/*
  JNNP ROBOTICS CLUB - DRIVE v10 = KEYBOARD DRIVE v9 + ROS 2 / Nav2 speed control
  ---------------------------------------------------------------------------
  NEW IN v10 (everything from v9 still works the same - the arrow teleop is unchanged):
    * Speed command from ROS (sent by robodojo_base.py on the Pi), one line:
          V <left> <right>\n        wheel speeds in encoder ticks per second
                                    (forward = positive on BOTH sides), e.g.  V 600 600
      The Arduino keeps each side at that speed with its encoders (feed-forward + PI).
      Like the keys, it stops by itself if no V line arrives for 0.5 s.
    * Encoder report every 50 ms (was 200 ms) so ROS gets smooth odometry.
    * The left encoder counts backwards when driving forward (it is mounted mirrored);
      the speed control corrects for that. The ENC report is still the raw count.
  ---------------------------------------------------------------------------
  (v9 notes below)
  Arduino Mega 2560 + Monster Moto Shield, 4 GA25-370 motors in parallel pairs
    Left pair  = Channel A (pins 7, 8, 5)     Right pair = Channel B (pins 4, 9, 6)

  Works with the Pi program robot_teleop_ubuntu.py (the Pi sends these letters):
    w = forward   s = backward   a = turn left   d = turn right   x = stop
    + / - = speed   u = dump (bin up, slowly)   n = bin down (slowly)   r = reset encoders

  SPEED: press + / - in the teleop to go faster / slower (40 ... 255, steps of 10).
  TURNS: spin on the spot (sides in opposite directions), strong start - instant turning.
  STOP : active brake - the motors are shorted to ground so the robot stops at once.

  WHAT CHANGED FROM v8 (to match your rebuilt robot and your test sketch)
    * Servo (dump bin) on pin 30, HOME 0 deg, DUMP 40 deg, moves slowly
    * Robot stops before dumping
    * Motor directions follow your test sketch: forward = INA HIGH, INB LOW on both sides
      (if a side runs backward on 'w', set its *_REVERSED switch to true)
    * Status LEDs moved off pin 30 (the servo uses it now)
    * Still has the 0.5 s safety stop: if the Pi stops sending, the motors stop
*/

#include <Servo.h>

// ---------- Motor pins ----------
const int INA_A = 7, INB_A = 8, PWM_A = 5;   // LEFT pair
const int INA_B = 4, INB_B = 9, PWM_B = 6;   // RIGHT pair

// ---------- Direction switches ----------
// Lift the robot and hold FORWARD. If a side spins BACKWARD, flip its switch.
const bool LEFT_REVERSED  = false;           // set true if the LEFT side runs backward on w
const bool RIGHT_REVERSED = false;           // set true if the RIGHT side runs backward on w

// ---------- Encoders ----------
const int ENC_L_A = 18, ENC_L_B = 22;        // left  yellow, white
const int ENC_R_A = 19, ENC_R_B = 24;        // right yellow, white

// ---------- Servo and LEDs ----------
const int SERVO_PIN = 30;                    // dump bin servo (from your test sketch)
const int DECK_DOWN = 0;                     // HOME_ANGLE: bin lowered
const int DECK_UP   = 40;                    // DUMP_ANGLE: bin tipped
const int LED_STATUS = 26, LED_REV1 = 28, LED_REV2 = 34;   // pin 30 is now the servo

// ---------- Speeds (0-255) ----------
int motorSpeed = 120;                        // starting speed (change live with + / -)
const int SPEED_STEP = 10;
const int MIN_SPEED = 40;                    // slowest you can go with -
const int MAX_SPEED = 255;

// Lowest PWM a moving wheel is allowed. Below this the GA25-370s just hum.
// Raise it if a wheel still stalls, lower it if slow driving is too fast.
const int MIN_PWM = 40;                      // lowest PWM for a moving wheel (raise if wheels hum and stop)

const int KICK_SPEED = 200;                  // short burst when a move starts (gets the heavy robot rolling)
const unsigned long KICK_TIME = 150;         // ms

// TURNING - SPIN IN PLACE: left and right sides run in OPPOSITE directions at equal power,
// so the robot turns on the spot straight away.
//   TURN_MIN_PWM : turns never use less than this, even at low drive speed (heavy robot needs it)
//   TURN_HIGH_ADD: at higher drive speeds, turn power = motorSpeed + this
const int TURN_MIN_PWM = 170;
const int TURN_HIGH_ADD = 40;
const int TURN_KICK_SPEED = 255;             // full power for the first moment of a turn
const unsigned long TURN_KICK_TIME = 200;    // ms

// ---------- Drive-straight correction ----------
const bool  STRAIGHT_CORRECTION = true;      // set false to switch it off
const float KP = 0.8;                        // how strongly to correct (0.3 - 1.5)
const int   MAX_TRIM = 40;                   // maximum extra PWM for the slow side
const unsigned long CORRECT_EVERY = 50;      // ms

// ---------- ROS speed control (V command) ----------
const int LEFT_ENC_SIGN  = -1;               // left encoder counts DOWN when driving forward
const int RIGHT_ENC_SIGN = +1;
const float V_FF  = 0.13;                    // PWM per (tick/s)  - first guess: PWM 120 ~ 920 ticks/s
const float V_KP  = 0.10;                    // PWM per (tick/s) of error
const float V_KI  = 0.30;                    // PWM per (tick/s * s) of error
const float V_I_MAX = 120;                   // limit of the integral part (PWM)
const int   V_MIN_STRAIGHT = 55;             // least PWM for a moving wheel when both sides go the same way
const int   V_MIN_SPIN     = 150;            // least PWM when the sides go opposite ways (skid-steer turn)
const int   V_DEADBAND     = 15;             // ticks/s - smaller targets count as "stop"
const unsigned long V_PERIOD = 50;           // ms between control updates

// ---------- Safety ----------
const bool SAFETY_TIMEOUT = true;            // false only for Serial Monitor testing
const unsigned long TIMEOUT_MS = 500;

// ---------- State ----------
volatile long leftCount = 0, rightCount = 0;
char motion = 'x';
unsigned long lastCmdTime = 0, lastReport = 0, lastCorrect = 0, kickUntil = 0;
bool kicking = false;
float trim = 0;                              // + means left is faster -> boost right
long prevL = 0, prevR = 0;
// V-mode state
float targetL = 0, targetR = 0;              // ticks/s, forward positive
float integL = 0, integR = 0;
long vPrevL = 0, vPrevR = 0;
unsigned long lastV = 0;
char lineBuf[32]; int lineLen = 0; bool inLine = false;
Servo deck;

void leftEncoder()  { if (digitalRead(ENC_L_B)) leftCount++;  else leftCount--;  }
void rightEncoder() { if (digitalRead(ENC_R_B)) rightCount++; else rightCount--; }

void setup() {
  Serial.begin(115200);
  pinMode(INA_A, OUTPUT); pinMode(INB_A, OUTPUT); pinMode(PWM_A, OUTPUT);
  pinMode(INA_B, OUTPUT); pinMode(INB_B, OUTPUT); pinMode(PWM_B, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoder, RISING);
  pinMode(LED_STATUS, OUTPUT); pinMode(LED_REV1, OUTPUT); pinMode(LED_REV2, OUTPUT);
  deck.attach(SERVO_PIN);
  deck.write(DECK_DOWN);
  stopMotors();
  Serial.println("READY v10 - keys: w a s d x + - u n r   ROS: V <left> <right>");
}

void loop() {
  // 1) Read keys
  while (Serial.available()) {
    char raw = Serial.read();
    if (inLine) {                                   // collecting a "V l r" line
      if (raw == '\n' || raw == '\r') { lineBuf[lineLen] = 0; inLine = false; handleV(lineBuf); }
      else if (lineLen < (int)sizeof(lineBuf) - 1) lineBuf[lineLen++] = raw;
      else inLine = false;                          // too long - throw it away
      continue;
    }
    if (raw == 'V') { inLine = true; lineLen = 0; continue; }
    char c = tolower(raw);
    switch (c) {
      case 'w': case 's': case 'a': case 'd': case 'x':
        lastCmdTime = millis();
        if (c != motion) startMotion(c);
        break;
      case '+': motorSpeed = min(MAX_SPEED, motorSpeed + SPEED_STEP); applyMotion(); break;
      case '-': motorSpeed = max(MIN_SPEED, motorSpeed - SPEED_STEP); applyMotion(); break;
      case 'u': startMotion('x'); moveDeck(DECK_UP); lastCmdTime = millis(); break;   // stop, then dump
      case 'n': moveDeck(DECK_DOWN); lastCmdTime = millis(); break;
      case 'r': noInterrupts(); leftCount = 0; rightCount = 0; interrupts(); break;
      default: break;
    }
  }

  // 2) Safety stop
  if (SAFETY_TIMEOUT && motion != 'x' && millis() - lastCmdTime > TIMEOUT_MS) startMotion('x');

  // 2b) ROS speed control
  if (motion == 'v' && millis() - lastV >= V_PERIOD) speedControl();

  // 3) End of kick-start
  if (kicking && millis() > kickUntil) {
    kicking = false;
    resetCorrection();
    applyMotion();
  }

  // 4) Drive-straight correction (only for W and S, after the kick)
  if (millis() - lastCorrect >= CORRECT_EVERY) {
    lastCorrect = millis();
    noInterrupts(); long l = leftCount, r = rightCount; interrupts();
    long dl = labs(l - prevL), dr = labs(r - prevR);   // ticks moved by each side
    prevL = l; prevR = r;
    if (STRAIGHT_CORRECTION && !kicking && (motion == 'w' || motion == 's')
        && dl > 0 && dr > 0) {                          // both encoders working
      trim += KP * (dl - dr);                           // left faster -> trim grows
      trim = constrain(trim, -MAX_TRIM, MAX_TRIM);
      applyMotion();
    }
  }

  // 5) LEDs
  bool blink = (millis() / 250) % 2;
  digitalWrite(LED_STATUS, motion != 'x' ? HIGH : (millis() / 1000) % 2);
  digitalWrite(LED_REV1, motion == 's' &&  blink);
  digitalWrite(LED_REV2, motion == 's' && !blink);

  // 6) Report to the Pi: ENC <left> <right> SPEED <speed>
  if (millis() - lastReport >= 50) {
    lastReport = millis();
    noInterrupts(); long l = leftCount, r = rightCount; interrupts();
    Serial.print("ENC "); Serial.print(l); Serial.print(' '); Serial.print(r);
    Serial.print(" SPEED "); Serial.println(motorSpeed);
  }
}

// ---------- V command: "V <left> <right>" (after the V) ----------
void handleV(char *txt) {
  char *end;
  long l = strtol(txt, &end, 10);
  if (end == txt) return;
  char *txt2 = end;
  long r = strtol(txt2, &end, 10);
  if (end == txt2) return;
  lastCmdTime = millis();
  targetL = abs(l) < V_DEADBAND ? 0 : l;
  targetR = abs(r) < V_DEADBAND ? 0 : r;
  if (targetL == 0 && targetR == 0) {
    if (motion != 'x') startMotion('x');           // brake
    return;
  }
  if (motion != 'v') {                             // entering speed mode
    motion = 'v';
    kicking = false;
    integL = integR = 0;
    noInterrupts(); vPrevL = leftCount; vPrevR = rightCount; interrupts();
    lastV = millis();
    speedControl();
  }
}

void speedControl() {
  unsigned long now = millis();
  float dt = (now - lastV) / 1000.0;
  lastV = now;
  noInterrupts(); long l = leftCount, r = rightCount; interrupts();
  float measL = 0, measR = 0;
  if (dt > 0.005 && dt < 0.5) {
    measL = LEFT_ENC_SIGN  * (l - vPrevL) / dt;
    measR = RIGHT_ENC_SIGN * (r - vPrevR) / dt;
  } else dt = 0;
  vPrevL = l; vPrevR = r;

  float errL = targetL - measL, errR = targetR - measR;
  integL = constrain(integL + V_KI * errL * dt, -V_I_MAX, V_I_MAX);
  integR = constrain(integR + V_KI * errR * dt, -V_I_MAX, V_I_MAX);
  float pwmL = V_FF * targetL + V_KP * errL + integL;
  float pwmR = V_FF * targetR + V_KP * errR + integR;

  bool spin = (targetL > 0) != (targetR > 0) && targetL != 0 && targetR != 0;
  int minPwm = spin ? V_MIN_SPIN : V_MIN_STRAIGHT;
  driveLeftV (shapePwm(pwmL, targetL, minPwm));
  driveRightV(shapePwm(pwmR, targetR, minPwm));
}

// keep the wheel turning the way it was asked, with at least minPwm
int shapePwm(float pwm, float target, int minPwm) {
  if (target == 0) return 0;
  int sign = target > 0 ? 1 : -1;
  float mag = pwm * sign;                           // PWM in the wanted direction
  if (mag < minPwm) mag = minPwm;
  if (mag > 255) mag = 255;
  return (int)(mag * sign);
}

// same as driveLeft/driveRight but a 0 here means "hold this side still" (brake that side)
void driveLeftV(int spd) {
  if (spd == 0) { digitalWrite(INA_A, LOW); digitalWrite(INB_A, LOW); analogWrite(PWM_A, 255); return; }
  driveLeft(spd);
}
void driveRightV(int spd) {
  if (spd == 0) { digitalWrite(INA_B, LOW); digitalWrite(INB_B, LOW); analogWrite(PWM_B, 255); return; }
  driveRight(spd);
}

// Move the bin slowly (1 degree every 10 ms) to protect the mechanism
void moveDeck(int target) {
  int now = deck.read();
  int step = target > now ? 1 : -1;
  for (int a = now; a != target; a += step) { deck.write(a); delay(10); }
  deck.write(target);
}

void resetCorrection() {
  trim = 0;
  noInterrupts(); prevL = leftCount; prevR = rightCount; interrupts();
}

void startMotion(char c) {
  motion = c;
  Serial.print("CMD "); Serial.println(c);
  resetCorrection();
  if (motion != 'x') {
    kicking = true;
    unsigned long k = KICK_TIME;
    if (motion == 'a' || motion == 'd') k = TURN_KICK_TIME;
    kickUntil = millis() + k;
  } else {
    kicking = false;
  }
  applyMotion();
}

void applyMotion() {
  if (motion == 'v') return;                        // ROS speed control is in charge
  // ----- Straight: boost the SLOW side instead of starving the fast one -----
  int straight = kicking ? max(KICK_SPEED, motorSpeed) : motorSpeed;
  int boostL = (trim < 0) ? (int)(-trim) : 0;   // right faster -> boost left
  int boostR = (trim > 0) ? (int)( trim) : 0;   // left faster  -> boost right
  int left  = straight + boostL;
  int right = straight + boostR;
  int over = max(left, right) - 255;            // keep the ratio if we hit the top
  if (over > 0) { left -= over; right -= over; }

  // ----- Turns: spin in place, both sides equal power in opposite directions -----
  int high = constrain(max(TURN_MIN_PWM, motorSpeed + TURN_HIGH_ADD), 0, 255);
  if (kicking) high = TURN_KICK_SPEED;
  int low = -high;

  switch (motion) {
    case 'w': driveLeft(+left);  driveRight(+right); break;   // forward
    case 's': driveLeft(-left);  driveRight(-right); break;   // backward
    case 'a': driveLeft(low);    driveRight(high);   break;   // turn left
    case 'd': driveLeft(high);   driveRight(low);    break;   // turn right
    default:  stopMotors();                          break;
  }
}

// Any non-zero speed is raised to at least MIN_PWM so the wheel actually turns
int withFloor(int spd) {
  if (spd == 0) return 0;
  int mag = constrain(abs(spd), MIN_PWM, 255);
  return spd > 0 ? mag : -mag;
}

// ----- One side at a time. Positive = forward, negative = backward. -----
// Directions as in your test sketch: forward = INA HIGH, INB LOW.
void driveLeft(int spd) {
  if (LEFT_REVERSED) spd = -spd;
  spd = withFloor(spd);
  if (spd >= 0) { digitalWrite(INA_A, HIGH); digitalWrite(INB_A, LOW);  }
  else          { digitalWrite(INA_A, LOW);  digitalWrite(INB_A, HIGH); spd = -spd; }
  analogWrite(PWM_A, spd);
}

void driveRight(int spd) {
  if (RIGHT_REVERSED) spd = -spd;
  spd = withFloor(spd);
  if (spd >= 0) { digitalWrite(INA_B, HIGH); digitalWrite(INB_B, LOW);  }
  else          { digitalWrite(INA_B, LOW);  digitalWrite(INB_B, HIGH); spd = -spd; }
  analogWrite(PWM_B, spd);
}

// ACTIVE BRAKE: INA = INB = LOW with PWM on shorts each motor to ground,
// so the robot stops at once instead of rolling on.
void stopMotors() {
  digitalWrite(INA_A, LOW); digitalWrite(INB_A, LOW);
  digitalWrite(INA_B, LOW); digitalWrite(INB_B, LOW);
  analogWrite(PWM_A, 255);
  analogWrite(PWM_B, 255);
}
