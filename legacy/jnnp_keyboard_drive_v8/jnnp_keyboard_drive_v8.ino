/*
  JNNP ROBOTICS CLUB - KEYBOARD DRIVE v8 (direction switches + spin turns)
  Arduino Mega 2560 + Monster Moto Shield
  Four GA25-370 motors wired in parallel pairs
    Left pair  = Channel A (pins 7, 8, 5)
    Right pair = Channel B (pins 4, 9, 6)

  KEYS (from the Pi program robot_teleop.py, hold the key down)
    w / RIGHT arrow = forward          s / LEFT arrow = backward
    a / UP arrow    = turn LEFT        d / DOWN arrow = turn RIGHT
    x / SPACE = stop   + / - = speed   u / n = deck up / down   r = reset encoders

  WHAT CHANGED FROM v6 (so the wheels stop struggling)
    * Higher default speeds: drive 170, turns 210 / 90
    * MIN_PWM floor: any moving wheel gets at least MIN_PWM, so a side never
      sits at a PWM too weak to turn the gearbox (it just hums)
    * Stronger, longer kick-start: 255 for 250 ms
    * Straight correction now SPEEDS UP the slow side instead of slowing the
      fast side, so neither side is starved of power
    * Turn speeds follow the + / - keys too
    * TURN_LOW may be negative: then the inner side reverses (spin turn)

  WHAT CHANGED FROM v7
    * LEFT_REVERSED / RIGHT_REVERSED switches: flip a side's direction in one
      place if it runs the wrong way (v7 drove BACKWARD on forward, so both
      are set to true here)
    * TURN_LOW = -150: inner side reverses, so all four motors help the robot
      turn (with +90 both sides went forward and the heavy robot did not turn)
*/

#include <Servo.h>

// ---------- Motor pins ----------
const int INA_A = 7, INB_A = 8, PWM_A = 5;   // LEFT pair
const int INA_B = 4, INB_B = 9, PWM_B = 6;   // RIGHT pair

// ---------- Direction switches ----------
// Lift the robot and hold FORWARD. If a side spins BACKWARD, flip its switch.
const bool LEFT_REVERSED  = true;            // v7 went backward on forward -> flipped
const bool RIGHT_REVERSED = true;            // v7 went backward on forward -> flipped

// ---------- Encoders ----------
const int ENC_L_A = 18, ENC_L_B = 22;        // left  yellow, white
const int ENC_R_A = 19, ENC_R_B = 24;        // right yellow, white

// ---------- Servo and LEDs ----------
const int SERVO_PIN = 32;
const int DECK_DOWN = 10;
const int DECK_UP   = 100;
const int LED_STATUS = 26, LED_REV1 = 28, LED_REV2 = 30;

// ---------- Speeds (0-255) ----------
int motorSpeed = 170;                        // straight forward / backward
const int SPEED_STEP = 10;
const int MIN_SPEED = 100;
const int MAX_SPEED = 255;

// Lowest PWM a moving wheel is allowed. Below this the GA25-370s just hum.
// Raise it if a wheel still stalls, lower it if slow driving is too fast.
const int MIN_PWM = 90;

const int KICK_SPEED = 255;                  // full-power burst when a move starts
const unsigned long KICK_TIME = 250;         // ms

// TURNING - set relative to motorSpeed so + / - changes turns too
//   TURN_HIGH_ADD : how much faster the outer side runs than motorSpeed
//   TURN_LOW      : inner side speed.
//                    positive  = smooth curve (both sides forward)
//                    0         = pivot on the inner wheels
//                    negative  = spin turn (inner side reverses) - tightest
const int TURN_HIGH_ADD = 40;                // outer side = motorSpeed + 40 (max 255)
const int TURN_LOW = -150;                   // negative = inner side reverses (spin turn)
const unsigned long TURN_KICK_TIME = 250;

// ---------- Drive-straight correction ----------
const bool  STRAIGHT_CORRECTION = true;      // set false to switch it off
const float KP = 0.8;                        // how strongly to correct (0.3 - 1.5)
const int   MAX_TRIM = 40;                   // maximum extra PWM for the slow side
const unsigned long CORRECT_EVERY = 50;      // ms

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
  Serial.println("READY v8 - keys: w a s d x + - u n r");
}

void loop() {
  // 1) Read keys
  while (Serial.available()) {
    char c = tolower(Serial.read());
    switch (c) {
      case 'w': case 's': case 'a': case 'd': case 'x':
        lastCmdTime = millis();
        if (c != motion) startMotion(c);
        break;
      case '+': motorSpeed = min(MAX_SPEED, motorSpeed + SPEED_STEP); applyMotion(); break;
      case '-': motorSpeed = max(MIN_SPEED, motorSpeed - SPEED_STEP); applyMotion(); break;
      case 'u': deck.write(DECK_UP);   break;
      case 'n': deck.write(DECK_DOWN); break;
      case 'r': noInterrupts(); leftCount = 0; rightCount = 0; interrupts(); break;
      default: break;
    }
  }

  // 2) Safety stop
  if (SAFETY_TIMEOUT && motion != 'x' && millis() - lastCmdTime > TIMEOUT_MS) startMotion('x');

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
  if (millis() - lastReport > 200) {
    lastReport = millis();
    noInterrupts(); long l = leftCount, r = rightCount; interrupts();
    Serial.print("ENC "); Serial.print(l); Serial.print(' '); Serial.print(r);
    Serial.print(" SPEED "); Serial.println(motorSpeed);
  }
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
  // ----- Straight: boost the SLOW side instead of starving the fast one -----
  int straight = kicking ? KICK_SPEED : motorSpeed;
  int boostL = (trim < 0) ? (int)(-trim) : 0;   // right faster -> boost left
  int boostR = (trim > 0) ? (int)( trim) : 0;   // left faster  -> boost right
  int left  = straight + boostL;
  int right = straight + boostR;
  int over = max(left, right) - 255;            // keep the ratio if we hit the top
  if (over > 0) { left -= over; right -= over; }

  // ----- Turns: outer side faster than straight speed, inner side TURN_LOW -----
  int high = min(255, motorSpeed + TURN_HIGH_ADD);
  int low  = TURN_LOW;
  if (kicking) {                                // same ratio, full power
    low  = (long)255 * low / high;
    high = 255;
  }

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
// Directions come from the club's tested route code (left side is reversed).
void driveLeft(int spd) {
  if (LEFT_REVERSED) spd = -spd;
  spd = withFloor(spd);
  if (spd >= 0) { digitalWrite(INA_A, LOW);  digitalWrite(INB_A, HIGH); }
  else          { digitalWrite(INA_A, HIGH); digitalWrite(INB_A, LOW);  spd = -spd; }
  analogWrite(PWM_A, spd);
}

void driveRight(int spd) {
  if (RIGHT_REVERSED) spd = -spd;
  spd = withFloor(spd);
  if (spd >= 0) { digitalWrite(INA_B, HIGH); digitalWrite(INB_B, LOW);  }
  else          { digitalWrite(INA_B, LOW);  digitalWrite(INB_B, HIGH); spd = -spd; }
  analogWrite(PWM_B, spd);
}

void stopMotors() {
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  digitalWrite(INA_A, LOW); digitalWrite(INB_A, LOW);
  digitalWrite(INA_B, LOW); digitalWrite(INB_B, LOW);
}
