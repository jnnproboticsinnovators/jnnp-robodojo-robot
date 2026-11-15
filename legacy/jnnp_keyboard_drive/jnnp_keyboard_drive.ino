/*
  JNNP ROBOTICS CLUB - KEYBOARD DRIVE
  Arduino Mega 2560 + Monster Moto Shield
  Four GA25-370 motors wired in parallel pairs
    Left pair  = Channel A (pins 7, 8, 5)
    Right pair = Channel B (pins 4, 9, 6)

  KEYS (from the Pi program robot_teleop.py, hold the key down)
    w / RIGHT arrow = forward          s / LEFT arrow = backward
    a / UP arrow    = turn LEFT        d / DOWN arrow = turn RIGHT
    x / SPACE = stop   + / - = speed   u / n = deck up / down   r = reset encoders

  FEATURES
    * Forward: both sides 120    Backward: both sides 80
    * Turn left : left -180 (back), right 230   Turn right: left 230, right -180 (back)
    * Kick-start: short full-power burst whenever a movement starts
    * Drive-straight correction: the encoders are compared every 50 ms and the
      faster side is slowed / slower side sped up, so W and S go straight
    * Safety stop if no key arrives for 0.5 s
    * Deck servo moves SLOWLY (1 degree every SERVO_STEP_MS) - driving keeps working while it moves
*/

#include <Servo.h>

// ---------- Motor pins ----------
const int INA_A = 7, INB_A = 8, PWM_A = 5;   // LEFT pair
const int INA_B = 4, INB_B = 9, PWM_B = 6;   // RIGHT pair

// ---------- Encoders ----------
const int ENC_L_A = 18, ENC_L_B = 22;        // left  yellow, white
const int ENC_R_A = 19, ENC_R_B = 24;        // right yellow, white

// ---------- Servo and LEDs ----------
const int SERVO_PIN = 32;
const int DECK_DOWN = 10;
const int DECK_UP   = 20;
// SLOW SERVO: the deck moves 1 degree every SERVO_STEP_MS milliseconds.
//   20 ms  -> about 50 degrees per second (gentle)
//   40 ms  -> about 25 degrees per second (slower)
//   10 ms  -> about 100 degrees per second (faster)
const unsigned long SERVO_STEP_MS = 40;
// The servo HOLDS the deck firmly closed all the time (from switch-on).
// It only opens when you press U, and closes when you press N.
// (A released servo is loose, so the deck could swing open when the robot
//  brakes or reverses - holding it stops that.)
const bool RELEASE_WHEN_DONE = false;
const unsigned long RELEASE_DELAY_MS = 500;  // wait this long after arriving, then release
const int LED_STATUS = 26, LED_REV1 = 28, LED_REV2 = 30;

// ---------- Speeds (0-255) ----------
int motorSpeed = 120;                        // FORWARD speed
int backSpeed  = 80;                         // BACKWARD speed
const int SPEED_STEP = 10;
const int MIN_SPEED = 50;
const int MAX_SPEED = 255;
const int KICK_SPEED = 200;                  // burst to start moving straight
const unsigned long KICK_TIME = 200;         // ms

// TURNING (UP = left, DOWN = right)
//   Turn LEFT : left wheels = TURN_LOW, right wheels = TURN_HIGH
//   Turn RIGHT: left wheels = TURN_HIGH, right wheels = TURN_LOW
//   Bigger difference between HIGH and LOW = shorter, faster turn
//   Negative TURN_LOW = inner wheels run BACKWARD, so both sides help rotate (best for a heavy robot)
const int TURN_HIGH = 230;                   // outer side (faster)
const int TURN_LOW  = -180;                  // inner side (slower) - lower = tighter turn
const unsigned long TURN_KICK_TIME = 200;    // same short start burst as straight


// ---------- Drive-straight correction ----------
const bool  STRAIGHT_CORRECTION = false;     // OFF: the left and right encoders count at different rates, so it was cutting power
const float KP = 0.8;                        // how strongly to correct (0.3 - 1.5)
const int   MAX_TRIM = 40;                   // maximum speed difference added per side
const unsigned long CORRECT_EVERY = 50;      // ms

// ---------- Safety ----------
const bool SAFETY_TIMEOUT = true;            // false only for Serial Monitor testing
const unsigned long TIMEOUT_MS = 500;

// ---------- State ----------
volatile long leftCount = 0, rightCount = 0;
char motion = 'x';
unsigned long lastCmdTime = 0, lastReport = 0, lastCorrect = 0, kickUntil = 0;
bool kicking = false;
float trim = 0;                              // + means slow the left, speed up the right
long prevL = 0, prevR = 0;
Servo deck;
int deckPos    = DECK_DOWN;                  // where the servo is now
int deckTarget = DECK_DOWN;                  // where it is going
unsigned long lastServoStep = 0;
unsigned long servoArrivedAt = 0;
bool servoOn = false;                        // false = servo not powered by the signal (limp)

void servoStart() {                          // switch the servo on at its current position
  if (!servoOn) {
    deck.write(deckPos);                     // set position BEFORE attaching, so it does not jump
    deck.attach(SERVO_PIN);
    servoOn = true;
  }
  servoArrivedAt = millis();                 // keep it switched on while it moves
}

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
  servoStart();                              // hold the deck closed from the start
  deckTarget = DECK_DOWN;
  stopMotors();
  Serial.println("READY - keys: w a s d x + - u n r");
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
      case '+': motorSpeed = min(MAX_SPEED, motorSpeed + SPEED_STEP);
                backSpeed  = min(MAX_SPEED, backSpeed  + SPEED_STEP); applyMotion(); break;
      case '-': motorSpeed = max(MIN_SPEED, motorSpeed - SPEED_STEP);
                backSpeed  = max(MIN_SPEED, backSpeed  - SPEED_STEP); applyMotion(); break;
      case 'u': servoStart(); deckTarget = DECK_UP;   break;   // only moves when U is pressed
      case 'n': servoStart(); deckTarget = DECK_DOWN; break;   // only moves when N is pressed
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

  // 4b) Move the deck servo one small step towards its target (slow, smooth)
  if (servoOn && deckPos != deckTarget && millis() - lastServoStep >= SERVO_STEP_MS) {
    lastServoStep = millis();
    deckPos += (deckTarget > deckPos) ? 1 : -1;
    deck.write(deckPos);
    servoArrivedAt = millis();
  }
  // switch the servo off after it has arrived, so it cannot twitch on its own
  if (RELEASE_WHEN_DONE && servoOn && deckPos == deckTarget
      && millis() - servoArrivedAt > RELEASE_DELAY_MS) {
    deck.detach();
    servoOn = false;
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
  int base     = (motion == 's') ? backSpeed : motorSpeed;     // forward 120, backward 80
  int straight = kicking ? max(KICK_SPEED, base) : base;
  int left  = constrain(straight - (int)trim, 0, 255);
  int right = constrain(straight + (int)trim, 0, 255);

  // during the start burst keep the same HIGH:LOW ratio, just stronger
  int high = kicking ? max(KICK_SPEED, TURN_HIGH) : TURN_HIGH;
  int low  = kicking ? (long)high * TURN_LOW / TURN_HIGH : TURN_LOW;

  switch (motion) {
    case 'w': driveLeft(+left);  driveRight(+right); break;   // forward
    case 's': driveLeft(-left);  driveRight(-right); break;   // backward
    case 'a': driveLeft(+low);   driveRight(+high);  break;   // turn left:  left back, right forward
    case 'd': driveLeft(+high);  driveRight(+low);   break;   // turn right: left forward, right back
    default:  stopMotors();                          break;
  }
}

// ----- One side at a time. Positive = forward, negative = backward. -----
// Directions come from the club's tested route code (left side is reversed).
void driveLeft(int spd) {
  if (spd >= 0) { digitalWrite(INA_A, LOW);  digitalWrite(INB_A, HIGH); }
  else          { digitalWrite(INA_A, HIGH); digitalWrite(INB_A, LOW);  spd = -spd; }
  analogWrite(PWM_A, spd);
}

void driveRight(int spd) {
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
