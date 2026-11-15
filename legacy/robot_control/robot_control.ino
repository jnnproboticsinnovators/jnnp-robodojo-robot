/*
  RoboDojo robot - Arduino Mega 2560 + Monster Moto Shield
  Receives one-letter commands from the Raspberry Pi over USB (115200 baud).

  KEYS (sent from the Pi):
    w = forward     s = backward     a = spin left     d = spin right
    x = stop        + = faster       - = slower        r = reset encoder counts
    u = tip cargo deck up (dump)      n = cargo deck down (normal)

  MOTORS: four GA25-370 gear motors, wired in parallel pairs
    Left pair  = shield channel A (pins 7, 8, 5)
    Right pair = shield channel B (pins 4, 9, 6)
    Direction for each side is set by MOTOR_DIR below (taken from the club's
    tested route code: the LEFT side runs reversed).
    Left encoder  : yellow -> 18, white -> 22
    Right encoder : yellow -> 19, white -> 24
    Encoders      : blue -> 5V, green -> GND
    Servo signal  : pin 32   (pins 9 and 11 are covered by the shield)
    LEDs          : 26 = status, 28 and 30 = reverse warning
*/

#include <Servo.h>

// ---------- Monster Moto Shield (fixed by the shield, do not change) ----------
const int INA[2] = {7, 4};    // direction A: Left, Right
const int INB[2] = {8, 9};    // direction B: Left, Right
const int PWM[2] = {5, 6};    // speed:       Left, Right
const int LEFT = 0, RIGHT = 1;

// Direction correction per side (from the tested route code):
//  1 = normal, -1 = reversed. If a side runs backward on W, flip its number.
const int MOTOR_DIR[2] = {-1, 1};   // Left, Right

// ---------- Encoders ----------
const int ENC_L_A = 18, ENC_L_B = 22;
const int ENC_R_A = 19, ENC_R_B = 24;
const int ENC_L_SIGN = 1;     // change to -1 if left count goes DOWN when driving forward
const int ENC_R_SIGN = 1;     // change to -1 if right count goes DOWN when driving forward

// ---------- Servo and LEDs ----------
const int SERVO_PIN = 32;     // uncovered extended header (pins 9 and 11 are under the shield)
const int DECK_DOWN = 10;     // servo angle with the deck flat (adjust after testing)
const int DECK_UP   = 100;    // servo angle with the deck tipped (adjust after testing)
const int LED_STATUS = 26, LED_REV1 = 28, LED_REV2 = 30;

// ---------- Settings ----------
const unsigned long TIMEOUT_MS = 500;   // stop motors if no command for 0.5 s
int speedVal = 80;                      // 0..255, starts gentle (route code used 40)

// ---------- State ----------
volatile long leftCount = 0, rightCount = 0;
unsigned long lastCmdTime = 0, lastReport = 0, lastBlink = 0;
char motion = 'x';
bool blinkState = false;
Servo deck;

void leftEncoder()  { if (digitalRead(ENC_L_B)) leftCount  += ENC_L_SIGN; else leftCount  -= ENC_L_SIGN; }
void rightEncoder() { if (digitalRead(ENC_R_B)) rightCount += ENC_R_SIGN; else rightCount -= ENC_R_SIGN; }

// spd > 0 forward, spd < 0 backward, 0 stop
void motor(int m, int spd) {
  spd *= MOTOR_DIR[m];
  if (spd > 0)      { digitalWrite(INA[m], HIGH); digitalWrite(INB[m], LOW); }
  else if (spd < 0) { digitalWrite(INA[m], LOW);  digitalWrite(INB[m], HIGH); spd = -spd; }
  else              { digitalWrite(INA[m], LOW);  digitalWrite(INB[m], LOW); }
  analogWrite(PWM[m], constrain(spd, 0, 255));
}

void applyMotion() {
  switch (motion) {
    case 'w': motor(LEFT,  speedVal); motor(RIGHT,  speedVal); break;
    case 's': motor(LEFT, -speedVal); motor(RIGHT, -speedVal); break;
    case 'a': motor(LEFT, -speedVal); motor(RIGHT,  speedVal); break;
    case 'd': motor(LEFT,  speedVal); motor(RIGHT, -speedVal); break;
    default:  motor(LEFT, 0);         motor(RIGHT, 0);         break;
  }
}

void updateLeds() {
  if (millis() - lastBlink > 250) { lastBlink = millis(); blinkState = !blinkState; }
  bool moving = (motion != 'x');
  // status LED: slow blink when idle, steady ON when moving
  digitalWrite(LED_STATUS, moving ? HIGH : (millis() / 1000) % 2);
  // reverse LEDs: alternate while reversing, off otherwise
  digitalWrite(LED_REV1, motion == 's' &&  blinkState);
  digitalWrite(LED_REV2, motion == 's' && !blinkState);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 2; i++) {
    pinMode(INA[i], OUTPUT); pinMode(INB[i], OUTPUT); pinMode(PWM[i], OUTPUT);
  }
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoder, RISING);

  pinMode(LED_STATUS, OUTPUT); pinMode(LED_REV1, OUTPUT); pinMode(LED_REV2, OUTPUT);

  deck.attach(SERVO_PIN);
  deck.write(DECK_DOWN);

  motion = 'x';
  applyMotion();
  Serial.println("READY");
}

void loop() {
  // 1) Commands from the Pi
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'w': case 's': case 'a': case 'd': case 'x':
        motion = c; lastCmdTime = millis(); applyMotion(); break;
      case '+': speedVal = min(255, speedVal + 20); applyMotion(); break;
      case '-': speedVal = max(40,  speedVal - 20); applyMotion(); break;
      case 'r': noInterrupts(); leftCount = 0; rightCount = 0; interrupts(); break;
      case 'u': deck.write(DECK_UP);   break;
      case 'n': deck.write(DECK_DOWN); break;
      default: break;   // ignore anything else (newlines, typos)
    }
  }

  // 2) Safety stop: no command for 0.5 s -> stop
  if (motion != 'x' && millis() - lastCmdTime > TIMEOUT_MS) {
    motion = 'x';
    applyMotion();
  }

  // 3) LEDs
  updateLeds();

  // 4) Report to the Pi 5 times per second:  ENC <left> <right> SPEED <speed>
  if (millis() - lastReport > 200) {
    lastReport = millis();
    noInterrupts(); long l = leftCount, r = rightCount; interrupts();
    Serial.print("ENC "); Serial.print(l); Serial.print(' '); Serial.print(r);
    Serial.print(" SPEED "); Serial.println(speedVal);
  }
}
