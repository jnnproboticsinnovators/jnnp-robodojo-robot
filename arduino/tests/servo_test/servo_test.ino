/*
  SERVO TEST - no Raspberry Pi needed.
  Moves the deck servo on pin 32 slowly back and forth so you can SEE it work.
  Open Tools > Serial Monitor at 115200 to see the angle.

  Wiring:
    Servo orange/white (signal) -> Arduino pin 32
    Servo red   (power)         -> Buck #2 OUT+  (6 V)
    Servo brown/black (ground)  -> Buck #2 OUT-
    Buck #2 OUT-                -> Arduino GND   (common ground - required!)
*/
#include <Servo.h>

const int SERVO_PIN = 32;
const int ANGLE_A = 10;      // start angle
const int ANGLE_B = 90;      // end angle (big movement so it is easy to see)

Servo deck;

void setup() {
  Serial.begin(115200);
  deck.attach(SERVO_PIN);
  Serial.println("Servo test starting...");
}

void sweep(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int a = from; a != to + step; a += step) {
    deck.write(a);
    if (a % 10 == 0) { Serial.print("angle "); Serial.println(a); }
    delay(15);
  }
}

void loop() {
  sweep(ANGLE_A, ANGLE_B);
  delay(1000);
  sweep(ANGLE_B, ANGLE_A);
  delay(1000);
}
