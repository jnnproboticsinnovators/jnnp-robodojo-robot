/*
  FULL POWER TEST - no Pi, no keyboard, no extra features.
  Uses the club's tested motor directions. Runs once, then stops.

    3 s  forward  at 255 (maximum)
    2 s  stop
    3 s  backward at 255
    2 s  stop
    3 s  turn left  (left back 255, right forward 255)
    2 s  stop
    3 s  turn right (left forward 255, right back 255)
    stop for good

  Put the robot ON THE FLOOR in a clear space. Press the Arduino RESET
  button to run it again.
*/
const int INA_A = 7, INB_A = 8, PWM_A = 5;   // LEFT pair
const int INA_B = 4, INB_B = 9, PWM_B = 6;   // RIGHT pair

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
void stopAll() { driveLeft(0); driveRight(0); digitalWrite(INA_A, LOW); digitalWrite(INB_A, LOW);
                 digitalWrite(INA_B, LOW); digitalWrite(INB_B, LOW); }

void step(const char* name, int l, int r, unsigned long ms) {
  Serial.println(name);
  driveLeft(l); driveRight(r);
  delay(ms);
  stopAll();
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  pinMode(INA_A, OUTPUT); pinMode(INB_A, OUTPUT); pinMode(PWM_A, OUTPUT);
  pinMode(INA_B, OUTPUT); pinMode(INB_B, OUTPUT); pinMode(PWM_B, OUTPUT);
  stopAll();
  Serial.println("Starting in 3 seconds...");
  delay(3000);
  step("FORWARD 255",     255,  255, 3000);
  step("BACKWARD 255",   -255, -255, 3000);
  step("TURN LEFT 255",  -255,  255, 3000);
  step("TURN RIGHT 255",  255, -255, 3000);
  Serial.println("DONE");
}

void loop() {}
