/*
  MOTOR TEST - no Raspberry Pi needed.
  Upload from the laptop, keep the Arduino on the laptop USB,
  connect the battery to the shield + / -, lift the wheels,
  then open Tools > Serial Monitor at 115200 baud.

  Sequence (repeats):
    LEFT  forward 2 s -> stop
    RIGHT forward 2 s -> stop
    BOTH  forward 2 s -> stop
    BOTH  backward 2 s -> stop
*/

const int INA[2] = {7, 4};
const int INB[2] = {8, 9};
const int PWM[2] = {5, 6};
const int EN[2]  = {A0, A1};   // shield enable/fault pins: HIGH = OK
const int CS[2]  = {A2, A3};   // shield current sense
const char* NAME[2] = {"LEFT", "RIGHT"};
const int SPEED = 200;         // fairly strong push (0-255)

void motor(int m, int spd) {
  if (spd > 0)      { digitalWrite(INA[m], HIGH); digitalWrite(INB[m], LOW); }
  else if (spd < 0) { digitalWrite(INA[m], LOW);  digitalWrite(INB[m], HIGH); spd = -spd; }
  else              { digitalWrite(INA[m], LOW);  digitalWrite(INB[m], LOW); }
  analogWrite(PWM[m], spd);
}

void report(const char* step) {
  Serial.print(step);
  for (int m = 0; m < 2; m++) {
    Serial.print(" | "); Serial.print(NAME[m]);
    Serial.print(" EN="); Serial.print(digitalRead(EN[m]) ? "OK" : "LOW(fault/disabled)");
    Serial.print(" current="); Serial.print(analogRead(CS[m]));
  }
  Serial.println();
}

void runFor(const char* step, int l, int r) {
  motor(0, l); motor(1, r);
  delay(300);            // let current settle, then measure
  report(step);
  delay(1700);
  motor(0, 0); motor(1, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  for (int m = 0; m < 2; m++) {
    pinMode(INA[m], OUTPUT); pinMode(INB[m], OUTPUT); pinMode(PWM[m], OUTPUT);
    pinMode(EN[m], INPUT);
  }
  motor(0, 0); motor(1, 0);
  delay(1000);
  report("IDLE");
}

void loop() {
  runFor("LEFT forward ", SPEED, 0);
  runFor("RIGHT forward", 0, SPEED);
  runFor("BOTH forward ", SPEED, SPEED);
  runFor("BOTH backward", -SPEED, -SPEED);
  Serial.println("---- repeat ----");
}
