/*
  LEFT SIDE TEST - finds why the left wheels do not go forward.
  Lift the robot. Only the LEFT motors are driven, at full power:
    3 s FORWARD  (pin 7 LOW,  pin 8 HIGH)
    2 s stop
    3 s BACKWARD (pin 7 HIGH, pin 8 LOW)
    2 s stop
  ...repeats. Watch the left wheels AND the small DIR LED near the
  shield's left motor terminals. Serial Monitor at 115200 shows each step.
*/
const int INA_A = 7, INB_A = 8, PWM_A = 5;   // LEFT pair

void setup() {
  Serial.begin(115200);
  pinMode(INA_A, OUTPUT); pinMode(INB_A, OUTPUT); pinMode(PWM_A, OUTPUT);
}

void run(const char* name, int a, int b) {
  Serial.println(name);
  digitalWrite(INA_A, a); digitalWrite(INB_A, b); analogWrite(PWM_A, 255);
  delay(3000);
  analogWrite(PWM_A, 0); digitalWrite(INA_A, LOW); digitalWrite(INB_A, LOW);
  Serial.println("stop");
  delay(2000);
}

void loop() {
  run("LEFT FORWARD  (pin7 LOW,  pin8 HIGH)", LOW,  HIGH);
  run("LEFT BACKWARD (pin7 HIGH, pin8 LOW)",  HIGH, LOW);
}
