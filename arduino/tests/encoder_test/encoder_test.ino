/*
  ENCODER TEST - checks that each encoder signal reaches the Arduino.
  No motors, no servo. Turn each wheel SLOWLY BY HAND and watch
  Tools > Serial Monitor at 115200.

  Each line shows:  L count | L pins A,B | R count | R pins A,B
  Turning a wheel should make its count change and its A/B pins flip 0/1.
*/
const int ENC_L_A = 18, ENC_L_B = 22;   // left  yellow, white
const int ENC_R_A = 19, ENC_R_B = 24;   // right yellow, white

volatile long leftCount = 0, rightCount = 0;
void leftEncoder()  { if (digitalRead(ENC_L_B)) leftCount++;  else leftCount--;  }
void rightEncoder() { if (digitalRead(ENC_R_B)) rightCount++; else rightCount--; }

void setup() {
  Serial.begin(115200);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoder, RISING);
  Serial.println("Turn each wheel slowly by hand...");
}

void loop() {
  noInterrupts(); long l = leftCount, r = rightCount; interrupts();
  Serial.print("LEFT count "); Serial.print(l);
  Serial.print("  pins A,B = "); Serial.print(digitalRead(ENC_L_A)); Serial.print(','); Serial.print(digitalRead(ENC_L_B));
  Serial.print("   |   RIGHT count "); Serial.print(r);
  Serial.print("  pins A,B = "); Serial.print(digitalRead(ENC_R_A)); Serial.print(','); Serial.println(digitalRead(ENC_R_B));
  delay(200);
}
