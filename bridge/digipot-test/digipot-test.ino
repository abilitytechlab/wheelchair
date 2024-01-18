// generate a voltage through a digital potentiometer
// center voltage should be 6V
// swing +/- 1 V

#include <Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  int value = analogRead(A0);
  Serial.println(value);
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00
  Wire.write(value/4);
  Wire.endTransmission();
  delay(100);
}
