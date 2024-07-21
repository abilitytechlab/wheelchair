// generate a voltage through a digital potentiometer
// center voltage should be 6V
// swing +/- 1 V

#include <Wire.h>

const int shutDownPin = 2;

void setup() {
  // put your setup code here, to run once:
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Serial.begin(115200);
  Wire.begin();

}
int val = 256;
void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 250; i<val; i++){
  //Serial.println(value);
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(i);
  int error = Wire.endTransmission();
  Serial.println(i);
  
  delay(1000);
  }
}


//  -- END OF FILE --