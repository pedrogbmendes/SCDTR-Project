//MASTER.INO
#include <Wire.h>

const int other_add = 4; //other dev address

void setup() {
  Serial.begin(9600);
  Wire.begin(); // join as a master
}

void loop() {
  char c;
  if(Serial.available() > 0 ) {
  c = Serial.read();
  Wire.beginTransmission(other_add);//get BUS
  Wire.write(c); //send byte to address on BUS
  Wire.endTransmission(); //release BUS
  }
}
