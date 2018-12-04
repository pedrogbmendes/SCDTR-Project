#include <Wire.h>

const int own_address = 4; //this dev address

void setup() {
  Serial.begin(115200);
  Wire.begin(own_address); // receive data
  Wire.onReceive(receiveEvent); //event handler
}

void loop() {
// EMPTY: all the work is done in receiveEvent
  }
  
void receiveEvent(int howMany) {
  while(Wire.available() > 0) { //check data on BUS
  char c = Wire.read(); //receive byte at I2S BUS
  Serial.write(c); // echo on terminal
  }
}
