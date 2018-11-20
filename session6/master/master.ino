//MASTER.INO
#include <Wire.h>

const int other_add = 4; //other dev address
char buf[5];
int i=0;


void setup() {
  Serial.begin(9600);
  Wire.begin(); // join as a master
}

void loop() {
  
  char c;
  
  /*
   Wire.beginTransmission(other_add);//get BUS
   char d[3] = {48, 48, 48};
   Wire.write(d, 3); //send string to address on BUS
   
   Wire.endTransmission(); //release BUS
  delay(100);
  */
  if(Serial.available() > 0 ) {
    c = Serial.read();
    buf[i] = c;
       
    i++;
    
  }
  if(c == '\n' && i!= 0 ){
    
    char trans[i];

    for(int j=0;j<i;j++)
      trans[j] = buf[j];
 
    Wire.beginTransmission(other_add);//get BUS
  
    Wire.write(trans, i-1); //send string to address on BUS
    
    Wire.endTransmission(); //release BUS
    
    i=0;
  }


}
