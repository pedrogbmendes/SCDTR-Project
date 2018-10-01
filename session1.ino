


const int ledPin= 6; // LED connected to digital pin 7
const int switchPin= 2; // LED connected to digital pin 7
const int sensorPin= 0; // connect sensor to analog input 0

// the next two lines set the min and max delay between blinks
const int minDuration= 100; // minimumwaitbetweenblinks
const int maxDuration= 1000; // maximum wait between blinks
bool toggle = LOW;

void setup()
{
  pinMode(ledPin, OUTPUT); // enable output on the led pin
  pinMode(switchPin, INPUT_PULLUP);
  Serial.begin(9600); // initializeSerial
}

void loop()
{
  int R1 = 10000;
  float m = log(0.2);
  float b = log(50000); 
  float V_r, R_ldr;
  double i_lux;

 int aux1 = digitalRead(switchPin);
  //Serial.println(aux1);

  if (aux1 == 0) {
    toggle = !toggle;
    delay(1000);
  }
      
  Serial.println("\n Toggle:");
  Serial.println(toggle);

  
  if(toggle) {
    int rate = analogRead(sensorPin); // read the analog input
    //scales the blink rate between the min and max   values
    rate = map(rate, 0, 1023, minDuration, maxDuration);  
    rate = constrain(rate, minDuration,maxDuration); // saturate
    Serial.println("\n Rate:");
    Serial.println(rate); // print rate to serial monitor

    V_r = (5*rate)/1023;
    R_ldr = (5-V_r)*(R1/V_r);
    i_lux = pow(10, ((log(R_ldr)-b)/m ) ); 
    Serial.println("\n LUX value:");
    Serial.println(i_lux);
    
    analogWrite(ledPin, 128); // set the LED on
    delay(rate); // wait duration dependent on light level
    //analogWrite(ledPin, LOW); // set the LED off
    //delay(rate);
  }
  



}
