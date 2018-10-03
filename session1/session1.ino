/**************************************************************************
*
*                 Distributed Real-time Control Systems
*   
*     Project: 
*        - Real-Time Cooperative Decentralized Control of a Smart
* Office Illumination System
* 
*     Authors:  
*       - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ulisboa.pt
*       - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt
*
*                      1st semestre, 2018/19
*                   Instítuto Superior Técnico  
* 
* 
**************************************************************************/





/**************************************************************************
*
*     Define Global variables
* 
**************************************************************************/
const int ledPin= 6; // LED connected to digital pin 6 (PWM)
const int switchPin= 2; // LED connected to digital pin 2 (Switch state)
const int sensorPin= 0; // connect sensor to analog input 0

// the next two lines set the min and max delay between blinks
const int minDuration= 100; // minimumwaitbetweenblinks
const int maxDuration= 1000; // maximum wait between blinks
bool toggle = LOW; //initial state of LED - turn off (toggle is LOW (0))




/**************************************************************************
*
*     Function: setup()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
void setup()
{
  pinMode(ledPin, OUTPUT); // enable output on the led pin
  pinMode(switchPin, INPUT_PULLUP);
  Serial.begin(9600); // initializeSerial
}



/**************************************************************************
*
*     Function: verify_toggle()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
void verify_toggle()
{
   int aux1 = digitalRead(switchPin);
   
   if (aux1 == 0) {
    toggle = !toggle;
    Serial.println("\n Toggle:");
    Serial.println(toggle);
    delay(1000);
   }
}



/**************************************************************************
*
*     Function: loop()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
void loop()
{

  //define varibles
  int R1 = 10000;
  float m = log(0.2);
  float b = log(50000); 
  float V_r, R_ldr;
  double i_lux;
  int rate;


  verify_toggle();
  
  if(toggle) {
    //toggle is HIGH
       
    rate = analogRead(sensorPin); // read the analog input

    //verify the toggle value periodically 
    verify_toggle();

    //scales the blink rate between the min and max values
    rate = map(rate, 0, 1023, minDuration, maxDuration);  
    rate = constrain(rate, minDuration,maxDuration); // saturate
    //Serial.println("\n Rate:");
    //Serial.println(rate); // print rate to serial monitor

    //verify the toggle value periodically 
    verify_toggle();

    V_r = (5*rate)/1023.0;
    R_ldr = (5-V_r)*(R1/V_r);
    i_lux = pow(10.0, ((log(R_ldr)-b)/m ));

    //verify the toggle value periodically 
    verify_toggle();
    
    analogWrite(ledPin, 255); // set the LED on

    //verify the toggle value periodically 
    verify_toggle();
    
    delay(rate); // wait duration dependent on light level
    //analogWrite(ledPin, LOW); // set the LED off
    //delay(rate);

    //verify the toggle value periodically 
    verify_toggle();
    
  }else{
    //toggle is LOW - LED is turn off 
     analogWrite(ledPin, 0);
  }
  
  
}
