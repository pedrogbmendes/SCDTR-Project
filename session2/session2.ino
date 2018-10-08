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
int ct = 0;




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
   boolean aux1 = debounce_toggle();
   
   if (aux1 == 0 && ct == 0){
    toggle = !toggle;
    ct++;
   } else if (aux1 == 1) {
    ct = 0;
   }
}



boolean  debounce_toggle()
{
    // debouncereturns true if the switch in the given pin
    // is closed and stable

    int debounceDelay = 100;
    
    boolean previousState;
    boolean state;
    
    // store switch state
    previousState= digitalRead(switchPin); 
    
    for(int counter=0; counter < debounceDelay; counter++) {
      // wait for 1 millisecond
      delay(1); 
      
      // read the pin
      state = digitalRead(switchPin); 
    
      if( state!= previousState){
        // reset the counter if the state changes
        counter = 0; 
        // and save the current state
        previousState = state; 
      }
    }
    // here when the switch state has been stable 
    // longer than the  debounceperiod
    return state;

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
  float m = (log10(4249.0/10876.0));
  float b = log10(32625.0); 
  float V_r, R_ldr;
  double i_lux;
  int rate;
  
  int aux = 0, aux11 = 0;
  int v_rate[26];
  double v_lux[26];
  
  toggle=1;

  if(toggle) {
    //toggle is HIGH
    while(aux<26){
      Serial.println(aux11);
      analogWrite(ledPin, aux11); // set the LED on
      delay(1000);
      
      rate = analogRead(sensorPin); // read the analog input
  
      //scales the blink rate between the min and max values
      //rate = map(rate, 0, 1023, minDuration, maxDuration);  
      //rate = constrain(rate, minDuration,maxDuration); // saturate
  
      v_rate[aux] = rate;
  
      V_r = (5*rate)/1023.0;
      R_ldr = (5-V_r)*(R1/V_r);
      i_lux = pow(10.0, ((log10(R_ldr)-b)/m ));

      v_lux[aux] = i_lux;
      aux= aux+1;
      aux11=aux11+10;
    }
    toggle = 0;
    analogWrite(ledPin, 0);
    delay(500);
    aux=0;
    
    Serial.println("\n v_rate:");
    while(aux<26){
      Serial.println(v_rate[aux]);
      aux=aux+1;
    }
    aux=0;
    Serial.println("\n\n v_lux:");
    while(aux<26){
      Serial.println(v_lux[aux]);
      aux=aux+1;
    }
    
  }else{
    //toggle is LOW - LED is turn off 
     analogWrite(ledPin, 0);
  }
  
}
