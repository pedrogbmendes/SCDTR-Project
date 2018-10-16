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
const int ledPin= 9; // LED connected to digital pin 6 (PWM)
const int switchPin= 2; // LED connected to digital pin 2 (Switch state)
const int sensorPin= 0; // connect sensor to analog input 0
const int inputPin= 5; // connect sensor to analog input 5

// the next two lines set the min and max delay between blinks
const int minDuration= 100; // minimumwaitbetweenblinks
const int maxDuration= 1000; // maximum wait between blinks
bool toggle = LOW; //initial state of LED - turn off (toggle is LOW (0))
int ct = 0;

unsigned long t_init;

const byte mask= B11111000;
// mask bits that are not prescale
int prescale = 1;
//fastest possible

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
  TCCR1B = (TCCR1B & mask) | prescale;
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


/**************************************************************************
*
*     Function: debounce_toggle()
*     
*     Arguments: No arguments
*     Return value: state of the toggle
*     
*     Description: 
* 
**************************************************************************/
boolean  debounce_toggle()
{
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

    return state;

}




/**************************************************************************
*
*     Function: read_lux()
*     
*     Arguments: No arguments
*     Return value: state of the toggle
*     
*     Description: 
* 
**************************************************************************/
double read_lux(int rate)
{
  int R1 = 10000;
  float V_r, R_ldr;
  double i_lux;

  //Calibration of LDR
  float m = -0.363;
  float b = log10(20515.0); 
    
  
  V_r = rate/205.205;
  R_ldr = (5.0-V_r)*(R1/V_r);
  i_lux = pow(10.0, ((log10(R_ldr)-b)/m ));
  // Serial.println("\n lux:");
  Serial.println(V_r);
  // Serial.println(R_ldr);
  //Serial.println(i_lux);

  return i_lux;
  
}




/**************************************************************************
*
*     Function: change_led()
*     
*     Arguments: No arguments
*     Return value: state of the toggle
*     
*     Description: 
* 
**************************************************************************/
void change_led()
{
  double i_lux;
  int rate;
  float v_rate[450];
  int i;
  unsigned long t[450];

  
  //test1
  //analogWrite(ledPin, 127); //duty cycle 50%
  //analogWrite(ledPin, 255); //escalao


      //50
   for (i=0; i<50; i++){  
      analogWrite(ledPin, 50); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205; 
      Serial.println( v_rate[i]);
      Serial.println( t[i]);
   }
   
   //100
   for (i=50; i<100; i++){  
      analogWrite(ledPin, 100); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }

   //150
   for (i=100; i<150; i++){  
      analogWrite(ledPin, 150); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }

   //200
   for (i=150; i<200; i++){  
      analogWrite(ledPin, 200); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }

   //255
   for (i=200; i<250; i++){  
      analogWrite(ledPin, 255); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }
       
   //200
   for (i=250; i<300; i++){  
      analogWrite(ledPin, 200); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }

   //150
    for (i=300; i<350; i++){  
      analogWrite(ledPin, 150); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
    }

   //100
   for (i=350; i<400; i++){  
      analogWrite(ledPin, 100); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }
  
   //50
   for (i=400; i<450; i++){  
      analogWrite(ledPin, 50); //steps
      rate = analogRead(sensorPin);
      t[i] = micros()- t_init;
      v_rate[i] = rate/205.205;
      Serial.println( v_rate[i]);
      Serial.println( t[i]); 
   }
    analogWrite(ledPin, 0);

  delay(100000);

  /*
   for(i=0;i<250;i++){
      Serial.println( v_rat[i]);
   }
   for(i=0;i<200;i++){
      Serial.println( v_rate[i]);
   }
   for(i=0;i<250;i++){
      Serial.println( t1[i]);
   }
   for(i=0;i<200;i++){
      Serial.println( t2[i]);
   }*/
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

  t_init = micros();
  
  //verify_toggle(); 
  toggle=1;
  if(toggle) {
    //toggle is HIGH
       
    change_led();

    //scales the blink rate between the min and max values
    //rate = map(rate, 0, 1023, minDuration, maxDuration);  
    //rate = constrain(rate, minDuration,maxDuration); // saturate
    //Serial.println("\n Rate:");
    //Serial.println(rate); // print rate to serial monitor
 
  }else{
    //toggle is LOW - LED is turn off 
     analogWrite(ledPin, 0);
  }
  
  
}
