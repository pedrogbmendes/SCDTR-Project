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

unsigned long t_init = micros();

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
  // Serial.println(R_ldr);
  //Serial.println(i_lux);

  return i_lux;
  
}


/**************************************************************************
*
*     Function: convert_lux_R()
*     
*     Arguments: No arguments
*     Return value: state of the toggle
*     
*     Description: 
* 
**************************************************************************/
float convert_lux_R(float i_lux)
{

    //Calibration of LDR
  float m = -0.363;
  float b = log10(20515.0); 
  
  return pow(10.0, (m*log10(i_lux) + b));

  
}


/**************************************************************************
*
*     Function: convert_R_V()
*     
*     Arguments: No arguments
*     Return value: state of the toggle
*     
*     Description: 
* 
**************************************************************************/
float convert_R_V(float R2)
{

  return  (5.0)/(1+(R2/10000));

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
void change_led(int u)
{
  double i_lux;
  int v_read;
 
  //test1
  analogWrite(ledPin, u); 
  v_read = analogRead(sensorPin);
  i_lux = read_lux(v_read);
     
}



/**************************************************************************
*
*     Function: calculate_gain()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
float calculate_gain(int pwm_value)
{
  return 0.8511*pwm_value - 39.48;
}



/**************************************************************************
*
*     Function: calculate_t_const()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
float calculate_t_const(int pwm_value)
{

  return (0.0106 + 0.028*pow(10.0, (-0.009*pwm_value)) );

  /*
  int R = 10000;
  double C = 1* pow(10.0,-6);
  float R_ldr, V_r;


  R_ldr = convert_lux_R(i_lux);
  return ((R*R_ldr)/(R+R_ldr))*C;
  */
}


/**************************************************************************
*
*     Function: feedforward_control()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
int feedforward_control(float ill_des)
{
  
  float pwm_des = (ill_des + 39.48) / 0.8511;
  return int(pwm_des);
}



/**************************************************************************
*
*     Function: simulator()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
float simulator(float ill_des, float v_i, unsigned long t_ini)
{
  int R1 = 10000;
  float pwm_des;
  float gain, tau, R2;
  float v_f, y ;
  
  pwm_des= (ill_des + 39.48) / 0.8511;
  //gain = calculate_gain(int (pwm_des) );
  tau = calculate_t_const(ill_des);


  R2 = convert_lux_R(ill_des);
  v_f = convert_R_V(R2);

  y = v_f - (v_f - v_i)*exp(-((micros()-t_ini)/tau)*pow(10.0,-6));
 
  return y;
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

  
  int u_des;
   
  //verify_toggle(); 
  toggle=1;
  if(toggle) {
    //toggle is HIGH

    u_des = feedforward_control(50);
    simulator(50, 0,t_init);
    change_led(u_des);
    

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
