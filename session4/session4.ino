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

//toggle
bool toggle = LOW; //initial state of LED - turn off (toggle is LOW (0))
int ct = 0;

//time
unsigned long t_init = micros();

//frequency
const byte mask= B11111000; // mask bits that are not prescale
int prescale = 1; //fastest possible

//PI variables
float y_ant = 0, i_ant = 0, e_ant = 0;




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
*     Function: initialization()
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
float initialization(){

  int v0, v1;
  float i0, i1, i, gain;
  
  v0 = analogRead(sensorPin);
  i0 = read_lux(v0);
  
  analogWrite(ledPin, 255); 
  v1 = analogRead(sensorPin);
  i1 = read_lux(v0);

  i = i1 - i0;
  gain = i/255.0;
  
  return gain;
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
  i_lux = convert_V_lux(V_r);
  
  return i_lux;
  
}



/**************************************************************************
*
*     Function: convert_V_lux(float V_r)
*     
*     Arguments: No arguments
*     Return value: state of the toggle
*     
*     Description: 
* 
**************************************************************************/
double convert_V_lux(float V_r){
  int R1 = 10000;
  float R_ldr;
  double i_lux;

  //Calibration of LDR
  float m = -0.363;
  float b = log10(20515.0); 
  
  R_ldr = (5.0-V_r)*(R1/V_r);
  i_lux = pow(10.0, ((log10(R_ldr)-b)/m ));

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
  Serial.println(i_lux);    
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
*     Function: feedback_control(float err)
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
int feedback_control(float lux_des, float lux_obs)
{
  float kp = 2 , ki = 30;
  float k1, k2, p, i, e, y, u;
  float T = 0.059; //3*constant of time(correspond to 95% of the response) for 50 lux (tau(50lux) = 0.0196)
  float b = 0.5;
 
  float err;
  
  k1 = kp * b;
  k2 = kp * ki * (T/2);
  
  err = lux_des - lux_obs;

  //deadzone
  //if(err <0 && abs(err)<3) err = 0;
  
  //proportional
  p = (k1*lux_des) - (kp*lux_obs); 

  //integral
  i = i_ant + k2*(e + e_ant);

  u = p + i;
  
  y_ant = lux_obs;
  i_ant = i;
  e_ant = err;
  
  return int (u);    
}



/**************************************************************************
*
*     Function: controller (float ill_des, float t_init, float v_obs, float v_i)
*     
*     Arguments: No arguments
*     Return value: No return value
*     
*     Description: 
* 
**************************************************************************/
int controller (float ill_des, float t_init, float v_obs, float v_i){
  
  float v_des, err, fdbk;
  int u_fb, u_ff, u;
  double lux_des, lux_obs; 

  delay(59);

  v_des = simulator(ill_des, v_i, t_init);
  u_ff = feedforward_control(ill_des);
  
  lux_des = convert_V_lux(v_des);
  lux_obs = convert_V_lux(v_obs);

  u_fb = feedback_control(lux_des,lux_obs);

  u = u_fb + u_ff;
  if(u > 255){
    u = 255;
  }else if(u<0){
    u = 0;   
  }
  //Serial.println(u_fb);
  return u;
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
  float v_read;
  float gain;

  //gain = initialization();
  
  //verify_toggle(); 

 
  toggle=1;
  if(toggle) {
    //toggle is HIGH

    v_read = analogRead(sensorPin)/205.205;
    u_des = controller(100, t_init, v_read, 0); 
    //Serial.println(v_read);
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
