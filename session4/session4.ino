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
unsigned long t_init;

//frequency
const byte mask= B11111000; // mask bits that are not prescale
int prescale = 1; //fastest possible

//PI variables
float y_ant = 0, i_ant = 0, e_ant = 0, u_ant = 0;
float u_wdp = 0;
float gain;


//define varibles of loop (main)
int u_des;
float v_read;
float gain;

//variables of controller
float ill_des, t_init, v_obs, v_i;

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

  Timer1.initialize(5000);
  Timer1.attachInterrupt(controller, );


  gain = initialization();
  t_init = micros();

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

  int v0, v1, v;
  float i, gain;

  delay(50);
  v0 = analogRead(sensorPin);

  analogWrite(ledPin, 255);
  delay(1000);
  v1 = analogRead(sensorPin);

  i = read_lux(v1) - read_lux(v0);


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
  //Serial.println(i_lux);
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
int feedforward_control(float ill_desire)
{

  float pwm_des = (ill_desire + 20.8) / 0.6235;

  //float pwm_des = ill_desire / gain;


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
float simulator(float ill_desire, float v_ini, unsigned long t_ini)
{
  int R1 = 10000;
  int pwm_des;
  float gain, tau, R2;
  float v_f, y ;

  tau = calculate_t_const(ill_desire);

  R2 = convert_lux_R(ill_desire);
  v_f = convert_R_V(R2);

  y = v_f - (v_f - v_ini)*exp(-((micros()-t_ini)/tau)*pow(10.0,-6));

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
  float kp = 55.8 , ki = 72;
  float k1, k2, p, i, e, y, u;
  float T = .059; //3*constant of time(correspond to 95% of the response) for 50 lux (tau(50lux) = 0.0196)
  float b = 0.5;
  float u_sat;

  float err;

  k1 = kp * b;
  k2 = kp * ki * (T/2);

  err = lux_des - lux_obs;

  //deadzone
  //if(abs(err)<10){
    //err = 0;
  //}

  //proportional
  p = (k1*lux_des) - (kp*lux_obs);

  //integral
  i = i_ant + k2*(e + e_ant) + u_wdp;

  u = p + i;

  if(u > 255){
    u_sat = 255;
  }else if(u<0){
    u_sat = 0;
  }else{
    u_sat = u;
  }

  u_wdp = u_sat - u;

  y_ant = lux_obs;
  i_ant = i;
  e_ant = err;

  return int (u);
}



/**************************************************************************
*
*     Function: controller ()
*
*     Arguments: No arguments
*     Return value: No return value
*
*     Description:
*
**************************************************************************/
void controller (){

  float v_des, err, fdbk;
  int u_fb, u_ff;
  double lux_des, lux_obs;


  v_des = simulator(ill_des, v_i, t_init);
  u_ff =  0 * feedforward_control(ill_des);

  lux_des = convert_V_lux(v_des);
  lux_obs = convert_V_lux(v_obs);
  Serial.println(v_obs);
  err = lux_des - lux_obs;
  u_fb =  feedback_control(v_des,v_obs);

  u_des = u_fb + u_ff;

  //flickering effect
  if(u_ant <= 0 && u_des <= 30){
        u_des = 0;
  }

  if(abs(err)<2){
    u_des = u_ant;
  }


  //saturation
  if(u_des > 255){
    u_des = 255;
  }else if(u<0){
    u_des = 0;
  }

  u_ant = u_des;

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



  //verify_toggle();
  ill_des = 30;
  v_i = 0;

  toggle=1;
  if(toggle) {
    //toggle is HIGH

    delay(59);

    v_obs = analogRead(sensorPin)/205.205;
    u_des = controller();
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
