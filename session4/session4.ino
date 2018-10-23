/**************************************************************************

                  Distributed Real-time Control Systems

      Project:
         - Real-Time Cooperative Decentralized Control of a Smart
  Office Illumination System: Part1

      Authors:
        - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ulisboa.pt
        - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt

                       1st semestre, 2018/19
                    Instítuto Superior Técnico


**************************************************************************/



/**************************************************************************

      Define Global variables

**************************************************************************/

//Ports used
const int ledPin = 3; // LED connected to digital pin 3 (PWM - timer2)
const int switchPin = 2; // LED connected to digital pin 2 (Switch state)
const int sensorPin = 0; // connect sensor to analog input 0

//toggle
bool toggle = LOW; //initial state of LED - turn off (toggle is LOW (0))
bool toggle_ant = HIGH; //previous state
int ct = 0; //counter toggle

//time
unsigned long t_init; 

//frequency
const byte mask = B11111000; // mask bits that are not prescale timer2
int prescale2 = 1; //fastest possible: f = 31372.55Hz

//interruptions 
int prescale1b = 4; //prescaler is 256
const uint16_t t1_reset = 0;
const uint16_t t1_comp = 625; 
/* time sampling 100Hz=0.01s
 * 16MHz frequency of oscilator
 * presacler = 256
 * (16M*0.01)/256 = 625     */

//define varibles of loop (main)
int u_des;  //pwm signal desire to apply on led
float gain; //static gain of the system 

//PI variables
float y_ant = 0, i_ant = 0, e_ant = 0, u_ant = 0; //previous values
float u_wdp = 0; //windup

//variables of controller
float ill_des, t_change; //luminace desire and new initial time
float v_obs = 0, v_i = 0, v_des = 0; //observed, initial and desired tension
float error = 0; //error of desired and observed luminance   
int u_fb, u_ff; //feedback and feedforward
double lux_des = 0, lux_obs = 0; //desired and observed luminance

//Debug and demonstration
String inChar; //string to read anf input
int flag_wdp = 0; //(des)activate windup function
int flag_pro = 0; //(des)activate proportional controler
int flag_int = 0; //(des)activate integral controler
int flag_ff = 0; //(des)activate feedforward
int flag_dz = 0; //(des)activate deadzone function
int flag_fl = 0; //(des)activate flickering function
int flag_dv = 0; //(des)activate demonstration/print of tension
int flag_dl = 0; //(des)activate demonstration/print of luminance
int flag_dg = 0; //(des)activate demonstration/print of gain
bool flag_setLed = 0; //(des)activate demosntration of Led dimming values

int pwm_towrite = 0;
String pwm_TW;


/**************************************************************************

      Function: setup()

      Arguments: No arguments
      Return value: No return value

      Description: Initializes the arduino

**************************************************************************/
void setup(){
  pinMode(ledPin, OUTPUT); // enable output on the led pin
  pinMode(switchPin, INPUT_PULLUP);
 
  Serial.begin(9600); // initializeSerial
  
  TCCR2B = (TCCR2B & mask) | prescale2;//set the frequency to 31372.55Hz

  //Enables the interruptions

  //reset timer control A
  TCCR1A = 0;
 
  // turn on CTC mode - TCNT1=0
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  
  //set the prescaler of 256
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
  
  //reset timer1 and set the compare value of 625 (=100Hz)
  TCNT1 = t1_reset;
  OCR1A = t1_comp;

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  initialization(); //determine gain 
  v_i = 0;
  t_init = micros();
  t_change = t_init;

  //Enable interruption 
  sei();
}


/**************************************************************************

      Function: INTERRUPTION: ISR(TIMER1_COMPA_vect)

      Arguments: TIMER1_COMPA_vect
      Return value: No return value

      Description: This function is ativated each time that the interruption
  is triggered and runs the function control_interrupt()

**************************************************************************/
ISR(TIMER1_COMPA_vect){
   control_interrupt();
}


/**************************************************************************

      Function: initialization()

      Arguments: No arguments
      Return value: No return value

      Description: Calculate the system's gain

**************************************************************************/
void initialization() {

  int v0, v1, v;
  float i;

  delay(50);
  v0 = analogRead(sensorPin); //read the noise

  analogWrite(ledPin, 255);
  delay(1000);
  v1 = analogRead(sensorPin); //read the iluminance value + noise

  i = read_lux(v1) - read_lux(v0);//subtrat the noise

  gain = i / 255.0;//calculates the linera gain (lux/pwm)

  analogWrite(ledPin, 0);
  delay(3000);
}


/**************************************************************************

      Function: verify_toggle()

      Arguments: No arguments
      Return value: No return value

      Description: Verify if the variable toggle was pressed down
  If true (aux1=0), changes the value of toggle
  If true (aux1=1), mantains the value of toggle
  
**************************************************************************/
void verify_toggle(){
  
  boolean aux1 = debounce_toggle();// determine if the state change

  if (aux1 == 0 && ct == 0) {
    toggle = !toggle;
    ct++;
  } else if (aux1 == 1) {
    ct = 0;
  }
}


/**************************************************************************

      Function: debounce_toggle()

      Arguments: No arguments
      Return value: return the state
        - 1 if the state didn't change
        - 0 if the state changed
        
      Description: read the state until the stabilization of the debounce

**************************************************************************/
boolean  debounce_toggle(){
  int debounceDelay = 100;

  boolean previousState;
  boolean state;

  // store switch state
  previousState = digitalRead(switchPin);
  delay(1);
  state = digitalRead(switchPin);

  if ( state != previousState) {
    for (int counter = 0; counter < debounceDelay; counter++) {
      // wait for 1 millisecond
      delay(1);

      // read the pin
      state = digitalRead(switchPin);

      if ( state != previousState) {
        // reset the counter if the state changes
        counter = 0;
        // and save the current state
        previousState = state;
      }
    }
  }

  return state;
}


/**************************************************************************

      Function: read_lux(int rate)

      Arguments: value read
      Return value: luminance value

      Description: The function receives the integer value read by the port
  (a integer between 0 and 1023) and converts it in the respective voltage 
  value; then if calls the function convert_V_lux() to convert the voltage 
  in luminance

**************************************************************************/
double read_lux(int rate){

  float V_r;
  double i_lux;

  V_r = rate / 204.6;
  i_lux = convert_V_lux(V_r);

  return i_lux;
}


/**************************************************************************

      Function: convert_V_lux(float V_r)

      Arguments: read voltage 
      Return value: luminance

      Description: This function receives a voltage and  calculates the 
  correspondent value in lux; The values of this function were calibrated
  according to the caractheristic of the LDR

**************************************************************************/
double convert_V_lux(float V_r) {
  int R1 = 10000;
  float R_ldr;
  double i_lux;

  //Calibration of LDR
  float m = -0.62;
  float b = 4.8;

  R_ldr = (5.0 - V_r) * (R1 / V_r); // resistance of LDR
  i_lux = pow(10.0, ((log10(R_ldr) - b) / m )); //non-linear function 

  return i_lux;
}


/**************************************************************************

      Function: convert_lux_R(float i_lux)

      Arguments: luminance
      Return value: resistance
      
      Description: This function receives a value of luminance in lux and   
  determines the respective resistance value of the LDR

**************************************************************************/
float convert_lux_R(float i_lux){

  //Calibration of LDR
  float m = -0.62;
  float b = 4.8;

  return pow(10.0, (m * log10(i_lux) + b));
}


/**************************************************************************

      Function: convert_R_V(float R2)

      Arguments: resistance
      Return value: voltage

      Description: This function has a resistance value of LDR as an argument 
  and returns the voltage on voltage divider circuit

**************************************************************************/
float convert_R_V(float R2){

  return  (5.0) / (1 + (R2 / 10000));
}



/**************************************************************************

      Function: convert_lux_V(float ill_desire)

      Arguments: resistance
      Return value: voltage

      Description: This function has a resistance value of LDR as an argument 
  and returns the voltage on voltage divider circuit

**************************************************************************/
float convert_lux_V(float ill_desire){

  float R2, Vc;
  
  R2 = convert_lux_R(ill_desire);
  Vc = convert_R_V(R2);

  return  Vc
}


/**************************************************************************

      Function: change_led(int u)

      Arguments: pwm value
      Return value: No return value

      Description: Receives the pwm value and change the Led's luminance 

**************************************************************************/
void change_led(int u){
  double i_lux;
  int v_read;

  //change the iluminance of the led
  analogWrite(ledPin, u);

}


/**************************************************************************

      Function: calculate_t_const()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
float calculate_t_const(int pwm_value)
{
  return (0.0106 + 0.028 * pow(10.0, (-0.009 * pwm_value)) );
}


/**************************************************************************

      Function: feedforward_control(float ill_desire)

      Arguments:luminance disere
      Return value: pwm desire value

      Description: Feedforward control: receives the desire value of 
  luminance and calculates the pwm value to reach that respective luminance

**************************************************************************/
int feedforward_control(float ill_desire){
  
  float pwm_des = ill_desire / gain;
  return int(pwm_des);
}


/**************************************************************************

      Function: simulator(float ill_desire, float v_ini, unsigned long t_ini)

      Arguments: desire luminance, initial voltage and initial time
      Return value: 

      Description:

**************************************************************************/
float simulator(float ill_desire, float v_ini, unsigned long t_ini)
{
  int R1 = 10000;
  float tau;
  float v_f, y ;

  tau = calculate_t_const(ill_desire);


  y = v_f - (v_f - v_ini) * exp(-((micros() - t_ini) / tau) * pow(10.0, -6));

  return y;
}


/**************************************************************************

      Function: feedback_control(float err)

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
int feedback_control(float lux_des, float lux_obs)
{
  float kp = 0.06, ki = 350;
  float k1, k2, p, i, e, y, u;
  float T = .01; 
  float b = 1;
  float u_sat;

  float err;

  k1 = kp * b;
  k2 = kp * ki * (T / 2);
  
  err = lux_des - lux_obs;

  //deadzone
  //if(abs(err)<10){
  //err = 0;
  //}

  //proportional
  p = kp * err ;

if (abs(err) < flag_dz * 4) {
    err = 0;
  }

   

  //integral
  i = i_ant + k2 * (err + e_ant) + flag_wdp * u_wdp;

  u = flag_pro * p + flag_int * i ;

  if (u > 255 - u_ff) {
    u_sat = 255 - u_ff;
  } else if (u < - u_ff) {
    u_sat = - u_ff;
  } else {
    u_sat = u;
  }

  u_wdp = u_sat - u;

  y_ant = lux_obs;
  i_ant = i;
  e_ant = err;
//Serial.println(lux_obs);
  return int (u);
}


/**************************************************************************

      Function: controller ()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void controller() {
    
  v_des = simulator(ill_des, v_i, t_change);
  u_ff = flag_ff * feedforward_control(ill_des);
  

  lux_des = convert_V_lux(v_des);
  lux_obs = convert_V_lux(v_obs);
  
   
  error = lux_des - lux_obs;

  
  
}


/**************************************************************************

      Function: controller_interrupt ()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void control_interrupt(){
  
  //average noise filter
  //v_obs = v_obs / counter;
  u_fb = feedback_control(lux_des, lux_obs);
  
  u_des = u_fb + u_ff;

  //flickering effect
  if (u_ant <= 0 && u_des <= 3 && flag_fl == 1) {
    u_des = 0;
  }

  //if (abs(error) < 2) {
    //u_des = u_ant;
 // }
  
  //saturation
  if (u_des > 255) {
    u_des = 255;
  } else if (u_des < 0) {
    u_des = 0;
  }

  u_ant = u_des;
  //v_obs = 0;
  counter = 0;
}


/**************************************************************************

      Function: acquire_samples()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void acquire_samples() {
 
  counter++;
  v_obs =analogRead(sensorPin) / 204.6;

}


/**************************************************************************

      Function: loop()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void loop()
{

  if (Serial.available() > 0) {
    // read incoming serial data:
    if (!flag_setLed) inChar = Serial.readString();
    else pwm_TW = Serial.readString();
    // Type the next ASCII value from what you received:
    //Serial.println(inChar);
    
  }

  if(inChar == "w\n") {
    flag_wdp = 1 - flag_wdp;
    inChar = " ";
  } else if (inChar == "p\n") {
    flag_pro = 1 - flag_pro;
    inChar = " ";
  } else if (inChar == "i\n") {
    flag_int = 1 - flag_int;
    inChar = " ";
  } else if (inChar == "f\n") {
    flag_ff = 1 - flag_ff;
    inChar = " ";
  } else if (inChar == "d\n") {
    flag_dz = 1 - flag_dz;
    inChar = " ";
  } else if (inChar == "h\n") {
    flag_fl = 1 - flag_fl;
    inChar = " ";
  } else if (inChar == "l\n") {
    flag_dl = 1 - flag_dl;
    inChar = " ";
  } else if (inChar == "v\n") {
    flag_dv = 1 - flag_dv;
    inChar = " ";
  } else if (inChar == "g\n") {
    flag_dg = 1 - flag_dg;
    inChar = " ";
  } else if (inChar == "s\n") {
    flag_setLed = !flag_setLed;
    inChar = " ";
  }
  
  
  verify_toggle();

  if (toggle) {
    //toggle is HIGH
    
    if (!toggle_ant) {
      v_i = analogRead(sensorPin) / 204.6;
      t_change = micros();
      ill_des = 50;
      toggle_ant = HIGH;
    }

    
    acquire_samples();
    controller();
    change_led(u_des); 
    
  } else {
    //toggle is LOW - LED is turn off

    if (toggle_ant) {
      v_i = analogRead(sensorPin) / 204.6;
      t_change = micros();
      ill_des = 20;
      toggle_ant = LOW;
    }

    acquire_samples();
    controller();
    change_led(u_des);    
    
  }

  if (flag_dv){
    flag_pro = flag_int = flag_wdp = flag_ff = flag_dz = flag_fl = 1; 
    Serial.print(v_obs);
    Serial.print(" ");
    Serial.print(convert_R_V(convert_lux_R(ill_des)));
    Serial.print("\n");
  }

  if (flag_dl){
    flag_pro = flag_int = flag_wdp = flag_ff = flag_dz = flag_fl = 1; 
    Serial.print(read_lux(v_obs*204.6));
    Serial.print(" ");
    Serial.print(ill_des);
    Serial.print("\n");
  }

  if(flag_dg) {
    flag_pro = flag_int = flag_wdp = flag_ff = flag_dz = flag_fl = 0;
    change_led(pwm_towrite);
    delay(100);
    Serial.println(read_lux(analogRead(sensorPin)));
    if (pwm_towrite == 255) flag_dg = 0;

    pwm_towrite += 5;  
  }

  if (flag_setLed) {
    if (pwm_TW == "s\n") {
      flag_setLed = LOW;
      pwm_TW = " ";
    }
    else change_led(pwm_TW.toInt());

    Serial.println(pwm_TW);
    
  }

}
