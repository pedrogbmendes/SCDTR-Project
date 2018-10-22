/**************************************************************************

                  Distributed Real-time Control Systems

      Project:
         - Real-Time Cooperative Decentralized Control of a Smart
  Office Illumination System

      Authors:
        - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ulisboa.pt
        - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt

                       1st semestre, 2018/19
                    Instítuto Superior Técnico


**************************************************************************/

/**************************************************************************

      Define of libraries

**************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>



/**************************************************************************

      Define Global variables

**************************************************************************/
const int ledPin = 3; // LED connected to digital pin 6 (PWM)
const int switchPin = 2; // LED connected to digital pin 2 (Switch state)
const int sensorPin = 0; // connect sensor to analog input 0


//toggle
bool toggle = LOW; //initial state of LED - turn off (toggle is LOW (0))
bool toggle_ant = HIGH;
int ct = 0;

//time
unsigned long t_init;

//frequency
const byte mask = B11111000; // mask bits that are not prescale timer2
int prescale2 = 1; //fastest possible

//interruptions 
int prescale1b = 4; 
const uint16_t t1_reset = 0;
/* time sampling 100Hz=0.01s
 * 16MHz frequency of oscilator
 * presacler = 256
 * (16M*0.01)/256 = 625 
*/
const uint16_t t1_comp = 625; 


//PI variables
float y_ant = 0, i_ant = 0, e_ant = 0, u_ant = 0;
float u_wdp = 0;


//define varibles of loop (main)
int u_des;
float gain;

//variables of controller
float ill_des, t_change, v_obs = 0, v_i;
bool flag_interrup = 0;

//variables needed to read samples
int counter = 0;
//bool flag_count = HIGH;


unsigned long t_read[100];
int q=0;

/**************************************************************************

      Function: setup()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void setup()
{
  pinMode(ledPin, OUTPUT); // enable output on the led pin
  pinMode(switchPin, INPUT_PULLUP);
  Serial.begin(9600); // initializeSerial
  
  TCCR2B = (TCCR2B & mask) | prescale2;//set the frequency to 31372.55 Hz

  //Enables the interruptions

  //reset timer control A
  TCCR1A = 0;
 
  // turn on CTC mode
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  
  //set the prescaler of 256
  //TCCR1B = (TCCR1B & mask) | prescale1b;
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
  
  //reset timer1 and set the compare value of 625 (=100Hz)
  TCNT1 = t1_reset;
  OCR1A = t1_comp;

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //Enable interruption 
 

  initialization();
  t_init = micros();
  t_change = t_init;
  v_i = 0;

   sei();
}


/**************************************************************************

      Function: INTERRUPTION

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/

ISR(TIMER1_COMPA_vect){
  controller();
}



/**************************************************************************

      Function: initialization()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void initialization() {

  int v0, v1, v;
  float i;

  delay(50);
  v0 = analogRead(sensorPin);

  analogWrite(ledPin, 255);
  delay(1000);
  v1 = analogRead(sensorPin);

  i = read_lux(v1) - read_lux(v0);


  gain = i / 255.0;

}



/**************************************************************************

      Function: verify_toggle()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void verify_toggle()
{
  boolean aux1 = debounce_toggle();

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
      Return value: state of the toggle

      Description:

**************************************************************************/
boolean  debounce_toggle()
{
  int debounceDelay = 100;

  boolean previousState;
  boolean state;

  // store switch state
  previousState = digitalRead(switchPin);

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

      Function: read_lux()

      Arguments: No arguments
      Return value: state of the toggle

      Description:

**************************************************************************/
double read_lux(int rate)
{
  int R1 = 10000;
  float V_r, R_ldr;
  double i_lux;

  //Calibration of LDR
  float m = -0.363;
  float b = log10(20515.0);


  V_r = rate / 205.205;
  i_lux = convert_V_lux(V_r);

  return i_lux;

}



/**************************************************************************

      Function: convert_V_lux(float V_r)

      Arguments: No arguments
      Return value: state of the toggle

      Description:

**************************************************************************/
double convert_V_lux(float V_r) {
  int R1 = 10000;
  float R_ldr;
  double i_lux;

  //Calibration of LDR
  float m = -0.363;
  float b = log10(20515.0);

  R_ldr = (5.0 - V_r) * (R1 / V_r);
  i_lux = pow(10.0, ((log10(R_ldr) - b) / m ));

  return i_lux;
}


/**************************************************************************

      Function: convert_lux_R()

      Arguments: No arguments
      Return value: state of the toggle

      Description:

**************************************************************************/
float convert_lux_R(float i_lux)
{

  //Calibration of LDR
  float m = -0.363;
  float b = log10(20515.0);

  return pow(10.0, (m * log10(i_lux) + b));


}


/**************************************************************************

      Function: convert_R_V()

      Arguments: No arguments
      Return value: state of the toggle

      Description:

**************************************************************************/
float convert_R_V(float R2)
{

  return  (5.0) / (1 + (R2 / 10000));

}





/**************************************************************************

      Function: change_led()

      Arguments: No arguments
      Return value: state of the toggle

      Description:

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

      Function: calculate_t_const()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
float calculate_t_const(int pwm_value)
{
  return (0.0106 + 0.028 * pow(10.0, (-0.009 * pwm_value)) );

  /*
    int R = 10000;
    double C = 1* pow(10.0,-6);
    float R_ldr, V_r;


    R_ldr = convert_lux_R(i_lux);
    return ((R*R_ldr)/(R+R_ldr))*C;
  */
}


/**************************************************************************

      Function: feedforward_control()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
int feedforward_control(float ill_desire)
{

  float pwm_des = (ill_desire + 20.8) / 0.6235;

  //float pwm_des = ill_desire / gain;


  return int(pwm_des);
}



/**************************************************************************

      Function: simulator()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
float simulator(float ill_desire, float v_ini, unsigned long t_ini)
{
  int R1 = 10000;
  float tau, R2;
  float v_f, y ;

  tau = calculate_t_const(ill_desire);

  R2 = convert_lux_R(ill_desire);
  v_f = convert_R_V(R2);

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
  float kp = 2, ki = 35;
  float k1, k2, p, i, e, y, u;
  float T = .01; //3*constant of time(correspond to 95% of the response) for 50 lux (tau(50lux) = 0.0196)
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
  p = (k1 * lux_des) - (kp * lux_obs);

    

  //integral
  i = i_ant + k2 * (err + e_ant) + u_wdp;

  u = p + i;

  if (u > 255) {
    u_sat = 255;
  } else if (u < 0) {
    u_sat = 0;
  } else {
    u_sat = u;
  }
//Serial.println(lux_obs);
  u_wdp = u_sat - u;

  y_ant = lux_obs;
  i_ant = i;
  e_ant = err;

  return int (u);
}



/**************************************************************************

      Function: controller ()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void controller () {

  float v_des, err, fdbk;
  int u_fb, u_ff;
  double lux_des, lux_obs;
  t_read[q] = micros();
  //average noise filter
  //v_obs = v_obs / counter;
  Serial.println(v_obs);
  
  v_des = simulator(ill_des, v_i, t_change);
  u_ff =  0 * feedforward_control(ill_des);

  lux_des = convert_V_lux(v_des);
  lux_obs = convert_V_lux(v_obs);
    

  err = lux_des - lux_obs;
  u_fb =  feedback_control(lux_des, lux_obs);
  

  u_des = u_fb + u_ff;

  //flickering effect
  if (u_ant <= 0 && u_des <= 3) {
    u_des = 0;
  }

  if (abs(err) < 2) {
    u_des = u_ant;
  }
  

  //saturation
  if (u_des > 255) {
    u_des = 255;
  } else if (u_des < 0) {
    u_des = 0;
  }

  change_led(u_des);
  

  u_ant = u_des;
  //flag_interrup = HIGH;
  //v_obs = 0;
  counter = 0;
  q++;
  if(q==100){
    q=0;
    }
  
}




/**************************************************************************

      Function: acquire_samples()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void acquire_samples() {
 
  counter++;
  v_obs =analogRead(sensorPin) / 205.205;
  /*
  if(flag_count){
      flag_count = LOW;
  }
*/
}




/**************************************************************************

      Function: loop()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void loop()
{

  verify_toggle();

  if (toggle) {
    //toggle is HIGH
    cli();
    if (!toggle_ant) {
      v_i = analogRead(sensorPin) / 205.205;
      t_change = micros();
      ill_des = 50;
      toggle_ant = HIGH;
    }
    for(int j=0;j<99;j++){
      Serial.println(t_read[j+1]-t_read[j]);
    }
    
    //acquire_samples();
    
    //scales the blink rate between the min and max values
    //rate = map(rate, 0, 1023, minDuration, maxDuration);
    //rate = constrain(rate, minDuration,maxDuration); // saturate
    //Serial.println("\n Rate:");
    //Serial.println(rate); // print rate to serial monitor

  } else {
    //toggle is LOW - LED is turn off

    if (toggle_ant) {
      v_i = analogRead(sensorPin) / 205.205;
      t_change = micros();
      ill_des = 20;
      toggle_ant = LOW;
    }

    acquire_samples();

  }


}
