/**************************************************************************

                  Distributed Real-time Control Systems

      Project:
         - Real-Time Cooperative Decentralized Control of a Smart
  Office Illumination System: Part1

      Authors:
        - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ulisboa.pt
        - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt

                       1st semester, 2018/19
                    Instítuto Superior Técnico


**************************************************************************/


/**************************************************************************

     Includes

**************************************************************************/
#include <Wire.h>
#include <EEPROM.h>


/**************************************************************************

      Defines

**************************************************************************/

//msg_type
#define READ_MY_LED "RML"
#define READ_YOUR_LED "RYL"
#define CONF_READ_YOUR_LED "CYL"
#define DONE_READ "DR_"

//consensus messages type
#define SEND_RESULT "SRD"

#define address 2




/**************************************************************************

      Define Global variables

**************************************************************************/

//Ports used
const int ledPin = 3; // LED connected to digital pin 3 (PWM - timer2)
const int switchPin = 2; // LED connected to digital pin 2 (Switch state)
const int sensorPin = 0; // connect sensor to analog input 0

const int bus_add = 1; //other dev address

//Initialization
int Vnoise = 0;

//comunication
int my_address;
volatile bool endcali = false;
volatile bool read_led = false;
volatile bool end_read = false;
volatile bool flag_turnON = false;
int orig_addr = 0;


//consensus
float dn[2];
float rho = 0.07;
float cost = 0;
float d_neigh[2] = {0,0};
volatile bool flag_consensus = false;
float lu = 0.0;


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
float gain[4]; //static gain for each led

//PI variables
float y_ant = 0, i_ant = 0, e_ant = 0, u_ant = 0; //previous values
float u_wdp = 0; //windup
float p = 0, i = 0;

//variables of controller
float ill_des, t_change; //luminace desire and new initial time
float v_obs = 0, v_i = 0, v_des = 0; //observed, initial and desired tension
float error = 0; //error of desired and observed luminance
int u_fb, u_ff; //feedback and feedforward
double lux_des = 0, lux_obs = 0; //desired and observed luminance
float dz;

//Debug and demonstration
String inChar; //string to read anf input
int flag_wdp = 1; //(des)activate windup function
int flag_pro = 1; //(des)activate proportional controler
int flag_int = 1; //(des)activate integral controler
int flag_ff = 0; //(des)activate feedforward
int flag_dz = 1; //(des)activate deadzone function
int flag_fl = 1; //(des)activate flickering function
int flag_dv = 0; //(des)activate demonstration/print of tension
int flag_dl = 1; //(des)activate demonstration/print of luminance
int flag_dg = 0; //(des)activate demonstration/print of gain
bool flag_setLed = LOW; //(des)activate demosntration of Led dimming values

int pwm_towrite = 0;
String pwm_TW;

//frequency
unsigned long t1=0, t2=0;



/**************************************************************************

      CLASSE DEFINITION

**************************************************************************/
class INIT_cali{
  public:
    void init_calibration();
  private:
    void init_noise(); //read the noise and external luminance
    void initialization();//calibration process

};


struct node_class{
  int index;
  float d[2];
  float d_av[2];
  float y[2];
  float k[2];
  float n;
  float m;
  float c;
  float o;
  float L;
};

class node_class node;



/**************************************************************************

      STEP UP

**************************************************************************/
void setup() {

  pinMode(ledPin, OUTPUT); // enable output on the led pin
  pinMode(switchPin, INPUT_PULLUP);

  Serial.begin(115200);

  EEPROM.write(0, address);
  my_address = EEPROM.read(0);

  Wire.begin(my_address);
  Wire.onReceive(receive_msg);

  set_frequency();

  INIT_cali calib;
  calib.init_calibration();

  Serial.println(gain[0]);
  Serial.println(gain[1]);

  concensus(50.0);
  Serial.println(lu);
    
  v_i = 0;
  t_init = micros();
  t_change = t_init;
  t1 = t_init;
  //Enable interruption
  //sei();

}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void set_frequency(){

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

void control_interrupt(){
}

/*****************************END_OF_SETUP********************************/





/**************************************************************************

      CLASSE calibration

**************************************************************************/



  /**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

  **************************************************************************/
  void INIT_cali::init_calibration(){
    init_noise(); //read the noise and external luminance
    delay(1000);
    initialization();//calibration process
  }


  /**************************************************************************

        Function:

        Arguments:
        Return value:

        Description: )

  **************************************************************************/
  void INIT_cali::init_noise(){

    analogWrite(ledPin, 0);//turn off the led
    delay(50);
    Vnoise = analogRead(sensorPin); //read the noise

  }


  /**************************************************************************

        Function:

        Arguments:
        Return value:

        Description: )

  **************************************************************************/
  void INIT_cali::initialization() {

    int v1, v;
    float i;
    char str_send[5];
    int Vread = 0;
    float lumi = 0;

    if (my_address == 1){
      flag_turnON = true;
    }

    while(endcali == false){

      if(flag_turnON == true){
        //leader - begin the calibration process sending a message
        analogWrite(ledPin, 255);
        delay(1000);

        //send a msg warning to the other nodes to read my value led plus my address
        sprintf(str_send, "%s%d",READ_MY_LED , my_address);

        Wire.beginTransmission(bus_add);
        Wire.write(str_send);
        Wire.endTransmission();
        flag_turnON = false;
      }
      else if(end_read ==  true){
        //when the node receives the confirmation from all the other nodes
        Vread = analogRead(sensorPin); //read the iluminance value + noise
        lumi = read_lux(Vread) - read_lux(Vnoise);//subtrat the noise
        gain[my_address-1] = lumi / 255.0;//calculates the linera gain (lux/pwm)
        //Serial.println(lumi);
        end_read = false;
        analogWrite(ledPin, 0);//turn off the led
        delay(30);

        if(my_address == 1){
          //send a message to the other arduino informing that it wants to read the other node led on value
          sprintf(str_send, "%s%d",READ_YOUR_LED , my_address);
          Wire.beginTransmission(bus_add);
          Wire.write(str_send);
          Wire.endTransmission();
        }else if(my_address == 2){
          //send a message to the other arduino informing that it wants to read the other node led on value
          sprintf(str_send, "%s%d",DONE_READ , my_address);
          Wire.beginTransmission(bus_add);
          
          Wire.write(str_send);
          Wire.endTransmission();
          endcali = true;
        }
      }
      else if(read_led == true){
        /*when receives this message means that the other arduino is going to read is own led value
        so this arduino has to turn off his own led and read the lux of the other led*/
        analogWrite(ledPin, 0);//turn off the led
        delay(30);

        Vread = analogRead(sensorPin); //read the iluminance value + noise
        lumi = read_lux(Vread) - read_lux(Vnoise);//subtrat the noise
        //Serial.println(lumi);
        gain[orig_addr-1] = lumi / 255.0;//calculates the liner gain (lux/pwm)
        read_led = false;

        //send a message to the other arduino informing that it wants to read the other node led on value
        sprintf(str_send, "%s%d",CONF_READ_YOUR_LED , my_address);
        Wire.beginTransmission(bus_add);
        Wire.write(str_send);
        Wire.endTransmission();
      }
    }
  }

/*****************************END_OF_CLASS***********************************/





/**************************************************************************

      PROTOCOL OF MESSAGES

**************************************************************************

type                  purpose
COUNT_NODES           count all nodes
NODE                  inform ending of counting nodes
READ_MY_LED           inform all the nodes to read the sender ledON value
READ_YOUR_LED         inform a node that another node wants to read its ledON value
CONF_READ_YOUR_LED    inform that a node read the ledON value of the receiving node
DONE_READ             inform calibration is finish


All messages are broadcast in the bus to all the nodes.

**************************************************************************/


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void receive_msg(int numBytes){

  char msg_recv[20];
  char type_msg [4], str_send[7];
  char num_d1_msg[8], num_d2_msg[8];
  int c = 0;
  int j = 0;

  while(Wire.available() > 0) { //check data on BUS
    msg_recv[c] = Wire.read(); //receive byte at I2S BUS
    c++;
  }
  msg_recv[c] = '\0';

  for (j=0; j<3; j++){
    type_msg[j] = msg_recv[j];
  }
  type_msg[3] = '\0';
  orig_addr = msg_recv[3] - '0';//converto to int
  Serial.println(msg_recv);

  if(strcmp(type_msg, READ_MY_LED) == 0){
      read_led = true;

  }else if(strcmp(type_msg, CONF_READ_YOUR_LED) == 0) {
      end_read = true;

  }else if(strcmp(type_msg, READ_YOUR_LED) == 0) {
      flag_turnON = true;

  }else if(strcmp(type_msg, DONE_READ) == 0){
     endcali = true;
     
  }else if(strcmp(type_msg, SEND_RESULT) == 0){   
    j = 4;
    c = 0;

    while(msg_recv[j] != '_'){
      num_d1_msg[c] = msg_recv[j];
      j ++;
      c ++;
    }
    num_d1_msg[c] = '\0';
    d_neigh[0] = atof(num_d1_msg);
  
    j ++;
    c = 0;
 
    while(msg_recv[j] != '\0'){
      num_d2_msg[c] = msg_recv[j];      
      j ++;
      c ++;
    }
    num_d2_msg[c] = '\0'; 
    d_neigh[1] = atof(num_d2_msg);
    
    flag_consensus = true; 
  }

}

/*****************************END_OF_PROTOCOL*******************************/




/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void concensus(float lumi_desire){

  char str_send[30];
  char char_d0[10], char_d1[10];
  
  node.index = my_address-1;
  node.d[0] = 0.0;
  node.d[1] = 0.0;
  node.d_av[0] = 0.0;
  node.d_av[1] = 0.0;
  node.y[0] = 0.0;
  node.y[1] = 0.0;  
  node.k[0] = gain[0];
  node.k[1] = gain[1];
  node.n = sq(node.k[0]) + sq(node.k[1]);//sq(sqrt(sq(node.k[0]) + sq(node.k[1])));
  node.m = node.n - sq(node.k[node.index]);
  node.c = 1;
  node.o = read_lux(Vnoise);
  node.L = lumi_desire;


  for(int h=0; h<10; h++){
    primal_solve();// return the cost and dn[2]
    node.d[0] = dn[0];
    node.d[1] = dn[1];
  
    //each node needs to send d
    if (node.d[0] < 10){
        dtostrf(node.d[0],4,2,char_d0);
    }else if (node.d[0] > 10 && node.d[0]<100){
        dtostrf(node.d[0],5,2,char_d0);
    }else if (node.d[0] > 100){
        dtostrf(node.d[0],6,2,char_d0);
    }
    if (node.d[1] < 10){
        dtostrf(node.d[1],4,2,char_d1);
    }else if (node.d[1] > 10 && node.d[1]<100){
        dtostrf(node.d[1],5,2,char_d1);
    }else if (node.d[1] > 100){
        dtostrf(node.d[1],6,2,char_d1);
    }

    //send a message with the updated data
    sprintf(str_send, "%s%d%s_%s", SEND_RESULT, my_address, char_d0, char_d1);
    Serial.println("send:");
    Serial.println(h);
    Serial.println(str_send);
    Wire.beginTransmission(bus_add);
    Wire.write(str_send);
    Wire.endTransmission();
    
    while(!flag_consensus){}//waiting for the response
    flag_consensus = false;
    
    //Compute average with available data
    node.d_av[0] = (node.d[0] + d_neigh[0])/2;
    node.d_av[1] = (node.d[1] + d_neigh[1])/2;

    //Update local lagrangians
    node.y[0]  = node.y[0] + rho*(node.d[0]-node.d_av[0]);
    node.y[1]  = node.y[1] + rho*(node.d[1]-node.d_av[1]);

  }

  lu = node.k[0]*node.d_av[0] + node.k[1]*node.d_av[1] + node.o;
}



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void primal_solve(){
  float d_best[2] = {-1.0, -1.0};
  float  cost_best = 1000000;

  int sol_unconstrained = 1;
  int sol_boundary_linear = 1;
  int sol_boundary_0 = 1;
  int sol_boundary_100 = 1;
  int sol_linear_0 = 1;
  int sol_linear_100 = 1;

  float cost_unconstrained = 0;
  float cost_boundary_linear = 0;
  float cost_boundary_0 = 0;
  float cost_boundary_100 = 0;
  float cost_linear_0 = 0;
  float cost_linear_100 = 0;

  float z[2];
  z[0]= rho*node.d_av[0] - node.y[0];
  z[1]= rho*node.d_av[1] - node.y[1];
  z[node.index] = z[node.index] - node.c;

  float d[2];
  float d_u[2];
  d_u[0]= (1/rho)*z[0];
  d_u[1]= (1/rho)*z[1];

  //unconstrained minimum
  sol_unconstrained = check_feasibility(node, d_u);
  if(sol_unconstrained == 1){
    cost_unconstrained = evaluate_cost(node, d_u);
    if (cost_unconstrained < cost_best){
      dn[0] = d_u[0];
      dn[1] = d_u[1];
      cost = cost_unconstrained;
      return;
    }
  }

  //compute minimum constrained to linear boundary
  float d_bl[2];
  float p1[2], p2[2], p22, p23;

  p1[0] = ((1/rho)*z[0]);
  p1[1] = ((1/rho)*z[1]);

  p23 = (p1[0]*node.k[0])+(p1[1]*node.k[1]);
  p22 = node.o-node.L +  p23;
  p2[0] = node.k[0]/node.n * p22;
  p2[1] = node.k[1]/node.n * p22;

  d_bl[0] = p1[0]-p2[0];
  d_bl[1] = p1[1]-p2[1];

  //check feasibility of minimum constrained to linear boundary
  sol_boundary_linear = check_feasibility(node, d_bl);
  //compute cost and if best store new optimum
  if(sol_boundary_linear == 1){
    cost_boundary_linear = evaluate_cost(node, d_bl);
    if(cost_boundary_linear < cost_best){
      d_best[0] = d_bl[0];
      d_best[1] = d_bl[1];
      cost_best = cost_boundary_linear;
    }
  }

  //compute minimum constrained to 0 boundary
  float d_b0[2];
  d_b0[0] = (1/rho)*z[0];
  d_b0[1] = (1/rho)*z[1];
  d_b0[node.index] = 0;
  //check feasibility of minimum constrained to 0 boundary
  sol_boundary_0 = check_feasibility(node, d_b0);
  //compute cost and if best store new optimum
  if(sol_boundary_0 == 1){
    cost_boundary_0 = evaluate_cost(node, d_b0);
    if(cost_boundary_0 < cost_best){
      d_best[0] = d_b0[0];
      d_best[1] = d_b0[1];
      cost_best = cost_boundary_0;
    }
  }

  //compute minimum constrained to 100 boundary
  float d_b1[2];
  d_b1[0] = (1/rho)*z[0];
  d_b1[1] = (1/rho)*z[1];
  d_b1[node.index] = 100;
  //check feasibility of minimum constrained to 100 boundary
  sol_boundary_100 = check_feasibility(node, d_b1);
  //compute cost and if best store new optimum
  if(sol_boundary_100 == 1){
    cost_boundary_100 = evaluate_cost(node, d_b1);
    if(cost_boundary_100 < cost_best){
      d_best[0] = d_b1[0];
      d_best[1] = d_b1[1];
      cost_best = cost_boundary_100;
    }
  }

  //compute minimum constrained to linear and 0 boundary
  float d_l0[2];
  float a1[2], a2[2], a3[2], a31, a32;

  a1[0] = ((1/rho)*z[0]);
  a1[1] = ((1/rho)*z[1]);

  a2[0] = (1/node.m)*node.k[0]*(node.o-node.L);
  a2[1] = (1/node.m)*node.k[1]*(node.o-node.L);

  a32 = z[0]*node.k[0] + z[1]*node.k[1];
  a31 = node.k[node.index]*z[node.index]- a32;

  a3[0] = (1/rho/node.m)*node.k[0]* a31;
  a3[1] = (1/rho/node.m)*node.k[1]* a31;

  d_l0[0] = a1[0]-a2[0]+a3[0];
  d_l0[1] = a1[1]-a2[1]+a3[1];

  d_l0[node.index] = 0;

  //check feasibility of minimum constrained to linear and 0 boundary
  sol_linear_0 = check_feasibility(node, d_l0);
  if(sol_linear_0 == 1){
    cost_linear_0 = evaluate_cost(node, d_l0);
    if(cost_linear_0 < cost_best){
      d_best[0] = d_l0[0];
      d_best[1] = d_l0[1];
      cost_best = cost_linear_0;
    }
  }

  //compute minimum constrained to linear and 100 boundary
  float d_l1[2];
  //a1[0] = ((1/rho)*z[0]);
  //a1[1] = ((1/rho)*z[1]);

  a2[0] = (1/node.m)*node.k[0]*(node.o-node.L+100*node.k[node.index]);
  a2[1] = (1/node.m)*node.k[1]*(node.o-node.L+100*node.k[node.index]);

  //a32 = z[0]*node.k[0] + z[1]*node.k[1];
  //a31 = node.k[node.index]*z[node.index]- a32;

  //a3[0] = (1/rho/node.m)*node.k[0]* a31;
  //a3[1] = (1/rho/node.m)*node.k[1]* a31;

  d_l1[0] = a1[0]-a2[0]+a3[0];
  d_l1[1] = a1[1]-a2[1]+a3[1];

  d_l1[node.index] = 100;

  //check feasibility of minimum constrained to linear and 100 boundary
  sol_linear_100 = check_feasibility(node, d_l1);
  if(sol_linear_100 == 1){
    cost_linear_100 = evaluate_cost(node, d_l1);
    if(cost_linear_100 < cost_best){
      d_best[0] = d_l1[0];
      d_best[1] = d_l1[1];
      cost_best = cost_linear_100;
    }
  }

  dn[0] = d_best[0];
  dn[1] = d_best[1];
  cost = cost_best;
  return;
}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int check_feasibility(node_class no, float dr[2]){

  float tol = 0.001;
  float diff;
  float aux;

  if(dr[no.index] < -tol){
    return 0;
  }else if(dr[no.index] > 100+tol ){
    return 0;
  }

  aux =  dr[0] * no.k[0] + dr[1] * no.k[1];;
  diff =  no.L - no.o - tol;
  if (aux < diff) {
    return 0;
  }

  return 1;
}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float evaluate_cost(node_class no, float dr[2]){
  float co;
  float part1, part2, part3, part4;
  float aux_dif[2];

  aux_dif[0] = dr[0]-no.d_av[0];
  aux_dif[1] = dr[1]-no.d_av[1];


  part1 = no.c*dr[no.index];
  part2 = no.y[0] * aux_dif[0] + no.y[1] * aux_dif[1];
  part3 = (rho/2)*(sq(aux_dif[0]) + sq(aux_dif[1]));//sqrt(sq(aux_dif[0]) + sq(aux_dif[1]))^2;

  co = part1 + part2 + part3;
  return co;
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

  return  Vc;
}









/**************************************************************************

      Function: change_led()

      Arguments: No arguments
      Return value: No return value

      Description: Receives the pwm value and change the Led's luminance

**************************************************************************/
void change_led(int u){
  //change the iluminance of the led
  analogWrite(ledPin, u);

}



void loop() {



}
