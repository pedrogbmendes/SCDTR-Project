/**************************************************************************

                  Distributed Real-time Control Systems

      Project:
         - Real-Time Cooperative Decentralized Control of a Smart
  Office Illumination System

      Authors:
        - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ulisboa.pt
        - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt

                       1st semester, 2018/19
                    Instítuto Superior Técnico


**************************************************************************/


/**************************************************************************

     Includes

**************************************************************************/
#include <WSWire.h>
#include <EEPROM.h>


/**************************************************************************

      Defines

**************************************************************************/

/*  PROTOCOL OF MESSAGES  */
//msg_type
#define READ_MY_LED "RML"
#define READ_YOUR_LED "RYL"
#define CONF_READ_YOUR_LED "CYL"
#define DONE_READ "DR_"
#define TURN_ON_CONSENSUS "TC_"

//consensus messages type
#define SEND_RESULT "SRD"

//address
#define address 1



/**************************************************************************

      Define Global variables

**************************************************************************/

//Ports used
const int ledPin = 3; // LED connected to digital pin 3 (PWM - timer2)
const int switchPin = 2; // LED connected to digital pin 2 (Switch state)
const int sensorPin = 0; // connect sensor to analog input 0

const int bus_add = 2; //other dev address

//Initialization
int Vnoise = 0;

//comunication
int my_address;
volatile bool endcali = false;
volatile bool read_led = false;
volatile bool end_read = false;
volatile bool flag_turnON = false;
volatile bool flag_turn_consensus = false;
int orig_addr = 0;

int rate;

//consensus
float dn[2];
float rho = 0.07;
float cost = 0;
float d_neigh[2] = {0,0};
volatile bool flag_consensus = false;
int d_con[2] = {0,0};


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
float gain[2]; //static gain for each led

//Calibration of LDR
float m=-0.62;
float b=4.8;

//PI variables
float y_ant = 0, i_ant = 0, e_ant = 0, u_ant = 0; //previous values
float u_wdp = 0; //windup
float p = 0, i = 0;

//variables of controller
float ill_des, t_change; //luminace desire and new initial time
float v_obs = 0, v_i = 0, v_des = 0; //observed, initial and desired tension
int u_fb, u_con; //feedback and feedforward
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
volatile bool flag_send_to_rasp = false;
volatile bool stop_int = true;

int pwm_towrite = 0;
String pwm_TW;

//frequency
unsigned long t1=0, t2=0;

int c = 0;



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


class consensus_class{
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
  
  public:
    void concensus(float node);
  private:
    void primal_solve(node_class node);
    int check_feasibility(node_class no, float dr[2]);
    float evaluate_cost(node_class no, float dr[2]);
    
  };

class controller{

  public:
    void control_interrupt();
  private:
    int feedback_control();
};

INIT_cali calib;
consensus_class algoritm_consensus;
controller control;



/**************************************************************************

      SETUP

**************************************************************************/
void setup() {

  pinMode(ledPin, OUTPUT); // enable output on the led pin
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  Serial.begin(2000000);

  //write the address in the eeprom
  EEPROM.write(0, address);
  my_address = EEPROM.read(0);

  //write the LDR calibration parameters in the eeprom
  /*EEPROM.write(1, -0.6403);
  m = EEPROM.read(1);  
  EEPROM.write(2, 4.9);
  b = EEPROM.read(2);*/

    
  Wire.begin(my_address);
   
  Wire.onReceive(receive_msg);
   
  set_frequency();

  calib.init_calibration();

  Serial.println(gain[0]);
  Serial.println(gain[1]);
  
  v_i = 0;
  t_init = millis();
  t_change = t_init;
  t1 = t_init;
  send_data_to_rasp('F');
  //Enable interruption
  sei();
  
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
   control.control_interrupt();
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
   
        send_msg(READ_MY_LED, my_address, 0, 0);
        
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
          send_msg(READ_YOUR_LED, my_address, 0, 0);

        } else if(my_address == 2){
          
          //send a message to the other arduino informing that it wants to read the other node led on value
          send_msg(DONE_READ, my_address, 0, 0);
          
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
        send_msg(CONF_READ_YOUR_LED, my_address, 0, 0);
      }
    }
  }

/*****************************END_OF_CLASS***********************************/




/**************************************************************************

      CLASS consensus

**************************************************************************/




  /**************************************************************************
  
        Function:
  
        Arguments:
        Return value:
  
        Description: )
  
  **************************************************************************/
  void consensus_class::concensus(float lumi_desire){
  
    char str_send[30];
    char char_d0[10], char_d1[10];
    node_class node;
  
    node.index = my_address-1;
    node.d[0] = 0.0;
    node.d[1] = 0.0;
    node.d_av[0] = 0.0;
    node.d_av[1] = 0.0;
    node.y[0] = 0.0;
    node.y[1] = 0.0;  
    //node.k[0] = gain[0];
    //node.k[1] = gain[1];
    node.k[0] = (gain[0]*255.0)/100;
    node.k[1] = (gain[1]*255.0)/100;
    node.n = sq(node.k[0]) + sq(node.k[1]);
    node.m = node.n - sq(node.k[node.index]);
    node.c = 1;
    node.o = read_lux(Vnoise);
    node.L = lumi_desire;
  
    
    
    for(int h=0; h<50; h++){

      primal_solve(node);// return the cost and dn[2]
      
      node.d[0] = dn[0];
      node.d[1] = dn[1];
      
      //each node needs to send d
      if (node.d[0] < 10){
          dtostrf(node.d[0],4,2,char_d0);
      }else if (node.d[0] >= 10 && node.d[0]<100){
          dtostrf(node.d[0],5,2,char_d0);
      }else if (node.d[0] >= 100){
          dtostrf(node.d[0],6,2,char_d0);
      }
      if (node.d[1] < 10){
          dtostrf(node.d[1],4,2,char_d1);
      }else if (node.d[1] >= 10 && node.d[1]<100){
          dtostrf(node.d[1],5,2,char_d1);
      }else if (node.d[1] >= 100){
          dtostrf(node.d[1],6,2,char_d1);
      }

      
      //send a message with the updated data
      send_msg(SEND_RESULT, my_address, node.d[0], node.d[1]);
      
      while(!flag_consensus){}//waiting for the response
      flag_consensus = false;
      
      //Compute average with available data
      node.d_av[0] = (node.d[0] + d_neigh[0])/2;
      node.d_av[1] = (node.d[1] + d_neigh[1])/2;
  
      //Update local lagrangians
      node.y[0]  = node.y[0] + rho*(node.d[0]-node.d_av[0]);
      node.y[1]  = node.y[1] + rho*(node.d[1]-node.d_av[1]);
  
  
    }
    d_con[0] = int((node.d_av[0]*255.0)/100);
    d_con[1] = int((node.d_av[1]*255.0)/100);

    u_con = flag_ff * d_con[my_address-1];
    
    lux_des = gain[0]* d_con[0] + gain[1]* d_con[1] + node.o; 
   
  }
  
  
  /**************************************************************************
  
        Function:
  
        Arguments:
        Return value:
  
        Description: )
  
  **************************************************************************/
  void consensus_class::primal_solve(node_class node){
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
        
        if (dn[1] > 100) {
          dn[1] = 100;
        }
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

    if (dn[0] > 100) {
      dn[0] = 100;
    }
    if (dn[1] > 100) {
      dn[1] = 100;
    }
    
    cost = cost_best;
    return;
  }
  
  
  /**************************************************************************
  
        Function:
  
        Arguments:
        Return value:
  
        Description: )
  
  **************************************************************************/
  int consensus_class::check_feasibility(node_class no, float dr[2]){
  
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
  
        Description: 
  
  **************************************************************************/
  float consensus_class::evaluate_cost(node_class no, float dr[2]){
    float co;
    float part1, part2, part3, part4;
    float aux_dif[2];
  
    aux_dif[0] = dr[0]-no.d_av[0];
    aux_dif[1] = dr[1]-no.d_av[1];
  
  
    part1 = no.c*dr[no.index];
    part2 = no.y[0] * aux_dif[0] + no.y[1] * aux_dif[1];
    part3 = (rho/2)*(sq(aux_dif[0]) + sq(aux_dif[1]));
  
    co = part1 + part2 + part3;
    return co;
  }

/*****************************END_OF_CLASS***********************************/



/**************************************************************************

      PROTOCOL OF MESSAGES

**************************************************************************

type                  purpose
READ_MY_LED           inform all the nodes to read the sender ledON value
READ_YOUR_LED         inform a node that another node wants to read its ledON value
CONF_READ_YOUR_LED    inform that a node read the ledON value of the receiving node
DONE_READ             inform calibration is finish

SEND_RESULT           send the values of consensus algorithm

-other messeges:
  - from the arduino to raspberry pi: update the database

.**************************************************************************/



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void receive_msg(int numBytes){

  char msg_recv[20];
  char type_msg [4], str_send[7];
  char num_d1_msg[4], num_d2_msg[4];
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
  orig_addr = (int) msg_recv[3];//converto to int
  //Serial.println(msg_recv);

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

    while(j != 8){
      num_d1_msg[c] = msg_recv[j];
      j ++;
      c ++;
    }

    d_neigh[0] = Bytes2float(num_d1_msg);
    
    //num_d1_msg[c] = '\0';    
    //d_neigh[0] = atof(num_d1_msg);

    //j ++;
    c = 0;
 
   while(j != 12){
      num_d2_msg[c] = msg_recv[j];
      j ++;
      c ++;
    }

    d_neigh[1] = Bytes2float(num_d2_msg);
    //num_d2_msg[c] = '\0'; 
    //d_neigh[1] = atof(num_d2_msg);
    
    flag_consensus = true;
  } else if(strcmp(type_msg, TURN_ON_CONSENSUS) == 0) {
    flag_turn_consensus = true;
  }

}


void send_msg(const char type[3], int add, float d0, float d1){
  byte to_float[4];
  byte to_send[12];
  
  if (strcmp(type, SEND_RESULT) == 0){
    

    to_send[0] = type[0];
    to_send[1] = type[1];
    to_send[2] = type[2];
    to_send[3] = (byte) add;

    float2Bytes(d0, to_float);

    to_send[4] = to_float[0];
    to_send[5] = to_float[1];
    to_send[6] = to_float[2];
    to_send[7] = to_float[3];

    float2Bytes(d1, to_float);

    to_send[8] = to_float[0];
    to_send[9] = to_float[1];
    to_send[10] = to_float[2];
    to_send[11] = to_float[3];

    Wire.beginTransmission(bus_add);
    Wire.write(to_send, 12);
    digitalWrite(13, LOW);
      Wire.endTransmission();
      digitalWrite(13, HIGH); 
      delay(1);
    
  } else {

    byte to_send[4];
    
    to_send[0] = type[0];
    to_send[1] = type[1];
    to_send[2] = type[2];
    to_send[3] = (byte) add;

    Wire.beginTransmission(bus_add);
    Wire.write(to_send, 4);
    digitalWrite(13, LOW);
      Wire.endTransmission();
      digitalWrite(13, HIGH);
      delay(1); 

  }
}


void float2Bytes(float val, byte * bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array

  memcpy(bytes_array, u.temp_array, 4);
  
}

float Bytes2float(char* to_conv){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.temp_array[0] = (byte) to_conv[0];
  u.temp_array[1] = (byte) to_conv[1];
  u.temp_array[2] = (byte) to_conv[2];
  u.temp_array[3] = (byte) to_conv[3];
  // Assign bytes to input array

  return u.float_variable;
  
}
/**************************************************************************

      Function:

      Arguments:mode
                  - mode=false -> no need to send the reference lux value
                  - mode=true ->  need to send the reference lux value
      Return value:

      Description: )

**************************************************************************/

void send_data_to_rasp(const char T){
  unsigned long t_send;
  char refLux_char[10], meaLux_char[10];
  char str_send[30];
  byte to_send[14];
  byte lux_obs_send[4], ref_lux_send[4], control_lux_send[4], noise_send[4];
  float ref_lux, meas_lux;
  int n;

  ref_lux = (int) ill_des;
  meas_lux = (float) lux_obs;

  t_send = millis() - t_init;

  switch (T) {
    case 'I':

      to_send[0] = (byte) T;
      
      to_send[1] = (byte) my_address;

      float2Bytes(meas_lux, lux_obs_send);
      
      to_send[2] = lux_obs_send[0];
      to_send[3] = lux_obs_send[1];
      to_send[4] = lux_obs_send[2];
      to_send[5] = lux_obs_send[3];

      to_send[6] = (byte) u_des;
      
      Wire.beginTransmission(0); //address of the raspberry pi
      Wire.write(to_send, 7);
      digitalWrite(13, LOW);
      n = Wire.endTransmission();
      digitalWrite(13, HIGH);
      Serial.println(n);
      delay(1);
      
      break;
    case 'C':

      to_send[0] = (byte) T;
      
      to_send[1] = (byte) my_address;

      float2Bytes(lux_des, control_lux_send);
      
      to_send[2] = control_lux_send[0];
      to_send[3] = control_lux_send[1];
      to_send[4] = control_lux_send[2];
      to_send[5] = control_lux_send[3];
      
      to_send[6] = (byte) ref_lux;
      
      Wire.beginTransmission(0); //address of the raspberry pi
      Wire.write(to_send, 7);
      digitalWrite(13, LOW);
      Wire.endTransmission();
      digitalWrite(13, HIGH);
      delay(1);
      
      break;
    case 'F':
      
      to_send[0] = (byte) T;
      
      to_send[1] = (byte) my_address;

      float2Bytes((float)convert_V_lux(Vnoise), noise_send);

      to_send[2] = (byte) (t_send >> 24);
      to_send[3] = (byte) (t_send >> 16);
      to_send[4] = (byte) (t_send >> 8);
      to_send[5] = (byte) (t_send);
      
      to_send[6] = noise_send[0];
      to_send[7] = noise_send[1];
      to_send[8] = noise_send[2];
      to_send[9] = noise_send[3];

      Wire.beginTransmission(0); //address of the raspberry pi
      Wire.write(to_send, 10);
      digitalWrite(13, LOW);
      Wire.endTransmission();
      digitalWrite(13, HIGH);
      delay(1); 
      
      break;
  }
}


void print_msg(byte to_send[14]) {
  float time_send, meas_lux, ref_lux;
  char aux[4];
  
  Serial.println(int (to_send[0]));

  aux[0] = to_send[5];
  aux[1] = to_send[6];
  aux[2] = to_send[7];
  aux[3] = to_send[8];
  
  meas_lux = Bytes2float(aux);

  aux[0] = to_send[10];
  aux[1] = to_send[11];
  aux[2] = to_send[12];
  aux[3] = to_send[13];
  
  ref_lux = Bytes2float(aux);
  
  Serial.println(meas_lux);
  Serial.println(ref_lux);
  
}




/***************************************************************************
  
                      FUNCTIONS OF THE CONTROLLER
 
 **************************************************************************/



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


/**************************************************************************

      Function: feedback_control(float lux_des, float lux_obs)

      Arguments: desired and observed luminance
      Return value: pwm value

      Description: The feedback control function receives the desired and
  observed iluminance and calculates the error between these two. This contro-
  ler is a PID, that means that has a proportional and a integral part. The
  error multiples by the proportional gain. The integral part integrates the
  error

**************************************************************************/
int controller::feedback_control(){
  float kp = 0.07, ki = 350;
  float k2, u;
  float T = .01;
  float u_sat, err;

  k2 = kp * ki * (T / 2);

  //calculates the error between the desired and observed luminance
  err = lux_des - lux_obs;

  /*deadzone - if teh error is very small, it's aproximanted by zero */
  if (abs(err) < flag_dz * dz) {
      err = 0;
    }

  //proportional part
  p = kp * err ;

  //integral part
  i = i_ant + k2 * (err + e_ant) + flag_wdp * u_wdp;

  //control signal
  u = flag_pro * p + flag_int * i ;

  //Saturation of the signal control
  if (u > 255 - u_con) {
    u_sat = 255 - u_con;
  } else if (u < - u_con) {
    u_sat = - u_con;
  } else {
    u_sat = u;
  }

  //Anti-windup: used to create a boundary for the integrated error
  u_wdp = u_sat - u;

  //stores the value for next time
  y_ant = lux_obs;
  i_ant = i;
  e_ant = err;

  return int (u);
}


/**************************************************************************

      Function: controller_interrupt ()

      Arguments: No arguments
      Return value: No return value

      Description: Function that runs when the interruption is triggered.
  Running the feedback function and using the feedforward calculates the
  control  value (pwm) to apply to the led.

**************************************************************************/
void controller::control_interrupt(){
  if(!stop_int){
    t2 = t1;
  t1 = micros();

  
  u_fb = feedback_control();
  u_des = u_fb + u_con;
  
  //flickering effect
  if (u_ant <= 0 && u_des <= 3 && flag_fl == 1) {
    u_des = 0;
  }

  //saturation
  if (u_des > 255) {
    u_des = 255;
  } else if (u_des < 0) {
    u_des = 0;
  }

  u_ant = u_des;

  flag_send_to_rasp = true;
  }
  
}


/**************************************************************************

      Function: acquire_samples()

      Arguments: No arguments
      Return value: No return value

      Description: Read the value of voltage using the analog input
  port 0 (sensorPin) and converts the read value in voltage.

**************************************************************************/
void acquire_samples(){
  v_obs = analogRead(sensorPin) / 204.6;
  lux_obs = convert_V_lux(v_obs);
}


/**************************************************************************

      Function: loop()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void new_consensus_result(){
  
  acquire_samples();
  send_data_to_rasp(true);
  
}

/**************************************************************************

      Function: loop()

      Arguments: No arguments
      Return value: No return value

      Description:

**************************************************************************/
void loop() {
  stop_int =false; 
  verify_toggle();
 
  
  if (flag_turn_consensus) {
    stop_int =true;
    algoritm_consensus.concensus(ill_des);
    flag_turn_consensus = false;
    stop_int =false;
    Serial.println("end");
    
  }
  
  if (toggle) {
    //toggle is HIGH

    if (!toggle_ant) {
      //change state
      v_i = analogRead(sensorPin) / 204.6;
      t_change = micros();
      ill_des = 50;
      dz = 1;
      toggle_ant = HIGH;
      
      //sprintf(str_send, "%s", TURN_ON_CONSENSUS);
      if (!flag_turn_consensus) {
        stop_int =true;
        send_msg(TURN_ON_CONSENSUS, my_address, 0, 0);
        algoritm_consensus.concensus(ill_des);
        send_data_to_rasp('C'); 
        stop_int =false; 
      }
      
      //Serial.println("end");
    }

    acquire_samples();
    change_led(u_des);

  } else {
    //toggle is LOW - LED is turn off

    if (toggle_ant) {
      
      v_i = analogRead(sensorPin) / 204.6;
      t_change = micros();
      ill_des = 20;
      dz = 0.7;
      toggle_ant = LOW;

      //sprintf(str_send, "%s", TURN_ON_CONSENSUS);
      if (!flag_turn_consensus) {
        stop_int =true;
        send_msg(TURN_ON_CONSENSUS, my_address, 0, 0);
        algoritm_consensus.concensus(ill_des);
        send_data_to_rasp('C'); 
        stop_int =false; 
      }
      //Serial.println("end");
    }

    acquire_samples();
    change_led(u_des);
  }

  if (flag_send_to_rasp && !flag_turn_consensus){
    if (c > 100){
      send_data_to_rasp('I');
      c = 0;
    }
    c++;
    flag_send_to_rasp = false;
  }
  
  rate = analogRead(sensorPin);
  Serial.print(read_lux(rate));
  Serial.print(" ");
  Serial.print(ill_des);
  Serial.print("\n");
  
  //print_results();


}
