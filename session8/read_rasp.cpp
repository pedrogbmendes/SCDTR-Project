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
                              INCLUDES
**************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <iterator>

#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio/steady_timer.hpp>


/**************************************************************************
                              DEFINES
**************************************************************************/
#define SLAVE_ADDR 0x0
#define Ts 0.01 //sampling time =  0.01 seconds





/**************************************************************************
                          GLOBAL VARIABLES
**************************************************************************/





/**************************************************************************
                          CLASSES DEFINITION
**************************************************************************/


typedef struct data_list{

  float time;
  float measure_lux;
  int pwm;

}data_list;


class msg_save{
  public:
    msg_save ():
    data_buf(60000)
    {
    }
    int address;
    boost::circular_buffer<data_list> data_buf;
    float control_lux;
    float ref_lux;
    float Inoise;
    float sum_Flicker;
    float sum_Energy;
    float sum_Cerror;
    int count_samples;
};


class process_msg{
  public:
    int read_bus(std::vector<msg_save*> &vec_nodes);
  private:
    int init_slave(bsc_xfer_t &xfer, int addr);
    int close_slave(bsc_xfer_t &xfer);
    int store_message(char *message, std::vector<msg_save*> &vec_node);
    float energy_consumed(int pwmi, float ti, float ti1, float v_sum_Energy );
    float comfort_flicker(float l1, float l2, float l3, float v_sum_Flicker);
    float comfort_error(float mea_lux, float v_sum_Cerror, float refLux);
    float Bytes2float(char aux[4]);
};





/**************************************************************************
                          FUNCTION
**************************************************************************/



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int process_msg::store_message(char*message, std::vector<msg_save*> &vec_nodes){
/*message protocol:
  the first three bytes are the type of the message
  the fourth byte is the addres of the sender's node
*/
  int orig_addr;
  data_list data_recv;
  msg_save *new_node;
  float l1,l2,l3;
  unsigned long time_milli;
  float t_prev;
  int pwm_prev = 0;
  char aux[4];
  float contrLux, refLux, noise;

  //type of message
  char type_msg = (unsigned char)message[0];
  printf("%c\n", type_msg);

  //sender address
  orig_addr = int((unsigned char)message[1]);
  printf("%d\n",orig_addr );
  //std::vector<msg_save*> vec_nodes[127] = vec_node;

  if(type_msg == 'I'){
    //message contain the lux and pwm

    //time
    time_milli = (unsigned long)((unsigned char)message[2] << 24 |
                                     (unsigned  char)message[3] << 16 |
                                     (unsigned char)message[4] << 8 |
                                     (unsigned char)message[5] );



    data_recv.time = 0.001*time_milli;

    //measure lux
    aux[0]= (char)message[6];
    aux[1]= (char)message[7];
    aux[2]= (char)message[8];
    aux[3]= (char)message[9];
    data_recv.measure_lux=Bytes2float(aux);


    data_recv.pwm = int((unsigned char)message[10]);


  }else if(type_msg == 'C'){
    //consensus result

    //control lux
    aux[0]= (char)message[2];
    aux[1]= (char)message[3];
    aux[2]= (char)message[4];
    aux[3]= (char)message[5];
    contrLux = Bytes2float(aux);

    //reference lux
    aux[0]= (char)message[6];
    aux[1]= (char)message[7];
    aux[2]= (char)message[8];
    aux[3]= (char)message[9];
    refLux = Bytes2float(aux);


  }else if(type_msg == 'F'){
    aux[0]= (char)message[2];
    aux[1]= (char)message[3];
    aux[2]= (char)message[4];
    aux[3]= (char)message[5];
    noise = Bytes2float(aux);

  }


  printf("type = %c\n", type_msg);
  if (vec_nodes[orig_addr] == nullptr){
    //New node
    new_node = new msg_save;
    new_node->address = orig_addr;
    new_node->sum_Energy = 0;
    new_node->sum_Flicker = 0;
    new_node->control_lux = 0;
    new_node->sum_Cerror = 0;
    new_node->count_samples = 0;

    if(type_msg == 'I'){
      new_node->data_buf.push_back(data_recv);
      new_node->ref_lux = 0.0;
      new_node->control_lux = 0.0;
      new_node->count_samples = 1;

    }else if(type_msg == 'C'){
      new_node->ref_lux = refLux;
      new_node->control_lux = contrLux;

    }else if(type_msg == 'F'){
      new_node->Inoise = 0.0;
      new_node->ref_lux = 0.0;
      new_node->control_lux = 0.0;

    }

    vec_nodes[orig_addr] = new_node;
  }else{

    if(type_msg == 'I'){
      vec_nodes[orig_addr]->count_samples ++;
      //accumulated energy
      if(vec_nodes[orig_addr]->count_samples >= 2){
        t_prev = (vec_nodes[orig_addr]->data_buf).end()[-1].time;
        pwm_prev = (vec_nodes[orig_addr]->data_buf).end()[-1].pwm;
        vec_nodes[orig_addr]->sum_Energy = energy_consumed(pwm_prev, data_recv.time, t_prev, vec_nodes[orig_addr]->sum_Energy);
      }
      //accumulated Flicker
      if(vec_nodes[orig_addr]->count_samples > 3){
        l1 = data_recv.measure_lux;
        l2 = (vec_nodes[orig_addr]->data_buf).end()[-1].measure_lux;
        l3 = (vec_nodes[orig_addr]->data_buf).end()[-2].measure_lux;
        vec_nodes[orig_addr]->sum_Flicker = comfort_flicker(l1, l2, l3,vec_nodes[orig_addr]->sum_Flicker);
      }

      //accumulated confort error
      vec_nodes[orig_addr]->sum_Cerror = comfort_error(data_recv.measure_lux, vec_nodes[orig_addr]->sum_Cerror, vec_nodes[orig_addr]->ref_lux);
      vec_nodes[orig_addr]->data_buf.push_back(data_recv);

    }else if(type_msg == 'C'){
      vec_nodes[orig_addr]->ref_lux = refLux;
      vec_nodes[orig_addr]->control_lux = contrLux;

    }else if(type_msg == 'F'){
      vec_nodes[orig_addr]->Inoise = noise;

    }
  }



  std::cout<< "addr= " <<   vec_nodes[orig_addr]->address <<";\tcount= " << vec_nodes[orig_addr]->count_samples<<"\n";
  if(type_msg ==  'C'){
    std::cout<< "control= "<<  vec_nodes[orig_addr]->control_lux<<";\tref_lux= "<<vec_nodes[orig_addr]->ref_lux<<"\n";

  }else if (type_msg ==  'F'){
    std::cout<< "noise_lux= "<<  vec_nodes[orig_addr]->Inoise<<"\n";

  }else if (type_msg == 'I'){
    std::cout<<"comfort_error= "<<vec_nodes[orig_addr]->sum_Cerror<<"\nflicker= "<<vec_nodes[orig_addr]->sum_Flicker<<"\nenergy= "<<vec_nodes[orig_addr]->sum_Energy<<"\n";
    std::cout<<"time= "<<vec_nodes[orig_addr]->data_buf.end()[-1].time<<"\nmeasure_lux= "<<vec_nodes[orig_addr]->data_buf.end()[-1].measure_lux<<"\npwm= "<<vec_nodes[orig_addr]->data_buf.end()[-1].pwm<<"\n";
    /*for(int j=0; j<vec_nodes[orig_addr]->count_samples; j++){
      std::cout <<j<<"\ntime=" <<vec_nodes[orig_addr]->data_buf[j].time << "\npwm="<< vec_nodes[orig_addr]->data_buf[j].pwm <<"\nmeas_lux=" << vec_nodes[orig_addr]->data_buf[j].measure_lux  << "\n";
    }*/
  }
  return 0;
}


float process_msg::Bytes2float(char to_conv[4]){

  union{
    float l;
    char m[4];
  }u;

  u.m[0]= (char)to_conv[0];
  u.m[1]= (char)to_conv[1];
  u.m[2]= (char)to_conv[2];
  u.m[3]= (char)to_conv[3];
  return u.l;

}



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float process_msg::energy_consumed(int pwmi, float ti, float ti1, float v_sum_Energy ){

  return v_sum_Energy + pwmi*(ti-ti1);
}




/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float process_msg::comfort_flicker(float l1, float l2, float l3, float v_sum_Flicker){

  float sum = v_sum_Flicker;
  float fi = 0;

  if((l3-l2)*(l2-l1) < 0){
    fi = (fabs(l3-l2) + fabs(l2-l1))/(2.0*Ts);
    printf("%f\n", fi);
    sum = v_sum_Flicker + fi;
  }
  return sum;
}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float process_msg::comfort_error(float mea_lux, float v_sum_Cerror, float refLux){

  float sum = v_sum_Cerror;

  if(refLux - mea_lux> 0 )
    sum = v_sum_Cerror + (refLux - mea_lux);

  return sum;
}




/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int process_msg::init_slave(bsc_xfer_t &xfer, int addr){
  gpioSetMode(18, PI_ALT3);
  gpioSetMode(19, PI_ALT3);
  xfer.control =  (addr<<16) | /* Slave address */
                  (0x00<<13) | /* invert transmit status flags */
                  (0x00<<12) | /* enable host control */
                  (0x00<<11) | /* enable test fifo */
                  (0x00<<10) | /* invert receive status flags */
                  (0x01<<9) | /* enable receive */
                  (0x00<<8) | /* enable transmit */
                  (0x00<<7) | /* abort and clear FIFOs */
                  (0x00<<6) | /* send control reg as 1st I2C byte */
                  (0x00<<5) | /* send status regr as 1st I2C byte */
                  (0x00<<4) | /* set SPI polarity high */
                  (0x00<<3) | /* set SPI phase high */
                  (0x01<<2) | /* enable I2C mode */
                  (0x00<<1) | /* enable SPI mode */
                  0x01 ; /* enable BSC peripheral */
  return bscXfer(&xfer);
}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int process_msg::close_slave(bsc_xfer_t &xfer) {
  xfer.control = 0;
  return bscXfer(&xfer);
}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int process_msg::read_bus(std::vector<msg_save*> &vec_nodes) {

  char message[25];
  int status;

  if (gpioInitialise() < 0) {
    printf("Erro 1\n");
    return 1;
  }

  bsc_xfer_t xfer;
  status = init_slave(xfer, SLAVE_ADDR);
  if(status < 0)
    return 1;

  while(1) {
    xfer.txCnt = 0;
    status = bscXfer(&xfer);
    if(status < 0)
      return 1;


    if(xfer.rxCnt > 0){

//      message = std::string(xfer.rxBuf).substr(0,xfer.rxCnt);
  for(int j=0;j<xfer.rxCnt;j++)
    message[j] = xfer.rxBuf[j];
    message[xfer.rxCnt]='\0';
    store_message(message, vec_nodes);
    }

  }
  status = close_slave(xfer);
  if(status < 0)
    return 1;
  gpioTerminate();
  return -1;
}




/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/

int main(int argc, char const *argv[]) {
  //initialization
  std::vector<msg_save*> vec_nodes(127);

  process_msg pro_msg;
  pro_msg.read_bus(vec_nodes);



  return 0;
}
