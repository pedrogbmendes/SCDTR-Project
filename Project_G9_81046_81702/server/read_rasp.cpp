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
                              Read data

        Read data from the arduinos
        And stores all the information in the correct space
**************************************************************************/


#include "read_rasp.hpp"

//extern std::mutex mtx;


/**************************************************************************
                          FUNCTION
**************************************************************************/



/**************************************************************************

      Function:store_message

      Arguments:vec_nodes and message
      Return value: 0 in sucess

      Description: the function receives the data structure and the message
from the arduino. It decodes de message .Depending of the messages it stores
and compute some statistics

**************************************************************************/
int process_msg::store_message(char*message, std::vector<msg_save*> &vec_nodes){

  int orig_addr;
  data_list data_recv;
  msg_save *new_node;
  float l1,l2,l3;
  int pwm_prev = 0;
  char aux[4];
  float contrLux, refLux, noise;
  float timeInit;
  unsigned long taux;

  //type of message
  char type_msg = (unsigned char)message[0];

  //sender address
  orig_addr = int((unsigned char)message[1]);

  if(type_msg == 'I'){
    //message contain the lux and pwm


    //measure lux
    aux[0]= (char)message[2];
    aux[1]= (char)message[3];
    aux[2]= (char)message[4];
    aux[3]= (char)message[5];
    data_recv.measure_lux=Bytes2float(aux);


    data_recv.pwm = int((unsigned char)message[6]);


  }else if(type_msg == 'C'){
    //consensus result

    //control lux
    aux[0]= (char)message[2];
    aux[1]= (char)message[3];
    aux[2]= (char)message[4];
    aux[3]= (char)message[5];
    contrLux = Bytes2float(aux);

    //reference lux
    refLux= int((unsigned char)message[6]);



  }else if(type_msg == 'F'){

    taux= (unsigned long)((unsigned char)message[2] << 24 |
                             (unsigned char)message[3] << 16 |
                             (unsigned char)message[4] << 8 |
                             (unsigned char)message[5] );
    timeInit = taux * 0.001;


    aux[0]= (char)message[6];
    aux[1]= (char)message[7];
    aux[2]= (char)message[8];
    aux[3]= (char)message[9];
    noise = Bytes2float(aux);

  }

  if (vec_nodes[orig_addr] == nullptr){
    //New node
    new_node = new msg_save;
    new_node->address = orig_addr;
    new_node->sum_Energy = 0;
    new_node->sum_Flicker = 0;
    new_node->control_lux = 0;
    new_node->sum_Cerror = 0;
    new_node->count_samples = 0;
    new_node->Inoise = 0;
    new_node->time_init = 0;

    if(type_msg == 'I'){
      new_node->data_buf.push_back(data_recv);
      new_node->ref_lux = 0.0;
      new_node->control_lux = 0.0;
      new_node->count_samples = 1;

    }else if(type_msg == 'C'){
      new_node->ref_lux = refLux;
      new_node->control_lux = contrLux;
      data_recv.pwm=0;
      data_recv.measure_lux = 0;
      new_node->data_buf.push_back(data_recv);

    }else if(type_msg == 'F'){
      new_node->Inoise = noise;
      new_node->time_init = timeInit;
      new_node->ref_lux = 0.0;
      new_node->control_lux = 0.0;
      data_recv.pwm=0;
      data_recv.measure_lux = 0;
      new_node->data_buf.push_back(data_recv);

    }

    vec_nodes[orig_addr] = new_node;
  }else{
    //lock the resource

    mtx.lock();
    if(type_msg == 'I'){
      vec_nodes[orig_addr]->count_samples ++;
      //accumulated energy
      if(vec_nodes[orig_addr]->count_samples >= 2){
        pwm_prev =  (vec_nodes[orig_addr]->data_buf).end()[-1].pwm;
        vec_nodes[orig_addr]->sum_Energy = energy_consumed(pwm_prev, vec_nodes[orig_addr]->sum_Energy);
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
    mtx.unlock();
  }

  //print results

  /*mtx.lock();
  std::cout<< "addr= " <<   vec_nodes[orig_addr]->address <<"\n";
  if(type_msg ==  'C'){
    std::cout<< "control= "<<  vec_nodes[orig_addr]->control_lux<<";\tref_lux= "<<vec_nodes[orig_addr]->ref_lux<<"\n";

  }else if (type_msg ==  'F'){
    std::cout<< "noise_lux= "<<  vec_nodes[orig_addr]->Inoise<<"time= "<<vec_nodes[orig_addr]->time_init<<"\n";

  }else if (type_msg == 'I'){
    std::cout<<"comfort_error= "<<vec_nodes[orig_addr]->sum_Cerror<<"\nflicker= "<<vec_nodes[orig_addr]->sum_Flicker<<"\nenergy= "<<vec_nodes[orig_addr]->sum_Energy<<"\n";
    std::cout<<"\nmeasure_lux= "<<vec_nodes[orig_addr]->data_buf.end()[-1].measure_lux<<"\npwm= "<<vec_nodes[orig_addr]->data_buf.end()[-1].pwm<<"\n";
    for(int j=0; j<vec_nodes[orig_addr]->count_samples; j++){
      std::cout <<j<<"\ntime=" <<vec_nodes[orig_addr]->data_buf[j].time << "\npwm="<< vec_nodes[orig_addr]->data_buf[j].pwm <<"\nmeas_lux=" << vec_nodes[orig_addr]->data_buf[j].measure_lux  << "\n";
    }
  }
  mtx.unlock();*/

  return 0;
}


/**************************************************************************

      Function:Bytes2float

      Arguments:to_conv
      Return value: float value

      Description: convert from type byte to type float

**************************************************************************/
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

      Function:energy_consumed

      Arguments: pwmi, v_sum_Energy
      Return value: accumulated energy

      Description: Receives a new value of the pwm and it computes the
accumulated energy

**************************************************************************/
float process_msg::energy_consumed(int pwmi, float v_sum_Energy ){

  return v_sum_Energy + pwmi*(0.01);
}



/**************************************************************************

      Function:comfort_flicker

      Arguments: l1, l2 , l3, v_sum_Flicker
      Return value: accumulated flicker

      Description: receives a new sample and it has to compute the accumulated
flicker

**************************************************************************/
float process_msg::comfort_flicker(float l1, float l2, float l3, float v_sum_Flicker){

  float sum = v_sum_Flicker;
  float fi = 0;

  if((l3-l2)*(l2-l1) < 0){
    fi = (fabs(l3-l2) + fabs(l2-l1))/(2.0*Ts);
    sum = v_sum_Flicker + fi;
  }
  return sum;
}


/**************************************************************************

      Function:comfort_error

      Arguments:mea_lux, ref_lux, v_sum_Cerror
      Return value:accumulated comfort error

      Description: receives a new sample and it has to compute the accumulated
comfort flicker

**************************************************************************/
float process_msg::comfort_error(float mea_lux, float v_sum_Cerror, float refLux){

  float sum = v_sum_Cerror;

  if(refLux - mea_lux> 0 )
    sum = v_sum_Cerror + (refLux - mea_lux);

  return sum;
}


/**************************************************************************

      Function:init_slave

      Arguments:xfer, addr
      Return value:bscXfer(&xfer)

      Description: enables the i2c communication

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

      Function:close_slave

      Arguments:xfer
      Return value:bscXfer(&xfer)

      Description: ends the i2c communication

**************************************************************************/
int process_msg::close_slave(bsc_xfer_t &xfer) {
  xfer.control = 0;
  return bscXfer(&xfer);
}


/**************************************************************************

      Function: read_bus

      Arguments:vec_nodes
      Return value: 1 if error occurs;
                    -1 at the end

      Description: reads the messages of the i2c bus using pigpio

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
