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

#include "read_rasp.hpp"


/**************************************************************************
                          FUNCTION
**************************************************************************/



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int process_msg::store_message(std::string msg_recv, std::vector<msg_save*> vec_nodes){
/*message protocol:
  the first three bytes are the type of the message
  the fourth byte is the addres of the sender's node
*/
  int orig_addr;
  data_list data_recv;
  msg_save *new_node;
  float l1,l2,l3;
  unsigned long t_prev;
  std::list<data_list>::iterator it;

  std::size_t size_msg = msg_recv.length();

  //sender address
  orig_addr = int((unsigned char)message[0]);

  int a = int((buffer[0]) << 24 |
              (unsigned char)(buffer[1]) << 16 |
              (unsigned char)(buffer[2]) << 8 |
              (unsigned char)(buffer[3]));

  data_recv.time = (unsigned long)((unsigned char)message[1] << 24 |
                                   (unsigned char)message[2] << 16 |
                                   (unsigned char)message[3] << 8 |
                                   (unsigned char)message[4] );




  data_recv.measure_lux = float((unsigned char)message[5] << 24 |
                                (unsigned char)message[6] << 16 |
                                (unsigned char)message[7] << 8 |
                                (unsigned char)message[8] );


  data_recv.pwm = int((unsigned char)message[9]);


  if (size_msg == 14){
    //no reference lux value on the message

    data_recv.ref_lux = float((unsigned char)message[10] << 24 |
                              (unsigned char)message[11] << 16 |
                              (unsigned char)message[12] << 8 |
                              (unsigned char)message[13] );

  }else{
    //reference lux value on the message

    it = vec_nodes[orig_addr]->data.end();
    data_recv.ref_lux = it.ref_lux;

  }

  mtx.lock();

  if (vec_nodes[orig_addr] == NULL){
    //New node
    //new_node =  (msg_save *)malloc( sizeof(msg_save) );
    new_node = new msg_save;

    new_node->sum_Energy = 0;
    new_node->sum_Flicker = 0;
    new_node->sum_Cerror = comfort_error(data_recv, 0);
    new_node->address = orig_addr;
    new_node->data_buf.push_back(data_recv);
    new_node->count_samples = 1;
    vec_nodes[orig_addr] = new_node;

  }else{
    t_prev = (vec_nodes[orig_addr]->data_buf).end()[-1].time;
    vec_nodes[orig_addr]->sum_Energy = energy_consumed(data_recv.pwm, data_recv.time, t_prev,vec_nodes[orig_addr]->sum_Energy);

    if(vec_nodes[orig_addr]->count_samples > 3){
      l1 = data_recv.measure_lux;
      l2 = (vec_nodes[orig_addr]->data_buf).end()[-1].measure_lux;
      l3 = (vec_nodes[orig_addr]->data_buf).end()[-2].measure_lux;
      vec_nodes[orig_addr]->sum_Flicker = comfort_flicker(l1, l2, l3,vec_nodes[orig_addr]->sum_Flicker);
    }

    vec_nodes[orig_addr]->sum_Cerror = comfort_error(data_recv, vec_nodes[orig_addr]->sum_Cerror);
    vec_nodes[orig_addr]->count_samples ++;
    vec_nodes[orig_addr]->data_buf.push_back(data_recv);

  }

  mtx.unlock();

  std::cout << "addr=" << orig_addr<< "\ncomfort_error="<<vec_nodes[orig_addr]->sum_Cerror<<"\nflicker="<<vec_nodes[orig_addr]->sum_Flicker<<"\nenergy"<<vec_nodes[orig_addr]->sum_Energy;

  for(int j=0; j<vec_nodes[orig_addr]->count_samples; j++){
    std::cout <<"\ntime=" <<vec_nodes[orig_addr]->data_buf[j].time << "\nref_lux="<<vec_nodes[orig_addr]->data_buf[j].ref_lux << "\npwm="<< vec_nodes[orig_addr]->data_buf[j].pwm<<"\nmeas_lux="<< vec_nodes[orig_addr]->data_buf[j].measure_lux;
  }
  return 0;
}





/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float process_msg::energy_consumed(int pwmi, unsigned long ti, unsigned long ti1, float v_sum_Energy ){

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
    fi = (abs(l3-l2) + abs(l2-l1))/(2*Ts);
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
float process_msg::comfort_error(data_list value_recv, float v_sum_Cerror){

  float sum = v_sum_Cerror;

  if(value_recv.ref_lux - value_recv.measure_lux > 0 )
    sum = v_sum_Cerror + value_recv.ref_lux - value_recv.measure_lux;

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
                  (0x01<<8) | /* enable transmit */
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
int process_msg::read_bus(std::vector<msg_save*> vec_nodes) {

  std::string message;
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

      message = std::string(xfer.rxBuf).substr(0,xfer.rxCnt);
      std::cout << "Received" << xfer.rxCnt << "bytes\t";
      std::cout << message << '\n';
      store_message(message, vec_nodes);
      message = '\0';
    }

  }
  status = close_slave(xfer);
  if(status < 0)
    return 1;
  gpioTerminate();
  return -1;
}
