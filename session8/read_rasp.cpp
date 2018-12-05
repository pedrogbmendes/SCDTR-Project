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



/**************************************************************************
                              DEFINES
**************************************************************************/
#define SLAVE_ADDR 0x00
#define Ts 0.01 //sampling time =  0.01 seconds





/**************************************************************************
                          GLOBAL VARIABLES
**************************************************************************/



/**************************************************************************
                          CLASSES DEFINITION
**************************************************************************/

class read_bus{
  public:
    void read_bus();
  private:
    int init_slave(bsc_xfer_t &xfer, int addr);
    int close_slave(bsc_xfer_t &xfer)
    void store_message(std::string msg_recv);
};




typedef struct data_list{

  float time;
  float measure_lux;
  float ref_lux;
  int pwm;

};

typedef struct msg_save{
    int address;
    std::list<data_list> data
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
void store_message(std::string msg_recv, msg_save* vec_nodes){
/*message protocol:
  the first three bytes are the type of the message
  the fourth byte is the addres of the sender's node

*/

  std::size_t length_error;
  std::size_t pos, posnext;
  std::string number1, addr_str;
  float n1;
  int orig_addr;
  data_list data_recv;
  msg_save *new_node;

  std::size_t size_msg = msg_recv.length()

  pos = str.find("-", 0);
  if (pos < 0){
    return -1
  }

  //sender address
  length_error = msg_recv.copy(addr_str, pos, 0);
  if(length_error != 1)
    return -1;

  orig_addr = std::stoi(addr_str);

  posnext = str.find(":", pos);
  if (posnext < 0){
    return -1
  }

  //message time
  length_error = msg_recv.copy(data_recv.time, posnext-1-pos, pos+1);
  if(length_error != 1)
    return -1;

  pos = posnext;
  posnext = str.find(",", pos);
  if (posnext < 0){
    return -1
  }

  //Reference lux
  length_error = msg_recv.copy(data_recv.ref_lux, posnext-1-pos, pos+1);
  if(length_error != 1)
    return -1;

  pos = posnext;
  posnext = str.find(",", pos);
  if (posnext < 0){
    return -1
  }

  //Measure lux
  length_error = msg_recv.copy(data_recv.measure_lux, posnext-1-pos, pos+1);
  if(length_error != 1)
    return -1;


  //PWM
  length_error = msg_recv.copy(data_recv.pwm, size_msg-1-posnext, posnext+1);
  if(length_error != 1)
    return -1;


  if (vec_nodes[orig_addr] == NULL){
    new_node->address = orig_addr;
    new_node->data.push_back(data_recv);
    vec_node[orig_addr] = new_node;
  }else{
    vec_node[orig_addr]->data.push_back(data_recv);
  }

}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void compute_statistics(msg_save* vec_nodes, int address_node, type){





}



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float energy_consumed(msg_save* vec_node, bool mode, int addr){

  float sum = 0.0, energy = 0;
  std::list<data_list>::iterator it1, it2;
  msg_save node;
  float power;        ????????????? POWER???????????????????????

  if(mode == false){
    //total energy (of one node)
    node = vec_node[addr];
    it1 = node.data.begin();

    for (it2=std::next(node.data.begin()); it2 != node.data.end(); ++it2){
      sum += it1.pwm*(it2.time - it1.time);

      it1 = it2;
    }
    energy = sum * power;

  }else{
    //accumulated comfort error (all nodes)
    for(int j=0; j<127; i++){
      node = vec_node[j];
      if(node != NULL){
        it1 = node.data.begin();

        for (it2=std::next(node.data.begin()); it2 != node.data.end(); ++it2){
          sum += it1.pwm*(it2.time - it1.time);

          it1 = it2;
        }
      }
    }
    energy = sum * power;
  }

  return c_error;
}




/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float comfort_flicker(msg_save* vec_node, bool mode, int addr){

  float sum = 0.0, c_flicker = 0, fi = 0;
  int i = 1;
  float l1 = 0, l2 = 0, l3 = 0;
  std::list<data_list>::iterator next_l1, next_l2, next_l2;
  msg_save node;

  if(mode == false){
    //total comfort flicker (of one node)
    node = vec_node[addr];
    next_l1 = node.data.begin()
    next_l2 = std::next(next_l1, 1);

    for(next_l3=std::next(node.data.begin(), 2); next_l3 != node.data.end(); ++next_l2){

      l1 = next_l1.measure_lux;
      l2 = next_l2.measure_lux;
      l3 = next_l3.measure_lux;

      if((l3-l2)*(l2-l1) < 0){
        fi = (abs(l3-l2) + abs(l2-l1))/(2*Ts)
        sum += fi;
      }
      i++;

      next_l1 = next_l2;
      next_l2 = next_l3;

    }
    c_flicker = 1.0*sum/i;
  }else{
    //accumulated comfort flicker (all nodes)
    for(int j=0; j<127; i++){
      node = vec_node[j];
      if(node != NULL){
        next_l1 = node.data.begin()
        next_l2 = std::next(next_l1, 1);

        for(next_l3=std::next(node.data.begin(), 2); next_l3 != node.data.end(); ++next_l2){

          l1 = next_l1.measure_lux;
          l2 = next_l2.measure_lux;
          l3 = next_l3.measure_lux;

          if((l3-l2)*(l2-l1) < 0){
            fi = (abs(l3-l2) + abs(l2-l1))/(2*Ts)
            sum += fi;
          }
          i++;

          next_l1 = next_l2;
          next_l2 = next_l3;

        }
      }
    }
    c_flicker = 1.0*sum/i;
  }

  return c_flicker;
}



/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
float comfort_error(msg_save* vec_node, bool mode, int addr){

  float sum = 0.0, c_error = 0;
  int i = 1;
  std::list<data_list>::iterator it;
  msg_save node;


  if(mode == false){
    //total comfort error (of one node)
    node = vec_node[addr];
    for (it = node.data.begin(); it != node.data.end(); ++it){
      if( (it.ref_lux - it.measure_lux) > 0 ){
        sum += it.ref_lux - it.measure_lux;
      }
      i++;
    }
    c_error = 1.0*sum/i;

  }else{
    //accumulated comfort error (all nodes)
    for(int j=0; j<127; i++){
      node = vec_node[j];
      if(node != NULL){
        for (it = node.data.begin(); it != node.data.end(); ++it){
          if( (it.ref_lux - it.measure_lux) > 0 ){
            sum += it.ref_lux - it.measure_lux;
          }
          i++;
        }
      }
    }
    c_error = 1.0*sum/i;
  }

  return c_error;
}





/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
int init_slave(bsc_xfer_t &xfer, int addr){
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
int close_slave(bsc_xfer_t &xfer) {
  xfer.control = 0;
}


/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/
void read_bus(msg_save* vec_nodes) {

  std::string message;
  int status;

  if (gpioInitialise() < 0) {
    printf("Erro 1\n"); return 1;}

  bsc_xfer_t xfer;
  status = init_slave(xfer, SLAVE_ADDR);

  while(1) {
    xfer.txCnt = 0;
    status = bscXfer(&xfer);

    if(xfer.rxCnt > 0){

      message = std::string(xfer.rxBuf).substr(0,xfer.rxCnt);
      store_message(message, *vec_nodes);

      std::cout << "Received" << xfer.rxCnt << "bytes\t";
      std::cout << message << '\n';
      message = '\0';
    }

  }
  status = close_slave(xfer);
  gpioTerminate();
}




/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description: )

**************************************************************************/

int main(int argc, char const *argv[]) {

  std::vector<msg_save*> vec_nodes[127];


  read_bus(*vec_nodes);


  return 0;
}
