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


#ifndef SERVER_HPP
#define SERVER_HPP


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
#include <memory>
#include <unistd.h>
#include <ctype.h>


#include <mutex>
#include <thread>
#include <chrono>
#include <ctime>
#include <functional>
#include <iomanip>
#include <sstream>

#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>


/**************************************************************************
                              DEFINES
**************************************************************************/
#define SLAVE_ADDR 0x0
#define Ts 0.01 //sampling time =  0.01 seconds


using namespace boost::asio;
using namespace boost;
using boost::system::error_code;
using boost::asio::ip::tcp;
using namespace std;


/**************************************************************************
                          GLOBAL VARIABLES
**************************************************************************/
extern std::mutex mtx;



/**************************************************************************
                          Classes and structs
**************************************************************************/

typedef struct data_list{

  float measure_lux;
  int pwm;

}data_list;


class msg_save{
  public:
    msg_save ():
    data_buf(6000)
    {
    }
    int address;
    boost::circular_buffer<data_list> data_buf;
    float control_lux;
    float ref_lux;
    float Inoise;
    float time_init;
    float sum_Flicker;
    float sum_Energy;
    float sum_Cerror;
    int count_samples;
};


#endif
