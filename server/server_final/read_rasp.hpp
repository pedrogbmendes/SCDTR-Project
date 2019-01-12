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

#ifndef READ_RASP_HPP
#define READRASP_HPP

#include "server.hpp"


/**************************************************************************
                          CLASS DEFINITION: process_msg

  Reads data from the bus (read messages sent by the arduinos) and store
these messages in the correct locations and compute the statictics

**************************************************************************/


class process_msg{
  public:
    int read_bus(std::vector<msg_save*> &vec_nodes);
  private:
    int init_slave(bsc_xfer_t &xfer, int addr);
    int close_slave(bsc_xfer_t &xfer);
    int store_message(char *message, std::vector<msg_save*> &vec_node);
    float energy_consumed(int pwmi, float v_sum_Energy );
    float comfort_flicker(float l1, float l2, float l3, float v_sum_Flicker);
    float comfort_error(float mea_lux, float v_sum_Cerror, float refLux);
    float Bytes2float(char aux[4]);
};



#endif
