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



#include "server.hpp"





/**************************************************************************
                          FUNCTION
**************************************************************************/

/*
 An asynchronous TCP daytime server
 code based on https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime3.html
*/

class tcp_server
{


  public:
    tcp_server(boost::asio::io_service& io)
      : acceptor_(io, tcp::endpoint(tcp::v4(), 17000))
    {
      start_accept();
    }


private:
  tcp::acceptor acceptor_;


  void start_accept()
  {
    connection_client::pointer new_connection = connection_client::create(acceptor_.io_service());
    acceptor_.async_accept(new_connection->socket(), boost::bind(&tcp_server::handle_accept, this, new_connection, boost::asio::placeholders::error));
  }


  void handle_accept(connection_client::pointer new_connection, const boost::system::error_code& error)
  {
    if (!error){
      new_connection->start();
      start_accept();
    }
  }

};





/**************************************************************************

      Function:

      Arguments:
      Return value:

      Description:

**************************************************************************/

int main(int argc, char const *argv[]) {

  boost::asio::io_service io;
  std::vector<msg_save*> vec_nodes(127);

  tcp_server server(io);

  process_msg pro_msg;
  std::thread pro_msg.read_bus(vec_nodes);

  io.run();

  return 0;
}
