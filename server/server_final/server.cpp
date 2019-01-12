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
#include "read_rasp.hpp"

/**************************************************************************
                          Variables
**************************************************************************/
std::vector<msg_save*> vec_nodes(127);

std::mutex mtx;


/**************************************************************************
                          BackGround Server

      Responde to resquest from clients
      Compute the requires statictics using the data basa
**************************************************************************/

//ALARMS
void sighandler() {
	//sighandler to ignore keyboard interruptions
  signal(SIGINT, SIG_IGN);		//ignore crtl+C
	signal(SIGTSTP, SIG_IGN);		//ignore crtl+Z
}



class list_stream{
  public:
    int desk;
    char type;
		int count;
};



class connection_client: public boost::enable_shared_from_this<connection_client>
{

  private:

    ip::tcp::socket socket_client;
    boost::asio::steady_timer timer_;

    boost::asio::streambuf buf_request;
    std::string msg_recv, msg2send;
    int  flag_stream = false;

    std::list<list_stream> active_stream;

    connection_client(io_service& io): socket_client(io), timer_(io) {

        }




/**************************************************************************

      Function: request_client

      Arguments:error
      Return value: void

      Description: receives a request from client

**************************************************************************/
    void request_client(const boost::system::error_code& error){
      /*waits for the request of the client
      reads util receive the \n*/
			if(!error){
      	async_read_until(socket_client, buf_request, "\n",	boost::bind(&connection_client::answer_request, shared_from_this(),  boost::asio::placeholders::error));
			}else{
				socket_client.close();
			}

    }


/**************************************************************************

      Function: answer_request

      Arguments: error
      Return value: void

      Description: Handles with the request of the client

**************************************************************************/
    void answer_request(const boost::system::error_code& error){

			if (error){
				socket_client.close();;
			}else{
      //response to client
      std::istream is(&buf_request);
      std::getline(is, msg_recv);
      std::string msg2send, str_aux;;
      int d1, d2, d3, desk_chosen = 0;

      std::cout << "Request from client: "  + msg_recv + "\n";

      size_t len = msg_recv.length();

      if (msg_recv.at(0) == 'r' && msg_recv.length()==1){
        //reset
        msg2send = reset_info();

      }else if(len<5 || len> 7){
        msg2send = "Error: wrong request";

      } else if(msg_recv.at(1) != ' ' || msg_recv.at(3) !=  ' '){
        msg2send = "Error: wrong request";

      }else{

				//fault tolerance: wrong messages from clients
        if(len == 5){
					str_aux = msg_recv.substr(4, 1);

					if(isdigit(str_aux[0])){
						desk_chosen = std::stoi(str_aux);
					}

        }else if(len== 6){
          str_aux = msg_recv.substr(4, 2);
					if(isdigit(str_aux[0]) && isdigit(str_aux[1])){
          	desk_chosen = std::stoi(str_aux);
					}
        }else if (len ==7){
          str_aux = msg_recv.substr(4, 3);
					if(isdigit(str_aux[0]) && isdigit(str_aux[1]) && isdigit(str_aux[2])){
          	desk_chosen = std::stoi(str_aux);
					}
        }

        if((desk_chosen < 1 || desk_chosen >126) && (str_aux.compare("T") != 0)){
          //desk does not exist
            msg2send =  "ERROR: desk or request does not exist";

        }else{

					if((str_aux.compare("T") != 0) ){
						if(vec_nodes[desk_chosen] == nullptr){
						 //desk does not exist
						 msg2send =  "ERROR: desk does not exist"+ std::to_string(desk_chosen);
						}else{

							if(msg_recv.compare("\n")!=0 && msg_recv.compare("\0")!=0 && msg_recv.empty() == false){
	             	mtx.lock();
	             	//message received correctlly: process the type of the message
	             	if(msg_recv.at(0) == 'g'){
	               	//get information
	               	msg2send = get_info(msg_recv.at(2), str_aux);

	             	}else if (msg_recv.at(0) == 'b'){
	               //last minute information
		               msg2send = get_last_minute(msg_recv.at(2), str_aux);

	             	}else if (msg_recv.at(0) == 's'){
	               //stream
	               	msg2send = get_stream_info(msg_recv.at(2), str_aux);

	             	}else{
	               	msg2send =  "ERROR: request does not exist";
	             	}
	             	mtx.unlock();

	           	}else{
	             //message not received: try againv
	             msg2send =  "ERROR: request does not exist";
	           }
					 	}
				 	}else if((str_aux.compare("T") == 0) && (msg_recv.at(0) == 'g')){
					  mtx.lock();
        		msg2send = get_info(msg_recv.at(2), str_aux);

            mtx.unlock();

						}else{
            //message not received: try againv
            msg2send =  "ERROR: request does not exist";
          	}
        	}
      }
      //send a message to the client
      msg2send = msg2send + "\n";
      async_write(socket_client,buffer(msg2send),boost::bind(&connection_client::request_client, shared_from_this(),  boost::asio::placeholders::error));
		}
    }

/**************************************************************************

      Function: get_info

      Arguments: desk_i, unit
      Return value: string

      Description: computes the message to reply to the client in the request
is of type g

**************************************************************************/
    std::string get_info(char unit, std::string desk_i){

      int desk=0;
      std::string msg_send;
      float data_readf = 0;
      int j = 0;
			size_t len = desk_i.length();
			std::string str_aux;

			if(len == 1){
				str_aux = desk_i.substr(0, 1);
				if(isdigit(str_aux[0])){
					desk = std::stoi(str_aux);
				}

			}else if(len== 2){
				str_aux = desk_i.substr(0, 2);
				if(isdigit(str_aux[0]) && isdigit(str_aux[1])){
					desk = std::stoi(str_aux);
				}
			}else if (len ==3){
				str_aux = desk_i.substr(0, 3);
				if(isdigit(str_aux[0]) && isdigit(str_aux[1]) && isdigit(str_aux[2])){
					desk = std::stoi(str_aux);
				}
			}


      switch (unit) {
        case 'l':
					if(desk>=1 && desk<=126 ){
						//current measure lux
						data_readf = vec_nodes[desk]->data_buf.end()[-1].measure_lux;
						msg_send = "l " + std::to_string(desk) + " " + std::to_string(data_readf);
					}else{
						msg_send =  "ERROR: request does not exist";
					}
        break;

        case 'd':
					if(desk>=1 && desk<=126 ){
          //current pwm: 0.392156863=100/255
          	data_readf = (vec_nodes[desk]->data_buf.end()[-1].pwm)*0.392156863;
          	msg_send = "d " + std::to_string(desk) + " " + std::to_string(data_readf);
					}else{
						msg_send =  "ERROR: request does not exist";
					}
				break;

        case 's':
          //state ocupancy
					if(desk>=1 && desk<=126 ){
	          data_readf = vec_nodes[desk]->ref_lux;
	          if(data_readf > 40){
	            msg_send = "s " + std::to_string(desk) + " 1";
	          }else if(data_readf < 30){
	            msg_send = "s " + std::to_string(desk) + " 0";
	          }
					}else{
						msg_send =  "ERROR: request does not exist";
					}
        break;

        case 'L':
					if(desk>=1 && desk<=126 ){
          	//current lux lower bound
          	data_readf = vec_nodes[desk]->ref_lux;
          	msg_send = "L " + std::to_string(desk) + " " + std::to_string(data_readf);
					}else{
						msg_send =  "ERROR: request does not exist";
					}
        break;

        case 'o':
					if(desk>=1 && desk<=126 ){
          	//external lux
          	data_readf = vec_nodes[desk]->Inoise;
          	msg_send = "0 " + std::to_string(desk) + " " + std::to_string(data_readf);
					}else{
						msg_send =  "ERROR: request does not exist";
					}
				break;

        case 'r':
					if(desk>=1 && desk<=126 ){
          	//current illuminance control ref
          	data_readf = vec_nodes[desk]->control_lux;
          	msg_send = "r " + std::to_string(desk) + " " + std::to_string(data_readf);
					}else{
						msg_send =  "ERROR: request does not exist";
					}
        break;

        case 'p':
          //power consumption
          if(desk_i.compare("T") == 0){
            //Total power in the system
            for(j = 1; j<127; j++){
              if( vec_nodes[j] != nullptr)
                data_readf = data_readf + (vec_nodes[desk]->data_buf.end()[-1].pwm)*0.392156863;
            }
            msg_send = "p T " + std::to_string(data_readf);

          }else{
            //power in a desk
						if(desk>=1 && desk<=126 ){
            	data_readf =(vec_nodes[desk]->data_buf.end()[-1].pwm)*0.392156863;
            	msg_send = "p " + std::to_string(desk) + " " + std::to_string(data_readf);
						}else{
							msg_send =  "ERROR: request does not exist";
						}
          }
        break;

        case 't':
					if(desk>=1 && desk<=126 ){
          	//time elapsed since last restart
          	data_readf = vec_nodes[desk]->time_init + vec_nodes[desk]->count_samples * 0.01;
          	msg_send = "t " + std::to_string(desk) + " " + std::to_string(data_readf);
					}else{
						msg_send =  "ERROR: request does not exist";
					}
        break;

        case 'e':
          //energy
          if(desk_i.compare("T") == 0){
            //Total energy in the system
            for(j = 1; j<127; j++){
              if( vec_nodes[j] != nullptr)
                data_readf = data_readf + vec_nodes[j]->sum_Energy;
            }
            msg_send = "e T " + std::to_string(data_readf);

          }else{
						if(desk>=1 && desk<=126 ){
            	//energy in a desk
            	data_readf = vec_nodes[desk]->sum_Energy;
            	msg_send = "e " + std::to_string(desk) + " " + std::to_string(data_readf);
						}else{
							msg_send =  "ERROR: request does not exist";
						}
					}
        break;

        case 'c':
          //comfort error
          if(desk_i.compare("T") == 0){
            //total comfort error of the system
            for(j = 1; j<127; j++){
              if( vec_nodes[j] != nullptr)
                data_readf = data_readf + vec_nodes[j]->sum_Cerror/vec_nodes[j]->count_samples;
            }
            msg_send = "c T " + std::to_string(data_readf);

          }else{
						if(desk>=1 && desk<=126 ){
          	//energy in a desk
          		data_readf = vec_nodes[desk]->sum_Cerror/vec_nodes[desk]->count_samples;
          		msg_send = "c " + std::to_string(desk) + " " + std::to_string(data_readf);
          	}else{
							msg_send =  "ERROR: request does not exist";
						}
					}
        break;

        case 'v':
          //comfort flicker
          if(desk_i.compare("T") == 0){
            //total comfort error of the system
            for(j = 1; j<127; j++){
              if( vec_nodes[j] != nullptr)
                data_readf = data_readf + vec_nodes[j]->sum_Flicker/vec_nodes[j]->count_samples;
            }
            msg_send = "v T " + std::to_string(data_readf);

          }else{
						if(desk>=1 && desk<=126 ){
          	//energy in a desk
          		data_readf = vec_nodes[desk]->sum_Flicker/vec_nodes[desk]->count_samples;
          		msg_send = "v " + std::to_string(desk) + " " + std::to_string(data_readf);
						}else{
							msg_send =  "ERROR: request does not exist";
						}
          }
        break;

        default:
          msg_send = "ERROR: request does not exist";

      }
      return msg_send;
    }


/**************************************************************************

      Function:get_last_minute

      Arguments:desk_i, unit
      Return value: string

      Description: Creates the message containing the last minute buffer
to reply to the client

**************************************************************************/
    std::string get_last_minute(char unit, std::string desk_i){

      int desk=0;
      std::string msg_send,data;
      float data_readf = 0;
      boost::circular_buffer<data_list>::iterator it;
			std::stringstream round_data;

			size_t len = desk_i.length();
			std::string str_aux;

			if(len == 1){
				str_aux = desk_i.substr(0, 1);
				if(isdigit(str_aux[0])){
					desk = std::stoi(str_aux);
				}

			}else if(len== 2){
				str_aux = desk_i.substr(0, 2);
				if(isdigit(str_aux[0]) && isdigit(str_aux[1])){
					desk = std::stoi(str_aux);
				}
			}else if (len ==3){
				str_aux = desk_i.substr(0, 3);
				if(isdigit(str_aux[0]) && isdigit(str_aux[1]) && isdigit(str_aux[2])){
					desk = std::stoi(str_aux);
				}
			}


			int i=0;

      switch (unit) {
        case 'l':
          //last minute Lux
          msg_send = "b l " + std::to_string(desk) + " ";
          for(it=vec_nodes[desk]->data_buf.begin(); it!=vec_nodes[desk]->data_buf.end(); ++it){
						i++;
						round_data << fixed << setprecision(1) << it->measure_lux;
            data = round_data.str();
						msg_send = msg_send + data + ", ";
						round_data.str("");
						data = "";
          }
        break;

        case 'd':
          //pwm of last minute
          msg_send = "b d " +  std::to_string(desk) + " ";
          for(it=vec_nodes[desk]->data_buf.begin(); it!=vec_nodes[desk]->data_buf.end(); ++it){
            msg_send = msg_send + std::to_string(it->pwm) + ", ";
          }
        break;

        default:
          msg_send = "ERROR: request does not exist";

      }
      return msg_send;
    }


/**************************************************************************

      Function: reset_info

      Arguments: non
      Return value: string

      Description: deletes all the data from the data base and sends a
ack to the client

**************************************************************************/
    std::string reset_info(){
      std::string msg_send;
      boost::circular_buffer<data_list>::iterator it;

      //clean the data on the data base
      for(int j=1; j<127; j++){
        if(vec_nodes[j]!=nullptr){
          vec_nodes[j]->control_lux = 0;
          vec_nodes[j]->ref_lux = 0;
          vec_nodes[j]->Inoise = 0;
          vec_nodes[j]->time_init = 0;
          vec_nodes[j]->sum_Flicker = 0;
          vec_nodes[j]->sum_Energy = 0;
          vec_nodes[j]->sum_Cerror = 0;
          vec_nodes[j]->count_samples = 0;

          for(it=vec_nodes[j]->data_buf.begin(); it!=vec_nodes[j]->data_buf.end(); ++it ){
            it->pwm = 0;
            it->measure_lux = 0;
          }

        }
      }

      msg_send = "ack";
      return msg_send;
    }


/**************************************************************************

      Function:get_stream_info

      Arguments: unit, desk_i
      Return value: string

      Description: The message received is from type s, that means that is a
new request for stream (and in this case is add to the list) or a stream needs
to stop (delete the respective stream information from the list)

**************************************************************************/
    std::string get_stream_info(char unit, std::string desk_i){

      list_stream data_str;
      int flag_erase = false;
      std::string msg2send;
			data_str.desk = 0;

			size_t len = desk_i.length();
			std::string str_aux;

			if(len == 1){
				str_aux = desk_i.substr(0, 1);
				if(isdigit(str_aux[0])){
					data_str.desk = std::stoi(str_aux);
				}

			}else if(len== 2){
				str_aux = desk_i.substr(0, 2);
				if(isdigit(str_aux[0]) && isdigit(str_aux[1])){
					data_str.desk = std::stoi(str_aux);
				}
			}else if (len ==3){
				str_aux = desk_i.substr(0, 3);
				if(isdigit(str_aux[0]) && isdigit(str_aux[1]) && isdigit(str_aux[2])){
					data_str.desk = std::stoi(str_aux);
				}
			}


      data_str.type = unit;
			data_str.count = 0;

      if(data_str.type !='l' && data_str.type !='d'){
        //error in the request of client
        msg2send = "ERROR: request does not exist";

      }else{


        for (std::list<list_stream>::iterator it=active_stream.begin(); it != active_stream.end(); ++it){
          if (it->desk ==  data_str.desk && it-> type == data_str.type ){
            //stop stream
            active_stream.erase(it);
            flag_erase = true;
            break;
          }
        }

        if(flag_erase == false ){
          //enable stream
					data_str.count = vec_nodes[data_str.desk]->count_samples - 1;
          msg2send = "stream";
          active_stream.push_back(data_str);
          flag_stream = true;

        }else{
          msg2send = "ack";
        }

        if(active_stream.empty() == true){
          flag_stream = false;
        }
      }
      return msg2send;
    }

/**************************************************************************

      Function: handle_stream

      Arguments: non
      Return value: void

      Description: Funstions that sees if there are stream requested and
if yes it send the respective stream for the client

**************************************************************************/
    void handle_stream(){

      std::string msg2send;
      float time_;
      float mealx;
      int duty_cycle;

      mtx.lock();
      for (std::list<list_stream>::iterator it=active_stream.begin(); it != active_stream.end(); ++it){
				if( it->count < vec_nodes[it->desk]->count_samples){
					time_  = vec_nodes[it->desk]->time_init + vec_nodes[it->desk]->count_samples*0.01;
					if(it->type == 'l'){
	          mealx = vec_nodes[it->desk]->data_buf.end()[-1].measure_lux;
	          msg2send = "s l " + std::to_string(it->desk)+ " " + std::to_string(mealx) + " " + std::to_string(time_) + "\n";
	        }else if (it->type == 'd'){
	          duty_cycle = vec_nodes[it->desk]->data_buf.end()[-1].pwm;
	          msg2send = "s d " + std::to_string(it->desk)+ " " + std::to_string(duty_cycle) + " "+ std::to_string(time_) + "\n";
	        }
					it->count = vec_nodes[it->desk]->count_samples;

					async_write(socket_client,buffer(msg2send),boost::bind(&connection_client::nothing, shared_from_this(),  boost::asio::placeholders::error));
	      }
			}
      mtx.unlock();
    }


    void nothing(const boost::system::error_code& error){
			/*if(error)
				socket_client.close();
*/
		}


/**************************************************************************

      Function: deadline_handler

      Arguments:error
      Return value: void

      Description: waits for a connection of the client

**************************************************************************/
    void deadline_handler(const boost::system::error_code& error){
			if(!error){
      	if(flag_stream == true)
        	handle_stream();

					timer_.expires_from_now(std::chrono::milliseconds(1000));
					timer_.async_wait(boost::bind(&connection_client::deadline_handler, shared_from_this(), boost::asio::placeholders::error));
			}else{
				socket_client.close();
			}

		}

/**************************************************************************/

  public:
    //class constructor
    static boost::shared_ptr<connection_client> create(boost::asio::io_service& io) {
        return boost::shared_ptr<connection_client>(new connection_client(io));
    }

    //socket creation
    ip::tcp::socket& socket() {return socket_client;}


    //start the connection with the client
    void start(){
      //set timer
      timer_.expires_from_now(std::chrono::milliseconds(1000));
      //Start an asynchronous wait on the timer- connection to the client
      timer_.async_wait(boost::bind(&connection_client::deadline_handler, shared_from_this(), boost::asio::placeholders::error ));

      std::cout << "New client: Connection Established" << "\n";

      //send a confirmation for the client
      async_write(socket_client, buffer("Welcome\n"),boost::bind(&connection_client::request_client,shared_from_this(),  boost::asio::placeholders::error));

    }
};





/**************************************************************************
                          Class: start connections
**************************************************************************/
/*
 An asynchronous TCP daytime server
 code based on https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime3.html
*/

class tcp_server
{
  private:
    ip::tcp::acceptor acceptor_;

  public:
    //constructor initialises an acceptor to listen a TCP connection on port 17000.
    tcp_server(boost::asio::io_service& io)
      : acceptor_(io, tcp::endpoint(tcp::v4(), 17000))
    {
      start_accept();
    }

  private:

    void start_accept()
    {
      //creates the sockett and initiates an asynchronous accept operation to wait for a new connection.
      boost::shared_ptr<connection_client> new_connection = connection_client::create(acceptor_.get_io_service());
      acceptor_.async_accept(new_connection->socket(), boost::bind(&tcp_server::handle_accept, this, new_connection, boost::asio::placeholders::error));
    }


    void handle_accept(boost::shared_ptr<connection_client> new_connection, const boost::system::error_code& error)
    {
      //asynchronous accept was initiate
      // services the client request
      //initalize new accept client operation
      if (!error ){
        new_connection->start();
				start_accept();
			}else{

				new_connection.reset();
			}
    }

};



/**************************************************************************

      Function: call_arduino_msg

      Arguments: non
      Return value: void

      Description: creates the thread to handle the messages of the arduino

**************************************************************************/
void call_arduino_msg(){

  process_msg proc_msg;
  proc_msg.read_bus(vec_nodes);
}


/**************************************************************************
      Function:main
**************************************************************************/

int main(int argc, char const *argv[]) {

  sighandler();

  boost::asio::io_service io;

  tcp_server server(io);

  std::thread thr {call_arduino_msg};

  io.run();

  return 0;
}
