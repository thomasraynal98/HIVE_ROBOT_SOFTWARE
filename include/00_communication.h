#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <sio_client.h>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <list>

#include "00_function.h"

struct Server_var
{
    std::string type_str, channel_str, value;
    Server_var(std::string _type_str, std::string _channel_str, std::string _value)
        : type_str(_type_str)
        , channel_str(_channel_str)
        , value(_value)
        {}
};

bool port_already_taken(sw::redis::Redis* redis, std::string curr_port_name);
bool port_is_ready_to_use(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
int port_opening_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
int port_closing_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
void reading_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager, std::string mcu_function_str);
void writing_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager, std::string mcu_function_str);
void bind_events(sio::socket::ptr current_socket);
void send_msg_server(sio::socket::ptr current_socket, std::string emit_title, std::vector<Server_var>& vect_msg);
void send_event_server(sio::socket::ptr current_socket, std::string mission_title, std::string mission_state);