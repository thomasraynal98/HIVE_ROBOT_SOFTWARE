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

#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>

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

struct axis_state {
    short x, y;

    // axis_state()
    //     : x(0)
    //     , y(0)
    //     {}
};

struct Js_state {
    /**
     * NOTE: Me permet de stocker mes valeurs de joysticks dans une struct.
     */

    std::vector<bool> button_state; // A - B - X - Y - LB - RB
    std::vector<axis_state> axis_state_vect; // Joystick gauche, arriere gauche, arriere droit.

    Js_state()
        {
            for(int i = 0; i < 8; i++) button_state.push_back(false);
            for(int i = 0; i < 3; i++) axis_state_vect.push_back(axis_state());
        }

    void show()
    {
        std::string output_show = std::to_string(get_curr_timestamp()) + "\n";
        for(int i = 0; i < 8; i++) output_show += std::to_string(button_state[i]) + "|";
        output_show += "\n"; 
        for(int i = 0; i < 3; i++) output_show += std::to_string(axis_state_vect[i].x) + "|" + std::to_string(axis_state_vect[i].y) + "\n";
        std::cout << output_show << "\n\n" << std::endl;;
    }

    std::string get_str()
    {
        std::string output;
        for(int i = 0; i < 8; i++) output += std::to_string(button_state[i]) + "|";
        for(int i = 0; i < 3; i++) output += std::to_string(axis_state_vect[i].x) + "|" + std::to_string(axis_state_vect[i].y) + "|";
        return output;
    }
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
int read_event(int fd, struct js_event *event);
size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);
float get_battery_level(float curr_voltage, float battery_voltage);