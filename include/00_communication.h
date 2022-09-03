#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <common/mavlink.h>
#include "autopilot_interface.h"
#include "serial_port.h"

#include "00_function.h"

bool port_already_taken(sw::redis::Redis* redis, std::string curr_port_name);
bool port_is_detected(std::string curr_port_name);
bool port_is_ready_to_use(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
int port_opening_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
int port_closing_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
void reading_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager);
void writing_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager, std::string mcu_function_str);