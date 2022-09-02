#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "00_function.h"

bool port_already_taken(sw::redis::Redis* redis, std::string curr_port_name);
bool com_is_available(sw::redis::Redis* redis, LibSerial::SerialPort* com_mcu, std::string channel_com_state, std::string channel_port_name);