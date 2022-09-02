#include "00_communication.h"

bool port_already_taken(sw::redis::Redis* redis, std::string curr_port_name)
{
    if(curr_port_name.compare(*redis->get("HARD_MCU_MOTOR_PORT_NAME")) == 0)
    {
        return true;
    }
    if(curr_port_name.compare(*redis->get("HARD_MCU_CARGO_PORT_NAME")) == 0)
    {
        return true;
    }
    if(curr_port_name.compare(*redis->get("HARD_MCU_INTER_PORT_NAME")) == 0)
    {
        return true;
    }
    return false;
}

bool com_is_available(sw::redis::Redis* redis, LibSerial::SerialPort* com_mcu, std::string channel_com_state, \
std::string channel_port_name)
{
    if(com_mcu != NULL)
    {
        if(get_redis_str(redis, channel_com_state).compare("CONNECTED") == 0)
        {
            if(com_mcu->IsOpen()) return true;
        }
    }
    return false;
}