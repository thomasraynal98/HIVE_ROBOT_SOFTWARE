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

bool port_is_detected(std::string curr_port_name)
{
    struct stat buffer;   
    if(stat(curr_port_name.c_str(), &buffer) == 0) return true;
    return false;
}

bool port_is_ready_to_use(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager)
{
    if(get_redis_str(redis, curr_port_state).compare("CONNECTED") == 0 && \
    port_is_detected(get_redis_str(redis, curr_port_name)) && \
    com_manager->IsOpen())
    {
        return true;
    }
    return false;
}

int port_opening_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager)
{
    /*
     * return info
     * -1 = pas de port spécifié
     * 0  = ouverture à échoué
     * 1  = ouverture
     * 2  = déja ouvert
     * 3  = problem, il est déja ouvert alors qu'on n'a pas encore ouvert, mal fermé.
     */

    if(get_redis_str(redis, curr_port_name).compare("NO_VAL") == 0) return -1;
    if(get_redis_str(redis, curr_port_state).compare("CONNECTED") == 0) return 2;

    if(get_redis_str(redis, curr_port_state).compare("PORT_DETECTED") == 0 && \
    port_is_detected(get_redis_str(redis, curr_port_name)))
    {
        if(!com_manager->IsOpen())
        {
            try
            {
                com_manager->Open(get_redis_str(redis, curr_port_name));
                set_redis_var(redis, curr_port_state, "CONNECTED");

                return 1;
            }
            catch(...)
            {
                set_redis_var(redis, curr_port_state, "DISCONNECTED");
                set_redis_var(redis, curr_port_name, "NO_VAL");

                return 0;
            }               
        }
    }

    return 3;
}

int port_closing_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager)
{
    /*
     * return info
     * -1 = 
     * 0  = pas de fermeture
     * 1  = fermeture
     */

    if(get_redis_str(redis, curr_port_state).compare("CONNECTED") == 0 && \
    !port_is_detected(get_redis_str(redis, curr_port_name)))
    {
        if(com_manager->IsOpen()) com_manager->Close();
        set_redis_var(redis, curr_port_state, "DISCONNECTED");
        set_redis_var(redis, curr_port_name, "NO_VAL");

        return 1;
    }
    return 0;
}

void reading_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager)
{
    size_t timeout_ms = 1000;
    std::string reponse;

    if(port_is_ready_to_use(redis, curr_port_name, curr_port_state, com_manager))
    {
        try
        {
            com_manager->ReadLine(reponse, '\n', timeout_ms);
            // std::cout << "DATA FROM XXX : " << reponse << std::endl;
            // ALL INFORMATION READING BY MCU ESP32.
        }
        catch(...)
        {
            // std::cout << "PAS REUSSI A LIRE." << std::endl;
        }
        
    }
    else{usleep(1000000);}
}

void writing_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager, std::string mcu_function_str)
{
    int flag_open, flag_close;

    std::string event_open_port_str  = "OPEN_COM_MCU_" + mcu_function_str;
    std::string event_close_port_str = "CLOSE_COM_MCU_" + mcu_function_str;

    flag_open =  port_opening_process(redis, curr_port_name ,curr_port_state, com_manager);
    if(flag_open == 0)  set_redis_var(redis, "EVENT", get_event_str(0, event_open_port_str, "FAIL"));
    if(flag_open == 1)  set_redis_var(redis, "EVENT", get_event_str(0, event_open_port_str, "SUCCESS"));

    if(port_is_ready_to_use(redis, curr_port_name, curr_port_state, com_manager))
    {
        // SPEAK
    }
    
    flag_close = port_closing_process(redis, curr_port_name, curr_port_state, com_manager);
    if(flag_close == 1) set_redis_var(redis, "EVENT", get_event_str(0, event_close_port_str, "SUCCESS"));
}

void send_msg_server(sio::socket::ptr current_socket, std::string emit_title, std::vector<Server_var>& vect_msg)
{
    sio::message::ptr socket_msg = sio::object_message::create();

    for(int i = 0; i < vect_msg.size(); i++)
    {
        if(vect_msg[i].type_str.compare("s") == 0)
        {
            socket_msg->get_map()[vect_msg[i].channel_str] = sio::string_message::create(vect_msg[i].value);
        }
        if(vect_msg[i].type_str.compare("i") == 0)
        {
            socket_msg->get_map()[vect_msg[i].channel_str] = sio::int_message::create(std::stoi(vect_msg[i].value));
        }
        if(vect_msg[i].type_str.compare("d") == 0)
        {
            socket_msg->get_map()[vect_msg[i].channel_str] = sio::double_message::create(std::stod(vect_msg[i].value));
        }
    }

    current_socket->emit(emit_title, socket_msg);
}

void send_mission_update_server(sio::socket::ptr current_socket, std::string mission_title, std::string mission_state, int flag)
{
    std::vector<Server_var> vect_msg_server;
    vect_msg_server.push_back(Server_var("s", "MISSION_INFO",       mission_title));
    vect_msg_server.push_back(Server_var("s", "MISSION_STATE",      mission_state));
    vect_msg_server.push_back(Server_var("i", "MISSION_START_FLAG", std::to_string(flag)));
    send_msg_server(current_socket, "ROBOT_MISSION_INFO", vect_msg_server);
}
