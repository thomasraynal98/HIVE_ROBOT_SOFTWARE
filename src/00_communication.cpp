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
                com_manager->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                com_manager->FlushIOBuffers();
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
    if(flag_open == 0)  pub_redis_var(redis, "EVENT", get_event_str(0, event_open_port_str, "FAIL"));
    if(flag_open == 1)  pub_redis_var(redis, "EVENT", get_event_str(0, event_open_port_str, "SUCCESS"));

    if(port_is_ready_to_use(redis, curr_port_name, curr_port_state, com_manager))
    {
        //====================
        // SPEAKING SPACE.
        //====================
        if(mcu_function_str.compare("MOTOR") == 0)
        {
            std::string msg_motor_str = "M|0|";

            std::vector<std::string> vect_motor_command;
            get_redis_multi_str(redis, "HARD_MOTOR_COMMAND", vect_motor_command);

            if(!time_is_over(get_curr_timestamp(), std::stoul(vect_motor_command[0]), 1000))
            {
                if(get_redis_str(redis, "ROBOT_INFO_MODEL").compare("MK4") == 0)
                {
                    for(int i = 1; i < 7; i++) msg_motor_str += vect_motor_command[1] + "|";
                }
                if(get_redis_str(redis, "ROBOT_INFO_MODEL").compare("MK4_LIGHT") == 0)
                {
                    msg_motor_str += vect_motor_command[2] + "|";
                    msg_motor_str += vect_motor_command[5] + "|";
                }
            }
            else
            {
                if(get_redis_str(redis, "ROBOT_INFO_MODEL").compare("MK4") == 0)
                {
                    msg_motor_str = "M|0|0.0|0.0|0.0|0.0|0.0|0.0|";
                }
                if(get_redis_str(redis, "ROBOT_INFO_MODEL").compare("MK4_LIGHT") == 0)
                {
                    msg_motor_str = "M|0|0.0|0.0|";
                }
            }
            
            std::cout << "SEND : " << msg_motor_str << std::endl;
            com_manager->Write(msg_motor_str+'\n'); 
        }
    }
    
    flag_close = port_closing_process(redis, curr_port_name, curr_port_state, com_manager);
    if(flag_close == 1) pub_redis_var(redis, "EVENT", get_event_str(0, event_close_port_str, "SUCCESS"));
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
        if(vect_msg[i].type_str.compare("l") == 0)
        {
            socket_msg->get_map()[vect_msg[i].channel_str] = sio::int_message::create(std::stoul(vect_msg[i].value));
        }
    }

    current_socket->emit(emit_title, socket_msg);
}

void send_event_server(sio::socket::ptr current_socket, std::string mission_title, std::string mission_state)
{
    std::vector<Server_var> vect_msg_server;
    vect_msg_server.push_back(Server_var("s", "TITLE"    , mission_title));
    vect_msg_server.push_back(Server_var("s", "INFO"     , mission_state));
    vect_msg_server.push_back(Server_var("l", "TIMESTAMP", std::to_string(get_curr_timestamp())));

    send_msg_server(current_socket, "EVENT", vect_msg_server);
}
