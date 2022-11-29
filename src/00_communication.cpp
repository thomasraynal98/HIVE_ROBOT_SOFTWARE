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

bool port_is_ready_to_use(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager)
{
    if(get_redis_str(redis, curr_port_state).compare("CONNECTED") == 0 && \
    file_exist(get_redis_str(redis, curr_port_name)) && \
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
    file_exist(get_redis_str(redis, curr_port_name)))
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
    !file_exist(get_redis_str(redis, curr_port_name)))
    {
        if(com_manager->IsOpen()) com_manager->Close();
        set_redis_var(redis, curr_port_state, "DISCONNECTED");
        set_redis_var(redis, curr_port_name, "NO_VAL");

        return 1;
    }
    return 0;
}

void reading_process(sw::redis::Redis* redis, std::string curr_port_name, std::string curr_port_state, LibSerial::SerialPort* com_manager, std::string mcu_function_str)
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

            if(mcu_function_str.compare("CARGO") == 0)
            {
                // Pour stocker et découper la nouvelle lecture.
                std::vector<std::string> vect_reponse_mcu_cargo;
                get_multi_str(reponse, vect_reponse_mcu_cargo);

                // Pour setup la nouvelle lecture sur redis.
                std::string redis_sensor_str = std::to_string(get_curr_timestamp()) + "|";

                // Pour comparer avec l'ancienne lecture.
                std::vector<std::string> new_sensor_vect;

                if(vect_reponse_mcu_cargo.size() == 6) // '\n'
                {
                    std::cout << "DATA FROM XXX : " << reponse << std::endl;
                    for(int i = 2; i < 5; i++)
                    {
                        if(vect_reponse_mcu_cargo[i].compare("1") == 0)
                        {
                            redis_sensor_str += "OPEN|";
                            new_sensor_vect.push_back("OPEN");
                        }
                        else
                        {
                            redis_sensor_str += "CLOSE|";
                            new_sensor_vect.push_back("CLOSE");
                        }
                    }
                }

                std::vector<std::string> previous_vect_reponse_mcu_cargo;
                get_redis_multi_str(redis, "HARD_CARGO_STATE", previous_vect_reponse_mcu_cargo);

                // Comparer la nouvelle à l'ancienne.
                for(int i = 0; i < new_sensor_vect.size(); i++)
                {
                    if(new_sensor_vect[i].compare(previous_vect_reponse_mcu_cargo[i+1]) != 0)
                    {
                        if(new_sensor_vect[i].compare("OPEN") == 0)
                        {
                            if(i == 0)
                            {
                                pub_redis_var(redis, "EVENT", get_event_str(3, "BOX_OPEN", "1"));
                                set_redis_var(redis, "EVENT_OPEN_BOX_A", std::to_string(get_curr_timestamp()) + "|OPEN|");
                            }
                            if(i == 1) 
                            {
                                pub_redis_var(redis, "EVENT", get_event_str(3, "BOX_OPEN", "2"));
                                set_redis_var(redis, "EVENT_OPEN_BOX_B", std::to_string(get_curr_timestamp()) + "|OPEN|");
                            }
                            if(i == 2) 
                            {
                                pub_redis_var(redis, "EVENT", get_event_str(3, "BOX_OPEN", "3"));
                                set_redis_var(redis, "EVENT_OPEN_BOX_C", std::to_string(get_curr_timestamp()) + "|OPEN|");
                            }
                        }
                        if(new_sensor_vect[i].compare("CLOSE") == 0)
                        {
                            if(i == 0) 
                            {
                                pub_redis_var(redis, "EVENT", get_event_str(3, "BOX_CLOSE", "1"));
                                set_redis_var(redis, "EVENT_OPEN_BOX_A", std::to_string(get_curr_timestamp()) + "|CLOSE|");
                            }
                            if(i == 1) 
                            {
                                pub_redis_var(redis, "EVENT", get_event_str(3, "BOX_CLOSE", "2"));
                                set_redis_var(redis, "EVENT_OPEN_BOX_B", std::to_string(get_curr_timestamp()) + "|CLOSE|");
                            }
                            if(i == 2) 
                            {
                                pub_redis_var(redis, "EVENT", get_event_str(3, "BOX_CLOSE", "3"));
                                set_redis_var(redis, "EVENT_OPEN_BOX_C", std::to_string(get_curr_timestamp()) + "|CLOSE|");
                            }

                            // Si on detect une fermeture on met à jour l'ordre de mission. MISSION_HARD_CARGO
                            std::vector<std::string> vect_mission_mcu_cargo;
                            get_redis_multi_str(redis, "MISSION_HARD_CARGO", vect_mission_mcu_cargo);

                            std::string str_mission_modify = vect_mission_mcu_cargo[0] + "|";
                            for(int ii = 1; ii < 4; ii++)
                            {
                                if(ii == i+1) str_mission_modify += "CLOSE|";
                                else
                                {
                                    str_mission_modify += vect_mission_mcu_cargo[ii] + "|";
                                }
                            }
                            set_redis_var(redis, "MISSION_HARD_CARGO", str_mission_modify);
                        }
                    }
                }

                // publier la nouvelle configuration sensor sur redis.
                if(vect_reponse_mcu_cargo.size() == 6) // '\n'
                {
                    set_redis_var(redis, "HARD_CARGO_STATE", redis_sensor_str);
                }
            }
            if(mcu_function_str.compare("MOTOR") == 0)
            {
                // Pour stocker et découper la nouvelle lecture.
                std::vector<std::string> vect_reponse_mcu_motor;
                get_multi_str(reponse, vect_reponse_mcu_motor);
                double tic_to_meter = std::stod(get_redis_str(redis, "HARD_ENCODER_TIC_TO_M"));

                if(vect_reponse_mcu_motor.size() == 9 && compare_redis_var(redis, "ROBOT_INFO_MODEL", "MK4"))
                {
                    if(vect_reponse_mcu_motor[1].compare("6") == 0)
                    {
                        // Message de type 6.
                        double dt_left_m  = tic_to_meter * std::stoi(vect_reponse_mcu_motor[2]);
                        double dt_right_m = tic_to_meter * std::stoi(vect_reponse_mcu_motor[5]);
                        double dt_moy_m   = (dt_left_m + dt_right_m) / 2;

                        // Lire la position actuelle.
                        std::vector<std::string> vect_redis_str;
                        get_redis_multi_str(redis, "NAV_LOCAL_POSITION", vect_redis_str);
                        
                        // Calculer la nouvelle position.
                        std::string new_local_position = std::to_string(get_curr_timestamp()) + "|";
                        new_local_position += std::to_string(std::stod(vect_redis_str[1]) + dt_moy_m * cos(deg_to_rad(std::stod(vect_redis_str[3])))) + "|";
                        new_local_position += std::to_string(std::stod(vect_redis_str[2]) + dt_moy_m * sin(deg_to_rad(std::stod(vect_redis_str[3])))) + "|";
                        new_local_position += vect_redis_str[3] + "|";
            
                        set_redis_var(redis, "NAV_LOCAL_POSITION", new_local_position);
                    }
                }
                if(vect_reponse_mcu_motor.size() == 5 && compare_redis_var(redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                {
                    if(vect_reponse_mcu_motor[1].compare("6") == 0)
                    {
                        // Message de type 6.
                        double dt_left_m  = tic_to_meter * std::stoi(vect_reponse_mcu_motor[2]);
                        double dt_right_m = tic_to_meter * std::stoi(vect_reponse_mcu_motor[3]);
                        double dt_moy_m   = (dt_left_m + dt_right_m) / 2;

                        // Lire la position actuelle.
                        std::vector<std::string> vect_redis_str;
                        get_redis_multi_str(redis, "NAV_LOCAL_POSITION", vect_redis_str);
                        
                        // Calculer la nouvelle position.
                        std::string new_local_position = std::to_string(get_curr_timestamp()) + "|";
                        new_local_position += std::to_string(std::stod(vect_redis_str[1]) + dt_moy_m * cos(deg_to_rad(std::stod(vect_redis_str[3])))) + "|";
                        new_local_position += std::to_string(std::stod(vect_redis_str[2]) + dt_moy_m * sin(deg_to_rad(std::stod(vect_redis_str[3])))) + "|";
                        new_local_position += vect_redis_str[3] + "|";
            
                        set_redis_var(redis, "NAV_LOCAL_POSITION", new_local_position);
                    }
                }
            }
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
                    for(int i = 1; i < 7; i++) msg_motor_str += vect_motor_command[i] + "|";
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
            
            // std::cout << "SEND : " << msg_motor_str << std::endl;
            com_manager->Write(msg_motor_str+'\n'); 
        }

        if(mcu_function_str.compare("CARGO") == 0)
        {
            std::string msg_box_str = "M|2|";

            std::vector<std::string> vect_box_command;
            get_redis_multi_str(redis, "MISSION_HARD_CARGO", vect_box_command);

            if(vect_box_command.size() == 4)
            {
                for(int i = 1; i < 4; i++)
                {
                    if(vect_box_command[i].compare("OPEN") == 0)
                    {
                        msg_box_str += "1|";
                    }
                    else
                    {
                        msg_box_str += "0|";
                    }
                }
            }

            std::cout << "SEND : " << msg_box_str << std::endl;
            com_manager->Write(msg_box_str+'\n'); 
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

int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 9)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

