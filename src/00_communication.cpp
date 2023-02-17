#include "00_communication.h"
#include "00_navigation.h"

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
    // if(curr_port_name.compare(*redis->get("HARD_PIXHAWK_PORT_NAME")) == 0)
    // {
    //     return true;
    // }
    if(curr_port_name.compare(*redis->get("HARD_LID1_PORT_NAME")) == 0)
    {
        return true;
    }
    if(curr_port_name.compare(*redis->get("HARD_LID2_PORT_NAME")) == 0)
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
        //   std::cout << "DATA FROM XXX : " << get_curr_timestamp() << reponse << std::endl;
            // ALL INFORMATION READING BY MCU ESP32.

            if(mcu_function_str.compare("CARGO") == 0)
            {
                // Pour stocker et découper la nouvelle lecture.
                std::vector<std::string> vect_reponse_mcu_cargo;
                get_multi_str(reponse, vect_reponse_mcu_cargo);

                std::cout << "DATA FROM XXX : " << get_curr_timestamp() << reponse << std::endl;

                if(vect_reponse_mcu_cargo.size() == 6)
                {
                    // Pour setup la nouvelle lecture sur redis.
                    std::string redis_sensor_str = std::to_string(get_curr_timestamp()) + "|";

                    // Pour comparer avec l'ancienne lecture.
                    std::vector<std::string> new_sensor_vect;

                    // Comparer à la mémoire. (1 + 3 x 5)
                    std::vector<std::string> vect_memory;
                    get_redis_multi_str(redis, "HARD_CARGO_STATE_MEMORY", vect_memory);
                    // std::cout << "SIZE " << vect_memory.size() << std::endl;
                    std::vector<int> vect_sum_memory;
                    vect_sum_memory.push_back(0);
                    vect_sum_memory.push_back(0);
                    vect_sum_memory.push_back(0);
                    for(int i = 1; i < vect_memory.size()-3; i += 3) // On ne prend pas la dernière.
                    {
                        if(vect_memory[i+0].compare("OPEN") == 0) vect_sum_memory[0] += 1;
                        if(vect_memory[i+1].compare("OPEN") == 0) vect_sum_memory[1] += 1;
                        if(vect_memory[i+2].compare("OPEN") == 0) vect_sum_memory[2] += 1;
                    }
                    if(vect_reponse_mcu_cargo[2].compare("1") == 0) vect_sum_memory[0] += 1;
                    if(vect_reponse_mcu_cargo[3].compare("1") == 0) vect_sum_memory[1] += 1;
                    if(vect_reponse_mcu_cargo[4].compare("1") == 0) vect_sum_memory[2] += 1;

                    if(vect_reponse_mcu_cargo.size() == 6) // '\n'
                    {
                      //std::cout << "DATA FROM XXX : " << reponse << std::endl;
                        for(int i = 0; i < vect_sum_memory.size(); i++)
                        {
                          //std::cout << "SUM " << i << " : " << vect_sum_memory[i] << std::endl;
                            if(vect_sum_memory[i] > 20)
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

                    // Save memoire.
                    std::string str_memory = std::to_string(get_curr_timestamp()) + "|";
                    str_memory += (vect_reponse_mcu_cargo[2].compare("1") == 0) ? "OPEN|" : "CLOSE|";
                    str_memory += (vect_reponse_mcu_cargo[3].compare("1") == 0) ? "OPEN|" : "CLOSE|";
                    str_memory += (vect_reponse_mcu_cargo[4].compare("1") == 0) ? "OPEN|" : "CLOSE|";
                    for(int i = 1; i < vect_memory.size() - 3; i++)
                    {
                        str_memory += vect_memory[i] + "|";
                    }
                    // std::cout << "MEMORY " << str_memory << std::endl;
                    set_redis_var(redis, "HARD_CARGO_STATE_MEMORY", str_memory);

                    std::vector<std::string> previous_vect_reponse_mcu_cargo;
                    get_redis_multi_str(redis, "HARD_CARGO_STATE", previous_vect_reponse_mcu_cargo);

                    // Comparer la nouvelle à l'ancienne.
                    for(int i = 0; i < new_sensor_vect.size(); i++)
                    {
                        // std::cout << new_sensor_vect[i] << " " << previous_vect_reponse_mcu_cargo[i+1] << std::endl;
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
                              //std::cout << "CHANGEMENT OPEN" << std::endl;
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

                              //std::cout << "CHANGEMENT CLOSE" << std::endl;

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
                        std::cout << get_curr_timestamp() << " " << reponse << std::endl;

                        // Message de type 6.
                        double dt_left_m  = tic_to_meter * std::stoi(vect_reponse_mcu_motor[2]);
                        double dt_right_m = tic_to_meter * std::stoi(vect_reponse_mcu_motor[5]);
                        double dt_moy_m   = (dt_left_m + dt_right_m) / 2;
                        if(abs(dt_moy_m) < 1.0)
                        {
                            /* Pour éviter les bugs. */
                            double kilometrage = std::stod(get_redis_str(redis, "ROBOT_SESSION_KILOMETRAGE"));
                            kilometrage += abs(dt_moy_m)/1000;
                            set_redis_var(redis, "ROBOT_SESSION_KILOMETRAGE", std::to_string(kilometrage));

                            kilometrage = std::stod(get_redis_str(redis, "ROBOT_TOTAL_KILOMETRAGE"));
                            kilometrage += abs(dt_moy_m)/1000;
                            set_redis_var(redis, "ROBOT_TOTAL_KILOMETRAGE", std::to_string(kilometrage));

                            kilometrage = std::stod(get_redis_str(redis, "ROBOT_TODAY_KILOMETRAGE"));
                            kilometrage += abs(dt_moy_m)/1000;
                            set_redis_var(redis, "ROBOT_TODAY_KILOMETRAGE", std::to_string(kilometrage));
                        }

                        double dt_central_left_m  = tic_to_meter * std::stoi(vect_reponse_mcu_motor[3]);
                        double dt_central_right_m = tic_to_meter * std::stoi(vect_reponse_mcu_motor[6]);
                        double dt_central_moy_m   = (dt_central_left_m + dt_central_right_m) / 2;

                        // if(compare_redis_var(redis, "NAV_HDG_WITH_ENCODER", "ACTIVATE"))
                        // {
                        //     double dt_angle   = (dt_central_right_m - dt_central_left_m) / std::stod(get_redis_str(redis, "HARD_WHEEL_DISTANCE"));
                        //     double m_dt_angle = std::stod(get_redis_str(redis, "NAV_DELTA_HDG_ENCODER"));
                        //     set_redis_var(redis, "NAV_DELTA_HDG_ENCODER", std::to_string(dt_angle+m_dt_angle));
                        //     std::cout << "ADD " << dt_angle << " TOTAL " << dt_angle+m_dt_angle << std::endl;
                        // }
                        // else
                        // {
                        //     set_redis_var(redis, "NAV_DELTA_HDG_ENCODER", "0.0");
                        // }

                        if(abs(dt_central_left_m) < 1.0 && abs(dt_central_right_m) < 1.0)
                        {
                            // NEW ANGLE.
                            std::vector<std::string> vect_redis_str;
                            get_redis_multi_str(redis, "NAV_LOCAL_POSITION", vect_redis_str);

                            double dt_angle    = ((dt_central_right_m - dt_central_left_m) * 0.656) / std::stod(get_redis_str(redis, "HARD_WHEEL_DISTANCE"));
                            double prev_angle  = std::stod(vect_redis_str[3]) - rad_to_deg(dt_angle);
                            if(prev_angle > 360) prev_angle -= 360;
                            if(prev_angle <   0) prev_angle += 360;

                            // [A] GLOBAL PART.
                            std::vector<std::string> vect_previous_global_pos;
                            get_redis_multi_str(redis, "NAV_GLOBAL_POSITION", vect_previous_global_pos);
                            prev_angle  = std::stod(vect_previous_global_pos[3]) - rad_to_deg(dt_angle);
                            if(prev_angle > 360) prev_angle -= 360;
                            if(prev_angle <   0) prev_angle += 360;

                            Geographic_point last_pos = Geographic_point(std::stod(vect_previous_global_pos[1]), std::stod(vect_previous_global_pos[2]));
                            Geographic_point new_pos  = get_new_position(&last_pos, prev_angle, dt_central_moy_m);

                            std::string new_pos_str   = std::to_string(get_curr_timestamp()) + "|";
                            new_pos_str               += std::to_string(new_pos.longitude) + "|";
                            new_pos_str               += std::to_string(new_pos.latitude)  + "|";
                            new_pos_str               += std::to_string(prev_angle) + "|";
                            set_redis_var(redis, "NAV_GLOBAL_POSITION", new_pos_str);

                            // [B] LOCAL PART.
                            std::string new_local_position = std::to_string(get_curr_timestamp()) + "|";
                            new_local_position += std::to_string(std::stod(vect_redis_str[1]) + dt_moy_m * cos(deg_to_rad(prev_angle))) + "|";
                            new_local_position += std::to_string(std::stod(vect_redis_str[2]) + dt_moy_m * sin(deg_to_rad(prev_angle))) + "|";
                            new_local_position += std::to_string(prev_angle) + "|";
                            set_redis_var(redis, "NAV_LOCAL_POSITION", new_local_position);

                            // Estimer la vitesse actuelle en km/h.
                            std::vector<std::string> last_value_vect;
                            get_redis_multi_str(redis, "NAV_CURR_SPEED", last_value_vect);
                            int64_t elasped_time = abs(get_elapsed_time(get_curr_timestamp(), std::stoul(last_value_vect[0])));
                            double speed_kmh = ((dt_moy_m / (elasped_time / 1000)) / 1000) * 3600;
                            std::string curr_speed_str = std::to_string(get_curr_timestamp()) + "|" + std::to_string(speed_kmh) + "|"; 
                            set_redis_var(redis, "NAV_CURR_SPEED", curr_speed_str);
                        }
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
            
                if(vect_reponse_mcu_motor.size() == 4)
                {
                    if(vect_reponse_mcu_motor[1].compare("7") == 0)
                    {    
                        set_redis_var(redis, "NAV_BATTERY_VOLTAGE", vect_reponse_mcu_motor[2]);
                        set_redis_var(redis, "NAV_BATTERY_PERCENTAGE", std::to_string(get_battery_level(std::stod(vect_reponse_mcu_motor[2]),24.0)));
                    }
                }

                if(vect_reponse_mcu_motor.size() == 18 && compare_redis_var(redis, "ROBOT_INFO_MODEL", "MK4"))
                {
                    if(vect_reponse_mcu_motor[1].compare("4") == 0)
                    {
                        std::string msg_str = std::to_string(get_curr_timestamp()) + "|";
                        msg_str += vect_reponse_mcu_motor[2]  + "|";
                        msg_str += vect_reponse_mcu_motor[7]  + "|";
                        msg_str += vect_reponse_mcu_motor[12] + "|";
                        set_redis_var(redis, "HARD_TEMPERATURE_INFO", msg_str);      

                        msg_str = std::to_string(get_curr_timestamp()) + "|";
                        msg_str += std::string((vect_reponse_mcu_motor[2].compare("0")  != 0) ? "1" : "0") + "|";
                        msg_str += std::string((vect_reponse_mcu_motor[7].compare("0")  != 0) ? "1" : "0") + "|";
                        msg_str += std::string((vect_reponse_mcu_motor[12].compare("0") != 0) ? "1" : "0") + "|";
                        set_redis_var(redis, "HARD_RCLAW_STATE", msg_str);  

                        msg_str = std::to_string(get_curr_timestamp()) + "|";
                        msg_str += vect_reponse_mcu_motor[5]  + "|";
                        msg_str += vect_reponse_mcu_motor[10] + "|";
                        msg_str += vect_reponse_mcu_motor[15] + "|";
                        msg_str += vect_reponse_mcu_motor[6]  + "|";
                        msg_str += vect_reponse_mcu_motor[11] + "|";
                        msg_str += vect_reponse_mcu_motor[16] + "|";
                        set_redis_var(redis, "HARD_ENCODER_STATE", msg_str);   

                        msg_str = std::to_string(get_curr_timestamp()) + "|";
                        msg_str += vect_reponse_mcu_motor[3]  + "|";
                        msg_str += vect_reponse_mcu_motor[8]  + "|";
                        msg_str += vect_reponse_mcu_motor[13] + "|";
                        msg_str += vect_reponse_mcu_motor[4]  + "|";
                        msg_str += vect_reponse_mcu_motor[9]  + "|";
                        msg_str += vect_reponse_mcu_motor[14] + "|";
                        set_redis_var(redis, "HARD_MOTOR_STATE", msg_str);     
                    
                    }
                }
            }
        }
        catch(...)
        {
            std::cout << "PAS REUSSI A LIRE." << std::endl;
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
            try
            {
                com_manager->Write(msg_motor_str+'\n'); 
            }
            catch(...)
            {

            }
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

            // std::cout << "SEND : " << msg_box_str << std::endl;

            try
            {
                com_manager->Write(msg_box_str+'\n'); 
            }
            catch(...)
            {

            }
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

float get_battery_level(float curr_voltage, float battery_voltage)
{
    if(battery_voltage == 24.0)
    {
        if(curr_voltage > 29.5) return 100.0;

        std::vector<float> border_vect;
        border_vect.push_back(29.5);
        border_vect.push_back(29.0);
        border_vect.push_back(28.5);
        border_vect.push_back(28.0);
        border_vect.push_back(27.5);
        border_vect.push_back(27.0);
        border_vect.push_back(25.5);
        border_vect.push_back(23.5);
        border_vect.push_back(21.5);
        border_vect.push_back(19.5);

        for(int i = 0; i < border_vect.size(); i++)
        {
            if(i == 0)
            {
                if(curr_voltage > border_vect[i]) return 100.0;
            }
            if(i != 0 && i != border_vect.size()-1)
            {
                if(curr_voltage <= border_vect[i-1] && curr_voltage > border_vect[i])
                {
                    return 10*(10-i) + (curr_voltage - border_vect[i]) * 10.0 / (border_vect[i-1] - border_vect[i]);  
                }
            }
            if(i == border_vect.size()-1)
            {
                if(curr_voltage <= border_vect[i-1] && curr_voltage > border_vect[i])
                {
                    return 10*(10-i) + (curr_voltage - border_vect[i]) * 10.0 / (border_vect[i-1] - border_vect[i]);  
                }
                else
                {
                    return 0.0;
                }
            }
        }
    }
}
