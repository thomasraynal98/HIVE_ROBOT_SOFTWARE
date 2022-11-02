#include "00_function.h"
#include "00_communication.h"

#include <common/mavlink.h>
#include "autopilot_interface.h"
#include "serial_port.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_port_detection;
std::thread thread_read_mcu_motor;
std::thread thread_read_mcu_cargo;
std::thread thread_read_mcu_inter;
std::thread thread_write_mcu_motor;
std::thread thread_write_mcu_cargo;
std::thread thread_write_mcu_inter;
std::thread thread_readwrite_pixhack;
std::thread thread_box_checking;

LibSerial::SerialPort* com_mcu_motor = new LibSerial::SerialPort;
LibSerial::SerialPort* com_mcu_cargo = new LibSerial::SerialPort;
LibSerial::SerialPort* com_mcu_inter = new LibSerial::SerialPort;

LibSerial::SerialPort* com_mcu_temp = new LibSerial::SerialPort;

//===================================================================
// PORT DETECTION THREAD
//===================================================================

void function_thread_port_detection()
{
    /*
     * Les MCU emettent dés leur allumages des status concernant l'ensemble des capteurs
     * et actionneurs qu'ils managent. Ce thread va écouter en permanence les ports 
     * ouvert, et va attribué les ports au 3 MCU intégré dans le robot.
     */ 

    std::string prefix_port      = "/dev/ttyUSB";
    std::string prefix_port2     = "/dev/ttyACM";
    std::string prefix_port_temp = prefix_port;

    size_t timeout_ms = 500;

    while(true)
    {
        for(int i = 0; i < 6; i++)
        {
            prefix_port_temp = prefix_port + std::to_string(i);
   
            if(file_exist(prefix_port_temp))
            {
                if(!port_already_taken(&redis, prefix_port_temp))
                {
                    try{
                        com_mcu_temp->Open(prefix_port_temp);
                        if(com_mcu_temp->IsOpen())
                        {
                            std::string reponse;
                            try
                            {
                                com_mcu_temp->ReadLine(reponse, '\n', timeout_ms);
                            }
                            catch(...){} // CAN'T RECEIVE DATA FROM MCU.

                            if(reponse.length() >= 1)
                            {
                                std::vector<std::string> vec_str;
                                if(get_multi_str(reponse, vec_str) >= 1)
                                {
                                    if(get_redis_str(&redis, "ROBOT_INFO_MCU_MOTOR_ID").compare(vec_str[0]) == 0 && \
                                    get_redis_str(&redis, "HARD_MCU_MOTOR_COM_STATE").compare("DISCONNECTED") == 0)
                                    {
                                        set_redis_var(&redis, "HARD_MCU_MOTOR_PORT_NAME", prefix_port_temp);
                                        set_redis_var(&redis, "HARD_MCU_MOTOR_COM_STATE", "PORT_DETECTED");
                                    }
                                    if(get_redis_str(&redis, "ROBOT_INFO_MCU_CARGO_ID").compare(vec_str[0]) == 0 && \
                                    get_redis_str(&redis, "HARD_MCU_CARGO_COM_STATE").compare("DISCONNECTED") == 0)
                                    {
                                        set_redis_var(&redis, "HARD_MCU_CARGO_PORT_NAME", prefix_port_temp);
                                        set_redis_var(&redis, "HARD_MCU_CARGO_COM_STATE", "PORT_DETECTED");
                                    }
                                    if(get_redis_str(&redis, "ROBOT_INFO_MCU_INTER_ID").compare(vec_str[0]) == 0 && \
                                    get_redis_str(&redis, "HARD_MCU_INTER_COM_STATE").compare("DISCONNECTED") == 0)
                                    {
                                        set_redis_var(&redis, "HARD_MCU_INTER_PORT_NAME", prefix_port_temp);
                                        set_redis_var(&redis, "HARD_MCU_INTER_COM_STATE", "PORT_DETECTED");
                                    }
                                }
                            }
                            com_mcu_temp->Close();
                        }
                    } catch(LibSerial::AlreadyOpen ex) {
                        std::cout << "SerialPort already open : " << prefix_port_temp << std::endl;
                    } catch(LibSerial::OpenFailed ex) {
                        std::cout << "Failed to open SerialPort : " << prefix_port_temp << std::endl;
                    }
                }
            }

            prefix_port_temp = prefix_port2 + std::to_string(i);
            if(file_exist(prefix_port_temp) && 
            get_redis_str(&redis, "HARD_PIXHAWK_COM_STATE").compare("DISCONNECTED") == 0) 
            {
                set_redis_var(&redis, "HARD_PIXHAWK_COM_STATE", "PORT_DETECTED");
                set_redis_var(&redis, "HARD_PIXHAWK_PORT_NAME", prefix_port_temp);
            }
        }
        usleep(500000);
    }
}

//===================================================================
// READ MCU THREAD
//===================================================================

void f_thread_read_mcu_motor()
{

    while(true)
    {
        reading_process(&redis, "HARD_MCU_MOTOR_PORT_NAME", "HARD_MCU_MOTOR_COM_STATE", com_mcu_motor, "MOTOR");
    }
}

void f_thread_read_mcu_cargo()
{
    while(true)
    {
        reading_process(&redis, "HARD_MCU_CARGO_PORT_NAME", "HARD_MCU_CARGO_COM_STATE", com_mcu_cargo, "CARGO");
    }
}

void f_thread_read_mcu_inter()
{
    while(true)
    {
        reading_process(&redis, "HARD_MCU_INTER_PORT_NAME", "HARD_MCU_INTER_COM_STATE", com_mcu_inter, "INTER");
    }
}

//===================================================================
// WRITE MCU THREAD
// La particularité des threads d'écriture c'est qu'ils sont chargé 
// de gerer l'ouverture la fermeture des ports en cas d'evenemnts.
//===================================================================

void f_thread_write_mcu_motor()
{
    double ms_for_loop = frequency_to_ms(std::stoi(get_redis_str(&redis, "HARD_MCU_MOTOR_COM_HZ")));
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        writing_process(&redis, "HARD_MCU_MOTOR_PORT_NAME", "HARD_MCU_MOTOR_COM_STATE", com_mcu_motor, "MOTOR");
    }
}

void f_thread_write_mcu_cargo()
{
    double ms_for_loop = frequency_to_ms(1);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        writing_process(&redis, "HARD_MCU_CARGO_PORT_NAME", "HARD_MCU_CARGO_COM_STATE", com_mcu_cargo, "CARGO");
    }
}

void f_thread_write_mcu_inter()
{
    double ms_for_loop = frequency_to_ms(5);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        writing_process(&redis, "HARD_MCU_INTER_PORT_NAME", "HARD_MCU_INTER_COM_STATE", com_mcu_inter, "INTER");
    }
}

//===================================================================
// PIXHAWK COMMUNICATION THREAD
//===================================================================

void f_thread_readwrite_pixhawk()
{
    Generic_Port *pixhawk_com_manager;
    int baudrate_pixhawk = 57600;
    Autopilot_Interface* autopilot_interface;

    double ms_for_loop = frequency_to_ms(std::stoi(get_redis_str(&redis, "HARD_PIXHAWK_COM_HZ")));
    ms_for_loop = 100;
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        // CLOSING PROCEDURE
        if(!file_exist(get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME")) && \
        get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME").compare("NO_VAL") != 0 && \
        get_redis_str(&redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED") == 0)
        {
            if(pixhawk_com_manager->is_running()) pixhawk_com_manager->stop();
            set_redis_var(&redis, "HARD_PIXHAWK_COM_STATE", "DISCONNECTED");
            set_redis_var(&redis, "HARD_PIXHAWK_PORT_NAME", "NO_VAL");
            pub_redis_var(&redis, "EVENT", get_event_str(0, "CLOSE_COM_PIXHAWK", "SUCCESS"));
        }

        // READING PROCEDURE
        if(get_redis_str(&redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED") == 0 && \
        file_exist(get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME")) && \
        pixhawk_com_manager->is_running())
        {
            // READ ICI
            Mavlink_Messages messages = autopilot_interface->current_messages;

            std::string debug_str = "";

            // LOCAL_POSITION_NED
            // debug_str = std::to_string(get_curr_timestamp()) + "|";
            // debug_str += std::to_string(messages.thomas_add2.x) + "|";
            // debug_str += std::to_string(messages.thomas_add2.y) + "|";
            // debug_str += std::to_string(messages.thomas_add2.yaw) + "|";
            // debug_str += std::to_string(messages.local_position_ned.vx) + "|";
            // debug_str += std::to_string(messages.local_position_ned.vy) + "|";
            // debug_str += std::to_string(messages.local_position_ned.vz) + "|";
            // set_redis_var(&redis, "NAV_LOCAL_POSITION", debug_str);

            // NEW LOCAL_POSITION_NED STRATEGIE.
            debug_str = std::to_string(get_curr_timestamp()) + "|";
            debug_str += std::to_string(messages.local_heading.heading) + "|";
            std::cout << messages.local_heading.heading << std::endl;

            // GLOBAL_POSITION_INT
            if(messages.global_position_int.lat != 0 && messages.global_position_int.lon != 0)
            {
                debug_str = std::to_string(get_curr_timestamp()) + "|";
                debug_str += std::to_string((double)(messages.global_position_int.lon)/10000000) + "|";
                debug_str += std::to_string((double)(messages.global_position_int.lat)/10000000) + "|";
                debug_str += std::to_string((double)(messages.global_position_int.hdg)/100) + "|";
                // debug_str += std::to_string(messages.global_position_int.vx) + "|";
                // debug_str += std::to_string(messages.global_position_int.vy) + "|";
                // debug_str += std::to_string(messages.global_position_int.vz) + "|";
                set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);
            }

            // MSG SYSTEM STATUS
            // debug_str = std::to_string(messages.sys_status.load) + "|";
            // debug_str += std::to_string(messages.sys_status.onboard_control_sensors_enabled) + "|";
            // debug_str += std::to_string(messages.sys_status.drop_rate_comm) + "|";
            // std::cout << debug_str << std::endl;

            // HIGHRES IMU
            // debug_str += std::to_string(messages.highres_imu.xgyro) + "|";
            // debug_str += std::to_string(messages.highres_imu.ygyro) + "|";
            // debug_str += std::to_string(messages.highres_imu.zgyro) + "|";
            // debug_str += std::to_string(messages.highres_imu.xacc) + "|";
            // debug_str += std::to_string(messages.highres_imu.yacc) + "|";
            // debug_str += std::to_string(messages.highres_imu.zacc) + "|";
            // debug_str += std::to_string(messages.highres_imu.temperature) + "|";
            // std::cout << debug_str << std::endl;

            // GPS HIL
            set_redis_var(&redis, "HARD_GPS_FIX_STATE", std::to_string(messages.gps_info.fix_type));
            set_redis_var(&redis, "HARD_GPS_NUMBER", std::to_string(messages.gps_info.satellites_visible));
            // std::cout << debug_str << std::endl;
        }

        // OPENING PROCEDURE
        if(get_redis_str(&redis, "HARD_PIXHAWK_COM_STATE").compare("PORT_DETECTED") == 0 && \
        file_exist(get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME")))
        {
            std::string m = get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME");

            pixhawk_com_manager = new Serial_Port(m.c_str(), baudrate_pixhawk);
            Autopilot_Interface temp_autopilot_interface(pixhawk_com_manager);
            autopilot_interface = &temp_autopilot_interface;

            pixhawk_com_manager->start();
            autopilot_interface->start();

            if(pixhawk_com_manager->is_running())
            {
                set_redis_var(&redis, "HARD_PIXHAWK_COM_STATE", "CONNECTED");
                pub_redis_var(&redis, "EVENT", get_event_str(0, "OPEN_COM_PIXHAWK", "SUCCESS"));
            }
            else
            {
                set_redis_var(&redis, "HARD_PIXHAWK_COM_STATE", "DISCONNECTED");
                set_redis_var(&redis, "HARD_PIXHAWK_PORT_NAME", "NO_VAL");
                pub_redis_var(&redis, "EVENT", get_event_str(0, "OPEN_COM_PIXHAWK", "FAIL"));
            }
        }

        if(get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME").compare("NO_VAL") == 0) usleep(500000);
    }
}

void f_thread_box_checking()
{
    double ms_for_loop = frequency_to_ms(1);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        std::vector<std::string> vect_redis_str;

        get_redis_multi_str(&redis, "EVENT_OPEN_BOX_A", vect_redis_str);
        if(vect_redis_str[1].compare("OPEN") == 0)
        {
            if(time_is_over(get_curr_timestamp(), std::stoul(vect_redis_str[0]), std::stoul(get_redis_str(&redis, "MISSION_BOX_MAX_OPEN_TIME"))))
            {
                pub_redis_var(&redis, "EVENT", get_event_str(1, "BOX_TIME_OUT", "A"));
                // refait timer dans 15 secondes.
                set_redis_var(&redis, "EVENT_OPEN_BOX_A", std::to_string(std::stoul(vect_redis_str[0]) + 15000) + " |OPEN|");
            }
        }

        get_redis_multi_str(&redis, "EVENT_OPEN_BOX_B", vect_redis_str);
        if(vect_redis_str[1].compare("OPEN") == 0)
        {
            if(time_is_over(get_curr_timestamp(), std::stoul(vect_redis_str[0]), std::stoul(get_redis_str(&redis, "MISSION_BOX_MAX_OPEN_TIME"))))
            {
                pub_redis_var(&redis, "EVENT", get_event_str(1, "BOX_TIME_OUT", "B"));
                set_redis_var(&redis, "EVENT_OPEN_BOX_B", std::to_string(std::stoul(vect_redis_str[0]) + 15000) + " |OPEN|");
            }
        }

        get_redis_multi_str(&redis, "EVENT_OPEN_BOX_C", vect_redis_str);
        if(vect_redis_str[1].compare("OPEN") == 0)
        {
            if(time_is_over(get_curr_timestamp(), std::stoul(vect_redis_str[0]), std::stoul(get_redis_str(&redis, "MISSION_BOX_MAX_OPEN_TIME"))))
            {
                pub_redis_var(&redis, "EVENT", get_event_str(1, "BOX_TIME_OUT", "C"));
                set_redis_var(&redis, "EVENT_OPEN_BOX_C", std::to_string(std::stoul(vect_redis_str[0]) + 15000) + " |OPEN|");
            }
        }
    }
}

//===================================================================
// MAIN PART
//===================================================================

void reset_value()
{
    set_redis_var(&redis, "HARD_MCU_MOTOR_COM_STATE", "DISCONNECTED");
    set_redis_var(&redis, "HARD_MCU_MOTOR_PORT_NAME", "NO_VAL");
    set_redis_var(&redis, "HARD_MCU_CARGO_COM_STATE", "DISCONNECTED");
    set_redis_var(&redis, "HARD_MCU_CARGO_PORT_NAME", "NO_VAL");
    set_redis_var(&redis, "HARD_MCU_INTER_COM_STATE", "DISCONNECTED");
    set_redis_var(&redis, "HARD_MCU_INTER_PORT_NAME", "NO_VAL");
    set_redis_var(&redis, "HARD_PIXHAWK_COM_STATE"  , "DISCONNECTED");
    set_redis_var(&redis, "HARD_PIXHAWK_PORT_NAME"  , "NO_VAL");

    set_redis_var(&redis, "MISSION_HARD_CARGO"      , "0000000000000|CLOSE|CLOSE|CLOSE|");
    set_redis_var(&redis, "HARD_CARGO_STATE"        , "0000000000000|CLOSE|CLOSE|CLOSE|");
}

int main(int argc, char *argv[])
{
    set_redis_var(&redis, "SOFT_PROCESS_ID_HARD", std::to_string(getpid()));

    reset_value();

    thread_port_detection    = std::thread(&function_thread_port_detection);
    thread_read_mcu_motor    = std::thread(&f_thread_read_mcu_motor);
    thread_read_mcu_cargo    = std::thread(&f_thread_read_mcu_cargo);
    thread_read_mcu_inter    = std::thread(&f_thread_read_mcu_inter);
    thread_write_mcu_motor   = std::thread(&f_thread_write_mcu_motor);
    thread_write_mcu_cargo   = std::thread(&f_thread_write_mcu_cargo);
    thread_write_mcu_inter   = std::thread(&f_thread_write_mcu_inter);
    thread_readwrite_pixhack = std::thread(&f_thread_readwrite_pixhawk);
    thread_box_checking      = std::thread(&f_thread_box_checking);

    thread_port_detection.join();
    thread_read_mcu_motor.join();
    thread_read_mcu_cargo.join();
    thread_read_mcu_inter.join();
    thread_write_mcu_motor.join();
    thread_write_mcu_cargo.join();
    thread_write_mcu_inter.join();
    thread_readwrite_pixhack.join();
    thread_box_checking.join();

    return 0;
}