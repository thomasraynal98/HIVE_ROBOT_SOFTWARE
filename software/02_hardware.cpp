#include "00_function.h"
#include "00_communication.h"
#include "00_navigation.h"

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
std::thread thread_local_joystick;

LibSerial::SerialPort* com_mcu_motor = new LibSerial::SerialPort;
LibSerial::SerialPort* com_mcu_cargo = new LibSerial::SerialPort;
LibSerial::SerialPort* com_mcu_inter = new LibSerial::SerialPort;

LibSerial::SerialPort* com_mcu_temp = new LibSerial::SerialPort;

//===================================================================
// DETECTION MCU PORT
// Permet de récuperer les ports utilisés et de définir le hardware
// qui est associé.
//===================================================================

void function_thread_port_detection()
{
    /**
     * NOTE:
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
// Permet de lire les informations en provenance des différent MCU.
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
// de gerer l'ouverture la fermeture des ports en cas d'evenements.
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
// Récupere les informations envoyer par la pixhawk.
//===================================================================

void f_thread_readwrite_pixhawk()
{
    Generic_Port *pixhawk_com_manager;
    int baudrate_pixhawk = 115200;
    Autopilot_Interface* autopilot_interface;

    double ms_for_loop = frequency_to_ms(std::stoi(get_redis_str(&redis, "HARD_PIXHAWK_COM_HZ")));
    ms_for_loop = 100;
    auto next = std::chrono::high_resolution_clock::now();

    // variable pour filtre local.
    int counter_outlier = 0;
    int max_counter_outlier = 5;
    bool accept_value = false;

    // variable pour filtre global.
    std::vector<std::string> vect_redis_str;
    get_redis_multi_str(&redis, "ROBOT_INFO_HOME_POSITION", vect_redis_str);
    Geographic_point homeland = Geographic_point(std::stod(vect_redis_str[0]), std::stod(vect_redis_str[1]));

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
            pub_redis_var(&redis, "EVENT", get_event_str(2, "CLOSE_COM_PIXHAWK", "SUCCESS"));
        }

        // READING PROCEDURE
        if(get_redis_str(&redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED") == 0 && \
        file_exist(get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME")) && \
        pixhawk_com_manager->is_running())
        {
            // READ ICI
            Mavlink_Messages messages = autopilot_interface->current_messages;

            std::string debug_str = "";

            /**
             * NOTE: Refaire ce code en filtrant les valeurs abérantes que peux renvoyer le
             * systême pixhawk.
             * [1] Local navigation.
             * [2] global navigation.
             */

            // [1] Local navigation.
            std::vector<std::string> vect_redis_str;
            get_redis_multi_str(&redis, "NAV_LOCAL_POSITION", vect_redis_str);
            accept_value = true;

            int tempo_hdg = messages.local_heading.heading;
            if(abs(tempo_hdg - std::stoi(vect_redis_str[3])) > 20)
            {
                counter_outlier++;
                accept_value = false;
            }
            if(counter_outlier > max_counter_outlier)
            {
                counter_outlier = 0;
                accept_value = true;
            }   

            if(tempo_hdg == 0) accept_value = false;

            if(accept_value)
            {
                counter_outlier = 0;
                debug_str = std::to_string(get_curr_timestamp()) + "|";
                debug_str += vect_redis_str[1] + "|";
                debug_str += vect_redis_str[2] + "|";
                debug_str += std::to_string(tempo_hdg) + "|";
                set_redis_var(&redis, "NAV_LOCAL_POSITION", debug_str);
            }

            // [2] global navigation.
            Geographic_point tempo_pos = Geographic_point(((double)(messages.global_position_int.lon)/10000000), ((double)(messages.global_position_int.lat)/10000000));
            tempo_hdg = (double)(messages.global_position_int.hdg)/100;

            if(get_angular_distance(&tempo_pos, &homeland) < 10000 && (tempo_hdg>0 && tempo_hdg<360))
            {
                debug_str = std::to_string(get_curr_timestamp()) + "|";
                debug_str += std::to_string(tempo_pos.longitude) + "|";
                debug_str += std::to_string(tempo_pos.latitude) + "|";
                debug_str += std::to_string(tempo_hdg) + "|";
                set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);
            }

            // [3] gps reading.
            set_redis_var(&redis, "HARD_GPS_FIX_STATE", std::to_string(messages.gps_raw.fix_type));
            set_redis_var(&redis, "HARD_GPS_NUMBER", std::to_string(messages.gps_raw.satellites_visible));
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
                pub_redis_var(&redis, "EVENT", get_event_str(2, "OPEN_COM_PIXHAWK", "SUCCESS"));
            }
            else
            {
                set_redis_var(&redis, "HARD_PIXHAWK_COM_STATE", "DISCONNECTED");
                set_redis_var(&redis, "HARD_PIXHAWK_PORT_NAME", "NO_VAL");
                pub_redis_var(&redis, "EVENT", get_event_str(2, "OPEN_COM_PIXHAWK", "FAIL"));
            }
        }

        if(get_redis_str(&redis, "HARD_PIXHAWK_PORT_NAME").compare("NO_VAL") == 0) usleep(500000);
    }
}

//===================================================================
// BOX MANAGEMENT
// Ce thread permet d'envoyer une notification si le box est ouvert
// depuis trop longtemps. Dans ce cas le thread enverra au server
// une notification toute les 15 secondes.
//===================================================================

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
                pub_redis_var(&redis, "EVENT", get_event_str(2, "BOX_TIME_OUT", "A"));
                // refait timer dans 15 secondes.
                set_redis_var(&redis, "EVENT_OPEN_BOX_A", std::to_string(std::stoul(vect_redis_str[0]) + 15000) + " |OPEN|");
            }
        }

        get_redis_multi_str(&redis, "EVENT_OPEN_BOX_B", vect_redis_str);
        if(vect_redis_str[1].compare("OPEN") == 0)
        {
            if(time_is_over(get_curr_timestamp(), std::stoul(vect_redis_str[0]), std::stoul(get_redis_str(&redis, "MISSION_BOX_MAX_OPEN_TIME"))))
            {
                pub_redis_var(&redis, "EVENT", get_event_str(2, "BOX_TIME_OUT", "B"));
                set_redis_var(&redis, "EVENT_OPEN_BOX_B", std::to_string(std::stoul(vect_redis_str[0]) + 15000) + " |OPEN|");
            }
        }

        get_redis_multi_str(&redis, "EVENT_OPEN_BOX_C", vect_redis_str);
        if(vect_redis_str[1].compare("OPEN") == 0)
        {
            if(time_is_over(get_curr_timestamp(), std::stoul(vect_redis_str[0]), std::stoul(get_redis_str(&redis, "MISSION_BOX_MAX_OPEN_TIME"))))
            {
                pub_redis_var(&redis, "EVENT", get_event_str(2, "BOX_TIME_OUT", "C"));
                set_redis_var(&redis, "EVENT_OPEN_BOX_C", std::to_string(std::stoul(vect_redis_str[0]) + 15000) + " |OPEN|");
            }
        }
    }
}

//===================================================================
// LOCAL BLUETOOTH JOYSTICK THREAD
// Ce thread permet au detenteur de la manette reine spécifique 
// au robot de le controller directement grace au bluetooth.
//===================================================================

void f_thread_local_joystick()
{
    double ms_for_loop = frequency_to_ms(4);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        if(file_exist("/dev/input/js0"))
        {
            /**
             * NOTE: Utiliser les notes de la sorte ces quand même assez cool.
             */
            set_redis_var(&redis, "HARD_LOCAL_JS_COM_STATE", "PORT_DETECTED");

            const char *device;
            int js;
            struct js_event event;
            struct axis_state axes[3] = {0};
            size_t axis;

            device = "/dev/input/js0";
            js = open(device, O_RDONLY);

            if(js != -1)
            {
                set_redis_var(&redis, "HARD_LOCAL_JS_COM_STATE", "CONNECTED");
                Js_state xbox_controller;

                while (read_event(js, &event) == 0)
                {
                    set_redis_var(&redis, "HARD_LOCAL_JS_COM_STATE", "CONNECTED");
                    switch (event.type)
                    {
                        case JS_EVENT_BUTTON:
                            xbox_controller.button_state[event.number] = event.value ? true : false;
                            break;
                        case JS_EVENT_AXIS:
                            axis = get_axis_state(&event, axes);
                            if(axis < 3)
                            {
                                xbox_controller.axis_state_vect[axis].x = axes[axis].x;
                                xbox_controller.axis_state_vect[axis].y = axes[axis].y;
                                set_redis_var(&redis, "ROBOT_MODE",           "MANUAL");
                                set_redis_var(&redis, "MISSION_MANUAL_TYPE",  "MANUAL_MOVE");
                                set_redis_var(&redis, "MISSION_MANUAL_STATE", "IN_PROGRESS");
                                set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");
                            }
                            break;
                        default:
                            /* Ignore init events. */
                            break;
                    }

                    // xbox_controller.show(); 

                    // // URGENCE STOP  
                    // if(xbox_controller.button_state[1])
                    // {
                    //     set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                    // }       

                    // // PILOTAGE JOYSTICK LOCAL START / STOP
                    // if(xbox_controller.button_state[6])
                    // {
                    //     set_redis_var(&redis, "NAV_LOCAL_JS_MODE", "ACTIVATE");
                    // } 
                    // if(xbox_controller.button_state[7])
                    // {
                    //     set_redis_var(&redis, "NAV_LOCAL_JS_MODE", "DEACTIVATE");
                    // } 

                    // // CHANGE MODE
                    // if(xbox_controller.button_state[4])
                    // {
                    //     set_redis_var(&redis, "NAV_MANUAL_MODE", "STANDARD_MAX");
                    // } 
                    // if(xbox_controller.button_state[5])
                    // {
                    //     set_redis_var(&redis, "NAV_MANUAL_MODE", "STANDARD");
                    // } 

                    // // OPEN ALL BOX
                    // if(xbox_controller.button_state[3])
                    // {
                    //     std::string new_mission_cargo_str = std::to_string(get_curr_timestamp()) + "|OPEN|OPEN|OPEN|";
                    //     set_redis_var(&redis, "MISSION_HARD_CARGO", new_mission_cargo_str);
                    // }

                    // URGENCE STOP  
                    if(xbox_controller.button_state[1])
                    {
                        set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                    }       

                    // PILOTAGE JOYSTICK LOCAL START / STOP
                    if(xbox_controller.button_state[6])
                    {
                        set_redis_var(&redis, "NAV_LOCAL_JS_MODE", "ACTIVATE");
                    } 
                    if(xbox_controller.button_state[7])
                    {
                        set_redis_var(&redis, "NAV_LOCAL_JS_MODE", "DEACTIVATE");
                    } 

                    // CHANGE MODE
                    if(xbox_controller.button_state[11])
                    {
                        if(compare_redis_var(&redis, "NAV_MANUAL_MODE", "STANDARD_MAX") == 0)
                        {
                            set_redis_var(&redis, "NAV_MANUAL_MODE", "STANDARD_MAX");                      
                        }
                        else
                        {
                            set_redis_var(&redis, "NAV_MANUAL_MODE", "STANDARD");    
                        }
                    } 

                    // OPEN ALL BOX
                    if(xbox_controller.button_state[4])
                    {
                        std::string new_mission_cargo_str = std::to_string(get_curr_timestamp()) + "|OPEN|OPEN|OPEN|";
                        set_redis_var(&redis, "MISSION_HARD_CARGO", new_mission_cargo_str);
                    }

                    // Send information to redis.
                    std::string redis_str = std::to_string(get_curr_timestamp()) + "|" + xbox_controller.get_str();
                    set_redis_var(&redis, "EVENT_LOCAL_JS_DATA", redis_str);
                    
                    fflush(stdout);
                }

                close(js);
                set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                set_redis_var(&redis, "NAV_LOCAL_JS_MODE", "DEACTIVATE");
                set_redis_var(&redis, "HARD_LOCAL_JS_COM_STATE", "DISCONNECTED");
            }
        }
        else
        {
            set_redis_var(&redis, "NAV_LOCAL_JS_MODE", "DEACTIVATE");
            set_redis_var(&redis, "HARD_LOCAL_JS_COM_STATE", "DISCONNECTED");
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
    thread_local_joystick    = std::thread(&f_thread_local_joystick);

    thread_port_detection.join();
    thread_read_mcu_motor.join();
    thread_read_mcu_cargo.join();
    thread_read_mcu_inter.join();
    thread_write_mcu_motor.join();
    thread_write_mcu_cargo.join();
    thread_write_mcu_inter.join();
    thread_readwrite_pixhack.join();
    thread_box_checking.join();
    thread_local_joystick.join();

    return 0;
}