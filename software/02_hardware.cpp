#include "00_function.h"
#include "00_communication.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_port_detection;
std::thread thread_read_mcu_motor;
std::thread thread_read_mcu_cargo;
std::thread thread_read_mcu_inter;
std::thread thread_write_mcu_motor;
std::thread thread_write_mcu_cargo;
std::thread thread_write_mcu_inter;

LibSerial::SerialPort* com_mcu_motor = new LibSerial::SerialPort;
LibSerial::SerialPort* com_mcu_cargo = new LibSerial::SerialPort;
LibSerial::SerialPort* com_mcu_inter = new LibSerial::SerialPort;

LibSerial::SerialPort* com_mcu_temp = new LibSerial::SerialPort;

void function_thread_port_detection()
{
    /*
     * Les MCU emettent dés leur allumages des status concernant l'ensemble des capteurs
     * et actionneurs qu'ils managent. Ce thread va écouter en permanence les ports 
     * ouvert, et va attribué les ports au 3 MCU intégré dans le robot.
     */ 

    std::string prefix_port = "/dev/ttyUSB";
    std::string prefix_port_temp = prefix_port;

    size_t timeout_ms = 500;

    while(true)
    {
        for(int i = 0; i < 6; i++)
        {
            prefix_port_temp = prefix_port + std::to_string(i);
            struct stat buffer;   
            if(stat(prefix_port_temp.c_str(), &buffer) == 0)
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
                            catch(...){}

                            if(reponse.length() >= 1)
                            {
                                std::cout << reponse << " " << prefix_port_temp << std::endl;
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
                        }
                    } catch(LibSerial::AlreadyOpen ex) {
                        std::cout << "SerialPort already open : " << prefix_port_temp << std::endl;
                    } catch(LibSerial::OpenFailed ex) {
                        std::cout << "Failed to open SerialPort : " << prefix_port_temp << std::endl;
                    }
                    com_mcu_temp->Close();
                }
            }
        }
    }
}

//===================================================================
// READ MCU THREAD
//===================================================================

// void f_thread_read_mcu_motor()
// {
//     while(true)
//     {
//         if(com_is_available(&redis, com_mcu_motor, "HARD_MCU_MOTOR_COM_STATE", "HARD_MCU_MOTOR_PORT_NAME"))
//         {
//             std::cout << "DETECT A" << std::endl;
//             usleep(500000);
//         }
//         else{usleep(1000000);}
//     }
// }

void f_thread_read_mcu_cargo()
{
    while(true)
    {
        if(com_is_available(&redis, com_mcu_cargo, "HARD_MCU_CARGO_COM_STATE", "HARD_MCU_CARGO_PORT_NAME"))
        {
            std::cout << "DETECT B" << std::endl;
            usleep(500000);
        }
        else{usleep(1000000);}
    }
}

// void f_thread_read_mcu_inter()
// {
//     while(true)
//     {
//         if(com_is_available(&redis, com_mcu_inter, "HARD_MCU_INTER_COM_STATE", "HARD_MCU_INTER_PORT_NAME"))
//         {
//             std::cout << "DETECT C" << std::endl;
//             usleep(500000);
//         }
//         else{usleep(1000000);}
//     }
// }

//===================================================================
// WRITE MCU THREAD
//===================================================================

// void f_thread_write_mcu_motor()
// {
//     double ms_for_loop = frequency_to_ms(10);
//     auto next = std::chrono::high_resolution_clock::now();

//     while(true)
//     {
//         next += std::chrono::milliseconds((int)ms_for_loop);
//         std::this_thread::sleep_until(next);

//         if(get_redis_str(&redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0)
//         {
//             if(com_mcu_motor != NULL)
//             {
//                 if(com_mcu_motor->IsOpen())
//                 {
//                     // BEST PROTECTION.
//                 }  
//             }
//         }
//         else if(get_redis_str(&redis, "HARD_MCU_MOTOR_COM_STATE").compare("PORT_DETECTED") == 0)
//         {
//             com_mcu_motor->Open(get_redis_str(&redis, "HARD_MCU_MOTOR_PORT_NAME"));
//             usleep(500);
//             if(com_mcu_motor->IsOpen())
//             {
//                 set_redis_var(&redis, "HARD_MCU_MOTOR_COM_STATE", "CONNECTED");
//             }
//             else
//             {
//                 com_mcu_motor->Close();
//                 com_mcu_motor = NULL;
//                 set_redis_var(&redis, "HARD_MCU_MOTOR_PORT_NAME", "NO_VAL");
//                 set_redis_var(&redis, "HARD_MCU_MOTOR_COM_STATE", "DISCONNECTED");
//             }
//         }
//     }
// }

void f_thread_write_mcu_cargo()
{
    double ms_for_loop = frequency_to_ms(4);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        if(get_redis_str(&redis, "HARD_MCU_CARGO_COM_STATE").compare("CONNECTED") == 0)
        {
            if(com_mcu_cargo != nullptr)
            {
                std::cout << "just berfore the crash ?" << com_mcu_cargo->IsOpen() << std::endl;
                if(com_mcu_cargo->IsOpen())
                {
                    // BEST PROTECTION.
                }
                else
                {
                    com_mcu_cargo = nullptr;
                    std::cout << "crash soon" << std::endl;
                    set_redis_var(&redis, "HARD_MCU_CARGO_PORT_NAME", "NO_VAL");
                    set_redis_var(&redis, "HARD_MCU_CARGO_COM_STATE", "DISCONNECTED");
                }
            }
        }
        else if(get_redis_str(&redis, "HARD_MCU_CARGO_COM_STATE").compare("PORT_DETECTED") == 0)
        {
            com_mcu_cargo = new LibSerial::SerialPort(get_redis_str(&redis, "HARD_MCU_CARGO_PORT_NAME"));
            // com_mcu_cargo->Open(get_redis_str(&redis, "HARD_MCU_CARGO_PORT_NAME"));
            usleep(500);
            if(com_mcu_cargo->IsOpen())
            {
                set_redis_var(&redis, "HARD_MCU_CARGO_COM_STATE", "CONNECTED");
            }
            else
            {
                com_mcu_cargo = nullptr;
                set_redis_var(&redis, "HARD_MCU_CARGO_PORT_NAME", "NO_VAL");
                set_redis_var(&redis, "HARD_MCU_CARGO_COM_STATE", "DISCONNECTED");
            }
        }
    }
}

// void f_thread_write_mcu_inter()
// {
//     double ms_for_loop = frequency_to_ms(4);
//     auto next = std::chrono::high_resolution_clock::now();

//     while(true)
//     {
//         next += std::chrono::milliseconds((int)ms_for_loop);
//         std::this_thread::sleep_until(next);

//         if(get_redis_str(&redis, "HARD_MCU_INTER_COM_STATE").compare("CONNECTED") == 0)
//         {
//             if(com_mcu_inter != NULL)
//             {
//                 if(com_mcu_inter->IsOpen())
//                 {
//                     // BEST PROTECTION.
//                 }  
//             }
//         }
//         else if(get_redis_str(&redis, "HARD_MCU_INTER_COM_STATE").compare("PORT_DETECTED") == 0)
//         {
//             com_mcu_inter->Open(get_redis_str(&redis, "HARD_MCU_INTER_PORT_NAME"));
//             usleep(500);
//             if(com_mcu_inter->IsOpen())
//             {
//                 set_redis_var(&redis, "HARD_MCU_INTER_COM_STATE", "CONNECTED");
//             }
//         }
//     }
// }

//===================================================================
// MAIN PART
//===================================================================

int main(int argc, char *argv[])
{
    thread_port_detection  = std::thread(&function_thread_port_detection);
    // thread_read_mcu_motor  = std::thread(&f_thread_read_mcu_motor);
    thread_read_mcu_cargo  = std::thread(&f_thread_read_mcu_cargo);
    // thread_read_mcu_motor  = std::thread(&f_thread_read_mcu_inter);
    // thread_write_mcu_motor = std::thread(&f_thread_write_mcu_motor);
    thread_write_mcu_cargo = std::thread(&f_thread_write_mcu_cargo);
    // thread_write_mcu_motor = std::thread(&f_thread_write_mcu_inter);

    thread_port_detection.join();
    // thread_read_mcu_motor.join();
    thread_read_mcu_cargo.join();
    // thread_read_mcu_motor.join();
    // thread_write_mcu_motor.join();
    thread_write_mcu_cargo.join();
    // thread_write_mcu_motor.join();
}