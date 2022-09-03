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
   
            if(port_is_detected(prefix_port_temp))
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
        reading_process(&redis, "HARD_MCU_MOTOR_PORT_NAME", "HARD_MCU_MOTOR_COM_STATE", com_mcu_motor);
    }
}

void f_thread_read_mcu_cargo()
{
    while(true)
    {
        reading_process(&redis, "HARD_MCU_CARGO_PORT_NAME", "HARD_MCU_CARGO_COM_STATE", com_mcu_cargo);
    }
}

void f_thread_read_mcu_inter()
{
    while(true)
    {
        reading_process(&redis, "HARD_MCU_INTER_PORT_NAME", "HARD_MCU_INTER_COM_STATE", com_mcu_inter);
    }
}

//===================================================================
// WRITE MCU THREAD
// La particularité des threads d'écriture c'est qu'ils sont chargé de gerer l'ouverture &
// la fermeture des ports en cas d'evenemnts.
//===================================================================

void f_thread_write_mcu_motor()
{
    double ms_for_loop = frequency_to_ms(10);
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
    double ms_for_loop = frequency_to_ms(5);
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
// PIXHAWK COMMUNICATION
//===================================================================

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
}

int main(int argc, char *argv[])
{
    reset_value();

    thread_port_detection  = std::thread(&function_thread_port_detection);
    thread_read_mcu_motor  = std::thread(&f_thread_read_mcu_motor);
    thread_read_mcu_cargo  = std::thread(&f_thread_read_mcu_cargo);
    thread_read_mcu_inter  = std::thread(&f_thread_read_mcu_inter);
    thread_write_mcu_motor = std::thread(&f_thread_write_mcu_motor);
    thread_write_mcu_cargo = std::thread(&f_thread_write_mcu_cargo);
    thread_write_mcu_inter = std::thread(&f_thread_write_mcu_inter);

    thread_port_detection.join();
    thread_read_mcu_motor.join();
    thread_read_mcu_cargo.join();
    thread_read_mcu_inter.join();
    thread_write_mcu_motor.join();
    thread_write_mcu_cargo.join();
    thread_write_mcu_inter.join();
}