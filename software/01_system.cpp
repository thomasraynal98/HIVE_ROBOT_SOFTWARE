#include "00_function.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_debug;

//===================================================================
// DESCISION : Le thread qui va SAVE les events. Le programme 03 va
// se charger du reste.
//===================================================================

//===================================================================
// DEBUG : Affichage des informations pour un opérateur console.
//===================================================================

void f_thread_debug()
{
    double ms_for_loop = frequency_to_ms(2);
    auto next = std::chrono::high_resolution_clock::now();

    std::string brut_input_str = "";
    std::vector<std::string>     vect_mode;
    std::string display_mode   = "NO_MODE";
    int display_time_ms        = 0;
    int64_t input_timestamp    = get_curr_timestamp();
    std::cout << "LET'S GO" << std::endl;
    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        if(time_is_over(get_curr_timestamp(), input_timestamp, display_time_ms))
        {
            std::system("clear");
            std::cout << "DISPLAY MODE > ";
            std::cin >> brut_input_str;
            get_multi_str(brut_input_str, vect_mode);

            try
            {
                display_mode    = vect_mode[0];
                display_time_ms = std::stoi(vect_mode[1]) * 1000;
                input_timestamp = get_curr_timestamp();
            }
            catch(...)
            {
                std::cout << "INPUT INVALID" << std::endl;
                display_mode    = "NO_MODE";
                display_time_ms = 0;
                usleep(500000);
            }    
        }

        if(display_mode.compare("MODE1") == 0)
        {
            std::system("clear");

            print_redis(&redis, "ROBOT_INFO_ID");
            print_redis(&redis, "ROBOT_INFO_MODEL");
            print_redis(&redis, "ROBOT_INFO_PSEUDO");
            print_redis(&redis, "ROBOT_INFO_EXPLOITATION");
            print_redis(&redis, "ROBOT_INFO_MCU_MOTOR_ID");
            print_redis(&redis, "ROBOT_INFO_MCU_CARGO_ID");
            print_redis(&redis, "ROBOT_INFO_MCU_INTER_ID");
            print_redis(&redis, "ROBOT_INFO_SERVER_ADRESS");
            print_redis(&redis, "ROBOT_MODE");
            std::cout << std::endl;

            print_redis(&redis, "SERVER_COM_STATE");
            std::cout << std::endl;

            print_redis(&redis, "EVENT");
            print_redis(&redis, "EVENT_MANUAL_CONTROLER_DATA");
            std::cout << std::endl;

            print_redis(&redis, "HARD_MCU_MOTOR_PORT_NAME");
            print_redis(&redis, "HARD_MCU_CARGO_PORT_NAME");
            print_redis(&redis, "HARD_MCU_INTER_PORT_NAME");
            print_redis(&redis, "HARD_PIXHAWK_PORT_NAME");
            print_redis(&redis, "HARD_MCU_MOTOR_COM_STATE");
            print_redis(&redis, "HARD_MCU_CARGO_COM_STATE");
            print_redis(&redis, "HARD_MCU_INTER_COM_STATE");
            print_redis(&redis, "HARD_PIXHAWK_COM_STATE");
            print_redis(&redis, "HARD_MOTOR_COMMAND");
            print_redis(&redis, "HARD_CARGO_STATE");
            print_redis(&redis, "HARD_MCU_MOTOR_COM_HZ");
            print_redis(&redis, "HARD_PIXHAWK_COM_HZ");
            std::cout << std::endl;

            print_redis(&redis, "NAV_HMR_MAP_UPDATE");
            print_redis(&redis, "NAV_HMR_DOWNLOAD_ADRESS");
            print_redis(&redis, "NAV_HMR_LOCAL_PATH");
            print_redis(&redis, "NAV_AUTO_DESTINATION");
            print_redis(&redis, "NAV_AUTO_MODE");
            print_redis(&redis, "NAV_AUTO_MODE_PARKING");
            print_redis(&redis, "NAV_MANUAL_MODE");
            print_redis(&redis, "NAV_MAX_SPEED");
            std::cout << std::endl;

            print_redis(&redis, "MISSION_MOTOR_BRAKE");
            print_redis(&redis, "MISSION_UPDATE_GLOBAL_PATH");
            print_redis(&redis, "MISSION_AUTO_TYPE");
            print_redis(&redis, "MISSION_AUTO_STATE");
            print_redis(&redis, "MISSION_MANUAL_TYPE");
            print_redis(&redis, "MISSION_MANUAL_STATE");
            print_redis(&redis, "MISSION_HARD_CARGO");
        }

        if(display_mode.compare("MODE2") == 0)
        {
            std::system("clear");

            print_redis(&redis, "ROBOT_MODE");
            print_redis(&redis, "SERVER_COM_STATE");
            std::cout << std::endl;

            print_redis(&redis, "EVENT");
            print_redis(&redis, "EVENT_MANUAL_CONTROLER_DATA");
            std::cout << std::endl;

            print_redis(&redis, "HARD_MCU_MOTOR_PORT_NAME");
            print_redis(&redis, "HARD_MCU_CARGO_PORT_NAME");
            print_redis(&redis, "HARD_MCU_INTER_PORT_NAME");
            print_redis(&redis, "HARD_PIXHAWK_PORT_NAME");
            print_redis(&redis, "HARD_MCU_MOTOR_COM_STATE");
            print_redis(&redis, "HARD_MCU_CARGO_COM_STATE");
            print_redis(&redis, "HARD_MCU_INTER_COM_STATE");
            print_redis(&redis, "HARD_PIXHAWK_COM_STATE");
            print_redis(&redis, "HARD_MOTOR_COMMAND");
            print_redis(&redis, "HARD_CARGO_STATE");
            print_redis(&redis, "HARD_MCU_MOTOR_COM_HZ");
            print_redis(&redis, "HARD_PIXHAWK_COM_HZ");
            std::cout << std::endl;

            print_redis(&redis, "NAV_HMR_MAP_UPDATE");
            print_redis(&redis, "NAV_HMR_DOWNLOAD_ADRESS");
            print_redis(&redis, "NAV_HMR_LOCAL_PATH");
            print_redis(&redis, "NAV_AUTO_DESTINATION");
            print_redis(&redis, "NAV_AUTO_MODE");
            print_redis(&redis, "NAV_AUTO_MODE_PARKING");
            print_redis(&redis, "NAV_MANUAL_MODE");
            print_redis(&redis, "NAV_MAX_SPEED");
            std::cout << std::endl;

            print_redis(&redis, "MISSION_MOTOR_BRAKE");
            print_redis(&redis, "MISSION_UPDATE_GLOBAL_PATH");
            print_redis(&redis, "MISSION_AUTO_TYPE");
            print_redis(&redis, "MISSION_AUTO_STATE");
            print_redis(&redis, "MISSION_MANUAL_TYPE");
            print_redis(&redis, "MISSION_MANUAL_STATE");
            print_redis(&redis, "MISSION_HARD_CARGO");

            std::cout << std::endl;
            print_redis(&redis, "HARD_GPS_NUMBER");
            print_redis(&redis, "HARD_GPS_FIX_STATE");
            print_redis(&redis, "NAV_GLOBAL_LOCALISATION_STATE");
            print_redis(&redis, "NAV_GLOBAL_POSITION");
            print_redis(&redis, "NAV_LOCAL_POSITION");
            print_redis(&redis, "NAV_ROAD_CURRENT_ID");
            print_redis(&redis, "NAV_AUTO_CROSSING_DIST_M");
        }
    }
}

int main(int argc, char *argv[])
{
    int opt_reset = 0;
    if(argc == 2) opt_reset = std::atoi(argv[1]);

    if(opt_reset == 0)
    {
        init_redis_var(&redis);
        thread_debug = std::thread(&f_thread_debug);
        thread_debug.join();
    }
}