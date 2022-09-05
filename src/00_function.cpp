#include "00_function.h"

void init_redis_var(sw::redis::Redis* redis)
{
    /* 
     * L'ensemble des variables utilisé par le system de gestion de ram redis
     * est stocké sur le fichier "robot_parameter.yaml" et télécharger à chaque
     * lancement du programme 01.
     */

    std::string path_file = "../data/robot_parameter.yaml";
    
    cv::FileStorage fsSettings(path_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        set_redis_var(redis, "EVENT", get_event_str(0, "LOAD_ROBOT_PARAM", "FAIL"));
        exit(0);
    }

    set_redis_var(redis, "EVENT", get_event_str(0, "LOAD_ROBOT_PARAM", "SUCCESS"));

    read_yaml(redis, &fsSettings, "ROBOT_INFO_ID");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_MODEL");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_PSEUDO");

    read_yaml(redis, &fsSettings, "ROBOT_INFO_MCU_MOTOR_ID");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_MCU_CARGO_ID");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_MCU_INTER_ID");

    read_yaml(redis, &fsSettings, "ROBOT_MODE");

    read_yaml(redis, &fsSettings, "HARD_MCU_MOTOR_PORT_NAME");
    read_yaml(redis, &fsSettings, "HARD_MCU_CARGO_PORT_NAME");
    read_yaml(redis, &fsSettings, "HARD_MCU_INTER_PORT_NAME");
    read_yaml(redis, &fsSettings, "HARD_MCU_MOTOR_COM_STATE");
    read_yaml(redis, &fsSettings, "HARD_MCU_CARGO_COM_STATE");
    read_yaml(redis, &fsSettings, "HARD_MCU_INTER_COM_STATE");

    read_yaml(redis, &fsSettings, "HARD_PIXHAWK_PORT_NAME");
    read_yaml(redis, &fsSettings, "HARD_PIXHAWK_COM_STATE");

    read_yaml(redis, &fsSettings, "ROBOT_INFO_SERVER_ADRESS");

    read_yaml(redis, &fsSettings, "NAV_HMR_MAP_UPDATE");
    read_yaml(redis, &fsSettings, "NAV_HMR_DOWNLOAD_ADRESS");
    read_yaml(redis, &fsSettings, "NAV_HMR_LOCAL_PATH");
    read_yaml(redis, &fsSettings, "NAV_AUTO_DESTINATION");
    read_yaml(redis, &fsSettings, "NAV_AUTO_MODE_PARKING");

    read_yaml(redis, &fsSettings, "MISSION_PARAM_AUTO_ENABLE");
    read_yaml(redis, &fsSettings, "MISSION_MOTOR_BRAKE");
    read_yaml(redis, &fsSettings, "MISSION_UPDATE_GLOBAL_PATH");

    read_yaml(redis, &fsSettings, "MISSION_AUTO_TYPE");
    read_yaml(redis, &fsSettings, "MISSION_AUTO_STATE");
    read_yaml(redis, &fsSettings, "MISSION_MANUAL_TYPE");
    read_yaml(redis, &fsSettings, "MISSION_MANUAL_STATE");

    read_yaml(redis, &fsSettings, "EVENT_MANUAL_CONTROLER_DATA");

    read_yaml(redis, &fsSettings, "HARD_CARGO_STATE");
    read_yaml(redis, &fsSettings, "MISSION_HARD_CARGO");

    read_yaml(redis, &fsSettings, "NAV_MANUAL_MODE");
    read_yaml(redis, &fsSettings, "NAV_MAX_SPEED");
    read_yaml(redis, &fsSettings, "HARD_MOTOR_COMMAND");
}

int64_t get_curr_timestamp()
{
    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::high_resolution_clock::now()).time_since_epoch()).count();

    return timestamp;
}

std::string get_event_str(int ID_event, std::string event_description, std::string event_info)
{
    return std::to_string(get_curr_timestamp()) + "|" + std::to_string(ID_event) + "|" + event_description + "|" + event_info + "|";
}

void set_redis_var(sw::redis::Redis* redis, std::string channel, std::string value)
{
    redis->set(channel, value);
}

std::string get_redis_str(sw::redis::Redis* redis, std::string channel)
{
    return *(redis->get(channel));
}

int get_redis_multi_str(sw::redis::Redis* redis, std::string channel, std::vector<std::string>& stockage)
{
    stockage.clear();
    
    std::string multi_str = *(redis->get(channel));

    std::string T;
    std::stringstream X(multi_str);

    int number_of_data = 0;

    while(std::getline(X, T, '|'))
    {
        stockage.push_back(T);
        number_of_data++;
    } 

    return number_of_data;
}

int get_multi_str(std::string str, std::vector<std::string>& vec_str)
{
    std::string T;
    std::stringstream X(str);

    int number_of_data = 0;

    while(std::getline(X, T, '|'))
    {
        vec_str.push_back(T);
        number_of_data++;
    } 

    return number_of_data;
}

void read_yaml(sw::redis::Redis* redis, cv::FileStorage* file_mng, std::string channel)
{
    std::string read_data;
    (*file_mng)[channel] >> read_data;
    redis->set(channel, read_data);
}

double frequency_to_ms(int frequency)
{
    return 1000 / frequency;
}

bool time_is_over(int64_t curr_timestamp, int64_t ref_timestamp, int64_t max_duration_ms)
{
    if(curr_timestamp - ref_timestamp > max_duration_ms) return true;
    return false;
}