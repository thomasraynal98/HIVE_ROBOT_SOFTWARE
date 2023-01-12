#include "00_function.h"

void init_redis_var_id(sw::redis::Redis* redis)
{
    /* 
     * L'ensemble des variables utilisé par le system de gestion de ram redis
     * est stocké sur le fichier "robot_parameter.yaml" et télécharger à chaque
     * lancement du programme 01.
     */

    std::string path_file = "../data/robot_id.yaml";
    
    cv::FileStorage fsSettings(path_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        pub_redis_var(redis, "EVENT", get_event_str(0, "LOAD_ROBOT_PARAM_ID", "FAIL"));
        exit(0);
    }

    pub_redis_var(redis, "EVENT", get_event_str(0, "LOAD_ROBOT_PARAM_ID", "SUCCESS"));

    read_yaml(redis, &fsSettings, "ROBOT_INFO_ID");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_MODEL");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_EXPLOITATION");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_PSEUDO");
    read_yaml(redis, &fsSettings, "HARD_CAM1_ID");
    read_yaml(redis, &fsSettings, "HARD_CAM2_ID");
}

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
        pub_redis_var(redis, "EVENT", get_event_str(0, "LOAD_ROBOT_PARAM", "FAIL"));
        exit(0);
    }

    pub_redis_var(redis, "EVENT", get_event_str(0, "LOAD_ROBOT_PARAM", "SUCCESS"));

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

    read_yaml(redis, &fsSettings, "HARD_LID1_PORT_NAME");
    read_yaml(redis, &fsSettings, "HARD_LID2_PORT_NAME");

    read_yaml(redis, &fsSettings, "HARD_PIXHAWK_PORT_NAME");
    read_yaml(redis, &fsSettings, "HARD_PIXHAWK_COM_STATE");

    read_yaml(redis, &fsSettings, "ROBOT_INFO_SERVER_ADRESS");

    read_yaml(redis, &fsSettings, "NAV_HMR_MAP_UPDATE");
    read_yaml(redis, &fsSettings, "NAV_HMR_DOWNLOAD_ADRESS");
    read_yaml(redis, &fsSettings, "NAV_HMR_LOCAL_PATH");
    read_yaml(redis, &fsSettings, "NAV_AUTO_DESTINATION");
    read_yaml(redis, &fsSettings, "NAV_AUTO_MODE_PARKING");

    read_yaml(redis, &fsSettings, "NAV_AUTO_MODE");
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

    read_yaml(redis, &fsSettings, "HARD_MCU_MOTOR_COM_HZ");
    read_yaml(redis, &fsSettings, "HARD_PIXHAWK_COM_HZ");

    read_yaml(redis, &fsSettings, "SERVER_COM_STATE");

    read_yaml(redis, &fsSettings, "HARD_GPS_NUMBER");
    read_yaml(redis, &fsSettings, "HARD_GPS_FIX_STATE");
    read_yaml(redis, &fsSettings, "NAV_GLOBAL_POSITION");
    read_yaml(redis, &fsSettings, "NAV_LOCAL_POSITION");
    read_yaml(redis, &fsSettings, "NAV_ROAD_CURRENT_ID");
    read_yaml(redis, &fsSettings, "NAV_GLOBAL_LOCALISATION_STATE");
    read_yaml(redis, &fsSettings, "NAV_AUTO_CROSSING_DIST_M");
    read_yaml(redis, &fsSettings, "NAV_AUTO_DESTINATION_ROAD_ID");

    read_yaml(redis, &fsSettings, "NAV_AUTO_ROAD_RADIUS");
    read_yaml(redis, &fsSettings, "NAV_AUTO_TARGET_EXTENSION");
    read_yaml(redis, &fsSettings, "NAV_AUTO_DESTINATION_CROSSING_M");

    read_yaml(redis, &fsSettings, "HARD_WHEEL_RADIUS");
    read_yaml(redis, &fsSettings, "HARD_WHEEL_DISTANCE");

    read_yaml(redis, &fsSettings, "NAV_AUTO_PROJECT_DESTINATION");

    read_yaml(redis, &fsSettings, "ENV_CAM1_OBJECTS");
    read_yaml(redis, &fsSettings, "ENV_CAM2_OBJECTS");
    read_yaml(redis, &fsSettings, "ENV_CAM1_HUMANS");
    read_yaml(redis, &fsSettings, "ENV_CAM2_HUMANS");
    read_yaml(redis, &fsSettings, "ENV_LID1_OBJECTS");
    read_yaml(redis, &fsSettings, "ENV_LID2_OBJECTS");

    read_yaml(redis, &fsSettings, "HARD_CAM1_DX");
    read_yaml(redis, &fsSettings, "HARD_CAM1_DY");
    read_yaml(redis, &fsSettings, "HARD_CAM1_ANGLE");
    read_yaml(redis, &fsSettings, "HARD_CAM2_DX");
    read_yaml(redis, &fsSettings, "HARD_CAM2_DY");
    read_yaml(redis, &fsSettings, "HARD_CAM2_ANGLE");
    read_yaml(redis, &fsSettings, "HARD_LID1_DX");
    read_yaml(redis, &fsSettings, "HARD_LID1_DY");
    read_yaml(redis, &fsSettings, "HARD_LID1_ANGLE");
    read_yaml(redis, &fsSettings, "HARD_LID2_DX");
    read_yaml(redis, &fsSettings, "HARD_LID2_DY");
    read_yaml(redis, &fsSettings, "HARD_LID2_ANGLE");

    read_yaml(redis, &fsSettings, "NAV_OBJ_MIN_DIST");
    read_yaml(redis, &fsSettings, "NAV_OBJ_MAX_DIST");
    read_yaml(redis, &fsSettings, "NAV_OBJ_MIN_SPACE");
    read_yaml(redis, &fsSettings, "NAV_OBJ_MIN_OBSERVATION");
    read_yaml(redis, &fsSettings, "NAV_OBJ_CLEARING_DIST");
    read_yaml(redis, &fsSettings, "NAV_OBJ_CLEARING_TIME_MS");

    read_yaml(redis, &fsSettings, "NAV_OBJ_SAFETY_DIST_M");

    read_yaml(redis, &fsSettings, "NAV_OPT_STREAM");

    read_yaml(redis, &fsSettings, "EVENT_OPEN_BOX_A");
    read_yaml(redis, &fsSettings, "EVENT_OPEN_BOX_B");
    read_yaml(redis, &fsSettings, "EVENT_OPEN_BOX_C");
    read_yaml(redis, &fsSettings, "MISSION_BOX_MAX_OPEN_TIME");

    read_yaml(redis, &fsSettings, "MISSION_ESTI_TIME_TO_TARGET");
    read_yaml(redis, &fsSettings, "MISSION_ESTI_DIST_TO_TARGET");

    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_SYS");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_HARD");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_SERV");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_NAV");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_PERCEP");

    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_SYS_STATUS");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_HARD_STATUS");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_NAV_STATUS");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_PERCEP_STATUS");
    read_yaml(redis, &fsSettings, "SOFT_PROCESS_ID_SERV_STATUS");

    read_yaml(redis, &fsSettings, "SERVER_MAX_TIME");
    read_yaml(redis, &fsSettings, "ROBOT_INFO_HOME_POSITION");
    read_yaml(redis, &fsSettings, "NAV_AUTO_MODE_PARKING_DIST_M");

    read_yaml(redis, &fsSettings, "ENCODED_CAM1");
    read_yaml(redis, &fsSettings, "ENCODED_CAM1_TIMESTAMP");
    read_yaml(redis, &fsSettings, "ENCODED_CAM2");
    read_yaml(redis, &fsSettings, "ENCODED_CAM2_TIMESTAMP");
    read_yaml(redis, &fsSettings, "ENCODED_CAM3");
    read_yaml(redis, &fsSettings, "ENCODED_CAM3_TIMESTAMP");
    read_yaml(redis, &fsSettings, "ENCODED_CAM4");
    read_yaml(redis, &fsSettings, "ENCODED_CAM4_TIMESTAMP");
    read_yaml(redis, &fsSettings, "ENCODED_CAM5");
    read_yaml(redis, &fsSettings, "ENCODED_CAM5_TIMESTAMP");
    read_yaml(redis, &fsSettings, "ENCODED_CAM6");
    read_yaml(redis, &fsSettings, "ENCODED_CAM6_TIMESTAMP");

    read_yaml(redis, &fsSettings, "SERVER_MAX_STREAM_VIDEO_HZ");
    read_yaml(redis, &fsSettings, "HARD_ENCODER_TIC_TO_M");

    read_yaml(redis, &fsSettings, "HARD_LOCAL_JS_COM_STATE");
    read_yaml(redis, &fsSettings, "NAV_LOCAL_JS_MODE");
    read_yaml(redis, &fsSettings, "EVENT_LOCAL_JS_DATA");

    read_yaml(redis, &fsSettings, "NAV_BATTERY_VOLTAGE");
    read_yaml(redis, &fsSettings, "NAV_BATTERY_PERCENTAGE");

    read_yaml(redis, &fsSettings, "NAV_MAX_ACCEL");
    read_yaml(redis, &fsSettings, "NAV_MAX_DECCEL");

    read_yaml(redis, &fsSettings, "HARD_WHEEL_SEPARATION");

    read_yaml(redis, &fsSettings, "SOFT_STATE_LIDAR_0");
    read_yaml(redis, &fsSettings, "SOFT_STATE_LIDAR_1");
    read_yaml(redis, &fsSettings, "SOFT_STATE_FRONT_CAMERA");
    read_yaml(redis, &fsSettings, "SOFT_STATE_BACK_CAMERA");

    read_yaml(redis, &fsSettings, "NAV_IMU_ACC");
    read_yaml(redis, &fsSettings, "NAV_INCLINE_CHANGE");
    read_yaml(redis, &fsSettings, "NAV_CURR_SPEED");

    read_yaml(redis, &fsSettings, "HARD_TEMPERATURE_INFO");

    read_yaml(redis, &fsSettings, "HARD_ENCODER_STATE");
    read_yaml(redis, &fsSettings, "HARD_MOTOR_STATE");

    read_yaml(redis, &fsSettings, "HARD_RCLAW_STATE");
    read_yaml(redis, &fsSettings, "HARD_CARGO_STATE_MEMORY");

    read_yaml(redis, &fsSettings, "NAV_AUTO_NEXT_POINT_DIST_MEMORY");
    read_yaml(redis, &fsSettings, "NAV_HDG_WITH_ENCODER");
    read_yaml(redis, &fsSettings, "NAV_HDG_CURR_ROAD");
    read_yaml(redis, &fsSettings, "NAV_DELTA_HDG_ENCODER");
    read_yaml(redis, &fsSettings, "NAV_CURRENT_ROAD_COOR");

    read_yaml(redis, &fsSettings, "MISSION_RESET_HDG");
    read_yaml(redis, &fsSettings, "MISSION_TRUST_GPS");

    read_yaml(redis, &fsSettings, "SERVER_GOOGLE_MEET_ID");
    read_yaml(redis, &fsSettings, "SERVER_GOOGLE_MEET_MODE");

    read_yaml(redis, &fsSettings, "ROBOT_SESSION_KILOMETRAGE");
    read_yaml(redis, &fsSettings, "ROBOT_TOTAL_KILOMETRAGE");
    read_yaml(redis, &fsSettings, "ROBOT_TODAY_KILOMETRAGE");

    read_yaml(redis, &fsSettings, "NAV_FIXE_ROAD");

    read_yaml(redis, &fsSettings, "NAV_CURR_ROAD_MAX_SPEED");
    read_yaml(redis, &fsSettings, "NAV_OPERATOR_MAX_SPEED_BONUS");
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

void pub_redis_var(sw::redis::Redis* redis, std::string channel, std::string value)
{
    redis->publish(channel, value);
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
    vec_str.clear();

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

bool time_is_over(int64_t curr_timestamp, int64_t end_timestamp)
{
    if(curr_timestamp - end_timestamp > 0) return true;
    return false;
}

int64_t get_elapsed_time(int64_t timesptamp1, int64_t timesptamp2)
{
    return abs(timesptamp1 - timesptamp2);
}

void print_redis(sw::redis::Redis* redis, std::string channel_str)
{
    int max_size = 30;
    int size_channel_title = channel_str.length();

    std::string format_channel_str = "";
    format_channel_str += channel_str;

    for(int i = size_channel_title; i < max_size; i++) format_channel_str += " ";
    format_channel_str += " = ";

    format_channel_str += get_redis_str(redis, channel_str);
    std::cout << format_channel_str << std::endl;
}

std::string get_standard_robot_id_str(sw::redis::Redis* redis)
{
    std::string official_id_str = get_redis_str(redis, "ROBOT_INFO_ID") + "-";
    official_id_str += get_redis_str(redis, "ROBOT_INFO_PSEUDO") + "-";
    official_id_str += get_redis_str(redis, "ROBOT_INFO_MODEL") + "-";
    official_id_str += get_redis_str(redis, "ROBOT_INFO_EXPLOITATION");
    return official_id_str;
}

bool compare_redis_var(sw::redis::Redis* redis, std::string channel, std::string compare)
{
    if(get_redis_str(redis, channel).compare(compare) == 0) return true;
    return false;
}

double rad_to_deg(double rad)
{
    double deg = rad * 180 / M_PI;
    if(deg > 360) deg = deg - 360;
    if(deg < 0)   deg = deg + 360;
    return deg;
}

double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

bool is_same_time(int64_t timesptamp1, int64_t timesptamp2)
{
    if(timesptamp1 == timesptamp2) return true;
    return false;
}

bool file_exist(std::string file_path)
{
    struct stat buffer;   
    if(stat(file_path.c_str(), &buffer) == 0) return true;
    return false;
}

void Write_TXT_file(std::string path, std::string file_data)
{
    std::ofstream myfile;
    myfile.open(path);
    myfile << file_data;
    myfile.close();
}

std::string get_date_of_today()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    return std::to_string(ltm->tm_mday) + "/" + std::to_string(ltm->tm_mon + 1) + "/" + std::to_string(ltm->tm_year + 1900);
}

double read_all_kilometrage(sw::redis::Redis* redis, std::string path)
{
    std::ifstream file(path);
    std::string str; 

    int data_type = 0;

    std::string date_of_today = get_date_of_today();

    double total_kilometrage = 0.0;

    while (std::getline(file, str))
    {
        std::vector<std::string> vect_str;
        get_multi_str(str, vect_str);

        bool data_available = true;

        // std::cout << str << " " << vect_str.size() << std::endl;

        if(vect_str.size() != 0)
        {
            if(vect_str[0].compare("KILOMETRAGE") == 0){data_type = 1; data_available = false;}

            if(data_available)
            {
                if(data_type == 1 && vect_str.size() == 2)
                {
                    // KILOMETRAGE DAY PER DAY.
                    if(vect_str[0].compare(date_of_today) == 0)
                    {
                        set_redis_var(redis, "ROBOT_TODAY_KILOMETRAGE", vect_str[1]);
                    }
                    total_kilometrage += std::stod(vect_str[1]);
                }
            }
        }
    }

    set_redis_var(redis, "ROBOT_TOTAL_KILOMETRAGE", std::to_string(total_kilometrage));
}

double save_kilometrage(sw::redis::Redis* redis, std::string path)
{
    std::ifstream file(path);
    std::string str; 

    int data_type = 0;

    bool detect_current_day = false;

    std::string date_of_today = get_date_of_today();

    std::vector<std::string> all_data;

    while (std::getline(file, str))
    {
        std::vector<std::string> vect_str;
        get_multi_str(str, vect_str);

        bool data_available = true;

        if(vect_str.size() != 0)
        {
            if(vect_str[0].compare("KILOMETRAGE") == 0){data_type = 1; data_available = false; all_data.push_back(str);}

            if(data_available)
            {
                // std::cout << str << " " << vect_str.size() << std::endl;
                if(data_type == 1 && vect_str.size() == 2)
                {
                    // KILOMETRAGE DAY PER DAY.
                    if(vect_str[0].compare(date_of_today) == 0)
                    {
                        double today_km = std::stod(vect_str[1]) + std::stod(get_redis_str(redis, "ROBOT_SESSION_KILOMETRAGE"));
                        set_redis_var(redis, "ROBOT_TODAY_KILOMETRAGE", std::to_string(today_km));
                        set_redis_var(redis, "ROBOT_SESSION_KILOMETRAGE", "0.0");
                        all_data.push_back(get_date_of_today() + "|" + std::to_string(today_km));
                        detect_current_day = true;
                    }
                    else
                    {
                        all_data.push_back(str);
                    }
                }
            }
        }
    }

    if(!detect_current_day)
    {
        std::string new_day = date_of_today + "|" + get_redis_str(redis, "ROBOT_SESSION_KILOMETRAGE");
        all_data.push_back(new_day);
    }

    file.close();

    std::string new_all_data = "";
    for(int i = 0; i < all_data.size(); i++)
    {
        new_all_data += all_data[i] + "\n";
    }
    Write_TXT_file("../data/statistique_utilisation.txt", new_all_data);
}
