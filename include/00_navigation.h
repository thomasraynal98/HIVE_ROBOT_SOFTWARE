#include <sw/redis++/redis++.h>
#include "00_function.h"

#include <OpenXLSX.hpp>

struct Geographic_point
{
    double longitude, latitude;

    Geographic_point(double a, double b)
        : longitude(a)
        , latitude(b)
        {}
};

struct Data_node
{   
    int node_ID;
    Geographic_point* point;
    double col_idx, row_idx;

    Data_node(int a, Geographic_point* b)
        : node_ID(a)
        , col_idx(0.0)
        , row_idx(0.0)
        {   point = new Geographic_point(b->longitude, b->latitude);}
};

struct Data_road
{
    int road_ID;
    Data_node* A;
    Data_node* B;
    double deg_to_A, deg_to_B;
    double length;
    bool available;
    double max_speed;

    Data_road(int a, Data_node* b, Data_node* c)
        : road_ID(a)
        , A(b)
        , B(c)
        , deg_to_A(0.1)
        , deg_to_B(0.1)
        {init_data_road();}

    long double toRadians(const long double degree)
    {
        long double one_deg = (M_PI) / 180;
        return (one_deg * degree);
    }

    void init_data_road()
    {
        // [!] Rien à faire car tout est setup dans le téléchargement du txt.
    }
};

struct Robot_position
{
    Geographic_point* point;
    double g_longitude, g_latitude, g_hdg;
    double l_x, l_y, l_hdg;
    int64_t g_timestamp, l_timestamp;

    Robot_position()
        : g_longitude(0.0)
        , g_latitude(0.0)
        , g_hdg(0.0)
        , l_x(0.0)
        , l_y(0.0)
        , l_hdg(0.0)
        , g_timestamp(0)
        , l_timestamp(0)
        {point = new Geographic_point(0.0,0.0);}
    
    void update_pos(sw::redis::Redis* redis)
    {
        std::vector<std::string> vect_str;

        get_redis_multi_str(redis, "NAV_GLOBAL_POSITION", vect_str);
        g_timestamp = std::stoul(vect_str[0]);
        g_longitude = std::stod(vect_str[1]);
        g_latitude  = std::stod(vect_str[2]);
        g_hdg       = std::stod(vect_str[3]);
        point->longitude = g_longitude;
        point->latitude  = g_latitude;

        vect_str.clear();
        get_redis_multi_str(redis, "NAV_LOCAL_POSITION", vect_str);
        l_timestamp = std::stoul(vect_str[0]);
        l_x         = std::stod(vect_str[1]);
        l_y         = std::stod(vect_str[2]);
        l_hdg       = std::stod(vect_str[3]);
        
        int gps_state = std::stoi(get_redis_str(redis, "HARD_GPS_FIX_STATE"));
        if(get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("DISCONNECTED") != 0)
        {
            if(gps_state == 0) set_redis_var(redis, "NAV_GLOBAL_LOCALISATION_STATE", "NO_AVAILABLE");
            if(gps_state == 1) set_redis_var(redis, "NAV_GLOBAL_LOCALISATION_STATE", "INSTABLE");
            if(gps_state == 2) set_redis_var(redis, "NAV_GLOBAL_LOCALISATION_STATE", "AVAILABLE");
            if(gps_state == 3) set_redis_var(redis, "NAV_GLOBAL_LOCALISATION_STATE", "OPTIMAL");
        }
        else
        {
            set_redis_var(redis, "NAV_GLOBAL_LOCALISATION_STATE", "NO_AVAILABLE");
        }
    }
};

// [!] Pour faire le A*.
struct Path_node{
    int index_node;
    Data_node* n;
    bool closet;
    double fscore;
    double gscore;
    std::vector<Path_node*> connection_data;
    std::vector<double> connection_weight;
    Path_node* come_from; 

    Path_node(int id, Data_node* a)
        : n(a)
        , closet(false)
        , fscore(99999)
        , gscore(99999)
        , index_node(id)
        , come_from(NULL)
        {}
};

int auto_mode_available(sw::redis::Redis* redis);
int manual_mode_available(sw::redis::Redis* redis);
std::string map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle, double max_speed_Ms);
void Read_TXT_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector);
double get_max_speed(sw::redis::Redis* redis, std::string robot_mode, std::string mode_param);
int get_road_ID_from_pos(sw::redis::Redis* redis, std::vector<Data_road>& vect_road, Geographic_point* curr_pos);
double get_bearing(Geographic_point* pointA, Geographic_point* pointB);
long double deg_to_rad(const long double degree);
double get_angular_distance(Geographic_point* pointA, Geographic_point* pointB);
double get_dist_from_pos_to_toad(Geographic_point* pointA, Geographic_point* pointB, Geographic_point* pointC);
void update_path_node(std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector, std::vector<Path_node>& graph);
double compute_weight_road(Data_road* road);
int get_node_ID_from_road(std::vector<Data_road>& vect_road, int road_ID);