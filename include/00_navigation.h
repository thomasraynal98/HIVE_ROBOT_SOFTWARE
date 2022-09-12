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
    Geographic_point point;
    int node_ID;
    double col_idx, row_idx;

    Data_node(int a, double b, double c)
        : point(b,c)
        , node_ID(a)
        , col_idx(0)
        , row_idx(0)
        {}
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
        max_speed = 7.001;
        available = true;
        
        // Calcul distance between point.
        double d = B->point.longitude - A->point.longitude;
        double x = cos(B->point.latitude) * sin(d);
        double y = cos(A->point.latitude) * sin(B->point.latitude) - (sin(A->point.latitude) * cos(B->point.latitude) * cos(d));
        deg_to_B = atan2(x,y) * 180 / 3.14;
        if(deg_to_B < 0) deg_to_B = 180 + (180 + deg_to_B);

        deg_to_A = deg_to_B + 180;
        if(deg_to_A > 360) deg_to_A = deg_to_A - 360;

        // Distance.
        double lat1  = toRadians(A->point.latitude);
        double long1 = toRadians(A->point.longitude);
        double lat2  = toRadians(B->point.latitude);
        double long2 = toRadians(B->point.longitude);
        // Haversine Formula
        long double dlong = long2 - long1;
        long double dlat = lat2 - lat1;
    
        long double ans = pow(sin(dlat / 2), 2) +
                            cos(lat1) * cos(lat2) *
                            pow(sin(dlong / 2), 2);
    
        ans = 2 * asin(sqrt(ans));
        long double R = 6371;
        length = ans * R;
    }
};

struct Navigation_road{
    Geographic_point* pt_origin;
    Geographic_point* pt_target;
    double length_road;
    double orientation_target;
    double speed_information;
    int road_id;

    Navigation_road(Geographic_point* last_node, Geographic_point* next_node, double a, double b, double c, double d)
        : pt_origin(last_node)
        , pt_target(next_node)
        , length_road(a)
        , orientation_target(b)
        , speed_information(c)
        , road_id(d)
        {}
};

struct Robot_position
{
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
        {}
    
    void update_pos(sw::redis::Redis* redis)
    {
        std::vector<std::string> vect_str;

        get_redis_multi_str(redis, "NAV_GLOBAL_POSITION", vect_str);
        g_timestamp = std::stoul(vect_str[0]);
        g_longitude = std::stod(vect_str[1]);
        g_latitude  = std::stod(vect_str[2]);
        g_hdg       = std::stod(vect_str[3]);

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

int auto_mode_available(sw::redis::Redis* redis);
int manual_mode_available(sw::redis::Redis* redis);
std::string map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle, double max_speed_Ms);
void read_xlsx_hmr(std::string file_path, std::vector<Data_node>& vect_node, std::vector<Data_road>& vect_road);
double get_max_speed(sw::redis::Redis* redis, std::string robot_mode, std::string mode_param);
void compute_current_road(sw::redis::Redis* redis, Robot_position* curr_pos, std::vector<Data_road>& road_vector, std::vector<Navigation_road>& destination_route, int opt_flag);
double compute_shortest_distance_to_road(Navigation_road* road_vector, Robot_position* curr_pos);
double compute_shortest_distance_to_road(Data_road road_vector, Robot_position* curr_pos);
double compute_dist_between_geo_point(double latA, double longA, double latB, double longB);
long double toRadians(const long double degree);
bool detect_road_shift(sw::redis::Redis* redis, std::string new_road);