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

int auto_mode_available(sw::redis::Redis* redis);
int manual_mode_available(sw::redis::Redis* redis);
std::string map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle, double max_speed_Ms);
void read_xlsx_hmr(std::string file_path, std::vector<Data_node>& vect_node, std::vector<Data_road>& vect_road);
double get_max_speed(sw::redis::Redis* redis, std::string robot_mode, std::string mode_param);