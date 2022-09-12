#include "00_navigation.h"

int auto_mode_available(sw::redis::Redis* redis)
{
    /*
     * return info
     * -1 = no param selected.
     * AUTO_SIMPLE param.
     * 10 = available.
     * 11 = not available because MCU_MOTOR not connected.
     * 12 = not available because PIXHAWK not connected.
     * 13 = not available because MCU_MOTOR & PIXHAWK not connected.
     * AUTO_STANDARD param.
     * 20 = available.
     */

    std::string opt_str = get_redis_str(redis, "NAV_AUTO_MODE");
    
    if(opt_str.compare("SIMPLE"))
    {
        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 10;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 11;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   != 0) return 12;

    }
    if(opt_str.compare("STANDARD"))
    {

    }
    return -1;
}

int manual_mode_available(sw::redis::Redis* redis)
{
    /*
     * return info
     * 10 = available
     * 11 = not available
     */

    if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0 && \
    get_redis_str(redis, "SERVER_COM_STATE").compare("CONNECTED") == 0) return 10;
    return 11;
}

std::string map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle, double max_speed_Ms)
{
    //TODO: Cette fonction va mapper les commandes brutes de la manelle en commande 
    //TODO: utilisable par le robot.

    std::string command_motor_str = std::to_string(get_curr_timestamp()) + "|";

    // std::cout << back_value << " " << front_value << " " << angle << " " << max_speed_Ms << std::endl;
    
    if(back_value < 0.05 && front_value < 0.05)
    {
        command_motor_str += "0|0|0|0|0|0|";
        return command_motor_str;
    }
    if(back_value > 0.05 && front_value > 0.05)
    {
        command_motor_str += "0|0|0|0|0|0|";
        return command_motor_str;
    }

    double vitesse_max = max_speed_Ms; // m/s

    if(front_value > 0.05)
    {
        /*
         * Info angle.
         * 0   - 90  = courbe vers la gauche
         * 90  - 180 = courbe vers la droite
         * 180 - 270 = rotation droite
         * 270 - 360 = rotation gauche
         */

        double current_speed = front_value * vitesse_max; 

        if(angle >= 0   && angle <= 90)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i >= 3) command_motor_str += std::to_string(current_speed) + "|";
                if(i < 3) command_motor_str += std::to_string(current_speed - current_speed*(90-angle)/90)  + "|";
            }
            return command_motor_str;
        }
        if(angle > 90  && angle <= 180)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i >= 3) command_motor_str += std::to_string(current_speed - current_speed*(angle-90)/90) + "|";
                if(i < 3) command_motor_str += std::to_string(current_speed)  + "|";
            }
            return command_motor_str;
        }
        if(angle > 180 && angle <= 270)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) command_motor_str += std::to_string(current_speed) + "|";
                if(i >= 3) command_motor_str += std::to_string(-1*current_speed) + "|";
            }
            return command_motor_str;
        }
        if(angle > 270 && angle <= 360)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) command_motor_str += std::to_string(-1*current_speed) + "|";
                if(i >= 3) command_motor_str += std::to_string(current_speed) + "|";
            }
            return command_motor_str;
        }
    }

    if(back_value > 0.05)
    {
        double current_speed = back_value * vitesse_max; 

        if(angle == 90)
        {
            for(int i = 0; i < 6; i++)
            {
                command_motor_str += std::to_string(-1*current_speed) + "|";
            }
            return command_motor_str;
        }
        if(angle > 180 && angle <= 270)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) command_motor_str += std::to_string(-1*current_speed) + "|";
                if(i >= 3) command_motor_str += std::to_string(-1*(current_speed - current_speed*(90-(angle-180))/90)) + "|";
            }
            return command_motor_str;
        }
        if(angle > 270 && angle <= 360)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) command_motor_str += std::to_string(-1*(current_speed - current_speed*(angle-270)/90)) + "|";
                if(i >= 3) command_motor_str += std::to_string(-1*current_speed) + "|";
            }
            return command_motor_str;
        }
    }
}

void read_xlsx_hmr(std::string file_path, std::vector<Data_node>& vect_node, std::vector<Data_road>& vect_road)
{
    OpenXLSX::XLDocument doc;
    doc.open(file_path);

    // ADD NODE.
    auto wks = doc.workbook().worksheet("GEO_POINT");

    for(auto& row : wks.rows())
    {
        bool end = false;
        int i = 0;

        int id = -1;
        double longitude = 0;
        double latitude = 0;

        for(auto cell : row.cells(3))
        {
            if(cell.value().type() == OpenXLSX::XLValueType::Empty) break;
            end = true;
            if(i == 0) id        = std::stoi(cell.value());
            if(i == 1) longitude = std::stod(cell.value());
            if(i == 2) latitude  = std::stod(cell.value());
            i++;
        }

        if(!end) break;

        vect_node.push_back(Data_node(id, longitude, latitude));
    }

    // ADD ROAD.
    auto wks2 = doc.workbook().worksheet("PATH");

    for(auto& row : wks2.rows())
    {
        bool end = false;
        int i = 0;

        int id_road = -1;
        int id_pointA = 0;
        int id_pointB = 0;
        double deg_to_A = 0;
        double deg_to_B = 0;
        double length = 0;
        bool available = true;
        double speed = 0;

        for(auto cell : row.cells(8))
        {
            if(cell.value().type() == OpenXLSX::XLValueType::Empty) break;

            end = true;
            if(i == 0) id_road   = std::stoi(cell.value());
            if(i == 1) id_pointA = std::stoi(cell.value());
            if(i == 2) id_pointB = std::stoi(cell.value());
            if(i == 3) deg_to_A  = std::stod(cell.value());
            if(i == 4) deg_to_B  = std::stod(cell.value());
            if(i == 5) length    = std::stod(cell.value());
            if(i == 6)
            {
                int tempo = std::stoi(cell.value());
                if(tempo == 1) available = true;
                else{available = false;}
            }
            if(i == 7) speed     = std::stod(cell.value());
            i++;
        }

        if(!end) break;

        Data_node* tempo_save;
        for(int i = 0; i < vect_node.size(); i++)
        {
            if(id_pointA == vect_node[i].node_ID) { tempo_save = &vect_node[i]; break;}
        }
        for(int i = 0; i < vect_node.size(); i++)
        {
            if(id_pointB == vect_node[i].node_ID)
            {
                Data_road new_road(id_road, tempo_save, &vect_node[i]);
                new_road.available = available;
                new_road.deg_to_A = deg_to_A;
                new_road.deg_to_B = deg_to_B;
                new_road.length = length;
                new_road.max_speed = speed;
                vect_road.push_back(new_road);
            }
        }
    }

    doc.close();
}

double get_max_speed(sw::redis::Redis* redis, std::string robot_mode, std::string mode_param)
{
    if(robot_mode.compare("MANUAL") == 0)
    {
        if(mode_param.compare("STANDARD")     == 0) return std::stod(get_redis_str(redis, "NAV_MAX_SPEED")) * 0.5;
        if(mode_param.compare("STANDARD_MAX") == 0) return std::stod(get_redis_str(redis, "NAV_MAX_SPEED")) * 0.9;
    }

    if(robot_mode.compare("AUTO") == 0)
    {
        return 0.0;
    }

    return 0.0;
}

void compute_current_road(sw::redis::Redis* redis, Robot_position* curr_pos, std::vector<Data_road>& road_vector, std::vector<Navigation_road>& destination_route, int opt_flag)
{
    /*
        Description : 
            Cette fonction très importante permet de déterminer la route sur laquelle le robot
            se trouve actuellement. Elle prend aussi en entré un systême d'option qui permet
            de faire la différence entre un robot en AUTONAV et un robot pas en AUTONAV.
    */

    if(opt_flag == 0)
    { 
        // INITIALISATION PROCESS. OR MANUAL
        double min_dist     = 9999; // km
        double dist         = 0;    // km
        int current_road_ID = 0;

        for(int i = 0; i < road_vector.size(); i++)
        {
            dist = compute_shortest_distance_to_road(road_vector[i], curr_pos);

            if(dist < min_dist)
            {
                min_dist = dist;
                current_road_ID = road_vector[i].road_ID;
            }
        }

        if(detect_road_shift(redis, std::to_string(current_road_ID)))
        {
            std::vector<std::string> vect_curr_road;
            get_redis_multi_str(redis, "NAV_ROAD_CURRENT_ID", vect_curr_road);
            pub_redis_var(redis, "EVENT", get_event_str(2, "CHANGE_ROAD", vect_curr_road[1] + ">" + std::to_string(current_road_ID)));
        }
        std::string redis_msg_str = std::to_string(get_curr_timestamp()) + "|";     
        redis_msg_str += std::to_string(current_road_ID) + "|";
        set_redis_var(redis, "NAV_ROAD_CURRENT_ID", redis_msg_str);
    }
    if(opt_flag == 1)
    {
        // AUTONAV PROCESS.

        std::vector<std::string> vect_curr_road;
        get_redis_multi_str(redis, "NAV_ROAD_CURRENT_ID", vect_curr_road);

        int current_road_ID = std::stoi(vect_curr_road[1]);

        double distance_next_node = 9999; // KM

        double distance_next_road = 9999;
        double distance_curr_road = 9999;

        // First, check if we are far of next NODE.
        for(int i = 0; i < destination_route.size(); i++)
        {
            if(destination_route[i].road_id == current_road_ID)
            {
                // IDEE 1 : next road plus proche que la current road.
                distance_curr_road = compute_shortest_distance_to_road(&destination_route[i]  , curr_pos);        
                distance_next_road = compute_shortest_distance_to_road(&destination_route[i+1], curr_pos);
                if(distance_next_road <= distance_curr_road)
                {
                    pub_redis_var(redis, "EVENT", get_event_str(2, "CHANGE_ROAD_1", vect_curr_road[1] + ">" + std::to_string(destination_route[i+1].road_id)));
                    std::string redis_msg_str = std::to_string(get_curr_timestamp()) + "|";
                    redis_msg_str += std::to_string(destination_route[i+1].road_id) + "|";
                    set_redis_var(redis, "NAV_ROAD_CURRENT_ID", redis_msg_str);
                    return;
                }

                // IDEE 2 : le next node est à moins de NAV_AUTO_CROSSING_DIST_M mètre.
                distance_next_node = compute_dist_between_geo_point(curr_pos->g_latitude, curr_pos->g_longitude, destination_route[i].pt_target->latitude, destination_route[i].pt_target->longitude);          
                if(distance_next_node < std::stod(get_redis_str(redis, "NAV_AUTO_CROSSING_DIST_M")) && \
                (i+1 < destination_route.size()))
                {
                    pub_redis_var(redis, "EVENT", get_event_str(2, "CHANGE_ROAD_2", vect_curr_road[1] + ">" + std::to_string(destination_route[i+1].road_id)));
                    std::string redis_msg_str = std::to_string(get_curr_timestamp()) + "|";
                    redis_msg_str += std::to_string(destination_route[i+1].road_id) + "|";
                    set_redis_var(redis, "NAV_ROAD_CURRENT_ID", redis_msg_str);
                    return;
                }
            }
        }
     
    }
}

double compute_shortest_distance_to_road(Data_road road_vector, Robot_position* curr_pos)
{
    // POTENTIELLEMENT UN PROBLEME AVEC LA DIFFERENCE D'ECHELLE ENTRE LONGITUDE ET LATITUDE.

    double px = road_vector.B->point.longitude - road_vector.A->point.longitude;
    double py = road_vector.B->point.latitude  - road_vector.A->point.latitude;
    
    double norm = px*px + py*py;

    double u = ((curr_pos->g_longitude - road_vector.A->point.longitude) * px + (curr_pos->g_latitude - road_vector.A->point.latitude) * py) / norm;

    if(u > 1.0) u = 1.0;
    else if (u < 0.0) u = 0.0;

    double x = road_vector.A->point.longitude + u * px;
    double y = road_vector.A->point.latitude  + u * py;

    //  (x,y) represent the nearest point from current position to segment.

    return compute_dist_between_geo_point(x, y, curr_pos->g_latitude, curr_pos->g_longitude);

    // // Distance.
    // double lat1  = toRadians(x);
    // double long1 = toRadians(y);
    // double lat2  = toRadians(curr_pos->g_latitude);
    // double long2 = toRadians(curr_pos->g_longitude);

    // // Haversine Formula
    // long double dlong = long2 - long1;
    // long double dlat = lat2 - lat1;

    // long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);

    // ans = 2 * asin(sqrt(ans));
    // long double R = 6371;
    // double length = ans * R;

    // return length;
}

double compute_shortest_distance_to_road(Navigation_road* road_vector, Robot_position* curr_pos)
{
    // POTENTIELLEMENT UN PROBLEME AVEC LA DIFFERENCE D'ECHELLE ENTRE LONGITUDE ET LATITUDE.

    double px = road_vector->pt_target->longitude - road_vector->pt_origin->longitude;
    double py = road_vector->pt_target->latitude  - road_vector->pt_origin->latitude;
    
    double norm = px*px + py*py;

    double u = ((curr_pos->g_longitude - road_vector->pt_origin->longitude) * px + (curr_pos->g_latitude - road_vector->pt_origin->latitude) * py) / norm;

    if(u > 1.0) u = 1.0;
    else if (u < 0.0) u = 0.0;

    double x = road_vector->pt_origin->longitude + u * px;
    double y = road_vector->pt_origin->latitude  + u * py;

    //  (x,y) represent the nearest point from current position to segment.

    return compute_dist_between_geo_point(x, y, curr_pos->g_latitude, curr_pos->g_longitude);

    // // Distance.
    // double lat1  = toRadians(x);
    // double long1 = toRadians(y);
    // double lat2  = toRadians(curr_pos->g_latitude);
    // double long2 = toRadians(curr_pos->g_longitude);

    // // Haversine Formula
    // long double dlong = long2 - long1;
    // long double dlat = lat2 - lat1;

    // long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);

    // ans = 2 * asin(sqrt(ans));
    // long double R = 6371;
    // double length = ans * R;

    // return length;
}

double compute_dist_between_geo_point(double latA, double longA, double latB, double longB)
{
    double lat1 = toRadians(latA);
    double long1 = toRadians(longA);
    double lat2 = toRadians(latB);
    double long2 = toRadians(longB);
    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;

    long double ans = pow(sin(dlat / 2), 2) +
                        cos(lat1) * cos(lat2) *
                        pow(sin(dlong / 2), 2);

    ans = 2 * asin(sqrt(ans));
    long double R = 6371;
    return ans * R;
}

long double toRadians(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

bool detect_road_shift(sw::redis::Redis* redis, std::string new_road)
{
    std::vector<std::string> vect_redis_str;
    get_redis_multi_str(redis, "NAV_ROAD_CURRENT_ID", vect_redis_str);
    if(vect_redis_str[1].compare(new_road) == 0)
    {
        return false;
    }
    return true;
}