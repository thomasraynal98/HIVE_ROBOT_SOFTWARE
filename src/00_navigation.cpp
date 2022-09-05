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

    std::string opt_str = get_redis_str(redis, "MISSION_PARAM_AUTO_ENABLE");
    
    if(opt_str.compare("AUTO_SIMPLE"))
    {
        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 10;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 11;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   != 0) return 12;

    }
    if(opt_str.compare("AUTO_STANDARD"))
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

    if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0) return 10;
    return 11;
}

std::string map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle, double max_speed_Ms)
{
    //TODO: Cette fonction va mapper les commandes brutes de la manelle en commande 
    //TODO: utilisable par le robot.

    std::string command_motor_str = std::to_string(get_curr_timestamp()) + "|";

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

        if(angle > 0   && angle <= 90)
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