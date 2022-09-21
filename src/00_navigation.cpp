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
    
    if(opt_str.compare("SIMPLE") == 0)
    {
        // HARD REMOVE
        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0) return 10;
        // HARD REMOVE

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 10;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 11;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   != 0) return 12;

    }
    if(opt_str.compare("STANDARD") == 0)
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

void Read_TXT_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector)
{ 
    vector_node.clear();
    road_vector.clear();

    std::ifstream file(path);
    std::string str; 

    std::string data_type;

    while (std::getline(file, str))
    {
        std::vector<std::string> vect_str;

        get_multi_str(str, vect_str);
        if(vect_str.size() == 1) data_type = vect_str[0];
        else
        {
            if(data_type.compare("NODE") == 0)
            {
                Geographic_point pt_temp = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));
                Data_node new_data(std::stoi(vect_str[0]), &pt_temp);
                vector_node.push_back(new_data);
            }
            
            if(data_type.compare("ROAD") == 0)
            {
                Data_node* tempo_save_A;
                Data_node* tempo_save_B;
                for(int i = 0; i < vector_node.size(); i++)
                {
                    if(vector_node[i].node_ID == std::stoi(vect_str[1]))
                    {
                        tempo_save_A = &vector_node[i];
                    }
                    if(vector_node[i].node_ID == std::stoi(vect_str[2]))
                    {
                        tempo_save_B = &vector_node[i];
                    }
                }

                Data_road new_road(std::stoi(vect_str[0]), tempo_save_A, tempo_save_B);

                if(std::stoi(vect_str[6]) == 1) new_road.available = true;
                else{new_road.available = false;}

                new_road.deg_to_A = std::stod(vect_str[3]);
                new_road.deg_to_B = std::stod(vect_str[4]);
                new_road.length = std::stod(vect_str[5]);
                new_road.max_speed = std::stod(vect_str[7]);
                road_vector.push_back(new_road);
            }
        }
    }
}

double get_max_speed(sw::redis::Redis* redis, std::string robot_mode, std::string mode_param, std::vector<Data_road>& road_vector)
{
    if(robot_mode.compare("MANUAL") == 0)
    {
        if(mode_param.compare("STANDARD")     == 0) return std::stod(get_redis_str(redis, "NAV_MAX_SPEED")) * 0.5;
        if(mode_param.compare("STANDARD_MAX") == 0) return std::stod(get_redis_str(redis, "NAV_MAX_SPEED")) * 0.9;
    }

    if(robot_mode.compare("AUTO") == 0)
    {
        if(compare_redis_var(redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
        {
            return 0.2;
        }
        if(compare_redis_var(redis, "ROBOT_INFO_MODEL", "MK4"))
        {
            std::vector<std::string> vect_str;
            get_redis_multi_str(redis, "NAV_ROAD_CURRENT_ID", vect_str);
            int curr_road_id = std::stoi(vect_str[1]);

            for(int i = 0; i < road_vector.size(); i++)
            {
                if(road_vector[i].road_ID == curr_road_id)
                {
                    return road_vector[i].max_speed;
                }
            }
        }
    }

    return 0.0;
}

int get_road_ID_from_pos(sw::redis::Redis* redis, std::vector<Data_road>& vect_road, Geographic_point* curr_pos)
{
    /*
        Description : Cette fonction va retourner le numéro d'ID de la route la plus
        proche du point géographique. Cependant la manière d'attribué une route va 
        dependre du mode dans lequel le robot se trouve.
    */

    //==============================================
    // MODE MANUEL
    //==============================================

    if(get_redis_str(redis, "ROBOT_MODE").compare("AUTO") != 0)
    {
        double min_dist_m = 99999000;
        double dist_m     = 99999000;
        int road_ID       = -1;

        for(int i = 0; i < vect_road.size(); i++)
        {
            dist_m = get_dist_from_pos_to_toad(vect_road[i].A->point, vect_road[i].B->point, curr_pos);

            if(min_dist_m >= dist_m)
            {
                road_ID = vect_road[i].road_ID;
                min_dist_m = dist_m;
            }
        }

        // [?] Nous avons quitter les routes officiel en mode manuel. 
        if(min_dist_m >= 15.0) 
        {
            std::vector<std::string> vect_redis_str;
            get_redis_multi_str(redis, "NAV_ROAD_CURRENT_ID", vect_redis_str);
            if(vect_redis_str[1].compare("-1") != 0)
            {
                pub_redis_var(redis, "EVENT", get_event_str(2, "MANUAL_NAV", "LEAVE_THE_MAP"));
            }
            return -1;
        }

        return road_ID;
    }

    //==============================================
    // MODE AUTOMATIQUE
    //==============================================

    if(get_redis_str(redis, "ROBOT_MODE").compare("AUTO") == 0)
    {
        double min_dist_m = 99999000;
        double dist_m     = 99999000;
        int road_ID       = -1;

        for(int i = 0; i < vect_road.size(); i++)
        {
            dist_m = get_dist_from_pos_to_toad(vect_road[i].A->point, vect_road[i].B->point, curr_pos);

            if(min_dist_m >= dist_m)
            {
                road_ID = vect_road[i].road_ID;
                min_dist_m = dist_m;
            }
        }
        return road_ID;
    }

    return -1;
}

double get_dist_from_pos_to_toad(Geographic_point* pointA, Geographic_point* pointB, Geographic_point* pointC)
{
    /*
        Description : Cette fonction est chargé de calculé la distance entre un point 
        géographique et une route en coordonnées GPS.
        [!] pointA et pointB sont les extrémités de la route et pointC le point GPS.
        [?] Les commentaires sont volontairement laissé pour faciliter la relecture.
    */

    /*
        La première étape et de prendre les extremiter de la route, ici point A et B et d'obtenir leur angle
        depuis le point C et leur distance.
    */

    double d_ca = get_angular_distance(pointC, pointA);
    double d_cb = get_angular_distance(pointC, pointB);

    double a_ca = get_bearing(pointC, pointA);
    double a_cb = get_bearing(pointC, pointB);

    /*
        On peux donc dire que l'on a les points A et B en coordonnées polaires par rapport au centre 0,0 ou
        se situe le point C. On va maintenant passé de ces coordonnées polaires à des coordonnées carthésiennes.
    */

    double xa = d_ca * cos(deg_to_rad(a_ca));
    double ya = d_ca * sin(deg_to_rad(a_ca));

    double xb = d_cb * cos(deg_to_rad(a_cb));
    double yb = d_cb * sin(deg_to_rad(a_cb));

    /*
        A partir d'ici on effectue la fonction classique pour trouver le point projeter sur le segment.
    */

    double A = 0 - xa;
    double B = 0 - ya;
    double C = xb - xa;
    double D = yb - ya;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;
    if(len_sq != 0) param = dot / len_sq;

    double XX, YY;

    if(param < 0) 
    {
        XX = xa;
        YY = ya;
    }
    else if(param > 1)
    {
        XX = xb;
        YY = yb;
    }
    else
    {
        XX = xa + param * C;
        YY = ya + param * D;
    }

    double dx = 0 - XX;
    double dy = 0 - YY;
    double dist_m = sqrt(pow(dx,2)+pow(dy,2));

    return dist_m;

    /*
        [?] Si besoin de cette data.
        Maintenant que l'on a les coordonnées dans l'espaces carthésien simplifiés il faut le repassé en polaire
        puis de polaire repassé en géographique.
        La première partie consiste à récuperer l'angle entre le point C et le point projeté (XX,YY).
        La deuxième partie (POUR LE DEBUG) consiste à définir le point P en coordonnées géographique.
    */

    // double new_angle = 2 * atan(YY/(XX+sqrt(pow(XX,2)+pow(YY,2))));
    // new_angle = new_angle*180/M_PI;
    
    // double lat_c_r  = deg_to_rad(pointC->latitude);
    // double long_c_r = deg_to_rad(pointC->longitude);

    // double lat_p  = asin(sin(lat_c_r)*cos(dist_m/6371000) + cos(lat_c_r)*sin(dist_m/6371000)*cos(deg_to_rad(new_angle)));
    // double long_p = long_c_r + atan2(sin(deg_to_rad(new_angle))*sin(dist_m/6371000)*cos(lat_c_r), cos(dist_m/6371000)-sin(lat_c_r)*sin(lat_p));

    // projec_pos->longitude = long_p;
    // projec_pos->latitude  = lat_p;

    // std::cout << std::setprecision(8);
    // // std::cout << "La distance est de " << dist_m << " (" << new_angle << "°) vers LONG:" << long_p*180/M_PI << " LAT:" << lat_p*180/M_PI << std::endl;
    // return dist_m;
}

double get_angular_distance(Geographic_point* pointA, Geographic_point* pointB)
{
    double lat1  = pointA->latitude;
    double long1 = pointA->longitude;
    double lat2  = pointB->latitude;
    double long2 = pointB->longitude;
    
    double R = 6371000;
    double r1 = lat1 * M_PI / 180;
    double r2 = lat2 * M_PI / 180;
    double dl = (lat2 - lat1) * M_PI/180;
    double dd = (long2 - long1) * M_PI/180;

    double a = sin(dl/2) * sin(dl/2) + cos(r1) * cos(r2) * sin(dd/2) * sin(dd/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    // [?] c represente la distance angulaire.
    // return c

    return R * c;
}

double get_bearing(Geographic_point* pointA, Geographic_point* pointB)
{
    double lat1  = deg_to_rad(pointA->latitude);
    double long1 = deg_to_rad(pointA->longitude);
    double lat2  = deg_to_rad(pointB->latitude);
    double long2 = deg_to_rad(pointB->longitude);

    double y = sin(long2 - long1) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2 - long1);
    double o = atan2(y, x);

    if(o*180/M_PI < 0.0)
    {
        return 360 + o*180/M_PI;
    }

    return o*180/M_PI;
}

long double deg_to_rad(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

int get_node_ID_from_road(std::vector<Data_road>& vect_road, int road_ID)
{
    for(auto road : vect_road)
    {
        if(road.road_ID == road_ID)
        {
            // [!] On retourne forcement l'index du node A.
            return road.A->node_ID;
        }
    }
    return -1;
}

void update_path_node(std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector, std::vector<Path_node>& graph)
{
    graph.clear();

    for(int idx_node = 0; idx_node < vector_node.size(); idx_node++)
    {
        Path_node new_path_node(vector_node[idx_node].node_ID, &vector_node[idx_node]);
        graph.push_back(new_path_node);
    }
    for(int idx_road = 0; idx_road < road_vector.size(); idx_road++)
    {
        if(road_vector[idx_road].available)
        {
            for(int idx_path = 0; idx_path < graph.size(); idx_path++)
            {
                if(graph[idx_path].index_node == road_vector[idx_road].A->node_ID)
                {
                    // 1 FOUND PATH NODE CONNECTION
                    for(int idx_path2 = 0; idx_path2 < graph.size(); idx_path2++)
                    {
                        if(graph[idx_path2].index_node == road_vector[idx_road].B->node_ID)
                        {
                            graph[idx_path].connection_data.push_back(&graph[idx_path2]);
                            graph[idx_path2].connection_data.push_back(&graph[idx_path]);
                            // 2 COMPUTE WEIGHT
                            graph[idx_path].connection_weight.push_back(compute_weight_road(&road_vector[idx_road]));
                            graph[idx_path2].connection_weight.push_back(compute_weight_road(&road_vector[idx_road]));
                        }
                    }
                }
            }
        }
    }
}

double compute_weight_road(Data_road* road)
{
    return road->length / road->max_speed; // en heure decimal.
}

bool compute_navigation_path(int idx_start, int idx_endof, std::vector<Path_node>& graph, std::vector<Data_road>& road_vector, std::vector<Data_road*>& path_road_vector)
{
    path_road_vector.clear();

    // GOOD INDEX.
    int detection = 0;
    for(int i = 0; i < graph.size(); i++)
    {
        if(graph[i].index_node == idx_start || graph[i].index_node == idx_endof) detection++;
    }
    if(detection < 2) return false;

    // INIT A* PARAM.
    for(int i = 0; i < graph.size(); i++)
    {
        graph[i].closet = false;
        graph[i].gscore = 99999;
        graph[i].fscore = 99999;
        graph[i].come_from = NULL;
    }

    std::priority_queue<TuplePath,std::vector<TuplePath>,std::greater<TuplePath>> openList;   

    // INITIALISATION START
    Path_node* start_node;
    Path_node* endof_node;

    for(int idx_graph = 0; idx_graph < graph.size(); idx_graph++)
    {
        if(graph[idx_graph].index_node == idx_start)
        {
            start_node = &graph[idx_graph];
            start_node->gscore = 0;
        }
        if(graph[idx_graph].index_node == idx_endof)
        {
            endof_node = &graph[idx_graph];
        }
    }
    
    TuplePath start_tuple(0.0, start_node);
    openList.emplace(start_tuple);

    while(!openList.empty())
    {
        TuplePath p = openList.top();
        std::get<1>(p)->closet = true;
        openList.pop();

        // POUR CHAQUE VOISIN
        for(int idx_voisin = 0; idx_voisin < std::get<1>(p)->connection_weight.size(); idx_voisin++)
        {
            // // VERIFIER SI ON EST A DESTINATION
            if(std::get<1>(p)->connection_data[idx_voisin]->index_node == endof_node->index_node)
            {
                std::get<1>(p)->connection_data[idx_voisin]->come_from = std::get<1>(p);
                Path_node* next_node = std::get<1>(p)->connection_data[idx_voisin];

                // double distance_km = 0;
                // double time_total_decimal = 0;
                while(next_node->come_from != NULL && next_node->index_node != idx_start)
                {
                    // FOUND ROAD BETWEEN.
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].A->node_ID == next_node->index_node && \
                        road_vector[i].B->node_ID == next_node->come_from->index_node)
                        {
                            // std::cout << " ROAD " << road_vector[i].road_ID << std::endl;
                            path_road_vector.push_back(&road_vector[i]);
                            // distance_km += road_vector[i].length;
                            // time_total_decimal += (road_vector[i].length/road_vector[i].max_speed);
                        }
                        if(road_vector[i].B->node_ID == next_node->index_node && \
                        road_vector[i].A->node_ID == next_node->come_from->index_node)
                        {
                            // std::cout << " ROAD " << road_vector[i].road_ID << std::endl;
                            path_road_vector.push_back(&road_vector[i]);
                            // distance_km += road_vector[i].length;
                            // time_total_decimal += (road_vector[i].length/road_vector[i].max_speed);
                        }
                    }

                    next_node = next_node->come_from;
                }
                // std::cout << std::setprecision(3);
                // std::cout << "DISTANCE TOTAL : " << distance_km << " KM (" << std::ceil(time_total_decimal*60) << "Min)" << std::endl;

                return true;
            }

            //  // CALCULER LE TENTATIVE_GSCORE = GSCORE[CURRENT] + DEPLACEMENT[CURRENT<>VOISIN]
            double tentative_gscode = std::get<1>(p)->gscore + std::get<1>(p)->connection_weight[idx_voisin];

            //  // IF TENTATIVE_GSCORE < GSCORE[VOISIN] //SETUP A 99999 DE BASE
            if(tentative_gscode < std::get<1>(p)->connection_data[idx_voisin]->gscore)
            {
                //  //  // VOISIN PARENT = CURRENT
                std::get<1>(p)->connection_data[idx_voisin]->come_from = std::get<1>(p);

                //  //  // GSCORE[VOISIN] = TENTATIVE_GSCORE
                std::get<1>(p)->connection_data[idx_voisin]->gscore = tentative_gscode;

                //  //  // FSCORE[VOISIN] = TENTATIVE_GSCORE + DISTANCE[VOISIN<>DESTINATION]
                // std::get<1>(p)->connection_data[idx_voisin]->fscore = tentative_gscode + compute_distance_to_end(*endof_node->n, *std::get<1>(p)->connection_data[idx_voisin]->n);
                std::get<1>(p)->connection_data[idx_voisin]->fscore = tentative_gscode + get_angular_distance(endof_node->n->point, std::get<1>(p)->connection_data[idx_voisin]->n->point)/1000;

                //  //  // IF VOISIN N'A PAS ENCORE ETAIT VISITER [CLOSET=FALSE]
                if(!std::get<1>(p)->connection_data[idx_voisin]->closet)
                {
                    //  //  //  // AJOUT VOISIN DANS OPENLIST
                    openList.emplace(std::get<1>(p)->connection_data[idx_voisin]->fscore, std::get<1>(p)->connection_data[idx_voisin]);
                }
            }
        }
    }

    return false;
}

void process_final_roadmap(sw::redis::Redis* redis, std::vector<Data_road*>& path_road_vector, std::vector<Data_road>& road_vector, std::vector<Roadmap_node>& vect_roadmap)
{
    /*
        Description : l'objectif de cette fonction va être de récuperer le chemin brut obtenue par
        l'algorythme de A* et le transformé en feuille de route navigable.
    */

    //==============================================
    // ROTATE VECTOR.
    //==============================================
    std::reverse(path_road_vector.begin(),path_road_vector.end());

    //==============================================
    // LES ROUTES MANQUANTES.
    //==============================================

    int curr_road_ID, dest_road_ID;

    std::vector<std::string> vect_str;
    get_redis_multi_str(redis, "NAV_ROAD_CURRENT_ID", vect_str);
    curr_road_ID = std::stoi(vect_str[1]); 
    get_redis_multi_str(redis, "NAV_AUTO_DESTINATION_ROAD_ID", vect_str);
    dest_road_ID = std::stoi(vect_str[1]); 

    if(curr_road_ID != path_road_vector[0]->road_ID)
    {
        for(int i = 0; i < road_vector.size(); i++)
        {
            if(curr_road_ID == road_vector[i].road_ID)
            {
                path_road_vector.insert(path_road_vector.begin(), &road_vector[i]);
                break;
            }
        }
    }

    if(dest_road_ID != path_road_vector[path_road_vector.size()-1]->road_ID)
    {
        for(int i = 0; i < road_vector.size(); i++)
        {
            if(dest_road_ID == road_vector[i].road_ID)
            {
                path_road_vector.push_back(&road_vector[i]);
                break;
            }
        }
    }
    
    //==============================================
    // TRANSFORMER LE BRUT EN NET (UTILISABLE)
    //==============================================

    for(int i = path_road_vector.size()-1; i >= 0; i--)
    {
        if(i == path_road_vector.size()-1)
        {
            get_redis_multi_str(redis, "NAV_AUTO_DESTINATION", vect_str);
            Geographic_point dest = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));

            Roadmap_node rm_node  = Roadmap_node();
            rm_node.road          = path_road_vector[i];

            std::vector<Data_node*> tempo_vect; 
            detect_connection(path_road_vector[i], path_road_vector[i-1], tempo_vect);

            rm_node.node_start    = tempo_vect[0];
            rm_node.node_target   = tempo_vect[1];

            rm_node.dest_dist_m   = get_angular_distance(rm_node.node_start->point , &dest);
            rm_node.dest_time_s   = get_time_to_travel_s(rm_node.dest_dist_m, rm_node.road->max_speed);

            vect_roadmap.push_back(rm_node);
        }
        if(i != path_road_vector.size()-1 && i != 0)
        {
            Roadmap_node rm_node  = Roadmap_node();
            rm_node.road          = path_road_vector[i];

            std::vector<Data_node*> tempo_vect; 
            detect_connection(path_road_vector[i], path_road_vector[i-1], tempo_vect);

            rm_node.node_start    = tempo_vect[0];
            rm_node.node_target   = tempo_vect[1];

            rm_node.dest_dist_m   = vect_roadmap[vect_roadmap.size()-1].dest_dist_m + get_angular_distance(rm_node.node_start->point , rm_node.node_target->point);
            rm_node.dest_time_s   = vect_roadmap[vect_roadmap.size()-1].dest_time_s + get_time_to_travel_s(rm_node.dest_dist_m, rm_node.road->max_speed);

            vect_roadmap.push_back(rm_node);
        }
        if(i == 0)
        {
            get_redis_multi_str(redis, "NAV_GLOBAL_POSITION", vect_str);
            Geographic_point curr = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));

            Roadmap_node rm_node  = Roadmap_node();
            rm_node.road          = path_road_vector[i];

            std::vector<Data_node*> tempo_vect; 
            detect_connection(path_road_vector[i], path_road_vector[i+1], tempo_vect);

            rm_node.node_start    = tempo_vect[1];
            rm_node.node_target   = tempo_vect[0];

            rm_node.dest_dist_m   = vect_roadmap[vect_roadmap.size()-1].dest_dist_m + get_angular_distance(rm_node.node_target->point , &curr);
            rm_node.dest_time_s   = vect_roadmap[vect_roadmap.size()-1].dest_time_s + get_time_to_travel_s(rm_node.dest_dist_m, rm_node.road->max_speed);

            vect_roadmap.push_back(rm_node);
        }
    }
}

bool detect_connection(Data_road* road1, Data_road* road2, std::vector<Data_node*>& tempo_vect)
{
    /*
        Description : Cette fonction permet de detecter le point qui relie la road1 à
        la road2.
    */

    tempo_vect.clear();

    if(road1->A->node_ID == road2->A->node_ID)
    {
        // Du coup on connais le sens de direction.
        tempo_vect.push_back(road1->A);
        tempo_vect.push_back(road1->B);
        return true;
    }
    if(road1->A->node_ID == road2->B->node_ID)
    {
        // Du coup on connais le sens de direction.
        tempo_vect.push_back(road1->A);
        tempo_vect.push_back(road1->B);
        return true;
    }
    if(road1->B->node_ID == road2->B->node_ID)
    {
        // Du coup on connais le sens de direction.
        tempo_vect.push_back(road1->B);
        tempo_vect.push_back(road1->A);
        return true;
    }
    if(road1->B->node_ID == road2->A->node_ID)
    {
        // Du coup on connais le sens de direction.
        tempo_vect.push_back(road1->B);
        tempo_vect.push_back(road1->A);
        return true;
    }
    return false;
}

int get_time_to_travel_s(double distance, double speed)
{
    // distance en mètre, speed en km/h, return en seconde. 
    return distance / (speed*1000/3600);
}

double get_distance(double xa, double ya, double xb, double yb)
{
    return sqrt(pow(xa-xb,2)+pow(ya-yb,2));
}