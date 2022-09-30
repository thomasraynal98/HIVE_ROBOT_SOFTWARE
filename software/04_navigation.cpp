#include "00_navigation.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

int main(int argc, char *argv[])
{
    set_redis_var(&redis, "NAV_HMR_MAP_UPDATE", "TRUE");

    std::vector<Data_node> vect_node;
    std::vector<Data_road> vect_road;

    double ms_for_loop = frequency_to_ms(10);
    auto next = std::chrono::high_resolution_clock::now();

    std::string motor_command_str = "0000000000000|0.0|0.0|0.0|0.0|0.0|0.0|";
    double curr_max_speed = 0.0;

    std::vector<std::string> vect_cmd_ctr;
    set_redis_var(&redis, "EVENT_MANUAL_CONTROLER_DATA", "0000000000000|0.0|0.0|0.0|");

    Robot_position curr_position = Robot_position();
    set_redis_var(&redis, "NAV_GLOBAL_POSITION", "0000000000000|0.0|0.0|0.0|");
    set_redis_var(&redis, "NAV_LOCAL_POSITION", "0000000000000|0.0|0.0|0.0|");
    // std::vector<Navigation_road> vect_navigation;

    std::vector<Path_node> graph;
    std::vector<Data_road*> vect_brut_road;
    std::vector<Roadmap_node> vect_roadmap;

    int64_t timesptamp_cam1 = 0;
    int64_t timesptamp_cam2 = 0;
    int64_t timesptamp_lid1 = 0;
    int64_t timesptamp_lid2 = 0;

    std::vector<Object_env> vect_obj;
    std::vector<Sensor_prm> vect_sensor_prm;

    update_sensor_prm(&redis, vect_sensor_prm);

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        //==============================================
        // HMR : Check si la map est bonne et télécharge.
        //==============================================

        if(get_redis_str(&redis, "NAV_HMR_MAP_UPDATE").compare("TRUE") == 0)
        {
            if(get_redis_str(&redis, "NAV_HMR_DOWNLOAD_ADRESS").compare("NO_VAL") == 0)
            {
                //!\\ DOWLOAD new map 
            }

            try
            {
                vect_node.clear();
                vect_road.clear();
                Read_TXT_file(get_redis_str(&redis, "NAV_HMR_LOCAL_PATH"), vect_node, vect_road);
                set_redis_var(&redis, "NAV_HMR_MAP_UPDATE", "FALSE");
                pub_redis_var(&redis, "EVENT", get_event_str(2, "LOAD_HMR", "SUCCESS"));
            }
            catch(...)
            {
                set_redis_var(&redis, "NAV_HMR_MAP_UPDATE", "TRUE");
                pub_redis_var(&redis, "EVENT", get_event_str(2, "LOAD_HMR", "FAIL"));
            }
        }

        //==============================================
        // LOCALISATION : Mettre à jour en lisant var.
        //==============================================
        
        curr_position.update_pos(&redis);

        std::string roadID_str = std::to_string(get_curr_timestamp()) + "|";
        roadID_str += std::to_string(get_road_ID_from_pos(&redis, vect_road, curr_position.point, vect_roadmap, 1)) + "|";
        set_redis_var(&redis, "NAV_ROAD_CURRENT_ID", roadID_str);

        //==============================================
        // GLOBAL PATH : Qu'importe le mode actuelle.
        //==============================================

        if(get_redis_str(&redis, "MISSION_UPDATE_GLOBAL_PATH").compare("TRUE") == 0)
        {
            std::vector<std::string> vect_str;
            get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str);
            if(std::stoi(vect_str[1]) != -1)
            {
                pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "START"));
                update_path_node(vect_node, vect_road, graph);

                int node_start_ID = get_node_ID_from_road(vect_road, std::stoi(vect_str[1]));

                std::vector<std::string> vect_str_2;
                get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION", vect_str_2);
                Geographic_point destination_pos = Geographic_point(std::stod(vect_str_2[1]), std::stod(vect_str_2[2]));
                int destination_road_ID = get_road_ID_from_pos(&redis, vect_road, &destination_pos, vect_roadmap, 2);

                std::string redis_str = std::to_string(get_curr_timestamp()) + "|" + std::to_string(destination_road_ID) + "|";
                set_redis_var(&redis, "NAV_AUTO_DESTINATION_ROAD_ID", redis_str);

                int node_endof_ID = get_node_ID_from_road(vect_road, destination_road_ID);

                if(compute_navigation_path(node_start_ID, node_endof_ID, graph, vect_road, vect_brut_road))
                {
                    process_final_roadmap(&redis, vect_brut_road, vect_road, vect_roadmap);

                    std::string global_path_str = "";
                    for(int i = vect_roadmap.size()-1; i >= 0; i--)
                    {
                        // global_path_str += std::to_string(vect_roadmap[i].node_start->node_ID) + "|";
                        global_path_str += std::to_string(vect_roadmap[i].road->road_ID) + "|";
                        // global_path_str += std::to_string(vect_roadmap[i].node_target->node_ID) + "|";
                        // global_path_str += std::to_string(vect_roadmap[i].dest_dist_m) + "m|";
                        // global_path_str += std::to_string(vect_roadmap[i].dest_time_s) + "s|";
                    }
                    // std::cout << global_path_str << std::endl;
                    // std::cout << vect_roadmap.size() << std::endl;
                    set_redis_var(&redis, "SIM_GLOBAL_PATH", global_path_str);

                    pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "SUCCESS"));
                    set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");
                    set_redis_var(&redis, "MISSION_AUTO_TYPE",   "GOTO");
                    set_redis_var(&redis, "MISSION_AUTO_STATE",  "IN_PROGRESS");
                }
                else
                {
                    set_redis_var(&redis, "SIM_GLOBAL_PATH", "");
                    pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "NO_PATH_POSSIBLE"));
                }
            }
            else
            {
                pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "NO_ROAD_ID"));
            }
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "FALSE");
        }

        //==============================================
        // ENV DATA : Lire les données de proximités.
        //==============================================
        if(true)
        {
            std::vector<std::string> vect_redis_str;
            get_redis_multi_str(&redis, "NAV_LOCAL_POSITION", vect_redis_str);

            if(!is_same_time(0, std::stoul(vect_redis_str[0])))
            {      
                std::vector<double> curr_local_pos;
                curr_local_pos.push_back(std::stod(vect_redis_str[0]));
                curr_local_pos.push_back(std::stod(vect_redis_str[1]));
                curr_local_pos.push_back(std::stod(vect_redis_str[2]));

                std::vector<std::string> vect_obj_brut;

                double min_dist     = std::stod(get_redis_str(&redis, "NAV_OBJ_MIN_DIST"));
                double max_dist     = std::stod(get_redis_str(&redis, "NAV_OBJ_MAX_DIST"));
                double min_space    = std::stod(get_redis_str(&redis, "NAV_OBJ_MIN_SPACE"));
                int min_observation = std::stoi(get_redis_str(&redis, "NAV_OBJ_MIN_OBSERVATION"));
                double clear_dist   = std::stod(get_redis_str(&redis, "NAV_OBJ_CLEARING_DIST"));
                int clear_time      = std::stod(get_redis_str(&redis, "NAV_OBJ_CLEARING_TIME_MS"));

                get_redis_multi_str(&redis, "ENV_CAM1_OBSTACLE", vect_redis_str);
                if(!is_same_time(timesptamp_cam1, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_cam1 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 4)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[0], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                    }
                }

                get_redis_multi_str(&redis, "ENV_CAM2_OBSTACLE", vect_redis_str);
                if(!is_same_time(timesptamp_cam2, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_cam2 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 4)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[1], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                    }
                }

                get_redis_multi_str(&redis, "ENV_LID1_OBSTACLE", vect_redis_str);
                if(!is_same_time(timesptamp_lid1, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_lid1 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 4)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[2], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                    }
                }

                get_redis_multi_str(&redis, "ENV_LID2_OBSTACLE", vect_redis_str);
                if(!is_same_time(timesptamp_lid2, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_lid2 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 4)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[3], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                    }
                }

                clear_obj_vect(curr_local_pos, vect_obj, clear_time, clear_dist);
            }
        }

        //==============================================
        // NAVIGATION :
        //==============================================

        if(get_redis_str(&redis, "MISSION_MOTOR_BRAKE").compare("FALSE") == 0)
        {
            //==========================================
            // Manuel mode nav.
            //==========================================
            if(get_redis_str(&redis, "ROBOT_MODE").compare("MANUAL") == 0)
            {
                if(get_redis_str(&redis, "MISSION_MANUAL_TYPE").compare("MANUAL_MOVE") == 0 && \
                get_redis_str(&redis, "MISSION_MANUAL_STATE").compare("IN_PROGRESS") == 0 && \
                manual_mode_available(&redis) == 10)
                {
                    get_redis_multi_str(&redis, "EVENT_MANUAL_CONTROLER_DATA", vect_cmd_ctr);

                    std::string flag_manual_mode = get_redis_str(&redis, "NAV_MANUAL_MODE");

                    if(flag_manual_mode.compare("STANDARD") == 0 || \
                    flag_manual_mode.compare("STANDARD_MAX") == 0 )
                    {
                        if(!time_is_over(get_curr_timestamp(), std::stoul(vect_cmd_ctr[0]), 1500))
                        {
                            curr_max_speed    = get_max_speed(&redis, "MANUAL", flag_manual_mode, vect_road);
                            motor_command_str = map_manual_command(&redis, std::stod(vect_cmd_ctr[1]), std::stod(vect_cmd_ctr[2]), std::stod(vect_cmd_ctr[3]), curr_max_speed);
                        }
                        else
                        {
                            set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                            pub_redis_var(&redis, "EVENT", get_event_str(2, "MISSION_MANUAL_MOVE", "STANDARD_MODE_OVER_TIME"));
                        }
                    }
                    else if(false)
                    {
                        //!\\ OTHER MODE.
                    }
                }
            }

            //==========================================
            // Autonomous mode nav.
            //==========================================
            if(compare_redis_var(&redis, "ROBOT_MODE", "AUTO"))
            {
                if(compare_redis_var(&redis, "MISSION_AUTO_TYPE", "GOTO") && \
                compare_redis_var(&redis, "MISSION_AUTO_STATE", "IN_PROGRESS") && \
                (auto_mode_available(&redis) == 10 || auto_mode_available(&redis) == 20))
                {
                    if(compare_redis_var(&redis, "NAV_AUTO_MODE", "SIMPLE"))
                    {
                        std::vector<std::string> vect_str;
                        get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str);
                        int curr_road_id = std::stoi(vect_str[1]);
                        get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION_ROAD_ID", vect_str);
                        int dest_road_id = std::stoi(vect_str[1]);
                        get_redis_multi_str(&redis, "NAV_AUTO_PROJECT_DESTINATION", vect_str);
                        Geographic_point dest = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));

                        if(curr_road_id == dest_road_id && \
                        get_angular_distance(curr_position.point, &dest) <= std::stod(get_redis_str(&redis, "NAV_AUTO_DESTINATION_CROSSING_M")))
                        {
                            set_redis_var(&redis, "SIM_GLOBAL_PATH", "");

                            set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                            set_redis_var(&redis, "MISSION_AUTO_STATE",  "COMPLETED");
                            pub_redis_var(&redis, "EVENT", get_event_str(2, "MISSION_AUTO_GOTO", "SUCCESS"));
                        }
                        else
                        {
                            // [?] Get node of the road.
                            Geographic_point* curr_start_node;
                            Geographic_point* curr_target_node;
                            Geographic_point* next_target_node;

                            // std::cout << vect_roadmap.size() << std::endl;
                            for(int i = 0; i < vect_roadmap.size(); i++)
                            {
                                if(vect_roadmap[i].road->road_ID == curr_road_id)
                                {
                                    // std::cout << "FOUND IT" << std::endl;
                                    curr_start_node  = vect_roadmap[i].node_start->point;
                                    curr_target_node = vect_roadmap[i].node_target->point;
                                    if(i != vect_roadmap.size()-1)
                                    {
                                        next_target_node = vect_roadmap[i+1].node_target->point;
                                    } else {next_target_node = curr_start_node; } // TODO: REMOVE
                                    break;
                                }
                            }

                            // [?] Project this point on polaire ref.

                            // std::cout << "CURRPOINT:" + std::to_string(curr_position.point->longitude) + " " + std::to_string(curr_position.point->latitude) << std::endl;//+ " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;
                            // std::cout << "START_NODE:" + std::to_string(curr_start_node->longitude) + " " + std::to_string(curr_start_node->latitude) << std::endl;//+ " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;

                            double d_ca = get_angular_distance(curr_position.point, curr_start_node);
                            double d_cb = get_angular_distance(curr_position.point, curr_target_node);

                            double a_ca = get_bearing(curr_position.point, curr_start_node);
                            double a_cb = get_bearing(curr_position.point, curr_target_node);

                            double xa   = d_ca * cos(deg_to_rad(a_ca));
                            double ya   = d_ca * sin(deg_to_rad(a_ca));

                            double xb   = d_cb * cos(deg_to_rad(a_cb));
                            double yb   = d_cb * sin(deg_to_rad(a_cb));

                            // std::cout << "DISTANCE:" + std::to_string(d_ca) + " " + std::to_string(d_cb) << std::endl;//+ " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;

                            // std::cout << std::to_string(xa) + " " + std::to_string(ya) + " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;

                            // [?] Estimate futur point [!] upgrade !
                            get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_str);
                            double robot_curr_speed = 0.0;
                            for(int i = 1; i < vect_str.size(); i++)
                            {
                                robot_curr_speed += std::stod(vect_str[i]);
                            }
                            robot_curr_speed = robot_curr_speed / 6;

                            double xf   = robot_curr_speed * cos(deg_to_rad(curr_position.g_hdg));
                            double yf   = robot_curr_speed * sin(deg_to_rad(curr_position.g_hdg));

                            // [SIM]
                            double distance_to_f = sqrt(pow(xf,2)+pow(yf,2));
                            Geographic_point new_position = get_new_position(curr_position.point, curr_position.g_hdg, distance_to_f);
                            set_redis_var(&redis, "SIM_AUTO_PT_FUTUR", std::to_string(new_position.longitude) + "|" + std::to_string(new_position.latitude) + "|");
                            // [ENDSIM]

                            // [?] Check if point futur is on the road, with projection point.
                            double A    = xf - xa;
                            double B    = yf - ya;
                            double C    = xb - xa;
                            double D    = yb - ya;

                            double dot    = A * C + B * D;
                            double len_sq = C * C + D * D;
                            double param  = -1;
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

                            // [SIM_AUTO_PROJECT_PT_FUTUR]
                            double distance_to_pf = sqrt(pow(XX,2)+pow(YY,2));
                            double bearing_to_pf  = rad_to_deg(2 * atan(YY / (XX + sqrt(pow(XX, 2) + pow(YY, 2)))));
                            Geographic_point new_position2 = get_new_position(curr_position.point, bearing_to_pf, distance_to_pf);
                            set_redis_var(&redis, "SIM_AUTO_PROJECT_PT_FUTUR", std::to_string(new_position2.longitude) + "|" + std::to_string(new_position2.latitude) + "|");
                            // [ENDSIM]

                            double dx             = xf - XX;
                            double dy             = yf - YY;
                            double dist_to_road_m = sqrt(pow(dx,2)+pow(dy,2));

                            double xt, yt;
                            double xs, ys;

                            // [?] Update seek point [xs,ys] (target point).
                            // if(dist_to_road_m >= std::stod(get_redis_str(&redis, "NAV_AUTO_ROAD_RADIUS")))
                            // {
                                double ddx = XX - xb;
                                double ddy = YY - yb;
                                double dist_proj_to_target = sqrt(pow(ddx,2)+pow(ddy,2));

                                if(dist_proj_to_target >= std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")))
                                {
                                    // TODO: Avancer de x mètres le point projeter sur road.
                                    double bearing_start_target = get_bearing(curr_start_node, curr_target_node);
                                    xt = XX + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * cos(deg_to_rad(bearing_start_target));
                                    yt = YY + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * sin(deg_to_rad(bearing_start_target));
                                }
                                else
                                {
                                    double bearing_start_target = get_bearing(curr_target_node, next_target_node);
                                    double road_extension = std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) - dist_proj_to_target;
                                    xt = xb + road_extension * cos(deg_to_rad(bearing_start_target));
                                    yt = yb + road_extension * sin(deg_to_rad(bearing_start_target));
                                }

                            //[NOTE] L'algo de base prend en compte la vélocité du robot pour choisir la prochaine cible. Ici non. Pour l'instant.
                            xs = xt ;//+ (-xf);
                            ys = yt ;//+ (-yf);
     

                            // [SIM_AUTO_POINT_TARGET]
                            double distance_to_pt = sqrt(pow(xs,2)+pow(ys,2));
                            double bearing_to_pt  = rad_to_deg(2 * atan(ys / (xs + sqrt(pow(xs, 2) + pow(ys, 2)))));
                            Geographic_point new_position3 = get_new_position(curr_position.point, bearing_to_pt, distance_to_pt);
                            set_redis_var(&redis, "SIM_AUTO_PT_TARGET", std::to_string(new_position3.longitude) + "|" + std::to_string(new_position3.latitude) + "|");
                            // [ENDSIM]   

                            // [?] Transform target point to motor command.
                            double perfect_motor_speed = sqrt(pow(xs,2)+pow(ys,2));
                            double curr_max_speed      = get_max_speed(&redis, "AUTO", "NO_MODE", vect_road);
                            double final_max_speed     = 0.0;
                            if(perfect_motor_speed > curr_max_speed)
                            {
                                final_max_speed = curr_max_speed;
                            }
                            else
                            {
                                final_max_speed = perfect_motor_speed;
                            }

                            // [?] final_angle : angle to go to target.
                            // [?] diff_angle  : diff bewteen curr angle and final_angle. [-pi,0,pi]
                            double final_angle = 2 * atan(ys / (xs + sqrt(pow(xs, 2) + pow(ys, 2))));
                            if(final_angle < 0) final_angle + 2 * M_PI;
                            final_angle = final_angle * 180 / M_PI;

                            double diff_angle = 0;

                            if(curr_position.g_hdg - final_angle > 0)
                            {
                                if(curr_position.g_hdg - final_angle > 180)
                                {
                                    // Va vers droite
                                    diff_angle = 360 - (curr_position.g_hdg - final_angle);
                                }
                                else
                                {
                                    // Va vers gauche
                                    diff_angle = -(curr_position.g_hdg - final_angle);
                                }
                            }
                            else
                            {
                                if(curr_position.g_hdg - final_angle < -180)
                                {
                                    // Va vers gauche
                                    diff_angle = -(360 - (final_angle - curr_position.g_hdg));
                                }
                                else
                                {   
                                    // Va vers droite
                                    diff_angle = final_angle - curr_position.g_hdg;
                                }   
                            }

                            double beta_angle    = 90 - abs(diff_angle);
                            double radius_circle = sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle));

                            //[!] Le threshold résoud le probleme de la rotation arriere.
                            double opt_treshold = 145;
                            if(diff_angle > opt_treshold)
                            {
                                //[!] Il faut faire demi tour vers la droite.
                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    motor_command_str += "0.5|0.2|0.5|-0.5|-0.2|-0.5|";
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "0.2|0.2|0.2|-0.2|-0.2|-0.2|";
                                }
                            }
                            if(diff_angle < -opt_treshold)
                            {
                                //[!] Il faut faire demi tour vers la gauche.
                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    motor_command_str += "-0.5|-0.2|-0.5|0.5|0.2|0.5|";
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "-0.2|-0.2|-0.2|0.2|0.2|0.2|";
                                }
                            }
                            if(diff_angle <= opt_treshold && diff_angle >= -opt_treshold)
                            {
                                //[?] Si l'angle est entre le threshold et la zone normal, on fait en sorte qu'une seul partie des
                                //[?] roues tournent.
                                if(abs(diff_angle) > 90) radius_circle = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2;

                                double rapport = 2 * M_PI * (radius_circle + std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2)  / final_max_speed;
                                double inter_motor_speed = 2 * M_PI * (radius_circle - std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2) / rapport;
                                if(diff_angle >= 0)
                                {
                                    motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                    for(int i = 0; i < 3; i++)
                                    {
                                        motor_command_str += std::to_string(final_max_speed) + "|";
                                    }
                                    for(int i = 3; i < 6; i++)
                                    {
                                        motor_command_str += std::to_string(inter_motor_speed) + "|";
                                    }
                                }
                                else
                                {
                                    motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                    for(int i = 0; i < 3; i++)
                                    {
                                        motor_command_str += std::to_string(inter_motor_speed) + "|";
                                    }
                                    for(int i = 3; i < 6; i++)
                                    {
                                        motor_command_str += std::to_string(final_max_speed) + "|";
                                    }
                                }
                                
                            }             
                        }
                    }
                    if(compare_redis_var(&redis, "NAV_AUTO_MODE", "OBSTACLE_AVOIDANCE_NIV1"))
                    {
                        /*
                            Description: Ce code prend en compte l'entrer de données de capteurs
                            d'environnement externe afin d'avoir une representation précise de l'environnement
                            direct du robot.
                        */

                        //==============================
                        // Projection des obj dans le ref local. [POUR VISUALISATION]
                        //==============================
                        // std::vector<Vect_2D> direct_map;

                        // for(int i = 0; i < vect_obj.size(); i++)
                        // {
                        //     double dist_pol = get_distance(curr_position.l_x, curr_position.l_y, vect_obj[i].pos->x, vect_obj[i].pos->y);
                        //     double angl_pol = rad_to_deg(2 * atan((curr_position.l_y-vect_obj[i].pos->y) / ((curr_position.l_x-vect_obj[i].pos->x) + dist_pol)));
                        //     direct_map.push_back(Vect_2D(dist_pol, angl_pol));
                        // }

                        Vect_2D target_kp = Vect_2D(0.0, 0.0); 
                        Vect_2D ICC_kp    = Vect_2D(0.0, 0.0);

                        //==============================
                        // STANDARD MODE.
                        //==============================

                        std::vector<std::string> vect_str;
                        get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str);
                        int curr_road_id = std::stoi(vect_str[1]);
                        get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION_ROAD_ID", vect_str);
                        int dest_road_id = std::stoi(vect_str[1]);
                        get_redis_multi_str(&redis, "NAV_AUTO_PROJECT_DESTINATION", vect_str);
                        Geographic_point dest = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));

                        if(curr_road_id == dest_road_id && \
                        get_angular_distance(curr_position.point, &dest) <= std::stod(get_redis_str(&redis, "NAV_AUTO_DESTINATION_CROSSING_M")))
                        {
                            set_redis_var(&redis, "SIM_GLOBAL_PATH", "");

                            set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                            set_redis_var(&redis, "MISSION_AUTO_STATE",  "COMPLETED");
                            pub_redis_var(&redis, "EVENT", get_event_str(2, "MISSION_AUTO_GOTO", "SUCCESS"));
                        }
                        else
                        {
                            // [?] Get node of the road.
                            Geographic_point* curr_start_node;
                            Geographic_point* curr_target_node;
                            Geographic_point* next_target_node;

                            // std::cout << vect_roadmap.size() << std::endl;
                            for(int i = 0; i < vect_roadmap.size(); i++)
                            {
                                if(vect_roadmap[i].road->road_ID == curr_road_id)
                                {
                                    // std::cout << "FOUND IT" << std::endl;
                                    curr_start_node  = vect_roadmap[i].node_start->point;
                                    curr_target_node = vect_roadmap[i].node_target->point;
                                    if(i != vect_roadmap.size()-1)
                                    {
                                        next_target_node = vect_roadmap[i+1].node_target->point;
                                    } else {next_target_node = curr_start_node; } // TODO: REMOVE
                                    break;
                                }
                            }

                            // [?] Project this point on polaire ref.

                            // std::cout << "CURRPOINT:" + std::to_string(curr_position.point->longitude) + " " + std::to_string(curr_position.point->latitude) << std::endl;//+ " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;
                            // std::cout << "START_NODE:" + std::to_string(curr_start_node->longitude) + " " + std::to_string(curr_start_node->latitude) << std::endl;//+ " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;

                            double d_ca = get_angular_distance(curr_position.point, curr_start_node);
                            double d_cb = get_angular_distance(curr_position.point, curr_target_node);

                            double a_ca = get_bearing(curr_position.point, curr_start_node);
                            double a_cb = get_bearing(curr_position.point, curr_target_node);

                            double xa   = d_ca * cos(deg_to_rad(a_ca));
                            double ya   = d_ca * sin(deg_to_rad(a_ca));

                            double xb   = d_cb * cos(deg_to_rad(a_cb));
                            double yb   = d_cb * sin(deg_to_rad(a_cb));

                            // std::cout << "DISTANCE:" + std::to_string(d_ca) + " " + std::to_string(d_cb) << std::endl;//+ " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;

                            // std::cout << std::to_string(xa) + " " + std::to_string(ya) + " " + std::to_string(xb) + " " + std::to_string(yb) << std::endl;

                            // [?] Estimate futur point [!] upgrade !
                            get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_str);
                            double robot_curr_speed = 0.0;
                            for(int i = 1; i < vect_str.size(); i++)
                            {
                                robot_curr_speed += std::stod(vect_str[i]);
                            }
                            robot_curr_speed = robot_curr_speed / 6;

                            double xf   = robot_curr_speed * cos(deg_to_rad(curr_position.g_hdg));
                            double yf   = robot_curr_speed * sin(deg_to_rad(curr_position.g_hdg));

                            // [SIM]
                            double distance_to_f = sqrt(pow(xf,2)+pow(yf,2));
                            Geographic_point new_position = get_new_position(curr_position.point, curr_position.g_hdg, distance_to_f);
                            set_redis_var(&redis, "SIM_AUTO_PT_FUTUR", std::to_string(new_position.longitude) + "|" + std::to_string(new_position.latitude) + "|");
                            // [ENDSIM]

                            // [?] Check if point futur is on the road, with projection point.
                            double A    = xf - xa;
                            double B    = yf - ya;
                            double C    = xb - xa;
                            double D    = yb - ya;

                            double dot    = A * C + B * D;
                            double len_sq = C * C + D * D;
                            double param  = -1;
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

                            // [SIM_AUTO_PROJECT_PT_FUTUR]
                            double distance_to_pf = sqrt(pow(XX,2)+pow(YY,2));
                            double bearing_to_pf  = rad_to_deg(2 * atan(YY / (XX + sqrt(pow(XX, 2) + pow(YY, 2)))));
                            Geographic_point new_position2 = get_new_position(curr_position.point, bearing_to_pf, distance_to_pf);
                            set_redis_var(&redis, "SIM_AUTO_PROJECT_PT_FUTUR", std::to_string(new_position2.longitude) + "|" + std::to_string(new_position2.latitude) + "|");
                            // [ENDSIM]

                            double dx             = xf - XX;
                            double dy             = yf - YY;
                            double dist_to_road_m = sqrt(pow(dx,2)+pow(dy,2));

                            double xt, yt;
                            double xs, ys;

                            // [?] Update seek point [xs,ys] (target point).
                            // if(dist_to_road_m >= std::stod(get_redis_str(&redis, "NAV_AUTO_ROAD_RADIUS")))
                            // {
                                double ddx = XX - xb;
                                double ddy = YY - yb;
                                double dist_proj_to_target = sqrt(pow(ddx,2)+pow(ddy,2));

                                if(dist_proj_to_target >= std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")))
                                {
                                    // TODO: Avancer de x mètres le point projeter sur road.
                                    double bearing_start_target = get_bearing(curr_start_node, curr_target_node);
                                    xt = XX + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * cos(deg_to_rad(bearing_start_target));
                                    yt = YY + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * sin(deg_to_rad(bearing_start_target));
                                }
                                else
                                {
                                    double bearing_start_target = get_bearing(curr_target_node, next_target_node);
                                    double road_extension = std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) - dist_proj_to_target;
                                    xt = xb + road_extension * cos(deg_to_rad(bearing_start_target));
                                    yt = yb + road_extension * sin(deg_to_rad(bearing_start_target));
                                }

                            //[NOTE] L'algo de base prend en compte la vélocité du robot pour choisir la prochaine cible. Ici non. Pour l'instant.
                            xs = xt ;//+ (-xf);
                            ys = yt ;//+ (-yf);
     

                            // [SIM_AUTO_POINT_TARGET]
                            double distance_to_pt = sqrt(pow(xs,2)+pow(ys,2));
                            double bearing_to_pt  = rad_to_deg(2 * atan(ys / (xs + sqrt(pow(xs, 2) + pow(ys, 2)))));
                            Geographic_point new_position3 = get_new_position(curr_position.point, bearing_to_pt, distance_to_pt);
                            set_redis_var(&redis, "SIM_AUTO_PT_TARGET", std::to_string(new_position3.longitude) + "|" + std::to_string(new_position3.latitude) + "|");
                            // [ENDSIM]   

                            // [?] Transform target point to motor command.
                            double perfect_motor_speed = sqrt(pow(xs,2)+pow(ys,2));
                            double curr_max_speed      = get_max_speed(&redis, "AUTO", "NO_MODE", vect_road);
                            double final_max_speed     = 0.0;
                            if(perfect_motor_speed > curr_max_speed)
                            {
                                final_max_speed = curr_max_speed;
                            }
                            else
                            {
                                final_max_speed = perfect_motor_speed;
                            }

                            // [?] final_angle : angle to go to target.
                            // [?] diff_angle  : diff bewteen curr angle and final_angle. [-pi,0,pi]
                            double final_angle = 2 * atan(ys / (xs + sqrt(pow(xs, 2) + pow(ys, 2))));
                            if(final_angle < 0) final_angle + 2 * M_PI;
                            final_angle = final_angle * 180 / M_PI;

                            double diff_angle = 0;

                            if(curr_position.g_hdg - final_angle > 0)
                            {
                                if(curr_position.g_hdg - final_angle > 180)
                                {
                                    // Va vers droite
                                    diff_angle = 360 - (curr_position.g_hdg - final_angle);
                                }
                                else
                                {
                                    // Va vers gauche
                                    diff_angle = -(curr_position.g_hdg - final_angle);
                                }
                            }
                            else
                            {
                                if(curr_position.g_hdg - final_angle < -180)
                                {
                                    // Va vers gaucheradius_circle- (final_angle - curr_position.g_hdg));
                                }
                                else
                                {   
                                    // Va vers droite
                                    diff_angle = final_angle - curr_position.g_hdg;
                                }   
                            }

                            //====================
                            // ROTATION SUR PLACE
                            //====================

                            double opt_treshold = 145;
                            if(diff_angle > opt_treshold)
                            {
                                //[!] Il faut faire demi tour vers la droite.
                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    motor_command_str += "0.5|0.2|0.5|-0.5|-0.2|-0.5|";
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "0.2|0.2|0.2|-0.2|-0.2|-0.2|";
                                }
                            }
                            if(diff_angle < -opt_treshold)
                            {
                                //[!] Il faut faire demi tour vers la gauche.
                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    motor_command_str += "-0.5|-0.2|-0.5|0.5|0.2|0.5|";
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "-0.2|-0.2|-0.2|0.2|0.2|0.2|";
                                }
                            }
                            if(diff_angle <= opt_treshold && diff_angle >= -opt_treshold)
                            {

                                // target_kp, ICC_kp;
                                target_kp.x = curr_position.l_x + distance_to_pt * cos(rad_to_deg(curr_position.l_hdg + diff_angle));
                                target_kp.y = curr_position.l_y + distance_to_pt * sin(rad_to_deg(curr_position.l_hdg + diff_angle));

                                bool trajectory_safe = false;
                                int trajectory_attempt = 0;
                                bool intern_search = true;

                                double beta_angle    = 90 - abs(diff_angle);
                                double radius_interne = sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle));
                                double radius_externe = sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle));

                                bool extern_switch = false;

                                double curr_radius;
                                double curr_side;

                                bool left_c = false;
                                bool right_c = false;

                                if(diff_angle > 0) right_c = true;
                                else
                                {
                                    left_c = true;
                                }

                                std::vector<std::string> vect_redis_str;
                                get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_redis_str);
                                double speed_motor_l = std::stod(vect_redis_str[2]);
                                double speed_motor_r = std::stod(vect_redis_str[5]);
                                long double speed_ms = (speed_motor_r + speed_motor_l) / 2;

                                while(trajectory_attempt = 60)
                                {   
                                    if(right_c)
                                    {
                                        if(intern_search)
                                        {
                                            radius_interne = radius_interne - (radius_interne/0.15);
                                            curr_radius = radius_interne;
                                            curr_side = 1.0;
                                        }
                                        else
                                        {
                                            if(extern_switch)
                                            {
                                                radius_externe = radius_externe - (radius_externe/0.15);
                                                curr_radius = radius_externe;
                                                curr_side = -1.0;
                                            }
                                            else
                                            {
                                                radius_externe = radius_externe + (radius_externe/0.15);
                                                curr_radius = radius_externe;
                                                curr_side = 1.0;
                                            }
                                            if(radius_externe > 20.0)
                                            {
                                                extern_switch = true;
                                                radius_externe = 20.0;
                                                curr_radius = radius_externe;
                                                curr_side = 1.0;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if(intern_search)
                                        {
                                            radius_interne = radius_interne - (radius_interne/0.15);
                                            curr_radius = radius_interne;
                                            curr_side = -1.0;
                                        }
                                        else
                                        {
                                            if(extern_switch)
                                            {
                                                radius_externe = radius_externe - (radius_externe/0.15);
                                                curr_radius = radius_externe;
                                                curr_side = 1.0;
                                            }
                                            else
                                            {
                                                radius_externe = radius_externe + (radius_externe/0.15);
                                                curr_radius = radius_externe;
                                                curr_side = -1.0;
                                            }
                                            if(radius_externe > 20.0)
                                            {
                                                extern_switch = true;
                                                radius_externe = 20.0;
                                                curr_radius = radius_externe;
                                                curr_side = -1.0;
                                            }
                                        }
                                    }

                                    intern_search = !intern_search;
                                    if(curr_side == 1.0)
                                    {
                                        ICC_kp.x    = curr_position.l_x + curr_radius * cos(rad_to_deg(curr_position.l_hdg-90));
                                        ICC_kp.y    = curr_position.l_y + curr_radius * sin(rad_to_deg(curr_position.l_hdg-90));
                                    }
                                    else
                                    {
                                        ICC_kp.x    = curr_position.l_x + curr_radius * cos(rad_to_deg(curr_position.l_hdg+90));
                                        ICC_kp.y    = curr_position.l_y + curr_radius * sin(rad_to_deg(curr_position.l_hdg+90));
                                    }

                                    double distance_m = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2 + std::stod(get_redis_str(&redis, "NAV_OBJ_SAFETY_DIST_M"));                            
                                    double center_m   = curr_radius;

                                    trajectory_safe = true;
                                    for(int i = 0; i < vect_obj.size(); i++)
                                    {
                                        double dist_to_ICC = get_distance(ICC_kp.x, ICC_kp.y, vect_obj[i].pos->x, vect_obj[i].pos->y);
                                        
                                        if(abs(dist_to_ICC - center_m) < distance_m)
                                        {
                                            if(get_distance(vect_obj[i].pos->x, vect_obj[i].pos->y, curr_position.l_x, curr_position.l_y) < speed_ms)
                                            {
                                                trajectory_safe = false;
                                                break;
                                            }
                                        }
                                    }

                                    if(trajectory_safe)
                                    {
                                        // if(abs(diff_angle) > 90) curr_radius = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/3;

                                        double rapport = 2 * M_PI * (curr_radius + std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2)  / final_max_speed;
                                        double inter_motor_speed = 2 * M_PI * (curr_radius - std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2) / rapport;
                                        if(curr_side == 1.0)
                                        {
                                            motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                            for(int i = 0; i < 3; i++)
                                            {
                                                motor_command_str += std::to_string(final_max_speed) + "|";
                                            }
                                            for(int i = 3; i < 6; i++)
                                            {
                                                motor_command_str += std::to_string(inter_motor_speed) + "|";
                                            }
                                        }
                                        else
                                        {
                                            motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                            for(int i = 0; i < 3; i++)
                                            {
                                                motor_command_str += std::to_string(inter_motor_speed) + "|";
                                            }
                                            for(int i = 3; i < 6; i++)
                                            {
                                                motor_command_str += std::to_string(final_max_speed) + "|";
                                            }
                                        }
                                    }       
                                }
                            }       
                        }
                    }
                }
            }
        }

        if(get_redis_str(&redis, "MISSION_MOTOR_BRAKE").compare("TRUE") == 0)
        {
            motor_command_str = std::to_string(get_curr_timestamp()) + "|0.0|0.0|0.0|0.0|0.0|0.0|";
        }

        //==============================================
        // PUBLISH RESULT : Envoyer à redis la reponse.
        //==============================================
        set_redis_var(&redis, "HARD_MOTOR_COMMAND", motor_command_str);

    }
}