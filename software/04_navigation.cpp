#include "00_navigation.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

cv::Mat debug_directmap(200, 200, CV_8UC3, cv::Scalar(255, 255, 255));

int main(int argc, char *argv[])
{
    cv::namedWindow( "DEBUG_DIRECT", 4);

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
                curr_local_pos.push_back(std::stod(vect_redis_str[1]));
                curr_local_pos.push_back(std::stod(vect_redis_str[2]));
                curr_local_pos.push_back(std::stod(vect_redis_str[3]));

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

                // get_redis_multi_str(&redis, "ENV_CAM2_OBSTACLE", vect_redis_str);
                // if(!is_same_time(timesptamp_cam2, std::stoul(vect_redis_str[0])))
                // {
                //     timesptamp_cam2 = std::stoul(vect_redis_str[0]);
                //     for(int i = 1; i < vect_redis_str.size(); i += 4)
                //     {
                //         vect_obj_brut.clear();
                //         vect_obj_brut.push_back(vect_redis_str[i+0]);
                //         vect_obj_brut.push_back(vect_redis_str[i+1]);
                //         vect_obj_brut.push_back(vect_redis_str[i+2]);
                //         vect_obj_brut.push_back(vect_redis_str[i+3]);

                //         process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[1], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                //     }
                // }

                // get_redis_multi_str(&redis, "ENV_LID1_OBSTACLE", vect_redis_str);
                // if(!is_same_time(timesptamp_lid1, std::stoul(vect_redis_str[0])))
                // {
                //     timesptamp_lid1 = std::stoul(vect_redis_str[0]);
                //     for(int i = 1; i < vect_redis_str.size(); i += 4)
                //     {
                //         vect_obj_brut.clear();
                //         vect_obj_brut.push_back(vect_redis_str[i+0]);
                //         vect_obj_brut.push_back(vect_redis_str[i+1]);
                //         vect_obj_brut.push_back(vect_redis_str[i+2]);
                //         vect_obj_brut.push_back(vect_redis_str[i+3]);

                //         process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[2], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                //     }
                // }

                // get_redis_multi_str(&redis, "ENV_LID2_OBSTACLE", vect_redis_str);
                // if(!is_same_time(timesptamp_lid2, std::stoul(vect_redis_str[0])))
                // {
                //     timesptamp_lid2 = std::stoul(vect_redis_str[0]);
                //     for(int i = 1; i < vect_redis_str.size(); i += 4)
                //     {
                //         vect_obj_brut.clear();
                //         vect_obj_brut.push_back(vect_redis_str[i+0]);
                //         vect_obj_brut.push_back(vect_redis_str[i+1]);
                //         vect_obj_brut.push_back(vect_redis_str[i+2]);
                //         vect_obj_brut.push_back(vect_redis_str[i+3]);

                //         process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[3], &min_dist, &max_dist, vect_obj, &min_space, &min_observation);
                //     }
                // }
                
                // std::cout << "INPUT S > " << vect_obj.size() << std::endl;
                clear_obj_vect(curr_local_pos, vect_obj, clear_time, clear_dist);
                // std::cout << "INPUT E > " << vect_obj.size() << std::endl;
            }
        }

        bool simulation = true;
        std::string sim_obj_str;
        // std::cout << "SIZE DIRECT MAP : " << vect_obj.size() << std::endl;
        if(simulation)
        {
            for(auto obj : vect_obj)
            {
                // std::cout << "DIRECT MAP POINT : " << obj.pos->x << " " << obj.pos->y << " " << obj.available << std::endl;
                sim_obj_str += std::to_string(obj.pos->x) + "|" + std::to_string(obj.pos->y) + "|";

                // double dist = sqrt(pow(curr_position.l_x - obj.pos->x,2) + pow(curr_position.l_y - obj.pos->y,2));
                // double x = obj.pos->x - curr_position.l_x;
                // double y = obj.pos->y - curr_position.l_y;
                // double angle = rad_to_deg(2 * atan(y / (x + dist))) + 360;
                // if(angle > 360) angle -= 360;
                // double angle_diff2;

                // if(curr_position.l_hdg - angle > 0)
                // {
                //     if(curr_position.l_hdg - angle > 180)
                //     {
                //         // Va vers droite
                //         angle_diff2 = 360 - (curr_position.l_hdg - angle);
                //     }
                //     else
                //     {
                //         // Va vers gauche
                //         angle_diff2 = -(curr_position.l_hdg - angle);
                //     }
                // }
                // else
                // {
                //     if(curr_position.l_hdg - angle < -180)
                //     {
                //         // Va vers gauche
                //         angle_diff2 = -(360- (angle - curr_position.l_hdg));
                //     }
                //     else
                //     {   
                //         // Va vers droite
                //         angle_diff2 = angle - curr_position.l_hdg;
                //     }   
                // }
                // std::cout << angle << " " << angle_diff2 << std::endl;
            }
            set_redis_var(&redis, "SIM_DIRECT_MAP", sim_obj_str);
        }

        //==============================================
        // NAVIGATION :
        //==============================================

        bool debug_cote = false;
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

                            // [SIM_CIRCLE]
                            Geographic_point center_circle = Geographic_point(0.0,0.0);
                            if(diff_angle < 0)
                            {
                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg - 90, radius_circle);
                            }
                            else
                            {
                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg + 90, radius_circle);
                            }
                            set_redis_var(&redis, "SIM_AUTO_PT_ICC", std::to_string(center_circle.longitude) + "|" + std::to_string(center_circle.latitude) + "|");
                            set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC", std::to_string(radius_circle));
                            // [SIM_END]

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
                        // std::cout << "TEST NEW" << std::endl;

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
                                    // Va vers gauche
                                    diff_angle = -(360- (final_angle - curr_position.g_hdg));
                                }
                                else
                                {   
                                    // Va vers droite
                                    diff_angle = final_angle - curr_position.g_hdg;
                                }   
                            }

                            // [SIM_CIRCLE]
                            double beta_angle    = 90 - abs(diff_angle);
                            double radius_circle = sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle));
                            Geographic_point center_circle = Geographic_point(0.0,0.0);
                            if(diff_angle < 0)
                            {
                                debug_cote = false;
                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg - 90, radius_circle);
                            }
                            else
                            {
                                debug_cote = true;
                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg + 90, radius_circle);
                            }
                            set_redis_var(&redis, "SIM_AUTO_PT_ICC", std::to_string(center_circle.longitude) + "|" + std::to_string(center_circle.latitude) + "|");
                            set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC", std::to_string(radius_circle));
                            // [SIM_END]


                            // std::cout << "NEW FRAME" << std::endl;
                            //====================
                            // ROTATION SUR PLACE
                            //====================
                            double opt_treshold = 130;
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

                                //==========================================
                                // OBSTACLE ALGO NIV 1 : 
                                //==========================================

                                bool trajectory_safe = false;
                                int trajectory_attempt = 0;

                                double beta_angle    = 90 - abs(diff_angle);

                                double radius_left = sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle));
                                double radius_right = sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle));
                                if(radius_left > 100) radius_left = 100;
                                if(radius_right > 100) radius_right = 100;

                                double curr_radius;
                                double curr_side;

                                bool left_c  = false;
                                bool right_c = false;

                                bool left_search   = true;
                                bool intern_switch = false;

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

                                double increment;

                                double min_radius = 1.0;
                                double switch_radius = 35.0;

                                while(trajectory_attempt < 1000)
                                {   
                                    bool radius_out = false;

                                    if(right_c)
                                    {
                                        if(left_search)
                                        {
                                            // Quand on commence a checher a droite, et que la on veut aller vers la gauche alors il faut augmenter la taille
                                            // du rayon.

                                            // variable = (condition) ? expressionTrue : expressionFalse
                                            // radius_left = (!intern_switch) ? radius_left + (radius_left*0.1) : radius_left - (radius_left*0.1);

                                            if(!intern_switch) 
                                            {
                                                radius_left += (radius_left*0.05);
                                                curr_side = 1.0;
                                            }
                                            else
                                            {
                                                radius_left -= (radius_left*0.01);
                                                curr_side = -1.0;
                                            }

                                            if(radius_left > switch_radius)
                                            {
                                                // Cela veut dire qu'on doit changer de coté.
                                                intern_switch = true;
                                            }

                                            if(radius_left < min_radius)
                                            {
                                                radius_out = true;
                                            }

                                            curr_radius = radius_left;
                                        }
                                        else
                                        {
                                            radius_right -= (radius_right*0.01);
                                            curr_side = 1.0;

                                            if(radius_right < min_radius)
                                            {
                                                radius_out = true;
                                            }

                                            curr_radius = radius_right;
                                        }
                                    }
                                    else
                                    {
                                        if(!left_search)
                                        {
                                            // Quand on commence a checher a droite, et que la on veut aller vers la gauche alors il faut augmenter la taille
                                            // du rayon.

                                            // variable = (condition) ? expressionTrue : expressionFalse
                                            // radius_left = (!intern_switch) ? radius_left + (radius_left*0.1) : radius_left - (radius_left*0.1);

                                            if(!intern_switch) 
                                            {
                                                radius_right += (radius_right*0.05);
                                                curr_side = -1.0;
                                            }
                                            else
                                            {
                                                radius_right -= (radius_right*0.01);
                                                curr_side = 1.0;
                                            }

                                            if(radius_right > switch_radius)
                                            {
                                                // Cela veut dire qu'on doit changer de coté.
                                                intern_switch = true;
                                            }

                                            if(radius_right < min_radius)
                                            {
                                                radius_out = true;
                                            }

                                            curr_radius = radius_right;
                                        }
                                        else
                                        {
                                            radius_left -= (radius_left*0.01);
                                            curr_side = -1.0;

                                            if(radius_left < min_radius)
                                            {
                                                radius_out = true;
                                            }

                                            curr_radius = radius_left;
                                        }
                                    }

                                    left_search = !left_search;

                                    if(!radius_out)
                                    {

                                        double final_max_speed_with_obstacle = final_max_speed;

                                        // if(curr_side == -1.0)
                                        // {
                                        //     ICC_kp.x    = curr_position.l_x + curr_radius * cos(rad_to_deg(curr_position.l_hdg-90));
                                        //     ICC_kp.y    = curr_position.l_y + curr_radius * sin(rad_to_deg(curr_position.l_hdg-90));
                                        // }
                                        // else
                                        // {
                                        //     ICC_kp.x    = curr_position.l_x + curr_radius * cos(rad_to_deg(curr_position.l_hdg+90));
                                        //     ICC_kp.y    = curr_position.l_y + curr_radius * sin(rad_to_deg(curr_position.l_hdg+90));
                                        // }

                                        // distance_m : represente le threshold en partant du rayon du cercle dans lequel aucun obstacle doit si trouver.
                                        double distance_m = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2 + std::stod(get_redis_str(&redis, "NAV_OBJ_SAFETY_DIST_M"));  
                                        // center_m   : represente le rayon actuelle de recherche de la nouvelle trajectoire.                          
                                        double center_m   = curr_radius;


                                        //====================================
                                        // RECHERCHE SI Y A UN OBJ SUR TRAJECTORY
                                        //====================================
                                        double row_idx;
                                        if(curr_side == -1.0) row_idx = 100 - center_m * 10;
                                        else{ row_idx = 100 + center_m * 10;}
                                        
                                        // double de;
                                        // if(center_m-distance_m < 0 )de = 0.2;
                                        // else { de = center_m-distance_m;}

                                        trajectory_safe = true;

                                        double min_speed = final_max_speed_with_obstacle;

                                        std::vector<double> distvect;

                                        for(auto obj : vect_obj)
                                        {
                                            double dist = sqrt(pow(curr_position.l_x - obj.pos->x,2) + pow(curr_position.l_y - obj.pos->y,2));
                                            double x = obj.pos->x - curr_position.l_x;
                                            double y = obj.pos->y - curr_position.l_y;
                                            double angle = rad_to_deg(2 * atan(y / (x + dist))) + 360;
                                            if(angle > 360) angle -= 360;
                                            double angle_diff2;

                                            if(curr_position.l_hdg - angle > 0)
                                            {
                                                if(curr_position.l_hdg - angle > 180)
                                                {
                                                    // Va vers droite
                                                    angle_diff2 = 360 - (curr_position.l_hdg - angle);
                                                }
                                                else
                                                {
                                                    // Va vers gauche
                                                    angle_diff2 = -(curr_position.l_hdg - angle);
                                                }
                                            }
                                            else
                                            {
                                                if(curr_position.l_hdg - angle < -180)
                                                {
                                                    // Va vers gauche
                                                    angle_diff2 = -(360- (angle - curr_position.l_hdg));
                                                }
                                                else
                                                {   
                                                    // Va vers droite
                                                    angle_diff2 = angle - curr_position.l_hdg;
                                                }   
                                            }



                                            double idx_col = 100 + dist*10*cos(deg_to_rad(angle_diff2));
                                            double idx_row = 100 + dist*10*sin(deg_to_rad(angle_diff2));

                                            double local_speed = final_max_speed_with_obstacle;
                                            if((dist < 2.0 && dist >= 1.5) && abs(angle_diff2) < 80) local_speed = final_max_speed_with_obstacle * 0.8;
                                            if((dist < 1.5 && dist >= 0.6) && abs(angle_diff2) < 80) local_speed = final_max_speed_with_obstacle * 0.5;
                                            if((dist < 0.6) && abs(angle_diff2) < 80) local_speed = final_max_speed_with_obstacle * 0.25;
                                            if(local_speed < min_speed) min_speed = local_speed;

                                            if(abs(sqrt(pow(100-idx_col,2)+pow(row_idx-idx_row,2))/10 - curr_radius) < distance_m && abs(angle_diff2) < 90)
                                            {
                                                if(dist < final_max_speed_with_obstacle*2.0)
                                                {
                                                    trajectory_safe = false;
                                                    break;
                                                }
                                            }

                                        }

                                        if(trajectory_safe)
                                        {
                                            trajectory_attempt = 99999;

                                            std::cout << "MAX SPEED IS " << min_speed << " ";
                                            double rapport = 2 * M_PI * (curr_radius + std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2)  / min_speed;
                                            double inter_motor_speed = 2 * M_PI * (curr_radius - std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2) / rapport;
                                            if(curr_side == 1.0)
                                            {
                                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                                for(int i = 0; i < 3; i++)
                                                {
                                                    motor_command_str += std::to_string(min_speed) + "|";
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
                                                    motor_command_str += std::to_string(min_speed) + "|";
                                                }
                                            }

                                            std::cout << motor_command_str << std::endl;
                                            
                                            // [SIM_CIRCLE NEW]
                                            double radius_circle = curr_radius;
                                            Geographic_point center_circle = Geographic_point(0.0,0.0);
                                            if(curr_side < 0)
                                            {
                                                debug_cote = false;
                                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg - 90, radius_circle);
                                            }
                                            else
                                            {
                                                debug_cote = true;
                                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg + 90, radius_circle);
                                            }
                                            set_redis_var(&redis, "SIM_AUTO_PT_ICC_NEW", std::to_string(center_circle.longitude) + "|" + std::to_string(center_circle.latitude) + "|");
                                            set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC_NEW", std::to_string(radius_circle));
                                            // [END SIM_CIRCLE NEW]
                                            break;
                                        } 
                                        else
                                        {
                                            trajectory_attempt++;
                                        }
                                    }
                                    else
                                    {
                                        trajectory_attempt++;
                                    }    
                                }
                                if(!trajectory_safe)
                                {std::cout << "NO TRAJECTORY FOUND. " << get_curr_timestamp() << " " << sqrt(pow(xs/2,2)+pow(ys/2,2))/cos(deg_to_rad(beta_angle)) << std::endl; set_redis_var(&redis, "SIM_AUTO_PT_ICC_NEW", "0.0|0.0|"); set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC_NEW", "0");}
                            }       
                        }
                    }

                }
            }
        }

        if(true)
        {
            cv::Mat copy = debug_directmap.clone();


            // ADD CIRCLE
            std::vector<std::string> vect_str_redis;
            get_redis_multi_str(&redis, "SIM_AUTO_PT_ICC_NEW", vect_str_redis);
            Geographic_point pt_circle = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));
            int radius = std::stoi(get_redis_str(&redis, "SIM_AUTO_RADIUS_ICC_NEW"));

            double row_idx;
            if(debug_cote) row_idx = 100 + radius * 10;
            else{ row_idx = 100 - radius * 10;}

            double distance_m = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2 + std::stod(get_redis_str(&redis, "NAV_OBJ_SAFETY_DIST_M"));
            
            cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)(radius*10), cv::Scalar(178, 102, 255), 1, cv::LineTypes::LINE_8);
            cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)((distance_m+radius)*10), cv::Scalar(178, 102/2, 255/2), 1, cv::LineTypes::LINE_8);

            double de;
            if(radius-distance_m < 0 )de = 0.2;
            else { de = radius-distance_m;}
            cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)((de)*10), cv::Scalar(178, 102/2, 255/2), 1, cv::LineTypes::LINE_8);

            // NEW CIRCLE
            // if(curr_side) row_idx = 100 - radius_circle * 10;
            // else{ row_idx = 100 + radius_circle * 10;}
            // cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)(radius_circle*10), cv::Scalar(255, 153, 31), 1, cv::LineTypes::LINE_8);
            // cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)((radius_circle+radius)*10), cv::Scalar(255, 153/2, 31), 1, cv::LineTypes::LINE_8);
            // if(radius_circle-distance_m < 0 )de = 0.2;
            // else { de = radius_circle-distance_m;}
            // cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)((de)*10), cv::Scalar(255, 153/2, 31), 1, cv::LineTypes::LINE_8);

            // ADD LOCAL POINT
            for(auto obj : vect_obj)
            {
                double dist = sqrt(pow(curr_position.l_x - obj.pos->x,2) + pow(curr_position.l_y - obj.pos->y,2));
                double x = obj.pos->x - curr_position.l_x;
                double y = obj.pos->y - curr_position.l_y;
                double angle = rad_to_deg(2 * atan(y / (x + dist))) + 360;
                if(angle > 360) angle -= 360;
                double angle_diff2;

                if(curr_position.l_hdg - angle > 0)
                {
                    if(curr_position.l_hdg - angle > 180)
                    {
                        // Va vers droite
                        angle_diff2 = 360 - (curr_position.l_hdg - angle);
                    }
                    else
                    {
                        // Va vers gauche
                        angle_diff2 = -(curr_position.l_hdg - angle);
                    }
                }
                else
                {
                    if(curr_position.l_hdg - angle < -180)
                    {
                        // Va vers gauche
                        angle_diff2 = -(360- (angle - curr_position.l_hdg));
                    }
                    else
                    {   
                        // Va vers droite
                        angle_diff2 = angle - curr_position.l_hdg;
                    }   
                }



                double idx_col = 100 + dist*10*cos(deg_to_rad(angle_diff2));
                double idx_row = 100 + dist*10*sin(deg_to_rad(angle_diff2));

                if(abs(sqrt(pow(100-idx_col,2)+pow(row_idx-idx_row,2))/10 - radius) < distance_m && abs(angle_diff2) < 80)
                {
                    // std::cout << "OBSTACLE" << std::endl;
                }

                std::vector<std::string> vect_redis_str;
                get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_redis_str);
                double speed_motor_l = std::stod(vect_redis_str[2]);
                double speed_motor_r = std::stod(vect_redis_str[5]);
                long double speed_ms = (speed_motor_r + speed_motor_l) / 2;

                if(dist < speed_ms*2)
                {
                    cv::circle(copy, cv::Point((int)(idx_col),(int)(idx_row)),0, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                }
                else
                {
                    cv::circle(copy, cv::Point((int)(idx_col),(int)(idx_row)),0, cv::Scalar(0,200,0), cv::FILLED, 1,0);
                }
            }
            // ADD ROBOT
            cv::circle(copy, cv::Point((int)(100),(int)(100)),2, cv::Scalar(0,0,0), cv::FILLED, 1,0);

            cv::imshow("DEBUG_DIRECT", copy);
            char d =(char)cv::waitKey(25);
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