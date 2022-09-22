#include "00_navigation.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

int main(int argc, char *argv[])
{
    set_redis_var(&redis, "NAV_HMR_MAP_UPDATE", "TRUE");

    std::vector<Data_node> vect_node;
    std::vector<Data_road> vect_road;

    double ms_for_loop = frequency_to_ms(2);
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
        roadID_str += std::to_string(get_road_ID_from_pos(&redis, vect_road, curr_position.point)) + "|";
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
                int destination_road_ID = get_road_ID_from_pos(&redis, vect_road, &destination_pos);
                
                std::string redis_str = std::to_string(get_curr_timestamp()) + "|" + std::to_string(destination_road_ID) + "|";
                set_redis_var(&redis, "NAV_AUTO_DESTINATION_ROAD_ID", redis_str);

                int node_endof_ID = get_node_ID_from_road(vect_road, destination_road_ID);

                if(compute_navigation_path(node_start_ID, node_endof_ID, graph, vect_road, vect_brut_road))
                {
                    // std::string global_path_str = "";
                    // for(auto p_road : vect_brut_road)
                    // {
                    //     global_path_str += std::to_string(p_road->road_ID) + "|";
                    // }
                    // std::cout << global_path_str << std::endl;

                    process_final_roadmap(&redis, vect_brut_road, vect_road, vect_roadmap);

                    // std::string global_path_str = "";
                    // for(int i = vect_roadmap.size()-1; i >= 0; i--)
                    // {
                    //     global_path_str += std::to_string(vect_roadmap[i].node_start->node_ID) + " > ";
                    //     global_path_str += std::to_string(vect_roadmap[i].road->road_ID) + " > ";
                    //     global_path_str += std::to_string(vect_roadmap[i].node_target->node_ID) + " > (";
                    //     global_path_str += std::to_string(vect_roadmap[i].dest_dist_to_road_m) + "m ~";
                    //     global_path_str += std::to_string(vect_roadmap[i].dest_time_s) + "s) > ";
                    // }
                    // std::cout << global_path_str << std::endl;

                    pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "SUCCESS"));
                    set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");
                    set_redis_var(&redis, "MISSION_AUTO_TYPE",   "WAITING");
                    set_redis_var(&redis, "MISSION_AUTO_STATE",  "IN_PROGRESS");
                }
                else
                {
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
                        get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION", vect_str);
                        Geographic_point dest = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));

                        if(curr_road_id == dest_road_id && \
                        get_angular_distance(curr_position.point, &dest) <= std::stod(get_redis_str(&redis, "NAV_AUTO_DESTINATION_CROSSING_M")))
                        {
                            set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                            set_redis_var(&redis, "MISSION_AUTO_STATE",  "COMPLETED");
                            pub_redis_var(&redis, "EVENT", get_event_str(2, "MISSION_AUTO_GOTO", "SUCCESS"));
                        }
                        else
                        {
                            // [?] Get node of the road.
                            Geographic_point* curr_start_node;
                            Geographic_point* curr_target_node;

                            for(int i = 0; i < vect_roadmap.size(); i++)
                            {
                                if(vect_roadmap[i].road->road_ID == curr_road_id)
                                {
                                    curr_start_node  = vect_roadmap[i].node_start->point;
                                    curr_target_node = vect_roadmap[i].node_target->point;
                                    break;
                                }
                            }

                            // [?] Project this point on polaire ref.
                            double d_ca = get_angular_distance(curr_position.point, curr_start_node);
                            double d_cb = get_angular_distance(curr_position.point, curr_target_node);

                            double a_ca = get_bearing(curr_position.point, curr_start_node);
                            double a_cb = get_bearing(curr_position.point, curr_target_node);

                            double xa   = d_ca * cos(deg_to_rad(a_ca));
                            double ya   = d_ca * sin(deg_to_rad(a_ca));

                            double xb   = d_cb * cos(deg_to_rad(a_cb));
                            double yb   = d_cb * sin(deg_to_rad(a_cb));

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

                            double dx             = xf - XX;
                            double dy             = yf - YY;
                            double dist_to_road_m = sqrt(pow(dx,2)+pow(dy,2));

                            double xt, yt;
                            double xs, ys;

                            // [?] Update seek point [xs,ys] (target point).
                            if(dist_to_road_m >= std::stod(get_redis_str(&redis, "NAV_AUTO_ROAD_RADIUS")))
                            {
                                double ddx = XX - xb;
                                double ddy = YY - yb;
                                double dist_proj_to_target = sqrt(pow(ddx,2)+pow(ddy,2));

                                if(dist_proj_to_target >= std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")))
                                {
                                    // TODO: Avancer de x mètres le point projeter sur road.
                                    double bearing_start_target = get_bearing(curr_start_node, curr_target_node);
                                    xt = xa + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * cos(deg_to_rad(bearing_start_target));
                                    yt = ya + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * sin(deg_to_rad(bearing_start_target));
                                }
                                else
                                {
                                    xt = xb;
                                    yt = yb;
                                }

                                xs = xt + (-xf);
                                ys = yt + (-yf);
                            }
                            else
                            {
                                xs = xf;
                                ys = yf;
                            }

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
                                    motor_command_str += "0.5|0.2|0.5|-0.5|-0.2|-0.5|"
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "0.2|0.2|0.2|-0.2|-0.2|-0.2|"
                                }
                            }
                            if(diff_angle < -opt_treshold)
                            {
                                //[!] Il faut faire demi tour vers la gauche.
                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    motor_command_str += "-0.5|-0.2|-0.5|0.5|0.2|0.5|"
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "-0.2|-0.2|-0.2|0.2|0.2|0.2|"
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