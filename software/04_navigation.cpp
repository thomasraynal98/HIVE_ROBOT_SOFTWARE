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

    // std::vector<Path_node> graph;
    // std::vector<Data_road*> vect_brut_road;

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

        // if(get_redis_str(&redis, "MISSION_UPDATE_GLOBAL_PATH").compare("TRUE") == 0)
        // {
        //     std::vector<std::string> vect_str;
        //     get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str);
        //     if(std::stoi(vect_str[1]) != 0)
        //     {
        //         pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "START"));
        //         std::cout << "DEB1" << std::endl;
        //         graph.clear();
        //         fill_path_node(vect_node, vect_road, graph);
        //         std::cout << "DEB2" << std::endl;
        //         int index_start = compute_index_start_node(&redis, vect_road);
        //         std::cout << "DEB20" << std::endl;
        //         int index_endof = compute_destination_node(&redis, vect_road);
        //         std::cout << "DEB21" << std::endl;
        //         std::cout << index_start << " " << index_endof << " " << std::endl;
        //         std::cout <<  graph.size() << " " << vect_road.size() << " " << std::endl;
        //         vect_brut_road.clear();
        //         vect_navigation.clear();
        //         std::cout << vect_brut_road.size() << std::endl;
        //         if(compute_navigation_path(index_start, index_endof, graph, vect_road, vect_brut_road))
        //         {
        //             std::cout << "DEB25" << std::endl;
        //             brut_navigation_to_navigable_route(vect_brut_road, vect_navigation);
        //             std::cout << "DEB3" << std::endl;
        //             destination_route_correction(&redis, vect_road, vect_navigation);
        //             std::cout << "DEB4" << std::endl;
        //             print_vect_navigation(vect_navigation);

        //             set_redis_var(&redis, "MISSION_MOTOR_BRAKE",    "FALSE");
        //             set_redis_var(&redis, "MISSION_AUTO_STATE",     "IN_PROGRESS");
        //             pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "COMPLETED"));
        //         }
        //         else
        //         {
        //             set_redis_var(&redis, "MISSION_AUTO_STATE",     "INTERRUPTED");
        //             pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "NO_SOLUTION"));
        //         }
        //     }
        //     else
        //     {
        //         pub_redis_var(&redis, "EVENT", get_event_str(2, "COMPUTE_GLOBAL_PATH", "FAIL_LOCALISATION"));
        //     }
        //     set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "FALSE");
        // }

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
                            curr_max_speed    = get_max_speed(&redis, "MANUAL", flag_manual_mode);
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
            if(get_redis_str(&redis, "ROBOT_MODE").compare("AUTO") == 0)
            {
                
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