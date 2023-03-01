#include "00_navigation.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

cv::Mat debug_directmap(200, 200, CV_8UC3, cv::Scalar(255, 255, 255));

int main(int argc, char *argv[])
{   
    set_redis_var(&redis, "SOFT_PROCESS_ID_NAV", std::to_string(getpid()));
    // SECURITY RESET
    set_redis_var(&redis, "ROBOT_MODE", "MANUAL");
    set_redis_var(&redis, "NAV_HMR_MAP_UPDATE", "TRUE");

    std::vector<Data_node> vect_node;
    std::vector<Data_road> vect_road;

    double ms_for_loop = frequency_to_ms(30);
    auto next = std::chrono::high_resolution_clock::now();

    std::string motor_command_str = "0000000000000|0.0|0.0|0.0|0.0|0.0|0.0|";
    double curr_max_speed = 0.0;

    std::vector<std::string> vect_cmd_ctr;
    set_redis_var(&redis, "EVENT_MANUAL_CONTROLER_DATA", "0000000000000|0.0|0.0|0.0|");

    Robot_position curr_position = Robot_position();

    std::vector<std::string> start_pos_msg;
    get_redis_multi_str(&redis, "ROBOT_INFO_HOME_POSITION", start_pos_msg);
    set_redis_var(&redis, "NAV_GLOBAL_POSITION", "0000000000000|" + start_pos_msg[0] + "|" + start_pos_msg[1] + "|140.0|");
    set_redis_var(&redis, "NAV_LOCAL_POSITION", "0000000000000|0.0|0.0|0.0|");
    // std::vector<Navigation_road> vect_navigation;

    std::vector<Path_node> graph;
    std::vector<Data_road*> vect_brut_road;
    std::vector<Roadmap_node> vect_roadmap;

    int64_t timesptamp_cam1 = 0;
    int64_t timesptamp_cam2 = 0;
    int64_t timesptamp_lid1 = 0;
    int64_t timesptamp_lid2 = 0;

    int64_t timesptamp_proj = 0;

    std::vector<Object_env> vect_obj;
    std::vector<Sensor_prm> vect_sensor_prm;

    update_sensor_prm(&redis, vect_sensor_prm);

    double deb_x = 0; 
    double deb_y = 0;
    double curr_side = 0;

    // [?] Cette variable setup ou non le OPENCV de debug.
    bool show_debug = true;
    if(argc == 2)
    {
        std::cout << "Mode Headless running." << std::endl;
        show_debug = false; 
    } 
    else
    {
        std::cout << "Mode Debug running." << std::endl;
        cv::namedWindow( "DEBUG_DIRECT", 4);
    }

    /**
     * NOTE: vect_traj
     * 
     * Ce vecteur permet de stocker l'ensemble des trajectoires automatiques
     * que le robot peux choisir de prendre.
     */ 


    /* New TRAJECTORY system ! */
    std::vector<Trajectory> vect_traj_80;
    std::vector<Trajectory> vect_traj_50;
    std::vector<Trajectory> vect_traj_25;
    std::vector<Trajectory> vect_traj_10;

    // vect_traj_80.push_back(Trajectory(   -4, 2.0, 1.40));
    vect_traj_80.push_back(Trajectory(   -6, 2.0, 1.20));
    vect_traj_80.push_back(Trajectory(   -8, 2.0, 1.20));
    vect_traj_80.push_back(Trajectory(  -10, 2.5, 0.80));
    vect_traj_80.push_back(Trajectory(  -15, 2.5, 0.80));
    vect_traj_80.push_back(Trajectory(  -20, 2.5, 0.80));
    vect_traj_80.push_back(Trajectory(  -30, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(  -40, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(  -60, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(  -80, 3.0, 0.30));
    vect_traj_80.push_back(Trajectory(  -80, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory( -100, 3.0, 0.30));
    vect_traj_80.push_back(Trajectory( -100, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(-1000, 3.0, 0.30));
    vect_traj_80.push_back(Trajectory(-1000, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory( 1000, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory( 1000, 3.0, 0.30));
    vect_traj_80.push_back(Trajectory(  100, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(  100, 3.0, 0.30));
    vect_traj_80.push_back(Trajectory(   80, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(   80, 3.0, 0.30));
    vect_traj_80.push_back(Trajectory(   60, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(   40, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(   30, 3.0, 0.80));
    vect_traj_80.push_back(Trajectory(   20, 2.5, 0.80));
    vect_traj_80.push_back(Trajectory(   15, 2.5, 0.80));
    vect_traj_80.push_back(Trajectory(   10, 2.5, 0.80));
    vect_traj_80.push_back(Trajectory(    8, 2.0, 1.20));
    vect_traj_80.push_back(Trajectory(    6, 2.0, 1.20));
    // vect_traj_80.push_back(Trajectory(    4, 2.0, 1.40));

    // vect_traj_50.push_back(Trajectory( -1.0, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory(-1.25, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory(-1.50, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory(-1.75, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory( -2.0, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory( -2.5, 1.0, 0.50));
    vect_traj_50.push_back(Trajectory(   -3, 1.0, 0.50));
    vect_traj_50.push_back(Trajectory(   -4, 2.0, 0.50));
    vect_traj_50.push_back(Trajectory(   -5, 2.0, 0.50));
    vect_traj_50.push_back(Trajectory(   -6, 2.5, 0.50));
    vect_traj_50.push_back(Trajectory(   -8, 2.0, 0.60));
    vect_traj_50.push_back(Trajectory(  -10, 2.5, 0.60));
    vect_traj_50.push_back(Trajectory(  -15, 2.5, 0.60));
    vect_traj_50.push_back(Trajectory(  -20, 2.5, 0.60));
    vect_traj_50.push_back(Trajectory(  -30, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(  -40, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(  -60, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(  -80, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(   80, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(   60, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(   40, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(   30, 3.0, 0.60));
    vect_traj_50.push_back(Trajectory(   20, 2.5, 0.60));
    vect_traj_50.push_back(Trajectory(   15, 2.5, 0.60));
    vect_traj_50.push_back(Trajectory(   10, 2.5, 0.60));
    vect_traj_50.push_back(Trajectory(    8, 2.0, 0.60));
    vect_traj_50.push_back(Trajectory(    6, 2.5, 0.50));
    vect_traj_50.push_back(Trajectory(    5, 2.0, 0.50));
    vect_traj_50.push_back(Trajectory(    4, 2.0, 0.50));
    vect_traj_50.push_back(Trajectory(    3, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory(  2.5, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory(  2.0, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory( 1.75, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory( 1.50, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory( 1.25, 1.0, 0.50));
    // vect_traj_50.push_back(Trajectory(  1.0, 1.0, 0.50));

    // vect_traj_25.push_back(Trajectory(-1.00, 1.0, 0.30));
    // vect_traj_25.push_back(Trajectory(-1.25, 1.0, 0.30));
    vect_traj_25.push_back(Trajectory(-1.50, 1.0, 0.30));
    vect_traj_25.push_back(Trajectory(-1.75, 1.0, 0.30));
    vect_traj_25.push_back(Trajectory(-2.00, 1.0, 0.30));
    vect_traj_25.push_back(Trajectory(-2.25, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory(-2.50, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory(-2.75, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory(-3.00, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory(-3.25, 1.5, 0.25));
    vect_traj_25.push_back(Trajectory( -3.5, 1.5, 0.25));
    vect_traj_25.push_back(Trajectory( -4.0, 2.0, 0.25));
    vect_traj_25.push_back(Trajectory( -4.5, 2.0, 0.25));
    vect_traj_25.push_back(Trajectory(   -5, 2.0, 0.25));
    vect_traj_25.push_back(Trajectory(   -6, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory(  -50, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory( -200, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory(-1000, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory( 1000, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory(  200, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory(   50, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory(    6, 2.5, 0.25));
    vect_traj_25.push_back(Trajectory(    5, 2.0, 0.25));
    vect_traj_25.push_back(Trajectory(  4.5, 2.0, 0.25));
    vect_traj_25.push_back(Trajectory(  4.0, 2.0, 0.25));
    vect_traj_25.push_back(Trajectory(  3.5, 1.5, 0.25));
    vect_traj_25.push_back(Trajectory( 3.25, 1.5, 0.25));
    vect_traj_25.push_back(Trajectory( 3.00, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory( 2.75, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory( 2.50, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory( 2.25, 1.0, 0.25));
    vect_traj_25.push_back(Trajectory( 2.00, 1.0, 0.30));
    vect_traj_25.push_back(Trajectory( 1.75, 1.0, 0.30));
    vect_traj_25.push_back(Trajectory( 1.50, 1.0, 0.30));
    // vect_traj_25.push_back(Trajectory( 1.25, 1.0, 0.30));
    // vect_traj_25.push_back(Trajectory( 1.00, 1.0, 0.30));

    vect_traj_10.push_back(Trajectory(  -0.40, 0.2, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.45, 0.2, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.50, 0.2, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.55, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.60, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.70, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.80, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(  -0.90, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(  -1.00, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(  -1.25, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(  -1.50, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory( -100.0, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory( -600.0, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(-1000.0, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory( 1000.0, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(  600.0, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(  100.0, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(   1.50, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(   1.25, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(   1.00, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(   0.90, 0.8, 0.10));
    vect_traj_10.push_back(Trajectory(   0.80, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(   0.70, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(   0.60, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(   0.55, 0.5, 0.10));
    vect_traj_10.push_back(Trajectory(   0.50, 0.2, 0.10));
    vect_traj_10.push_back(Trajectory(   0.45, 0.2, 0.10));
    vect_traj_10.push_back(Trajectory(   0.40, 0.2, 0.10));

    std::vector<std::vector<Trajectory>> trajectory_registre;
    trajectory_registre.push_back(vect_traj_80);
    trajectory_registre.push_back(vect_traj_50);
    trajectory_registre.push_back(vect_traj_25);
    trajectory_registre.push_back(vect_traj_10);

    double memory_dist = 1.2;

    /* End TRAJECTORY new system ! */

    double final_side = 0;
    double memo_side = 0;
    bool recul_forcer = false;
    int64_t start_recul = get_curr_timestamp();

    // variable pour eviter que l'alerte OPERATOR_SUPERVISION_REQUIRED soit spam.
    bool alert_msg_send = false;
    int alert_road = 9999;

    // variable pour stocker la position du robot sur les 15 dernières secondes.
    std::vector<std::tuple<int64_t,Geographic_point>> vect_geo_save;
    int64_t last_geo_save_ts = get_curr_timestamp();
    double min_dist_threshold_m = 0.0;

    // variable qui permet de stocker l'ID de la current road sur les x dernières secondes.
    int road_moy_time = 2000;
    std::vector<std::tuple<int64_t, int>> vect_last_curr_road_id;

    // variable qui va indiquer les risques de colisions et le type de colision.
    int collision_risk = 0;

    // variable propre au phase de realignement
    double realig_rapport = 1;
    double thresold_stop_realig = 15; // +- 15 deg
    bool realig_process_start = false;

    // variable de monitoring.
    int64_t monitoring_ts = get_curr_timestamp();

    // variable de detection de trotoires.
    int64_t warning_incline_ts = get_curr_timestamp();
    bool detect_warning_incline = false;

    std::cout << "[" << get_curr_timestamp() << "] [info] navigation program is running." << std::endl;

    //==================================================
    // MAIN LOOP :
    // Cette boucle contient l'unique thread du programme
    // numéro 4 qui gère la navigation automatique & manuel.
    //==================================================
    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        std::cout << get_elapsed_time(get_curr_timestamp(), monitoring_ts) << " FPS: " <<  1000/((double)get_elapsed_time(get_curr_timestamp(), monitoring_ts)) << std::endl;
        monitoring_ts = get_curr_timestamp();

        //==============================================
        // HMR : 
        // Manage la mise à jour de la Hive Map Representation
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
                pub_redis_var(&redis, "EVENT", get_event_str(4, "LOAD_HMR", "SUCCESS"));
            }
            catch(...)
            {
                set_redis_var(&redis, "NAV_HMR_MAP_UPDATE", "TRUE");
                pub_redis_var(&redis, "EVENT", get_event_str(4, "LOAD_HMR", "FAIL"));
            }
        }

        //==============================================
        // LOCALISATION : 
        // Mettre à jour la position en lisant redis var.
        // Faire la moyenne de la route actuelle.
        //==============================================

        if(true)
        {
            curr_position.update_pos(&redis);

            int curr_road_ID = get_road_ID_from_pos(&redis, vect_road, curr_position.point, vect_roadmap, 1);
            std::tuple<int64_t,int> curr_road_ID_save(get_curr_timestamp(), curr_road_ID);
            vect_last_curr_road_id.push_back(curr_road_ID_save);

            // [!] New strategie.
            curr_road_ID = get_filtred_road_ID(&redis, vect_last_curr_road_id, road_moy_time);
            
            std::string roadID_str = std::to_string(get_curr_timestamp()) + "|";
            roadID_str += std::to_string(curr_road_ID) + "|";
            set_redis_var(&redis, "NAV_ROAD_CURRENT_ID", roadID_str);

            // if(get_redis_str(&redis, "NAV_FIXE_ROAD").compare("TRUE") == 0)
            // {
            //     for(int i = 0; i < vect_roadmap.size(); i++)
            //     {
            //         if(curr_road_ID == vect_roadmap[i].road->road_ID)
            //         {
            //             curr_position.point->longitude += 0.000002;
            //             curr_position.point->latitude += 0.000002;
            //             Geographic_point pt = get_projected_point(vect_roadmap[i].node_start->point, vect_roadmap[i].node_target->point, curr_position.point);
            //             curr_position.point->longitude = pt.longitude + 0.000002;
            //             curr_position.point->latitude  = pt.latitude + 0.000002;;

            //             std::vector<std::string> vect_red;
            //             get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_red);

            //             std::string str = vect_red[0] + "|" + std::to_string(curr_position.point->longitude) + "|" + std::to_string(curr_position.point->latitude) + "|" + vect_red[3] + "|";
            //             set_redis_var(&redis, "NAV_GLOBAL_POSITION", str);
            //         }
            //     }
            // }
        }

        //==============================================
        // GLOBAL PATH : 
        // Compute un global path si demander par le sys.
        //==============================================

        if(get_redis_str(&redis, "MISSION_UPDATE_GLOBAL_PATH").compare("TRUE") == 0)
        {
            std::vector<std::string> vect_str;
            get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str);
            if(std::stoi(vect_str[1]) != -1)
            {
                // [?] update var pour le mode semi-manual.
                alert_road = 9999;
                alert_msg_send = false;

                pub_redis_var(&redis, "EVENT", get_event_str(4, "COMPUTE_GLOBAL_PATH", "START"));

                // [?] permet de MAJ les changements de la HMR.
                update_path_node(vect_node, vect_road, graph);

                // [?] selectionne le node le plus proche du robot sur la route actuel.
                int node_start_ID = get_node_ID_from_road(vect_road, std::stoi(vect_str[1]), curr_position.point);

                std::vector<std::string> vect_str_2;
                get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION", vect_str_2);
                Geographic_point destination_pos = Geographic_point(std::stod(vect_str_2[1]), std::stod(vect_str_2[2]));

                vect_roadmap.clear();
                int destination_road_ID = get_road_ID_from_pos(&redis, vect_road, &destination_pos, vect_roadmap, 2);

                std::string redis_str = std::to_string(get_curr_timestamp()) + "|" + std::to_string(destination_road_ID) + "|";
                set_redis_var(&redis, "NAV_AUTO_DESTINATION_ROAD_ID", redis_str);

                int node_endof_ID = get_node_ID_from_road(vect_road, destination_road_ID, &destination_pos);

                /**
                 * NOTE: 
                 * Le prochain if permet de gerer les cas ou les nodes d'arriver et de départ sont les mêmes (A)
                 * mais pas la même route ou le cas ou les nodes sont sont situé sur la même route (B).
                 * 
                 * TODO:
                 * Gerer le cas (C) ou la route est la même et le node de départ et d'arrivé aussi.
                 */
                if((node_start_ID) == node_endof_ID || (std::stoi(vect_str[1]) == destination_road_ID))
                {
                    vect_brut_road.clear();

                    // SAME NODE PROBLEM
                    if(node_start_ID == node_endof_ID && (std::stoi(vect_str[1]) != destination_road_ID))
                    {
                        for(int i = 0; i < vect_road.size(); i++)
                        {
                            if(vect_road[i].road_ID == (std::stoi(vect_str[1])))
                            {
                                vect_brut_road.push_back(&vect_road[i]);
                                break;
                            }
                        }
                        for(int i = 0; i < vect_road.size(); i++)
                        {
                            if(vect_road[i].road_ID == destination_road_ID)
                            {
                                vect_brut_road.push_back(&vect_road[i]);
                                break;
                            }
                        }

                        Roadmap_node rm_node  = Roadmap_node();
                        rm_node.road          = vect_brut_road[0];

                        std::vector<Data_node*> tempo_vect; 
                        detect_connection(vect_brut_road[0], vect_brut_road[1], tempo_vect);

                        rm_node.node_start    = tempo_vect[1];
                        rm_node.node_target   = tempo_vect[0];
                        vect_roadmap.push_back(rm_node);

                        Roadmap_node rm_node2  = Roadmap_node();
                        rm_node2.road          = vect_brut_road[1];
                        rm_node2.node_start    = rm_node.node_target;
                        if(rm_node2.node_start->node_ID != vect_brut_road[1]->A->node_ID)
                        {
                            rm_node2.node_target = vect_brut_road[1]->A;
                        }
                        else
                        {
                            rm_node2.node_target = vect_brut_road[1]->B;
                        }
                        vect_roadmap.push_back(rm_node2);

                        // TODO TIME AND DIST.
                    }

                    // SAME ROAD PROBLEM.
                    if((std::stoi(vect_str[1]) == destination_road_ID))
                    {
                        for(int i = 0; i < vect_road.size(); i++)
                        {
                            if(vect_road[i].road_ID == destination_road_ID)
                            {
                                vect_brut_road.push_back(&vect_road[i]);
                                Roadmap_node rm_node  = Roadmap_node();
                                rm_node.road          = &vect_road[i];

                                // select the direction.
                                double distRA = get_angular_distance(curr_position.point, vect_road[i].A->point);
                                double distDA = get_angular_distance(&destination_pos   , vect_road[i].A->point);
                                
                                if(distRA < distDA)
                                {
                                    rm_node.node_start  = vect_road[i].A;
                                    rm_node.node_target = vect_road[i].B;
                                }
                                else
                                {
                                    rm_node.node_start  = vect_road[i].B;
                                    rm_node.node_target = vect_road[i].A;
                                }

                                // TODO TIME AND DIST.

                                vect_roadmap.push_back(rm_node);
                                break;
                            }
                        }
                    }

                    // [?] Permet de visualiser sur la simulation le global path.
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

                    set_redis_var(&redis, "SIM_GLOBAL_PATH", global_path_str);

                    pub_redis_var(&redis, "EVENT", get_event_str(4, "COMPUTE_GLOBAL_PATH", "SUCCESS"));
                    set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");
                    set_redis_var(&redis, "MISSION_AUTO_TYPE",   "GOTO");
                    set_redis_var(&redis, "MISSION_AUTO_STATE",  "IN_PROGRESS");
                }
                else
                {
                    if(compute_navigation_path(node_start_ID, node_endof_ID, graph, vect_road, vect_brut_road))
                    {
                        process_final_roadmap(&redis, vect_brut_road, vect_road, vect_roadmap);

                        // [?] Permet de visualiser sur la simulation le global path.
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

                        pub_redis_var(&redis, "EVENT", get_event_str(4, "COMPUTE_GLOBAL_PATH", "SUCCESS"));
                        set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");
                        set_redis_var(&redis, "MISSION_AUTO_TYPE",   "GOTO");
                        set_redis_var(&redis, "MISSION_AUTO_STATE",  "IN_PROGRESS");
                    }
                    else
                    {
                        set_redis_var(&redis, "MISSION_ESTI_TIME_TO_TARGET", "0");
                        set_redis_var(&redis, "MISSION_ESTI_DIST_TO_TARGET", "0");
                        set_redis_var(&redis, "SIM_GLOBAL_PATH", "");
                        pub_redis_var(&redis, "EVENT", get_event_str(4, "ERR", "2_COMPUTE_GLOBAL_PATH NO_PATH_POSSIBLE"));
                    }
                }
            }
            else
            {
                set_redis_var(&redis, "MISSION_ESTI_TIME_TO_TARGET", "0");
                set_redis_var(&redis, "MISSION_ESTI_DIST_TO_TARGET", "0");
                pub_redis_var(&redis, "EVENT", get_event_str(4, "ERR", "2_COMPUTE_GLOBAL_PATH NO_ROAD_ID"));
            }
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "FALSE");
        }

        //==============================================
        // ENV DATA :
        // Cette partie va lire l'ensemble des channels
        // ou les différents capteurs publish leur data.
        // Va aussi permettre de detecter les colisions
        // qui sont évitable.
        //==============================================

        if(true)
        {
            std::vector<std::string> vect_redis_str;
            get_redis_multi_str(&redis, "NAV_LOCAL_POSITION", vect_redis_str);

            // std::cout << "A " << get_curr_timestamp() << std::endl;
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
                
                /**
                 * NOTE: On va verifier si le robot se situe sur un dodane, dans ce cas, les valeurs
                 * devant lui ne seront pas pris en compte. 
                 * 
                 */

                // if(time_is_over( get_curr_timestamp(), std::stoul(get_redis_str(&redis, "NAV_INCLINE_CHANGE")), 1000)) { detect_warning_incline = false;}
                // else{ detect_warning_incline = true;}

                get_redis_multi_str(&redis, "ENV_CAM1_OBJECTS", vect_redis_str);
                if(!is_same_time(timesptamp_cam1, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_cam1 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 3)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        // vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[0], &min_dist, &max_dist, vect_obj, &min_space, &min_observation, detect_warning_incline);
                    }
                }

                get_redis_multi_str(&redis, "ENV_CAM2_OBJECTS", vect_redis_str);
                if(!is_same_time(timesptamp_cam2, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_cam2 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 3)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        // vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[1], &min_dist, &max_dist, vect_obj, &min_space, &min_observation, detect_warning_incline);
                    }
                }

                get_redis_multi_str(&redis, "ENV_LID1_OBJECTS", vect_redis_str);
                if(!is_same_time(timesptamp_lid1, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_lid1 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 3)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        // vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[2], &min_dist, &max_dist, vect_obj, &min_space, &min_observation, detect_warning_incline);
                    }
                }

                get_redis_multi_str(&redis, "ENV_LID2_OBJECTS", vect_redis_str);
                if(!is_same_time(timesptamp_lid2, std::stoul(vect_redis_str[0])))
                {
                    timesptamp_lid2 = std::stoul(vect_redis_str[0]);
                    for(int i = 1; i < vect_redis_str.size(); i += 3)
                    {
                        vect_obj_brut.clear();
                        vect_obj_brut.push_back(vect_redis_str[i+0]);
                        vect_obj_brut.push_back(vect_redis_str[i+1]);
                        vect_obj_brut.push_back(vect_redis_str[i+2]);
                        // vect_obj_brut.push_back(vect_redis_str[i+3]);

                        process_brut_obj(curr_local_pos, vect_obj_brut, &vect_sensor_prm[3], &min_dist, &max_dist, vect_obj, &min_space, &min_observation, detect_warning_incline);
                    }
                }
                
                // [?] Le code suivant permet de nettoyer les datas qui se supperpose ou qui sont vieil.
                // std::cout << "INPUT S > " << vect_obj.size() << std::endl;
                clear_obj_vect(curr_local_pos, vect_obj, clear_time, clear_dist);
                // std::cout << "INPUT E > " << vect_obj.size() << std::endl;
            
                collision_risk = emergency_collision_detector(curr_local_pos, vect_obj);
            }


        }

        //==============================================
        // NAVIGATION : 
        // Cette partie permet de prendre la descision
        // de la prochaine commande moteur.
        //==============================================

        if(get_redis_str(&redis, "MISSION_MOTOR_BRAKE").compare("FALSE") == 0)
        {
            //==========================================
            // MANUAL NAVIGATION PARTIE :
            //==========================================
            if(get_redis_str(&redis, "ROBOT_MODE").compare("MANUAL") == 0)
            {
                if(get_redis_str(&redis, "MISSION_MANUAL_TYPE").compare("MANUAL_MOVE") == 0 && \
                get_redis_str(&redis, "MISSION_MANUAL_STATE").compare("IN_PROGRESS") == 0 && \
                manual_mode_available(&redis) == 10)
                {
                    if(!compare_redis_var(&redis, "NAV_LOCAL_JS_MODE", "ACTIVATE"))
                    {
                        // MANUAL FROM SERVER.
                        get_redis_multi_str(&redis, "EVENT_MANUAL_CONTROLER_DATA", vect_cmd_ctr);

                        std::string flag_manual_mode = get_redis_str(&redis, "NAV_MANUAL_MODE");

                        if(flag_manual_mode.compare("STANDARD") == 0 || \
                        flag_manual_mode.compare("STANDARD_MAX") == 0 )
                        {
                            if(!time_is_over(get_curr_timestamp(), std::stoul(vect_cmd_ctr[0]), 1000))
                            {
                                std::string flag_manual_mode  = get_redis_str(&redis, "NAV_MANUAL_MODE");
                                curr_max_speed    = get_max_speed(&redis, "MANUAL", flag_manual_mode, vect_road);

                                // motor_command_str = map_manual_command(&redis, std::stod(vect_cmd_ctr[1]), std::stod(vect_cmd_ctr[2]), std::stod(vect_cmd_ctr[3]), curr_max_speed);
                                std::vector<std::string> inutil;
                                motor_command_str = map_local_manual_command(&redis, curr_max_speed, inutil, 30, 0, std::stod(vect_cmd_ctr[2]), std::stod(vect_cmd_ctr[1]), std::stod(vect_cmd_ctr[3]));
                            }
                            else
                            {
                                set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                                pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_MANUAL_MOVE", "STANDARD_MODE_OVER_TIME"));
                            }
                        }
                        else if(false)
                        {
                            //!\\ OTHER MODE.
                        }
                    }
                    else
                    {
                        // MANUAL FROM LOCAL JS.
                        get_redis_multi_str(&redis, "EVENT_LOCAL_JS_DATA", vect_cmd_ctr);
                        if(!time_is_over(get_curr_timestamp(), std::stoul(vect_cmd_ctr[0]), 1000))
                        {
                            std::string flag_manual_mode  = get_redis_str(&redis, "NAV_MANUAL_MODE");
                            curr_max_speed    = get_max_speed(&redis, "MANUAL", flag_manual_mode, vect_road);

                            // motor_command_str = map_local_manual_command(&redis, curr_max_speed, vect_cmd_ctr);
                            motor_command_str = map_local_manual_command(&redis, 3.0, vect_cmd_ctr, 30, 0);
                         //std::cout << motor_command_str << std::endl;
                        }
                        else
                        {
                            // set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                            // pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_MANUAL_MOVE", "STANDARD_MODE_OVER_TIME"));

                            std::string flag_manual_mode  = get_redis_str(&redis, "NAV_MANUAL_MODE");
                            curr_max_speed    = get_max_speed(&redis, "MANUAL", flag_manual_mode, vect_road);

                            // motor_command_str = map_local_manual_command(&redis, curr_max_speed, vect_cmd_ctr);
                            motor_command_str = map_local_manual_command(&redis, 3.0, vect_cmd_ctr, 30, 1);
                         //std::cout << motor_command_str << std::endl;
                        }
                    }
                }
            }

            //==========================================
            // AUTOMATIQUE NAVIGATION PARTIE :
            //==========================================
            if(compare_redis_var(&redis, "ROBOT_MODE", "AUTO") && compare_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "FALSE"))
            {
                if(compare_redis_var(&redis, "MISSION_AUTO_TYPE", "GOTO") && \
                compare_redis_var(&redis, "MISSION_AUTO_STATE", "IN_PROGRESS") && \
                (auto_mode_available(&redis) == 10 || auto_mode_available(&redis) == 20))
                {
                    /**
                     * NOTE:
                     * 
                     * Cette partie va permettre de faire 3 choses :
                     * 
                     * 1. Elle va calculer la distance et le temps jusqu'a destination.
                     * 2. Elle permet de faire fonctionner le mode parking qui consiste
                     * à appeler un opérateur pour venir garer le robot.
                     * 3. Permet d'ajouter le mode de conduite semi-manuel qui consiste
                     * à appeler un opérateur sur certaine route qui demande une supervision
                     * ou un pilotage complet.
                     * 4. Permet de sauvegarder les positions sur les 15 dernières secondes
                     * afin de vérifier si le robot n'ai pas bloqué.
                     * 
                     * NOTE:
                     * Le mode 3 peux permettre de se passer du mode 1.
                     * 
                     */
                    if(true)
                    {
                        // [1] PART.
                        bool start = false;
                        std::vector<std::string> vect_str;
                        get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str);
                        int curr_road_id = std::stoi(vect_str[1]);

                        get_redis_multi_str(&redis, "NAV_AUTO_PROJECT_DESTINATION", vect_str);
                        Geographic_point dest = Geographic_point(std::stod(vect_str[1]), std::stod(vect_str[2]));

                        get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION_ROAD_ID", vect_str);
                        int dest_road_id = std::stoi(vect_str[1]);

                        double dist_total_m = 0;
                        double time_total_s = 0;
                        double m            = 0;

                        if(curr_road_id == dest_road_id)
                        {
                            for(int i = 0; i < vect_roadmap.size(); i++)
                            {
                                if(vect_roadmap[i].road->road_ID == curr_road_id)
                                {
                                    m = get_angular_distance(curr_position.point, &dest);
                                    dist_total_m = m;
                                    time_total_s = get_time_to_travel_s(m, vect_roadmap[i].road->max_speed);
                                }
                            }

                        }
                        else
                        {
                            for(int i = 0; i < vect_roadmap.size(); i++)
                            {
                                if(start)
                                {
                                    if(i == vect_roadmap.size()-1)
                                    {
                                        m             = get_angular_distance(vect_roadmap[i].node_start->point , &dest);
                                        dist_total_m += m;
                                        time_total_s += get_time_to_travel_s(m, vect_roadmap[i].road->max_speed);
                                    }
                                    else
                                    {
                                        m             = get_angular_distance(vect_roadmap[i].node_target->point, vect_roadmap[i].node_start->point);
                                        dist_total_m += m;
                                        time_total_s += get_time_to_travel_s(m, vect_roadmap[i].road->max_speed);
                                    }
                                }
                                if(vect_roadmap[i].road->road_ID == curr_road_id)
                                {
                                    start         = true;
                                    m             = get_angular_distance(vect_roadmap[i].node_target->point, curr_position.point);
                                    dist_total_m += m;
                                    time_total_s += get_time_to_travel_s(m, vect_roadmap[i].road->max_speed);
                                }
                            }
                        }

                        set_redis_var(&redis, "MISSION_ESTI_TIME_TO_TARGET", std::to_string((int)(time_total_s)));
                        set_redis_var(&redis, "MISSION_ESTI_DIST_TO_TARGET", std::to_string((int)(dist_total_m)));

                        // [2] PART.
                        if((int)(dist_total_m) <= std::stoi(get_redis_str(&redis, "NAV_AUTO_MODE_PARKING_DIST_M")) && 
                        get_redis_str(&redis, "NAV_AUTO_MODE_PARKING").compare("OPERATOR") == 0)
                        {
                            // demande de pilotage manuel pour un parking.
                            pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_AUTO_GOTO", "NEED_OPERATOR_PARKING"));
                            set_redis_var(&redis, "MISSION_MOTOR_BRAKE"   , "TRUE");
                            set_redis_var(&redis, "MISSION_AUTO_STATE"    , "PAUSE");
                            set_redis_var(&redis, "MISSION_MANUAL_STATE"  , "PAUSE");
                            pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_PAUSE", "START"));
                        }

                        // [3] PART.
                        for(int i = 0; i < vect_roadmap.size(); i++)
                        {
                            if(vect_roadmap[i].road->road_ID == curr_road_id)
                            {
                                if(vect_roadmap[i].road->opt_auto == 1 && !alert_msg_send && alert_road != curr_road_id)
                                {
                                    alert_road = curr_road_id;
                                    alert_msg_send = true;

                                    // demande de supervision.
                                    pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_AUTO_GOTO", "NEED_OPERATOR_SUPERVISION"));
                                }
                                if(vect_roadmap[i].road->opt_auto == 2)
                                {
                                    // demande de pilotage manuel.
                                    set_redis_var(&redis, "MISSION_MOTOR_BRAKE"   , "TRUE");
                                    set_redis_var(&redis, "MISSION_AUTO_STATE"    , "PAUSE");
                                    set_redis_var(&redis, "MISSION_MANUAL_STATE"  , "PAUSE");
                                    pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_AUTO_GOTO", "NEED_OPERATOR_DRIVE"));
                                }
                                else
                                {
                                    alert_msg_send = false;
                                }
                                break;
                            }
                        }

                        // [4] PART.
                        // if(time_is_over(get_curr_timestamp(), last_geo_save_ts, 1000))
                        // {
                        //     std::tuple<int64_t,Geographic_point> new_geo_save(get_curr_timestamp(), *curr_position.point);
                        //     vect_geo_save.push_back(new_geo_save);

                        //     auto it = vect_geo_save.begin();
                        //     while (it != vect_geo_save.end())
                        //     {
                        //         if(get_elapsed_time(get_curr_timestamp(), std::get<0>(*it)) > 16000)
                        //         {
                        //             it = vect_geo_save.erase(it); 
                        //         }
                        //         else
                        //         {
                        //             it++;
                        //         }
                        //     }

                        //     // On vérifie si la position du robot il y a 15 secondes environ dépasse un certain threshold.
                        //     for(int i = 0; i < vect_geo_save.size(); i++)
                        //     {
                        //         if(get_elapsed_time(get_curr_timestamp(), std::get<0>(vect_geo_save[i])) > 14000)
                        //         {
                        //             if(get_angular_distance(curr_position.point, &std::get<1>(vect_geo_save[i])) <= min_dist_threshold_m)
                        //             {
                        //                 // Il y a un probleme, notre deplacement n'ai pas assez grand, on previent l'opérateur
                        //                 // et on immobilise le robot.

                        //                 pub_redis_var(&redis, "EVENT", get_event_str(4, "ERR", "3|ROBOT_BLOCKED"));
                        //                 set_redis_var(&redis, "MISSION_MOTOR_BRAKE"   , "TRUE");
                        //                 set_redis_var(&redis, "MISSION_AUTO_STATE"    , "PAUSE");
                        //                 set_redis_var(&redis, "MISSION_MANUAL_STATE"  , "PAUSE");
                        //             }
                        //         }
                        //     }

                        //     last_geo_save_ts = get_curr_timestamp();
                        // }
                    }

                    //==================================
                    // SELECT SPECIFIC AUTOMATIQUE MODE :
                    // 1. Mode classique "SIMPLE"
                    // 2. Mode obstacle classique "OBSTACLE_AVOIDANCE_NIV1"
                    //==================================
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
                            pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_AUTO_GOTO", "SUCCESS"));
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
                            set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC", std::to_string(abs(radius_circle)));
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

                        int64_t debug_start_time = get_curr_timestamp();

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

                        // [?] Check si on est arriver à destination.
                        if(curr_road_id == dest_road_id && \
                        get_angular_distance(curr_position.point, &dest) <= std::stod(get_redis_str(&redis, "NAV_AUTO_DESTINATION_CROSSING_M")))
                        {
                            set_redis_var(&redis, "SIM_GLOBAL_PATH", "");

                            set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                            set_redis_var(&redis, "MISSION_AUTO_STATE",  "COMPLETED");
                            pub_redis_var(&redis, "EVENT", get_event_str(4, "MISSION_AUTO_GOTO", "SUCCESS"));
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
                                    } else 
                                    { next_target_node = curr_target_node; } // TODO: REMOVE
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
                            
                            // std::string str_global = std::to_string(get_curr_timestamp()) + "|";
                            // str_global += std::to_string(new_position2.longitude) + "|" + std::to_string(new_position2.latitude) + "|"; 
                            // std::cout << str_global << std::endl;

                            if(get_redis_str(&redis, "NAV_FIXE_ROAD").compare("TRUE") == 0 && time_is_over(get_curr_timestamp(), timesptamp_proj, 4000))
                            {
                                timesptamp_proj = get_curr_timestamp();
                                std::cout << "[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]" << std::endl;

                                std::vector<std::string> vect_redis; 
                                get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_redis);
                                std::string str_global = std::to_string(get_curr_timestamp()) + "|";
                                str_global += std::to_string(new_position2.longitude+0.0000001) + "|" + std::to_string(new_position2.latitude) + "|"; 
                                str_global += vect_redis[3] + "|";
                                std::cout << str_global << std::endl;

                                if(!std::isnan(new_position2.longitude) && new_position2.longitude != -1)
                                {
                                    set_redis_var(&redis, "NAV_GLOBAL_POSITION", str_global);
                                }
                            }
                            // [ENDSIM]

                            double dx             = xf - XX;
                            double dy             = yf - YY;
                            double dist_to_road_m = sqrt(pow(dx,2)+pow(dy,2));

                            double xt, yt;
                            double xs, ys;

                            // [?] Update seek point [xs,ys] (target point).
                            double ddx = XX - xb;
                            double ddy = YY - yb;
                            double dist_proj_to_target = sqrt(pow(ddx,2)+pow(ddy,2));
                            // std::cout << "dist: " << dist_proj_to_target << std::endl;
                            if(dist_proj_to_target >= std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")))
                            {
                                // TODO: Avancer de x mètres le point projeter sur road.
                                double bearing_start_target = get_bearing(curr_start_node, curr_target_node);
                                
                                set_redis_var(&redis, "NAV_HDG_CURR_ROAD", std::to_string(bearing_start_target));

                                xt = XX + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * cos(deg_to_rad(bearing_start_target));
                                yt = YY + std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) * sin(deg_to_rad(bearing_start_target));
                            }
                            else
                            {
                                set_redis_var(&redis, "NAV_HDG_CURR_ROAD", std::to_string(get_bearing(curr_start_node, curr_target_node)));

                                double bearing_start_target = get_bearing(curr_target_node, next_target_node);
                                double road_extension = std::stod(get_redis_str(&redis, "NAV_AUTO_TARGET_EXTENSION")) - dist_proj_to_target;
                                // road_extension = 5.0;

                                // if(road_extension < 0) road_extension = 0;

                                xt = xb + (abs(road_extension)) * cos(deg_to_rad(bearing_start_target));
                                yt = yb + (abs(road_extension)) * sin(deg_to_rad(bearing_start_target));

                            }

                            //[NOTE] L'algo de base prend en compte la vélocité du robot pour choisir la prochaine cible. Ici non. Pour l'instant.
                            
                            std::vector<std::string> last_command_motor;
                            get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", last_command_motor);
                            double force = (std::stod(last_command_motor[1]) + std::stod(last_command_motor[3])) / 2;
                            if(force < 3.0) force = 3.0;
                            xf   = force * cos(deg_to_rad(curr_position.g_hdg));
                            yf   = force * sin(deg_to_rad(curr_position.g_hdg));

                            xs = xt + (-xf);
                            ys = yt + (-yf);
     

                            // [SIM_AUTO_POINT_TARGET]
                            double distance_to_pt = sqrt(pow(xs,2)+pow(ys,2));
                            double bearing_to_pt  = rad_to_deg(2 * atan(ys / (xs + sqrt(pow(xs, 2) + pow(ys, 2)))));
                            Geographic_point new_position3 = get_new_position(curr_position.point, bearing_to_pt, distance_to_pt);
                            set_redis_var(&redis, "SIM_AUTO_PT_TARGET", std::to_string(new_position3.longitude) + "|" + std::to_string(new_position3.latitude) + "|");
                            // [ENDSIM]   

                            // [?] Transform target point to motor command.
                            double perfect_motor_speed = sqrt(pow(xs,2)+pow(ys,2));
                            double curr_max_speed      = get_max_speed(&redis, "AUTO", "NO_MODE", vect_road);
                            set_redis_var(&redis, "NAV_CURR_ROAD_MAX_SPEED", std::to_string(curr_max_speed));
                            
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
                                    diff_angle = -(curr_position.g_hdg - final_angle
                                    );
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
                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg - 90, radius_circle);
                            }
                            else
                            {
                                center_circle = get_new_position(curr_position.point, curr_position.g_hdg + 90, radius_circle);
                            }
                            set_redis_var(&redis, "SIM_AUTO_PT_ICC", std::to_string(center_circle.longitude) + "|" + std::to_string(center_circle.latitude) + "|");
                            set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC", std::to_string(abs(radius_circle)));
                            // [SIM_END]


                            // std::cout << "NEW FRAME" << std::endl;
                            //====================
                            // ROTATION SUR PLACE
                            //====================
                            double opt_treshold = 130;
                            if(diff_angle > opt_treshold || (realig_process_start && diff_angle > 0))
                            {
                                //[!] Verifier le process de realignement.
                                realig_process_start = true;
                                if(diff_angle < thresold_stop_realig) realig_process_start = false;

                                //[!] Il faut faire demi tour vers la droite.
                                double max_roat_speed = 0.2;

                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    /**
                                     * NOTE:
                                     * Lorsque en mode automatique, le robot souhaite faire une rotation sur place,
                                     * il arrive de deux cas :
                                     * 1 - Le robot est à l'arret (départ)
                                     * 2 - Le robot était entrain d'avancer (changement de route)
                                     * 
                                     * Il faut d'abord le faire ralentir si ce n'ai pas le cas.
                                     */

                                    double max_accel  = std::stod(get_redis_str(&redis,"NAV_MAX_ACCEL"));
                                    double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));

                                    std::vector<std::string> last_command_motor;
                                    get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", last_command_motor);
                                    int previous_mode = 0;
                                    if(std::stod(last_command_motor[1]) >= 0 && std::stod(last_command_motor[4]) >= 0) previous_mode = 2;
                                    if(std::stod(last_command_motor[1]) <= 0 && std::stod(last_command_motor[4]) <= 0) previous_mode = 3;
                                    if(std::stod(last_command_motor[1]) <  0 && std::stod(last_command_motor[4]) >  0) previous_mode = 4;
                                    if(std::stod(last_command_motor[1]) >  0 && std::stod(last_command_motor[4]) <  0) previous_mode = 5;
                                    if(std::stod(last_command_motor[1]) == 0 && std::stod(last_command_motor[4]) == 0) previous_mode = 1;

                                 //std::cout << previous_mode << std::endl;

                                    std::vector<double> last_command_motor_double;
                                    last_command_motor_double.push_back(std::stod(last_command_motor[1]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[2]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[3]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[4]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[5]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[6]));

                                    double previous_speed = 0;
                                    double previous_lspeed = 0;
                                    if(previous_mode == 4 || previous_mode == 5) previous_speed = abs(std::stod(last_command_motor[1]));
                                    else
                                    {
                                        if(abs(std::stod(last_command_motor[1])) >  abs(std::stod(last_command_motor[4]))) 
                                        {
                                            previous_speed = std::stod(last_command_motor[1]);
                                            previous_lspeed = std::stod(last_command_motor[4]);
                                        }
                                        else
                                        {
                                            previous_speed = std::stod(last_command_motor[4]); 
                                            previous_lspeed = std::stod(last_command_motor[1]);
                                        }
                                    }

                                    if(previous_mode == 1)
                                    {
                                        motor_command_str += std::to_string(previous_speed + (max_accel*0.2)/30) + "|";
                                        motor_command_str += std::to_string((previous_speed + (max_accel*0.2)/30)/4) + "|";
                                        motor_command_str += std::to_string(previous_speed + (max_accel*0.2)/30) + "|";
                                        motor_command_str += std::to_string(-(previous_speed + (max_accel*0.2)/30)) + "|";
                                        motor_command_str += std::to_string(-((previous_speed + (max_accel*0.2)/30)/4)) + "|";
                                        motor_command_str += std::to_string(-(previous_speed + (max_accel*0.2)/30)) + "|";
                                    }
                                    if(previous_mode == 4 || previous_mode == 5)
                                    {
                                        double new_speed;
                                        new_speed = previous_speed + ((max_accel*0.2)/30);
                                        if(new_speed > max_roat_speed) new_speed = max_roat_speed;

                                        motor_command_str += std::to_string(new_speed) + "|";
                                        motor_command_str += std::to_string(new_speed/1.5) + "|";
                                        motor_command_str += std::to_string(new_speed) + "|";
                                        motor_command_str += std::to_string(-(new_speed)) + "|";
                                        motor_command_str += std::to_string(-(new_speed/1.5)) + "|";
                                        motor_command_str += std::to_string(-(new_speed)) + "|";
                                    }
                                    if(previous_mode == 2)
                                    {
                                        /**
                                         * NOTE: 
                                         * A ce moment le robot doit frainer avant de se realigner, le probleme
                                         * c'est qu'il doit conserver le rapport qu'il avait avant le réalignement
                                         * car si on diminue de manière uniforme les moteurs, la courbe qu'il était 
                                         * entrain de faire va augmenter et causer des problemes.
                                         */

                                        std::vector<double> opti_command; // opti : dans le sens pour frainer le plus vite.
                                        for(int j = 0; j < 6; j++)
                                        {
                                            double v = last_command_motor_double[j] - (max_deccel/30);
                                            if(v < 0) v = 0;
                                            opti_command.push_back(v);
                                        }

                                        // std::string deb = "0000408640000|";
                                        // for(int i = 0; i < 6; i++)
                                        // {
                                        //     deb += std::to_string(opti_command[i]) + "|";
                                        // }
                                        // std::cout << "FIR: " << deb << std::endl;

                                        bool rot_left = true;
                                        if(last_command_motor_double[0] - last_command_motor_double[3] < 0) rot_left = false;

                                        double rapport_opti = realig_rapport;

                                        if(rot_left)
                                        {
                                            // les moteurs moins rapides doivent frainer plus doucement car ceux rapide fraine
                                            // au max.
                                            double good_speed_right = opti_command[0] / rapport_opti;
                                            if(abs(opti_command[3] - good_speed_right) > (max_accel/30))
                                            {
                                                opti_command[0] += (max_accel/30);
                                                opti_command[1] += (max_accel/30);
                                                opti_command[2] += (max_accel/30);
                                            }
                                            else
                                            {
                                                opti_command[3] = good_speed_right;
                                                opti_command[4] = good_speed_right;
                                                opti_command[5] = good_speed_right;
                                            }
                                        }
                                        else
                                        {
                                            double good_speed_left = opti_command[3] / rapport_opti;
                                            if(abs(opti_command[0] - good_speed_left) > (max_accel/30))
                                            {
                                                opti_command[0] += (max_accel/30);
                                                opti_command[1] += (max_accel/30);
                                                opti_command[2] += (max_accel/30);
                                            }
                                            else
                                            {
                                                opti_command[0] = good_speed_left;
                                                opti_command[1] = good_speed_left;
                                                opti_command[2] = good_speed_left;
                                            }
                                        }
                                        
                                        for(int i = 0; i < 6; i++) motor_command_str += std::to_string(opti_command[i]) + "|";
                                        // std::cout << "END: " << motor_command_str << std::endl;

                                    }
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "0.2|0.2|0.2|-0.2|-0.2|-0.2|";
                                }
                          //std::coutt << motor_command_str << std::endl;
                            }
                            if(diff_angle < -opt_treshold || (realig_process_start && diff_angle < 0))
                            {
                                //[!] Verifier le process de realignement.
                                realig_process_start = true;
                                if(diff_angle > -thresold_stop_realig) realig_process_start = false;

                                //[!] Il faut faire demi tour vers la gauche.
                                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                double max_roat_speed = 0.2;
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4"))
                                {
                                    double max_accel  = std::stod(get_redis_str(&redis,"NAV_MAX_ACCEL"));
                                    double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));

                                    std::vector<std::string> last_command_motor;
                                    get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", last_command_motor);
                                 //std::cout << "JUST BEFORE PREVIOUS " << get_redis_str(&redis, "HARD_MOTOR_COMMAND") << std::endl;
                                    int previous_mode = 0;
                                    if(std::stod(last_command_motor[1]) >= 0 && std::stod(last_command_motor[4]) >= 0) previous_mode = 2;
                                    if(std::stod(last_command_motor[1]) <= 0 && std::stod(last_command_motor[4]) <= 0) previous_mode = 3;
                                    if(std::stod(last_command_motor[1]) <  0 && std::stod(last_command_motor[4]) >  0) previous_mode = 4;
                                    if(std::stod(last_command_motor[1]) >  0 && std::stod(last_command_motor[4]) <  0) previous_mode = 5;
                                    if(std::stod(last_command_motor[1]) == 0 && std::stod(last_command_motor[4]) == 0) previous_mode = 1;

                                    std::vector<double> last_command_motor_double;
                                    last_command_motor_double.push_back(std::stod(last_command_motor[1]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[2]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[3]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[4]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[5]));
                                    last_command_motor_double.push_back(std::stod(last_command_motor[6]));

                                    double previous_speed = 0;
                                    double previous_lspeed = 0;
                                 //std::cout << previous_mode << std::endl;
                                    if(previous_mode == 4 || previous_mode == 5) previous_speed = abs(std::stod(last_command_motor[1]));
                                    else
                                    {
                                        if(abs(std::stod(last_command_motor[1])) >  abs(std::stod(last_command_motor[4]))) 
                                        {
                                            previous_speed = std::stod(last_command_motor[1]);
                                            previous_lspeed = std::stod(last_command_motor[4]);
                                        }
                                        else
                                        {
                                            previous_speed = std::stod(last_command_motor[4]); 
                                            previous_lspeed = std::stod(last_command_motor[1]);
                                        }
                                    }

                                    if(previous_mode == 1)
                                    {
                                        motor_command_str += std::to_string(-(previous_speed + (max_accel*0.2))) + "|";
                                        motor_command_str += std::to_string(-((previous_speed + (max_accel*0.2))/4)) + "|";
                                        motor_command_str += std::to_string(-(previous_speed + (max_accel*0.2))) + "|";
                                        motor_command_str += std::to_string(previous_speed + (max_accel*0.2)) + "|";
                                        motor_command_str += std::to_string((previous_speed + (max_accel*0.2))/4) + "|";
                                        motor_command_str += std::to_string(previous_speed + (max_accel*0.2)) + "|";
                                    }
                                    if(previous_mode == 4 || previous_mode == 5)
                                    {
                                        double new_speed;
                                        new_speed = previous_speed + ((max_accel*0.2)/30);
                                        if(new_speed > max_roat_speed) new_speed = max_roat_speed;

                                        motor_command_str += std::to_string(-new_speed) + "|";
                                        motor_command_str += std::to_string(-new_speed/1.5) + "|";
                                        motor_command_str += std::to_string(-new_speed) + "|";
                                        motor_command_str += std::to_string((new_speed)) + "|";
                                        motor_command_str += std::to_string((new_speed/1.5)) + "|";
                                        motor_command_str += std::to_string((new_speed)) + "|";
                                    }
                                    if(previous_mode == 2)
                                    {
                                        /**
                                         * NOTE: 
                                         * A ce moment le robot doit frainer avant de se realigner, le probleme
                                         * c'est qu'il doit conserver le rapport qu'il avait avant le réalignement
                                         * car si on diminue de manière uniforme les moteurs, la courbe qu'il était 
                                         * entrain de faire va augmenter et causer des problemes.
                                         */

                                        std::vector<double> opti_command; // opti : dans le sens pour frainer le plus vite.
                                        for(int j = 0; j < 6; j++)
                                        {
                                            double v = last_command_motor_double[j] - (max_deccel/30);
                                            if(v < 0) v = 0;
                                            opti_command.push_back(v);
                                        }

                                        // std::string deb = "0000408640000|";
                                        // for(int i = 0; i < 6; i++)
                                        // {
                                        //     deb += std::to_string(opti_command[i]) + "|";
                                        // }
                                        // std::cout << "FIR: " << deb << std::endl;

                                        bool rot_left = true;
                                        if(last_command_motor_double[0] - last_command_motor_double[3] < 0) rot_left = false;

                                        double rapport_opti = realig_rapport;

                                        if(rot_left)
                                        {
                                            // les moteurs moins rapides doivent frainer plus doucement car ceux rapide fraine
                                            // au max.
                                            double good_speed_right = opti_command[0] / rapport_opti;
                                            if(abs(opti_command[3] - good_speed_right) > (max_accel/30))
                                            {
                                                opti_command[0] += (max_accel/30);
                                                opti_command[1] += (max_accel/30);
                                                opti_command[2] += (max_accel/30);
                                            }
                                            else
                                            {
                                                opti_command[3] = good_speed_right;
                                                opti_command[4] = good_speed_right;
                                                opti_command[5] = good_speed_right;
                                            }
                                        }
                                        else
                                        {
                                            double good_speed_left = opti_command[3] / rapport_opti;
                                            if(abs(opti_command[0] - good_speed_left) > (max_accel/30))
                                            {
                                                opti_command[0] += (max_accel/30);
                                                opti_command[1] += (max_accel/30);
                                                opti_command[2] += (max_accel/30);
                                            }
                                            else
                                            {
                                                opti_command[0] = good_speed_left;
                                                opti_command[1] = good_speed_left;
                                                opti_command[2] = good_speed_left;
                                            }
                                        }
                                        
                                        for(int i = 0; i < 6; i++) motor_command_str += std::to_string(opti_command[i]) + "|";
                                        // std::cout << "END: " << motor_command_str << std::endl;

                                    }
                                }
                                if(compare_redis_var(&redis, "ROBOT_INFO_MODEL", "MK4_LIGHT"))
                                {
                                    motor_command_str += "-0.2|-0.2|-0.2|0.2|0.2|0.2|";
                                }
                          //std::coutt << motor_command_str << std::endl;
                            }
                            if(diff_angle <= opt_treshold && diff_angle >= -opt_treshold && (!realig_process_start))
                            {
                                //==========================================
                                // OBSTACLE ALGO NIV 1 : 
                                //==========================================
                                
                                /**
                                 * NOTE:
                                 * Variables pour statistique debug.
                                 */
                                int trajectory_tested = 0;
                                int64_t start_trajectory_selection_ts = get_curr_timestamp();


                                /**
                                 * NOTE: 
                                 * Les variables suivantes sont résevé au calcul de lissage des commandes moteurs.
                                 */

                                std::vector<std::string> last_command_motor;
                                get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", last_command_motor);
                                int previous_mode = 0;
                                if(std::stod(last_command_motor[1]) >= 0 && std::stod(last_command_motor[4]) >= 0) previous_mode = 2;
                                if(std::stod(last_command_motor[1]) <= 0 && std::stod(last_command_motor[4]) <= 0) previous_mode = 3;
                                if(std::stod(last_command_motor[1]) <  0 && std::stod(last_command_motor[4]) >  0) previous_mode = 4;
                                if(std::stod(last_command_motor[1]) >  0 && std::stod(last_command_motor[4]) <  0) previous_mode = 5;
                                if(std::stod(last_command_motor[1]) == 0 && std::stod(last_command_motor[4]) == 0) previous_mode = 1;

                                std::vector<double> last_command_motor_double;
                                last_command_motor_double.push_back(std::stod(last_command_motor[1]));
                                last_command_motor_double.push_back(std::stod(last_command_motor[2]));
                                last_command_motor_double.push_back(std::stod(last_command_motor[3]));
                                last_command_motor_double.push_back(std::stod(last_command_motor[4]));
                                last_command_motor_double.push_back(std::stod(last_command_motor[5]));
                                last_command_motor_double.push_back(std::stod(last_command_motor[6]));

                                double previous_speed = 0;
                                double previous_lspeed = 0;
                                if(previous_mode == 4 || previous_mode == 5) previous_speed = abs(std::stod(last_command_motor[1]));
                                else
                                {
                                    if(abs(std::stod(last_command_motor[1])) >  abs(std::stod(last_command_motor[4]))) 
                                    {
                                        previous_speed = std::stod(last_command_motor[1]);
                                        previous_lspeed = std::stod(last_command_motor[4]);
                                    }
                                    else
                                    {
                                        previous_speed = std::stod(last_command_motor[4]); 
                                        previous_lspeed = std::stod(last_command_motor[1]);
                                    }
                                }
                                double max_accel  = std::stod(get_redis_str(&redis,"NAV_MAX_ACCEL"));
                                double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));
                                double dbg_curr_speed = (previous_speed + previous_lspeed) / 2;

                                // ETAPE 00: INIT VAR FOR PERFORMANCE.
                                double dist = 0.0;
                                double x = 0.0; double y = 0.0; double angle = 0.0;
                                double angle_diff2 = 0.0;
                                double idx_col = 0.0; double idx_row = 0.0;
                                memory_dist = 1.2;
                                double distance_m = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2 + memory_dist;
                                double center_row = 0.0;
                                bool first_trajectory_safe = true;
                                double min_diff_radius = 9999;
                                int min_diff_radius_idx = -1;
                                int comptor = 0;
                                double sum_moy = 0.0;
                                double dist_obj_center = 0.0;
                                double diff_dist_obj_robot_center = 0.0;
                                double dist_obj_robot = 0.0;
                                double min_dist_obj_robot = 0.0;
                                int comptor_pt = 0;
                                double speed_with_obj = final_max_speed;
                                double min_speed_detect = final_max_speed;
                                double rapport_int_ext = 1.0;
                                double speed_int_with_obj = final_max_speed;
                                double final_radius = radius_circle;
                                final_side = 0;


                                if(diff_angle > 0) final_side = 1;
                                else{final_side = -1;}

                                // ETAPE 0 : INIT LA PROJECTION DES POINTS.
                                std::vector<Vect_2D> DirectMap_obs;
                                std::vector<double> DirectMap_obs_diff_angle;
                                Trajectory Final_traj = Trajectory(0.0, 0.0, 0.0);
                                Final_traj.niv = 200;

                                // ETAPE 1 : TEST ORIGINAL TRAJECTORY
                                double dbg_min_dist_obj = 99999;
                                for(auto obj : vect_obj)
                                {
                                    if(obj.available)
                                    {
                                        // ETAPE 1B : CONNAITRE L'ANGLE DE DIFF.
                                        dist = sqrt(pow(curr_position.l_x - obj.pos->x,2) + pow(curr_position.l_y - obj.pos->y,2));
                                        x = obj.pos->x - curr_position.l_x;
                                        y = obj.pos->y - curr_position.l_y;
                                        angle = rad_to_deg(2 * atan(y / (x + dist))) + 360;
                                        if(angle > 360) angle -= 360;
                                        angle_diff2;

                                        if(curr_position.l_hdg - angle > 0)
                                        {
                                            if(curr_position.l_hdg - angle > 180)
                                            {
                                                // DROITE
                                                angle_diff2 = 360 - (curr_position.l_hdg - angle);
                                            }
                                            else
                                            {
                                                // GAUCHE
                                                angle_diff2 = -(curr_position.l_hdg - angle);
                                            }
                                        }
                                        else
                                        {
                                            if(curr_position.l_hdg - angle < -180)
                                            {
                                                // GAUCHE
                                                angle_diff2 = -(360- (angle - curr_position.l_hdg));
                                            }
                                            else
                                            {   
                                                // DROITE
                                                angle_diff2 = angle - curr_position.l_hdg;
                                            }   
                                        }

                                        if(abs(angle_diff2) < 90)
                                        {
                                            // ETAPE 1C : PROJETER DANS MAP DIRECT (EN DOUBLE).
                                            idx_col = 100 + dist*10*cos(deg_to_rad(angle_diff2));
                                            idx_row = 100 + dist*10*sin(deg_to_rad(angle_diff2));
                                            DirectMap_obs.push_back(Vect_2D(idx_col, idx_row));
                                            DirectMap_obs_diff_angle.push_back(abs(angle_diff2));

                                            // ETAPE 1D : DETERMINER CENTRE DU CERCLE.
                                            // [!] angle_diff '1' et pas '2'
                                            if(diff_angle >= 0) center_row = 100 + abs(radius_circle) * 10;
                                            else{center_row = 100 - abs(radius_circle) * 10;}

                                            // ETAPE 1E : VERIFIER SI ILS SONT DANS TRAJECTOIRE
                                            if(abs(sqrt(pow(100-idx_col,2)+pow(center_row-idx_row,2))/10 - radius_circle) < distance_m && \
                                            dist < 6.0)
                                            {
                                                first_trajectory_safe = false;
                                            }

                                            // ETAPE 1F : SETUP LA VITESSE MAX EN FONCTION DES OBTACLES.
                                            // double temp = 8888;
                                            // if(dist < 1.5) temp = speed_with_obj * pow(dist / 1.5, 1);
                                            // if(temp < min_speed_detect) min_speed_detect = temp;
                                            // if(min_speed_detect < 0.3) min_speed_detect = 0.3;

                                            // TODO NEW: SETUP SPEED EN FONCTION DE LA DISTANCE AVEC L'AVANT DU ROBOT.
                                            if(abs(angle_diff2) < 65)
                                            {
                                                double dist_from_front = abs(sqrt(pow(103-idx_col,2)+pow(100-idx_row,2)))/10;

                                                if(dist_from_front < dbg_min_dist_obj)
                                                {
                                                    dbg_min_dist_obj = dist_from_front;
                                                    /**
                                                     * NOTE: Plusieurs traitement différent.
                                                     * [1] - dist_from_front < 1.0 => La vitesse max est de 0.27 m/s (1km/h)
                                                     * [2] - dist_from_front > 1.3 x Current speed => La vitesse max ne change pas.
                                                     * [3] - Entre les deux, la vitesse max evolue
                                                     */

                                                    double max_dist = dbg_curr_speed * 2.0;
                                                    
                                                    if(dist_from_front < 1.0) min_speed_detect = 0.27;
                                                    if(dist_from_front >= 1.0 && dist_from_front < max_dist)
                                                    {
                                                        min_speed_detect = ((dist_from_front-1.0)/((dbg_curr_speed*2.0)-1)) + 0.27;
                                                        if(min_speed_detect > dbg_curr_speed) min_speed_detect = dbg_curr_speed;
                                                    }
                                                }
                                                
                                            }
                                        }
                                    }
                                }

                                
                                speed_with_obj = (min_speed_detect < speed_with_obj) ? min_speed_detect : speed_with_obj;

                                trajectory_tested++;

                                double l_radius_circle;
                                if(diff_angle >= 0) l_radius_circle = radius_circle;
                                else{l_radius_circle = -radius_circle;}
                                min_diff_radius = 9999;
                                for(int i = 0; i < trajectory_registre[0].size(); i++)
                                {
                                    if(abs(trajectory_registre[0][i].r - l_radius_circle) < min_diff_radius)
                                    {
                                        min_diff_radius = abs(trajectory_registre[0][i].r - l_radius_circle);
                                        min_diff_radius_idx = i;
                                    }

                                    // ETAPE 2B : RESET
                                    trajectory_registre[0][i].niv  = -1;
                                    trajectory_registre[0][i].moy  = 0.0;
                                    trajectory_registre[0][i].pt_M = 0.0;
                                }

                                bool trajectory_found_in_bash = false;
                                if(first_trajectory_safe)
                                {
                                   trajectory_found_in_bash = true;
                                   Final_traj = trajectory_registre[0][min_diff_radius_idx];
                                }

                                if(!first_trajectory_safe)
                                {
                                    trajectory_found_in_bash = false;
                                    /**
                                     * NOTE: Nouvelle algorythme de selection de trajectoire.
                                     * 
                                     */

                                    std::vector<double> activation_dist_vect;
                                    activation_dist_vect.push_back(5.5);
                                    activation_dist_vect.push_back(4.0);
                                    activation_dist_vect.push_back(1.2);
                                    activation_dist_vect.push_back(0.6);

                                    for(int y = 0; y < 4 && (!trajectory_found_in_bash); y++)
                                    {
                                        /* Pour ce bacht, identifier l'index le plus proche de la perfect trajectory. */
                                        /* La variable est "min_diff_radius_idx" */
                                        
                                        distance_m = (std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2) + trajectory_registre[y][0].security_dist;
                                        memory_dist = distance_m;

                                        double l_radius_circle;
                                        if(diff_angle >= 0) l_radius_circle = radius_circle;
                                        else{l_radius_circle = -radius_circle;}
                                        for(int i = 0; i < trajectory_registre[y].size(); i++)
                                        {
                                            if(abs(trajectory_registre[y][i].r - l_radius_circle) < min_diff_radius)
                                            {
                                                min_diff_radius = abs(trajectory_registre[y][i].r - l_radius_circle);
                                                min_diff_radius_idx = i;
                                            }

                                            // ETAPE 2B : RESET INTERN VALUE.
                                            trajectory_registre[y][i].niv  = -1;
                                            trajectory_registre[y][i].moy  = 0.0;
                                            trajectory_registre[y][i].pt_M = 0.0;
                                        }

                                        /**
                                         * NOTE: grace au minimun, on va initialiser l'ordre de priorité des trajectoires de ce bacht.
                                         * 
                                         */

                                        for(int i = min_diff_radius_idx; i >= 0; i--)
                                        {
                                            trajectory_registre[y][i].niv = comptor;
                                            comptor++;
                                        }

                                        comptor = 0;

                                        for(int i = min_diff_radius_idx; i < trajectory_registre[y].size(); i++)
                                        {
                                            trajectory_registre[y][i].niv = comptor;
                                            comptor++;
                                        }

                                        /**
                                         * NOTE: on parcour l'ensemble des trajectoires du bacht pour trouver une trajectoire.
                                         * 
                                         */

                                        for(int i = 0; i < trajectory_registre[y].size(); i++)
                                        {
                                            trajectory_tested++;

                                            min_dist_obj_robot = 20;

                                            for(int ii = 0; ii < DirectMap_obs.size(); ii++)
                                            {
                                                // ETAPE 4 : POUR CHAQUE POINT PROJETER ON REGARDE SI ILS SONT DANS TRAJECTOIRE.
                                                // ETAPE 4A: ON DETERMINE LE CENTRE DU CERCLE. 
                                                // PS: ADD DIRECT A L'INIT
                                                if(trajectory_registre[y][i].r >= 0) center_row = 100 + abs(trajectory_registre[y][i].r) * 10;
                                                else{center_row = 100 - abs(trajectory_registre[y][i].r) * 10;}
                                                
                                                // ETAPE 4B : VERIFIER SI ILS SONT DANS TRAJECTOIRE
                                                dist_obj_center = sqrt(pow(100-DirectMap_obs[ii].x,2)+pow(center_row-DirectMap_obs[ii].y,2))/10;
                                                diff_dist_obj_robot_center = abs(dist_obj_center - abs(trajectory_registre[y][i].r));
                                                if(diff_dist_obj_robot_center < distance_m)
                                                {
                                                    // REVOIR
                                                    // dist_obj_robot = 2 * M_PI * abs(vect_traj[i].r) * ((180 - 2 * DirectMap_obs_diff_angle[i]) / 360);
                                                    dist_obj_robot = sqrt(pow(100 - DirectMap_obs[ii].x ,2) + pow(100 - DirectMap_obs[ii].y ,2)) / 10;


                                                    // ETAPE 4Ci : DETERMINE SI C LE POINT LE PLUS PROCHE
                                                    if(dist_obj_robot < min_dist_obj_robot) min_dist_obj_robot = dist_obj_robot;

                                                    // if(min_dist_obj_robot < activation_dist_vect[y]) break;
                                                }
                                            }

                                            trajectory_registre[y][i].pt_M = min_dist_obj_robot;

                                            if(Final_traj.pt_M <= trajectory_registre[y][i].pt_M)
                                            {
                                                Final_traj = trajectory_registre[y][i];
                                            }
                                        }

                                        /**
                                         * NOTE: Une fois l'ensemble du bacht traité c'est ok. Il faut voir si une trajectoire et valider ou non.
                                         * 
                                         * 1 - On parcours le tableau pour savoir si toute les données sont égale. (on compare au premier)
                                         * 2 - On note l'orientation actuelle de la trajectoire final ? (Peux être fait a la fin je pense)
                                         * 
                                         * NOTE: Si cela retourne la même valeur est qu'elle est différente de 20, cela veut dire que toute les trajectoires
                                         * percute le même obstacle.
                                         */
                                        
                                        bool same_dist_bool = true;
                                        double same_dist_detection = trajectory_registre[y][0].pt_M;
                                        for(int i = 0; i < trajectory_registre[y].size(); i++)
                                        {
                                            // [1]
                                            if(same_dist_detection != trajectory_registre[y][i].pt_M)
                                            {
                                                same_dist_bool = false;
                                            }

                                            // [2]
                                            // if(traj.r == Final_traj.r) 
                                            // {
                                            //     no_obs_traj = true;
                                                
                                            //     if(traj.r >= 0) memo_side = 1;
                                            //     else{memo_side = -1;}
                                            // }
                                        }

                                        /**
                                         * NOTE: Si on a plusieurs trajectoires avec la distance (20) il faut faire un choix.
                                         * 1 - On utilise la distance à la courbe parfaite.
                                         */
                                        
                                        int best_dist_orig = Final_traj.niv;
                                        if(Final_traj.pt_M == 20)
                                        {
                                            for(int i = 0; i < trajectory_registre[y].size(); i++)
                                            {
                                                if(trajectory_registre[y][i].pt_M == 20 && trajectory_registre[y][i].niv < best_dist_orig)
                                                {
                                                    best_dist_orig = trajectory_registre[y][i].niv;
                                                    Final_traj = trajectory_registre[y][i];
                                                    // std::cout << " [C] Changement." << std::endl;
                                                }
                                            }
                                        }

                                        /**
                                         * NOTE: A la différence de l'autre algorythme, on peux choisir de n'en prendre aucune, même si elle
                                         * ont toute une distance to robot différentes.
                                         */

                                        if(y == 0 && Final_traj.pt_M >= activation_dist_vect[y]) trajectory_found_in_bash = true;
                                        if(y == 1 && Final_traj.pt_M >= activation_dist_vect[y]) trajectory_found_in_bash = true;
                                        if(y == 2 && Final_traj.pt_M >= activation_dist_vect[y]) trajectory_found_in_bash = true;
                                        if(y == 3 && Final_traj.pt_M >= activation_dist_vect[y]) trajectory_found_in_bash = true;

                                        /**
                                         * NOTE: Rajout test.
                                         * 
                                         */

                                        // if(y == 0 && Final_traj.niv > 7) trajectory_found_in_bash = false;
                                        // if(y == 1 && Final_traj.niv > 8) trajectory_found_in_bash = false;
                                        // if(y == 2 && Final_traj.niv > 15) trajectory_found_in_bash = false;

                                        /**
                                         * NOTE:
                                         * On veut être sur que si plusieurs trajectoire on la même distance à un point, 
                                         * de prendre celle la plus proche de l'index.
                                         */

                                        if(trajectory_found_in_bash)
                                        {
                                            for(int i = 0; i < trajectory_registre[y].size(); i++)
                                            {
                                                if(Final_traj.pt_M == trajectory_registre[y][i].pt_M)
                                                {
                                                    if(Final_traj.niv > trajectory_registre[y][i].niv)
                                                    {
                                                        Final_traj = trajectory_registre[y][i];
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    // END ALGORYTHME.
                                }

                                std::cout << "count traj tested: " << trajectory_tested << " total time : " << get_elapsed_time(get_curr_timestamp(), start_trajectory_selection_ts) << std::endl;
                                if(trajectory_found_in_bash) 
                                {
                                    // std::cout << Final_traj.r << " " << Final_traj.pt_M << " " << Final_traj.security_dist << std::endl;                                     
                                    if(Final_traj.r >= 0) memo_side = 1;
                                    else{memo_side = -1;}
                                }
                                else
                                {
                                 //   std::cout << "[!] Aucune trajectoire trouver [!]" << std::endl;
                                }
                                // CHECK IF LE MEILLEUR A UNE TAILLE DE 20 PRENDRE LE PLUS PROCHE DE L4ORIGNIAL.
                                // int best_dist_orig = Final_traj.niv;
                                // if(Final_traj.pt_M == 20)
                                // {
                                //     for(int i = 0; i < vect_traj.size(); i++)
                                //     {
                                //         if(vect_traj[i].pt_M == 20 && vect_traj[i].niv < best_dist_orig)
                                //         {
                                //             best_dist_orig = vect_traj[i].niv;
                                //             Final_traj = vect_traj[i];
                                //             // std::cout << " [C] Changement." << std::endl;
                                //         }
                                //     }
                                // }

                                /**
                                 * NOTE: Etape 5
                                 * On va transformer la trajectoire selectionner en commande moteur.
                                 */
                                bool final_obstacle_detected = false;
                                if(first_trajectory_safe || (Final_traj.r != 0.0))
                                {
                                    if(first_trajectory_safe)
                                    {
                                        if(diff_angle > 0)
                                        {
                                          memo_side = 1;  
                                          final_side = 1;
                                        }
                                        else
                                        {
                                            memo_side = -1;
                                            final_side = -1;
                                        }

                                        speed_with_obj = Final_traj.max_speed;
                                    }

                                    if(Final_traj.r != 0.0) 
                                    {
                                        final_radius = abs(Final_traj.r);
                                        
                                        if(Final_traj.r >= 0) final_side = 1;
                                        else{ final_side = -1;}
                                    }

                                    /**
                                     * NOTE:
                                     * En fonction de la trajectoire final, on va venir modifier la vitesse max du robot, 
                                     * cependant il faut faire attention à ne pas "effacer" la diminution de vitesse du au obstacle.
                                     */
                                    
                                    // std::coutt << Final_traj.max_speed << " " << speed_with_obj << std::endl;
                                    
                                    if(speed_with_obj > Final_traj.max_speed) speed_with_obj = Final_traj.max_speed;
                                    if(speed_with_obj > curr_max_speed) speed_with_obj = curr_max_speed;
                                    std::cout << speed_with_obj << " " << curr_max_speed << std::endl;
                                    speed_with_obj += std::stod(get_redis_str(&redis, "NAV_OPERATOR_MAX_SPEED_BONUS"));

                                    /**
                                     * NOTE:
                                     * A ce niveau voici les valeurs optimals pour effectuer la trajectoire souhaité.
                                     * Cependant il faut filtrer les valeurs.
                                     */

                                    rapport_int_ext    = 2 * M_PI * (final_radius + std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2) / speed_with_obj;
                                    speed_int_with_obj = 2 * M_PI * (final_radius - std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2) / rapport_int_ext;
                                    
                                    /**
                                     * NOTE:
                                     * Afin de diminuer les frottements du au systême de 6 roues, les 4 roues diagonal doivent tourner
                                     * plus vite que les 2 roues centrales.
                                     */
                                    double r_ext_diag = sqrt(pow((final_radius + std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2),2) + pow(std::stod(get_redis_str(&redis, "HARD_WHEEL_SEPARATION")),2));
                                    double r_int_diag = sqrt(pow((final_radius - std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2),2) + pow(std::stod(get_redis_str(&redis, "HARD_WHEEL_SEPARATION")),2));
                                    double speed_ext_diag = 2 * M_PI * r_ext_diag / rapport_int_ext;
                                    double speed_int_diag = 2 * M_PI * r_int_diag / rapport_int_ext;

                                    // std::cout << "RADIUS: " << r_ext_diag << " " << r_int_diag << " SPEEDEXT: " << speed_ext_diag << " " << speed_with_obj << " SPEEDINT: " << speed_int_diag << " " << speed_int_with_obj << std::endl;

                                    /**
                                     * NOTE:
                                     * On calcul les commandes optimals.
                                     */

                                    std::vector<double> optimal_command_vect;
                                    if(final_side == 1)
                                    {
                                        // les valeurs de i non pas vraiment d'interet. Leur ordre oui.
                                        for(int i = 0; i < 3; i++)
                                        {
                                            if(i==1) optimal_command_vect.push_back(speed_with_obj);
                                            else
                                            {
                                                optimal_command_vect.push_back(speed_ext_diag);
                                            }
                                        }
                                        for(int i = 3; i < 6; i++)
                                        {
                                            if(i==4) optimal_command_vect.push_back(speed_int_with_obj);
                                            else
                                            {
                                                optimal_command_vect.push_back(speed_int_diag);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        for(int i = 3; i < 6; i++)
                                        {
                                            if(i==4) optimal_command_vect.push_back(speed_int_with_obj);
                                            else
                                            {
                                                optimal_command_vect.push_back(speed_int_diag);
                                            }
                                        }
                                        for(int i = 0; i < 3; i++)
                                        {
                                            if(i==1) optimal_command_vect.push_back(speed_with_obj);
                                            else
                                            {
                                                optimal_command_vect.push_back(speed_ext_diag);
                                            }
                                        }
                                    }

                                    // std::cout << speed_with_obj << " " << speed_int_with_obj << " " << Final_traj.max_speed << " " << Final_traj.r << " " << first_trajectory_safe << std::endl;

                                    std::vector<double> final_command_vector;
                                    for(int j = 0; j < 6; j++) final_command_vector.push_back(0.0);

                                    for(int j = 0; j < 6; j++)
                                    {
                                        double diff = last_command_motor_double[j] - optimal_command_vect[j];
                                        if(diff > 0)
                                        {
                                            /* Ce moteur doit ralentir.*/
                                            if(diff > max_deccel/30) final_command_vector[j] = last_command_motor_double[j] - (max_deccel/30);
                                            else
                                            {
                                                final_command_vector[j] = optimal_command_vect[j];
                                            }
                                            if(final_command_vector[j] < 0) final_command_vector[j] = 0;
                                        }
                                        if(diff < 0)
                                        {
                                            /* Ce moteur doit accelerer.*/
                                            if(abs(diff) > max_accel/30) final_command_vector[j] = last_command_motor_double[j] + (max_accel/30);
                                            else
                                            {
                                                final_command_vector[j] = optimal_command_vect[j];
                                            }
                                            if(final_command_vector[j] > speed_with_obj) final_command_vector[j] = speed_with_obj;
                                        }
                                        if(diff == 0)
                                        {
                                            /* Ce moteur doit maintenir.*/
                                            final_command_vector[j] = optimal_command_vect[j];
                                        }
                                    }

                                    /**
                                     * NOTE: Dans les virages les moteurs doivent garder un certain "rapport",
                                     * si l'ordre est 2.5 | 3.0, il ne faut pas que leur prograission soit de 
                                     * 0.5 | 0.5 > 1.0 | 1.0 
                                     * 
                                     * Dans ce cas il faut que le moteur le plus en avance garde le rapport.
                                     */
                                    double rapport = speed_with_obj / speed_int_with_obj;
                                    bool previous_to_left = true;
                                    if(last_command_motor_double[1] > last_command_motor_double[4]) previous_to_left = false;
                                    bool curr_to_left = true;
                                    if(final_command_vector[1] > final_command_vector[4]) curr_to_left = false;

                                    int previous_mode = 0;
                                    if(last_command_motor_double[0] >= 0 && last_command_motor_double[3] >= 0) previous_mode = 2;
                                    if(last_command_motor_double[0] <= 0 && last_command_motor_double[3] <= 0) previous_mode = 3;
                                    if(last_command_motor_double[0] == 0 && last_command_motor_double[3] == 0) previous_mode = 1;
                                    if(last_command_motor_double[0] <  0 && last_command_motor_double[3] >  0) previous_mode = 4;
                                    if(last_command_motor_double[0] >  0 && last_command_motor_double[3] <  0) previous_mode = 5;

                                 //   std::cout << "MODE : " << previous_mode << std::endl;

                                    std::string opti;
                                    for(int i = 0; i < 6; i++) opti += std::to_string(optimal_command_vect[i]) + "|";
                                    //std::coutt << opti << std::endl;

                                    std::string prop;
                                    for(int i = 0; i < 6; i++) prop += std::to_string(final_command_vector[i]) + "|";
                                    //std::coutt << prop << std::endl;

                                    // if(previous_mode == 4)
                                    // {
                                    //     /* Rotation vers la gauche. */

                                    //     /* Moteur de droite doivent ralentir pour attendre. On compute leur vitesse. */
                                    //     double transition_speed_ext = last_command_motor_double[3] - (max_accel/30);
                                    //     if(transition_speed_ext < 0.01) transition_speed_ext = 0.01;

                                    //     double transition_speed_central = last_command_motor_double[4] - (max_accel/30);
                                    //     if(transition_speed_central < 0.01) transition_speed_central = 0.01;

                                    //     if(final_command_vector[1] > 0) final_command_vector[1] = 0.0;

                                    //     final_command_vector[3] = transition_speed_ext;
                                    //     final_command_vector[4] = transition_speed_central;
                                    //     final_command_vector[5] = transition_speed_ext;

                                    // }
                                    // if(previous_mode == 5)
                                    // {
                                    //     /* Rotation vers la droite. */

                                    //     /* Moteur de gauche doivent ralentir pour attendre. On compute leur vitesse. */
                                    //     double transition_speed_ext = last_command_motor_double[0] - (max_accel/30);
                                    //     if(transition_speed_ext < 0.01) transition_speed_ext = 0.01;

                                    //     double transition_speed_central = last_command_motor_double[1] - (max_accel/30);
                                    //     if(transition_speed_central < 0.01) transition_speed_central = 0.01;

                                    //     if(final_command_vector[4] > 0) final_command_vector[1] = 0.0;

                                    //     final_command_vector[0] = transition_speed_ext;
                                    //     final_command_vector[1] = transition_speed_central;
                                    //     final_command_vector[2] = transition_speed_ext;
                                    // }
                                    if(previous_mode == 4 || previous_mode == 5)
                                    {
                                     //   std::cout << "Ralentir avant accélération." << std::endl;
                                        for(int j = 0; j < 6; j++)
                                        {
                                            double diff = last_command_motor_double[j];
                                            if(diff > 0)
                                            {
                                                /* Ce moteur doit ralentir.*/
                                                final_command_vector[j] = last_command_motor_double[j] - (max_deccel/30);

                                                if(final_command_vector[j] < 0) final_command_vector[j] = 0;
                                            }
                                            if(diff < 0)
                                            {
                                                /* Ce moteur doit accelerer.*/
                                                final_command_vector[j] = last_command_motor_double[j] + (max_accel/30);

                                                if(final_command_vector[j] > 0) final_command_vector[j] = 0;
                                            }
                                            if(diff == 0)
                                            {
                                                /* Ce moteur doit maintenir.*/
                                                final_command_vector[j] = 0.0;
                                            }
                                        }
                                    }
                                    if(previous_mode == 2)
                                    {
                                     //   std::cout << final_side << " r:" << Final_traj.r << " securi:" << Final_traj.security_dist <<  " index:" << Final_traj.niv << std::endl;
                                        if(final_side == -1)
                                        {
                                            /* Tout droit vers la gauche.*/
                                            double rapport_opti = optimal_command_vect[3] / optimal_command_vect[0];
                                            realig_rapport = rapport_opti;
                                            double rapport_reel = final_command_vector[3] / final_command_vector[0];
                                            double threshold_rapport = 0.02;

                                            if(abs(rapport_opti - rapport_reel) > threshold_rapport)
                                            {
                                                if(rapport_opti > rapport_reel)
                                                {
                                                    double perfect_speed = final_command_vector[0] / rapport_opti;
                                                    double new_speed;
                                                    if(abs(final_command_vector[0] - perfect_speed) > max_deccel/30)
                                                    // if(abs(final_command_vector[0] - perfect_speed) > 0)
                                                    {
                                                        new_speed = final_command_vector[0] - (max_deccel/30);
                                                    }
                                                    else
                                                    {
                                                        new_speed = perfect_speed;
                                                    }

                                                    final_command_vector[0] = new_speed;
                                                    final_command_vector[1] = new_speed;
                                                    final_command_vector[2] = new_speed;
                                                }
                                                // else
                                                // {
                                                //     double perfect_speed = final_command_vector[0] * rapport_opti;
                                                //     double new_speed;
                                                //     if(abs(final_command_vector[3] - perfect_speed) > max_deccel/30)
                                                //     // if(abs(final_command_vector[0] - perfect_speed) > 0)
                                                //     {
                                                //         new_speed = final_command_vector[3] + (max_deccel/30);
                                                //     }
                                                //     else
                                                //     {
                                                //         new_speed = perfect_speed;
                                                //     }

                                                //     final_command_vector[3] = new_speed;
                                                //     final_command_vector[4] = new_speed;
                                                //     final_command_vector[5] = new_speed;
                                                // }
                                            }
                                        }
                                        else
                                        {
                                            /* Tout droit vers la droite.*/
                                            double rapport_opti = optimal_command_vect[0] / optimal_command_vect[3];
                                            realig_rapport = rapport_opti;
                                            double rapport_reel = final_command_vector[0] / final_command_vector[3];
                                            double threshold_rapport = 0.02;

                                            if(rapport_opti > rapport_reel)
                                            {
                                                if(abs(rapport_opti - rapport_reel) > threshold_rapport)
                                                {
                                                    double perfect_speed = final_command_vector[0] / rapport_opti;
                                                    double new_speed;
                                                    if(abs(final_command_vector[3] - perfect_speed) > max_deccel/30)
                                                    // if(abs(final_command_vector[0] - perfect_speed) > 0)
                                                    {
                                                        new_speed = final_command_vector[3] - (max_deccel/30);
                                                    }
                                                    else
                                                    {
                                                        new_speed = perfect_speed;
                                                    }

                                                    final_command_vector[3] = new_speed;
                                                    final_command_vector[4] = new_speed;
                                                    final_command_vector[5] = new_speed;
                                                }
                                                // else
                                                // {
                                                //     double perfect_speed = final_command_vector[3] * rapport_opti;
                                                //     double new_speed;
                                                //     if(abs(final_command_vector[0] - perfect_speed) > max_deccel/30)
                                                //     // if(abs(final_command_vector[0] - perfect_speed) > 0)
                                                //     {
                                                //         new_speed = final_command_vector[0] + (max_deccel/30);
                                                //     }
                                                //     else
                                                //     {
                                                //         new_speed = perfect_speed;
                                                //     }

                                                //     final_command_vector[0] = new_speed;
                                                //     final_command_vector[1] = new_speed;
                                                //     final_command_vector[2] = new_speed;
                                                // }
                                            }
                                        }
                                    }

                                    if(realig_rapport == 0) realig_rapport = 1.0;
                                    
                                    //////////////////////////////////

                                    /**
                                     * NOTE:
                                     * Maintenant que la trajectoire est lisser et propre on doit verifier si
                                     * un obstacle ce trouve dedans.
                                     * 1 - On calcul le rayon du cercle que la trajectoire va faire.
                                     * 2 - On determine si des obtacles si trouve sans prendre en compte la zone 
                                     * de sécurité.
                                     */
                                    
                                    final_obstacle_detected = false;
                                    /* 1 - Trouver le rayon. */
                                    double A = speed_with_obj;
                                    double B = speed_int_with_obj;
                                    bool vers_la_gauche = true;

                                    if(final_command_vector[0] > final_command_vector[3])
                                    {
                                        A = final_command_vector[0];
                                        B = final_command_vector[3];
                                    }
                                    else
                                    {
                                        vers_la_gauche = false;
                                        A = final_command_vector[3];
                                        B = final_command_vector[0]; 
                                    }
                                    double c = (std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"))/2);
                                    double r = (-(A*c)-(B*c))/(B-A);

                                    /* 2 - On cherche des obstacles dedans. */
                                    double distance_m2 = (std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2) + 0.05;
                                    double dst_robot_obj = 0;
                                    for(int ii = 0; ii < DirectMap_obs.size(); ii++)
                                    {
                                        // ETAPE 4 : POUR CHAQUE POINT PROJETER ON REGARDE SI ILS SONT DANS TRAJECTOIRE.
                                        // ETAPE 4A: ON DETERMINE LE CENTRE DU CERCLE. 

                                        if(vers_la_gauche) center_row = 100 + abs(r) * 10;
                                        else{center_row = 100 - abs(r) * 10;}
                                        
                                        // ETAPE 4B : VERIFIER SI ILS SONT DANS TRAJECTOIRE
                                        dist_obj_center = sqrt(pow(100-DirectMap_obs[ii].x,2)+pow(center_row-DirectMap_obs[ii].y,2))/10;
                                        diff_dist_obj_robot_center = abs(dist_obj_center - abs(r));

                                        if(abs(diff_dist_obj_robot_center) < distance_m2)
                                        {
                                            dst_robot_obj = sqrt(pow(100-DirectMap_obs[ii].x,2)+pow(100-DirectMap_obs[ii].y,2))/10;
                                            if(dst_robot_obj < 1.0)
                                            {
                                                final_obstacle_detected = true;
                                                /* J'AI REMOVE !*/
                                             //std::coutt << "OBSTACLE DETECTED" << std::endl;
                                            }
                                        }
                                    }



                                    /* Fabriquer le messages string vect. */
                                    motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                    for(int j = 0; j < 6; j++)
                                    {
                                        motor_command_str += std::to_string(final_command_vector[j]) + "|";
                                        
                                    }
                                    //std::coutt << Final_traj.r << " " << motor_command_str << std::endl;


                                    // [SIM_CIRCLE NEW]

                                    /**
                                     * NOTE: Cette partie est uniquement pour la simulation. 
                                     */
                                    if(trajectory_found_in_bash) final_radius = Final_traj.r;
                                    else
                                    {
                                        final_radius = 0.32; 
                                    }
                                    Geographic_point center_circle = Geographic_point(0.0,0.0);
                                    if(final_side == -1)
                                    {
                                        center_circle = get_new_position(curr_position.point, curr_position.g_hdg - 90, final_radius);
                                    }
                                    if(final_side == 1)
                                    {
                                        center_circle = get_new_position(curr_position.point, curr_position.g_hdg + 90, final_radius);
                                    }
                                    set_redis_var(&redis, "SIM_AUTO_PT_ICC_NEW", std::to_string(center_circle.longitude) + "|" + std::to_string(center_circle.latitude) + "|");
                                    set_redis_var(&redis, "SIM_AUTO_RADIUS_ICC_NEW", std::to_string(abs(final_radius)));
                                    // [END SIM_CIRCLE NEW]
                                }
                                else
                                {
                                 //   std::cout << "NE DOIT PAS ARRIVER " << get_curr_timestamp() << std::endl;
                                }

                                // ETAPE 8 : SECURITY
                                if(((!first_trajectory_safe) && (!trajectory_found_in_bash)) && final_obstacle_detected)
                                {
                                    // std::cout << "[O] SECURITY" << std::endl;

                                    if((((!first_trajectory_safe) && (!trajectory_found_in_bash)) && !recul_forcer) || (final_obstacle_detected && !recul_forcer))
                                    {
                                        // std::cout << same_dist_detection << std::endl;
                                        if(diff_angle >= 0)memo_side = 1;
                                        else{memo_side = -1;}
                                        start_recul = get_curr_timestamp();
                                        recul_forcer = true;
                                    }
                                }
                                else
                                {
                                    // std::cout << "LET'S MAKE CODE CRASH" << std::endl;
                                }

                                // ETAPE 8 SECURITY
                                if(recul_forcer)
                                {
                                    if(time_is_over(get_curr_timestamp(), start_recul, 200))
                                    {
                                        recul_forcer = false;
                                    }
                                    else
                                    {
                                        double back_speed_max = -0.2;

                                        motor_command_str = std::to_string(get_curr_timestamp()) + "|";

                                        int previous_mode = 0;
                                        if(last_command_motor_double[0] >= 0 && last_command_motor_double[3] >= 0) previous_mode = 2;
                                        if(last_command_motor_double[0] <= 0 && last_command_motor_double[3] <= 0) previous_mode = 3;
                                        if(last_command_motor_double[0] == 0 && last_command_motor_double[3] == 0) previous_mode = 1;
                                        if(last_command_motor_double[0] <  0 && last_command_motor_double[3] >  0) previous_mode = 4;
                                        if(last_command_motor_double[0] >  0 && last_command_motor_double[3] <  0) previous_mode = 5;

                                        if(previous_mode == 2)
                                        {
                                            std::vector<double> opti_command; // opti : dans le sens pour frainer le plus vite.
                                            for(int j = 0; j < 6; j++)
                                            {
                                                double v = last_command_motor_double[j] - (max_deccel/30);
                                                if(v < 0) v = 0;
                                                opti_command.push_back(v);
                                            }

                                            bool rot_left = true;
                                            if(last_command_motor_double[0] - last_command_motor_double[3] < 0) rot_left = false;

                                            double rapport_opti = realig_rapport;

                                            if(rot_left)
                                            {
                                                // les moteurs moins rapides doivent frainer plus doucement car ceux rapide fraine
                                                // au max.
                                                double good_speed_right = opti_command[0] / rapport_opti;
                                                if(abs(opti_command[3] - good_speed_right) > (max_accel/30))
                                                {
                                                    opti_command[0] += (max_accel/30);
                                                    opti_command[1] += (max_accel/30);
                                                    opti_command[2] += (max_accel/30);
                                                }
                                                else
                                                {
                                                    opti_command[3] = good_speed_right;
                                                    opti_command[4] = good_speed_right;
                                                    opti_command[5] = good_speed_right;
                                                }
                                            }
                                            else
                                            {
                                                double good_speed_left = opti_command[3] / rapport_opti;
                                                if(abs(opti_command[0] - good_speed_left) > (max_accel/30))
                                                {
                                                    opti_command[0] += (max_accel/30);
                                                    opti_command[1] += (max_accel/30);
                                                    opti_command[2] += (max_accel/30);
                                                }
                                                else
                                                {
                                                    opti_command[0] = good_speed_left;
                                                    opti_command[1] = good_speed_left;
                                                    opti_command[2] = good_speed_left;
                                                }
                                            }
                                            
                                            for(int i = 0; i < 6; i++) motor_command_str += std::to_string(opti_command[i]) + "|";
                                        }
                                        else
                                        {
                                            for(int j = 0; j < 6; j++)
                                            {
                                                double v = 0;
                                                if(last_command_motor_double[j] < 0) v = last_command_motor_double[j] - ((max_accel*1.0)/30);
                                                else
                                                {
                                                    /* Il doit frainer avant la marche arrière, mais de la bonne manière en respectant le ratio. */
                                                    v = last_command_motor_double[j] - ((max_deccel*1.0)/30);
                                                }
                                                if(v < back_speed_max) v = back_speed_max;
                                                motor_command_str += std::to_string(v) + "|";
                                            }
                                        }
                                    }
                                }
                             //   std::cout << motor_command_str << std::endl;

                                // ETAPE 9 : Pour ne pas faire crash le code.
                                std::vector<std::string> vect_verif;
                                get_multi_str(motor_command_str, vect_verif);
                                if(vect_verif.size() <= 2)
                                {
                                    double back_speed_max = -0.2;
                                 //   std::cout << "BUG BUG BUG" << std::endl << std::endl << std::endl << std::endl;
                                    motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                                    for(int j = 0; j < 6; j++)
                                    {
                                        double v = 0;
                                        if(last_command_motor_double[j] < 0) v = last_command_motor_double[j] - ((max_accel*1.0)/30);
                                        else
                                        {
                                            v = last_command_motor_double[j] - ((max_accel*1.0)/30);
                                        }
                                        if(v < back_speed_max) v = back_speed_max;
                                        motor_command_str += std::to_string(v) + "|";
                                    }
                                }
                            }       
                        }
                    }
                }
                else
                {
                    vect_geo_save.clear();
                }
            }
        }
        
        //==============================================
        // DEBUG NAVIGATION : 
        // Utilisation de OPENCV pouvant mener à des bugs.
        //==============================================

        if(show_debug)
        {
            cv::Mat copy = debug_directmap.clone();

            // ADD CIRCLE
            std::vector<std::string> vect_str_redis;
            get_redis_multi_str(&redis, "SIM_AUTO_PT_ICC_NEW", vect_str_redis);
            Geographic_point pt_circle = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));

            // std::cout << vect_obj.size() << std::endl;
            double radius = std::stoi(get_redis_str(&redis, "SIM_AUTO_RADIUS_ICC_NEW"));
            // double radius = std::stod(get_redis_str(&redis, "DEB_R"));

            //REMOVE
            // radius = std::stod(get_redis_str(&redis, "angle"));
            // double a = std::stod(get_redis_str(&redis, "dist"));

            double row_idx = 0;
            if(final_side > 0) row_idx = 100 + abs(radius) * 10;
            else{ row_idx = 100 - abs(radius) * 10;}

            if(abs(radius) > 10000) radius = 10000;
            if(row_idx < 0.005 && row_idx > -0.005) row_idx = 0;

            // std::cout << vect_obj.size() << std::endl;
            double distance_m = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2 + std::stod(get_redis_str(&redis, "NAV_OBJ_SAFETY_DIST_M"));
            distance_m = memory_dist;
            // double distance_m = std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE")) / 2 + a;
            
            // std::cout << row_idx << " " << radius << " " << distance_m << " " << copy.cols << std::endl;
            cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)(abs(radius)*10)             , cv::Scalar(178, 102, 255)    , 1, cv::LineTypes::LINE_8);
            cv::circle(copy, cv::Point((int)(100),(int)(row_idx)), (int)((distance_m+abs(radius))*10), cv::Scalar(178, 102/2, 255/2), 1, cv::LineTypes::LINE_8);
            // std::cout << vect_obj.size() << std::endl;
            double de;
            if(abs(radius)-distance_m < 0 ){row_idx = -row_idx; de = abs(abs(radius)-distance_m);}
            else { de = abs(radius)-distance_m;}
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
            // std::cout << vect_obj.size() << std::endl;
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
                long double speed_ms = get_max_speed(&redis, "AUTO", "NO_MODE", vect_road);;
                
                if(obj.available)
                {
                    if(dist < speed_ms*2.0)
                    {
                        cv::circle(copy, cv::Point((int)(idx_col),(int)(idx_row)),0, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                    }
                    else
                    {
                        cv::circle(copy, cv::Point((int)(idx_col),(int)(idx_row)),0, cv::Scalar(0,200,0), cv::FILLED, 1,0);
                    }
                    if(abs(sqrt(pow(100-idx_col,2)+pow(row_idx-idx_row,2))/10 - radius) < distance_m)
                    {
                        cv::circle(copy, cv::Point((int)(idx_col),(int)(idx_row)),0, cv::Scalar(0,0,0), cv::FILLED, 1,0);
                    }
                }
                else
                {
                    cv::circle(copy, cv::Point((int)(idx_col),(int)(idx_row)),0, cv::Scalar(100,100,100), cv::FILLED, 1,0);
                }
            }

            cv::circle(copy, cv::Point((int)(deb_x),(int)(deb_y)),1, cv::Scalar(0,0,0), cv::FILLED, 1,0);
            // ADD ROBOT
            cv::circle(copy, cv::Point((int)(100),(int)(100)),2, cv::Scalar(0,0,0), cv::FILLED, 1,0);

            cv::imshow("DEBUG_DIRECT", copy);
            char d =(char)cv::waitKey(25);
        }
        
        //==============================================
        // COLISION DETECTION : 
        // Va vérifier que le robot n'ai pas entrain de se
        // mettre en position de contact/choc.
        //==============================================

        if(false)
        {
            if(collision_risk == 1)
            {
                // colision frontal.
                std::vector<std::string> vect_motor_command;
                get_multi_str(motor_command_str, vect_motor_command);

                int compteur_direction = 0;

                for(int i = 1; i < vect_motor_command.size(); i++)
                {
                    if(std::stoi(vect_motor_command[i]) > 0) compteur_direction++;
                }

                if(compteur_direction == 6)
                {
                    set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                    pub_redis_var(&redis, "EVENT", get_event_str(4, "ERR", "3_COLISION_DETECTED_BEFORE_CRASH_FRONT"));
                    set_redis_var(&redis, "MISSION_AUTO_STATE"    , "PAUSE");
                    set_redis_var(&redis, "MISSION_MANUAL_STATE"  , "PAUSE");
                }
            }
            if(collision_risk == 2)
            {
                // colision arrière.
                std::vector<std::string> vect_motor_command;
                get_multi_str(motor_command_str, vect_motor_command);

                int compteur_direction = 0;

                for(int i = 1; i < vect_motor_command.size(); i++)
                {
                    if(std::stoi(vect_motor_command[i]) < 0) compteur_direction++;
                }

                if(compteur_direction == 6)
                {
                    set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");
                    pub_redis_var(&redis, "EVENT", get_event_str(4, "ERR", "3_COLISION_DETECTED_BEFORE_CRASH_BACK"));

                    set_redis_var(&redis, "MISSION_AUTO_STATE"    , "PAUSE");
                    set_redis_var(&redis, "MISSION_MANUAL_STATE"  , "PAUSE");
                }
            }
        }

        //==============================================
        // MOTOR BRAKE SECURITY : 
        // Permet de stoper le robot net en cas de demande
        // du programme.
        //==============================================

        if(get_redis_str(&redis, "MISSION_MOTOR_BRAKE").compare("TRUE") == 0)
        {
            // motor_command_str = std::to_string(get_curr_timestamp()) + "|0.0|0.0|0.0|0.0|0.0|0.0|";

            double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));
            double max_accel  = std::stod(get_redis_str(&redis,"NAV_MAX_ACCEL"));

            std::vector<std::string> last_command_motor;
            get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", last_command_motor);

            motor_command_str = std::to_string(get_curr_timestamp()) + "|";

            std::vector<double> last_command_motor_double;
            last_command_motor_double.push_back(std::stod(last_command_motor[1]));
            last_command_motor_double.push_back(std::stod(last_command_motor[2]));
            last_command_motor_double.push_back(std::stod(last_command_motor[3]));
            last_command_motor_double.push_back(std::stod(last_command_motor[4]));
            last_command_motor_double.push_back(std::stod(last_command_motor[5]));
            last_command_motor_double.push_back(std::stod(last_command_motor[6]));

            /**
             * NOTE: 
             * A ce moment le robot doit frainer avant de se realigner, le probleme
             * c'est qu'il doit conserver le rapport qu'il avait avant le réalignement
             * car si on diminue de manière uniforme les moteurs, la courbe qu'il était 
             * entrain de faire va augmenter et causer des problemes.
             */

            int previous_mode = 0;
            if(last_command_motor_double[0] >= 0 && last_command_motor_double[3] >= 0) previous_mode = 2;

            if(previous_mode == 2)
            {
                std::vector<double> opti_command; // opti : dans le sens pour frainer le plus vite.
                for(int j = 0; j < 6; j++)
                {
                    double v = last_command_motor_double[j] - (max_deccel/30);
                    if(v < 0) v = 0;
                    opti_command.push_back(v);
                }

                bool rot_left = true;
                if(last_command_motor_double[0] - last_command_motor_double[3] < 0) rot_left = false;

                double rapport_opti = realig_rapport;

                if(rot_left)
                {
                    // les moteurs moins rapides doivent frainer plus doucement car ceux rapide fraine
                    // au max.
                    double good_speed_right = opti_command[0] / rapport_opti;
                    if(abs(opti_command[3] - good_speed_right) > (max_accel/30))
                    {
                        opti_command[0] += (max_accel/30);
                        opti_command[1] += (max_accel/30);
                        opti_command[2] += (max_accel/30);
                    }
                    else
                    {
                        opti_command[3] = good_speed_right;
                        opti_command[4] = good_speed_right;
                        opti_command[5] = good_speed_right;
                    }
                }
                else
                {
                    double good_speed_left = opti_command[3] / rapport_opti;
                    if(abs(opti_command[0] - good_speed_left) > (max_accel/30))
                    {
                        opti_command[0] += (max_accel/30);
                        opti_command[1] += (max_accel/30);
                        opti_command[2] += (max_accel/30);
                    }
                    else
                    {
                        opti_command[0] = good_speed_left;
                        opti_command[1] = good_speed_left;
                        opti_command[2] = good_speed_left;
                    }
                }
                
                for(int i = 0; i < 6; i++) motor_command_str += std::to_string(opti_command[i]) + "|";
            }
            else
            {
                double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));
                for(int j = 0; j < 6; j++)
                {
                    double v;
                    if(last_command_motor_double[j] > 0)
                    {
                        v = last_command_motor_double[j] - ((max_deccel*1.2)/30);
                        if(v < 0) v = 0;
                    }
                    else
                    {
                        v = last_command_motor_double[j] + ((max_deccel*1.2)/30);
                        if(v > 0) v = 0;
                    }
                    
                    motor_command_str += std::to_string(v) + "|";
                }
            }
        }

        //==============================================
        // PROTECTION CRASH : 
        // Permet de proteger contre pas de selection de trajectoire
        // en mode automatique.
        //==============================================

        if(true)
        {
            std::vector<std::string> vect_control;
            get_multi_str(motor_command_str, vect_control);
            double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));
            double max_accel  = std::stod(get_redis_str(&redis,"NAV_MAX_ACCEL"));

            if(vect_control.size() <= 2)
            {
             //   std::cout << "[!][!] L'algorythme à commis une erreur que le code à pu corriger. [!][!]" << std::endl;
                std::vector<std::string> last_command_motor;
                get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", last_command_motor);
                motor_command_str = std::to_string(get_curr_timestamp()) + "|";

                std::vector<double> last_command_motor_double;
                last_command_motor_double.push_back(std::stod(last_command_motor[1]));
                last_command_motor_double.push_back(std::stod(last_command_motor[2]));
                last_command_motor_double.push_back(std::stod(last_command_motor[3]));
                last_command_motor_double.push_back(std::stod(last_command_motor[4]));
                last_command_motor_double.push_back(std::stod(last_command_motor[5]));
                last_command_motor_double.push_back(std::stod(last_command_motor[6]));

                int previous_mode = 0;
                if(last_command_motor_double[0] >= 0 && last_command_motor_double[3] >= 0) previous_mode = 2;

                if(previous_mode == 2)
                {
                    std::vector<double> opti_command; // opti : dans le sens pour frainer le plus vite.
                    for(int j = 0; j < 6; j++)
                    {
                        double v = last_command_motor_double[j] - (max_deccel/30);
                        if(v < 0) v = 0;
                        opti_command.push_back(v);
                    }

                    bool rot_left = true;
                    if(last_command_motor_double[0] - last_command_motor_double[3] < 0) rot_left = false;

                    double rapport_opti = realig_rapport;

                    if(rot_left)
                    {
                        // les moteurs moins rapides doivent frainer plus doucement car ceux rapide fraine
                        // au max.
                        double good_speed_right = opti_command[0] / rapport_opti;
                        if(abs(opti_command[3] - good_speed_right) > (max_accel/30))
                        {
                            opti_command[0] += (max_accel/30);
                            opti_command[1] += (max_accel/30);
                            opti_command[2] += (max_accel/30);
                        }
                        else
                        {
                            opti_command[3] = good_speed_right;
                            opti_command[4] = good_speed_right;
                            opti_command[5] = good_speed_right;
                        }
                    }
                    else
                    {
                        double good_speed_left = opti_command[3] / rapport_opti;
                        if(abs(opti_command[0] - good_speed_left) > (max_accel/30))
                        {
                            opti_command[0] += (max_accel/30);
                            opti_command[1] += (max_accel/30);
                            opti_command[2] += (max_accel/30);
                        }
                        else
                        {
                            opti_command[0] = good_speed_left;
                            opti_command[1] = good_speed_left;
                            opti_command[2] = good_speed_left;
                        }
                    }
                    
                    for(int i = 0; i < 6; i++) motor_command_str += std::to_string(opti_command[i]) + "|";
                }
                else
                {
                    double max_deccel = std::stod(get_redis_str(&redis,"NAV_MAX_DECCEL"));
                    for(int j = 0; j < 6; j++)
                    {
                        double v;
                        if(last_command_motor_double[j] > 0)
                        {
                            v = last_command_motor_double[j] - ((max_deccel*1.2)/30);
                            if(v < 0) v = 0;
                        }
                        else
                        {
                            v = last_command_motor_double[j] + ((max_deccel*1.2)/30);
                            if(v > 0) v = 0;
                        }
                        
                        motor_command_str += std::to_string(v) + "|";
                    }
                }
            }

            // -nan protection.
            std::vector<std::string> vect;
            bool nan_detect = false;

            for (int i = 1; i < vect.size(); i++) 
            {
                double val = std::stod(vect[i]);
                if (std::isnan(val)) 
                {
                    vect[i] = "0.0";
                    nan_detect = true;
                }
            }

            if(nan_detect)
            {
                motor_command_str = std::to_string(get_curr_timestamp()) + "|";
                for(int i = 1; i < vect.size(); i++)
                {
                    motor_command_str += vect[i] + "|";
                }
            }
        }

        //==============================================
        // PUBLISH RESULT : 
        // Envoyer a redis la commande final qui sera lu
        // par le programme 02.
        //==============================================
        set_redis_var(&redis, "HARD_MOTOR_COMMAND", motor_command_str);
    }
}
