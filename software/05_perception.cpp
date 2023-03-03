#include "00_function.h"
#include "00_communication.h"

#include <chrono>
#include <iostream>
#include <optional>
#include <signal.h>
#include <thread>
#include <vector>
using namespace std;

#include "LD06Kit/lidarkit.hpp"
#include <librealsense2/rs.hpp>

cv::Mat debug_directmap(100, 100, CV_8UC1, cv::Scalar(255));
cv::Mat debug_directmap_clone(100, 100, CV_8UC1, cv::Scalar(255));


// /**
//  * TODO:
//  * [ ] 1545 - Detect caméra.
//  * [X] 1415 - Detect lidar.
//  * [X] 1600 - Lidar_change.
//  * [ ] 1700 - Build 2D map with caméra.
//  * [-] 1715 - Send to redis.
//  * [ ] 1200 - Use PCL.
// */
using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::string front_cam_id = get_redis_str(&redis, "HARD_CAM1_ID");

std::vector<std::string> esp_port;
std::vector<std::string> lidar_port;

std::thread thread_cam1;
std::thread thread_cam2;
std::thread thread_lid1;
std::thread thread_lid2;
std::thread thread_manager;

std::vector<Lidar_brut> clean_vect_1;
std::vector<Lidar_brut> clean_vect_2;
std::vector<Lidar_brut> clean_vect_1front;
std::vector<Lidar_brut> clean_vect_2front;

bool headless = false;

static bool is_running1 = true;
static bool is_running2 = true;

bool calibration_activate = false;
int step_calibration = 0;

void sigint_handler(int)
{
    is_running1 = false;
    is_running2 = false;
}

double inv_angle(double angle)
{
    if(angle < 90) return 360 - angle;
    if(angle > 270) return 360 - angle;
}

bool filter_lidar_brut(Lidar_brut data)
{
    if((data.angle < 85 || data.angle > 275) && data.dist > 0.30 && data.confidence > 100) return true;
    return false;
}

bool filter_lidar_brut_front(Lidar_brut data)
{
    if((data.angle < 5 || data.angle > 355) && data.dist > 0.30 && data.dist < 5.0) return true;
    return false;
}

void front_lidar_selection()
{
    float moy_dist_t1 = 0;
    int counter1 = 0;
    float moy_dist_t2 = 0;
    int counter2 = 0;

    for(int i = 0; i < clean_vect_1front.size(); i++)
    {
        if(filter_lidar_brut_front(clean_vect_1front[i]))
        {
            counter1++;
            moy_dist_t1 += clean_vect_1front[i].dist;
        }
    }
    clean_vect_1front.clear();
    moy_dist_t1 = moy_dist_t1 / counter1;

    for(int i = 0; i < clean_vect_2front.size(); i++)
    {
        if(filter_lidar_brut_front(clean_vect_2front[i]))
        {
            counter2++;
            moy_dist_t2 += clean_vect_2front[i].dist;
        }
    }
    clean_vect_2front.clear();
    moy_dist_t2 = moy_dist_t2 / counter2;

    std::cout << "thread 1 : " << moy_dist_t1 << "\n" << "thread 2 : " << moy_dist_t2 << std::endl;

    if(moy_dist_t1 > moy_dist_t2) 
    {
        std::string t1 = lidar_port[0];
        std::string t2 = lidar_port[1];
        lidar_port.clear();
        lidar_port.push_back(t2);
        lidar_port.push_back(t1);
        set_redis_var(&redis, "LIDAR_CHANGE", "false");
        step_calibration = 0;
        calibration_activate = false;
    }
    step_calibration = 0;
    calibration_activate = false;
    set_redis_var(&redis, "LIDAR_CHANGE", "false");
}

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/cloud_viewer.h>

double process_angle(double idx_c1, double idx_r1, double idx_c2, double idx_r2)
{
    // Description : Process angle between camera position and new data detection.
    return atan2(idx_r2 - idx_r1, idx_c2 - idx_c1);
}

void f_thread_cam1()
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    rs2::config cfg; 
    // cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 320, 180,rs2_format::RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 360,rs2_format::RS2_FORMAT_Z16, 30);

    // Configure and start the pipeline
    p.start(cfg);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    double y_down = 0.3;
    double y_up   = 0.1;

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        points = pc.calculate(depth);
        auto vertices = points.get_vertices();

        debug_directmap_clone = debug_directmap.clone();

        for(int i = 0; i < points.size(); i++)
        {
            if(-0.3 > vertices[i].y && vertices[i].z != 0 && vertices[i].z < 4.0 && -1.5 < vertices[i].y && abs(vertices[i].x) < 2.5)
            {
                cv::circle(debug_directmap_clone, cv::Point((int)(100-(vertices[i].x*20+50)),(int)(vertices[i].z*20)),1, cv::Scalar(debug_directmap_clone.at<uchar>((int)(vertices[i].z*20), (int)(100-(vertices[i].x*20+50)))-1), cv::FILLED, 0,0);
            }
        }

        std::string msg_redis = std::to_string(get_curr_timestamp()) + "|";

        int v = 50; // POINT PAR CELL
        v = std::stoi(get_redis_str(&redis, "UUU"));
        for(int j=0;j<debug_directmap_clone.rows;j++) 
        {
            for (int i=0;i<debug_directmap_clone.cols;i++)
            {
                if(debug_directmap_clone.at<uchar>(j,i) < v)
                {
                    debug_directmap_clone.at<uchar>(j,i) = 0; //black
                    msg_redis += "o|" + std::to_string(sqrt((double)(pow(50-i,2)+pow(0-j,2)))/20.0).substr(0,4) + "|" + std::to_string(rad_to_deg(process_angle(0,50,j,i))).substr(0,5) + "|";
                }
                else
                {
                    debug_directmap_clone.at<uchar>(j,i) = 255; //white
                }
            }
        }

        std::cout << points.size() << " timestamp : " << get_curr_timestamp() << std::endl;
        set_redis_var(&redis, "ENV_CAM1_OBJECTS", msg_redis);

        cv::namedWindow( "DEBUG_MDL_ENV_SENSING", 4);
        cv::imshow("DEBUG_MDL_ENV_SENSING", debug_directmap_clone);
        char d =(char)cv::waitKey(1);
    }

}

void f_thread_lid1()
{
    int counter = 0;

    while(true)
    {
        if(step_calibration == 0)
        {
            try {
                LidarKit lk(lidar_port[0]);
                if(lk.start()) is_running1 = true;

                
                std::this_thread::sleep_for(0.1s);
                while (is_running1) {

                    auto v = lk.get_points();
                    clean_vect_1.clear();

                    for (auto& p : v)
                    {
                        clean_vect_1.push_back(Lidar_brut(p.angle, p.distance, p.confidence));
                    } 

                    if(step_calibration == 1 || step_calibration == 2)
                    {
                        counter++;
                        for (auto& p : v)
                        {
                            clean_vect_1front.push_back(Lidar_brut(p.angle, p.distance, p.confidence));
                        } 
                    }
                    if(counter > 6) {is_running1 = false; counter = 0;}

                    std::this_thread::sleep_for(0.1s);

                    std::string redis_msg;
                    redis_msg = std::to_string(get_curr_timestamp()) + "|";
                    
                    for(int i = 0; i < clean_vect_1.size(); i++)
                    {
                        if(filter_lidar_brut(clean_vect_1[i]))
                        {
                            redis_msg += "o|" + std::to_string(clean_vect_1[i].dist).substr(0,4) + "|" + std::to_string(inv_angle(clean_vect_1[i].angle)).substr(0,5) + "|";
                        }
                    }
                    set_redis_var(&redis, "ENV_LID1_OBJECTS", redis_msg);
                }
                lk.stop();
                std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() + std::chrono::milliseconds((int)1000));
                if(step_calibration == 1 || step_calibration == 2) step_calibration++;
            } 
            catch (exception& e) 
            {} 
        }

        if(step_calibration == 3)
        {
            step_calibration = 4;
            front_lidar_selection();
        }
        else
        {
            std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() + std::chrono::milliseconds((int)500));
            std::cout << "WAIT " << step_calibration << std::endl;
        }
    }
}

void f_thread_lid2()
{
    // LIDAR ARRIERE

    int counter = 0;

    while(true)
    {
        if(step_calibration == 0)
        {
            try {
                LidarKit lk(lidar_port[1]);
                if(lk.start()) is_running2 = true;
                
                std::this_thread::sleep_for(0.1s);
                while (is_running2) {

                    auto v = lk.get_points();
                    clean_vect_2.clear();

                    for (auto& p : v)
                    {
                        clean_vect_2.push_back(Lidar_brut(p.angle, p.distance, p.confidence));
                    } 

                    if(step_calibration == 1 || step_calibration == 2)
                    {
                        counter++;
                        for (auto& p : v)
                        {
                            clean_vect_2front.push_back(Lidar_brut(p.angle, p.distance, p.confidence));
                        } 
                    }
                    if(counter > 6) {is_running2 = false; counter = 0;}

                    std::this_thread::sleep_for(0.1s);

                    std::string redis_msg;
                    redis_msg = std::to_string(get_curr_timestamp()) + "|";
                    
                    for(int i = 0; i < clean_vect_2.size(); i++)
                    {
                        if(filter_lidar_brut(clean_vect_2[i]))
                        {
                            redis_msg += "o|" + std::to_string(clean_vect_2[i].dist) + "|" + std::to_string(inv_angle(clean_vect_2[i].angle)) + "|";
                        }
                    }
                    set_redis_var(&redis, "ENV_LID2_OBJECTS", redis_msg);
                }
                lk.stop();
                std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() + std::chrono::milliseconds((int)1000));
                if(step_calibration == 1 || step_calibration == 2) step_calibration++;
                if(!calibration_activate) step_calibration = 0;
            } 
            catch (exception& e) 
            {std::cout << "JE PEUX PAS" << std::endl; } 
        }

        if(step_calibration == 3)
        {
            step_calibration = 4;
            front_lidar_selection();
        }
        else
        {
            std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() + std::chrono::milliseconds((int)500));
            std::cout << "WAITB " << step_calibration << std::endl;
        }
    }
}

void f_thread_manager()
{
    double ms_for_loop = frequency_to_ms(1);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        if(compare_redis_var(&redis, "LIDAR_CHANGE", "true")) 
        {
            set_redis_var(&redis, "LIDAR_CHANGE", "false");
            calibration_activate = true;
            step_calibration = 1;
        }
    }
}

int main(int argc, char *argv[])
{
    set_redis_var(&redis, "UUU", "50");

    // register signal handler, for smooth CTRL+C interrupt
    // signal(SIGINT, sigint_handler);
    if(argc == 2)
    {
        std::cout << "Mode Headless running." << std::endl;
        headless = true;
    } 
    else
    {
        std::cout << "Mode Debug running." << std::endl;
    }

    // if(compare_redis_var(&redis, "HARD_MCU_MOTOR_COM_STATE", "CONNECTED")) esp_port.push_back(get_redis_str(&redis, "HARD_MCU_MOTOR_PORT_NAME"));
    // if(compare_redis_var(&redis, "HARD_MCU_CARGO_COM_STATE", "CONNECTED")) esp_port.push_back(get_redis_str(&redis, "HARD_MCU_CARGO_PORT_NAME"));
    esp_port.push_back(get_redis_str(&redis, "HARD_MCU_MOTOR_PORT_NAME"));
    esp_port.push_back(get_redis_str(&redis, "HARD_MCU_CARGO_PORT_NAME"));

    std::string port_name = "/dev/ttyUSB";
    std::string tempo_port_name;
    for(int i = 0; i < 5; i++)
    {
        tempo_port_name = port_name + std::to_string(i);
        bool is_same = false;
        for(int j = 0; j < esp_port.size(); j++)
        {
            if(esp_port[j].compare(tempo_port_name) == 0) is_same = true;
        }
        if(!is_same && file_exist(tempo_port_name)) lidar_port.push_back(tempo_port_name);
    }

    for(int i = 0; i < lidar_port.size(); i++) std::cout << lidar_port[i] << std::endl;
    std::string cam1_id = get_redis_str(&redis, "HARD_CAM1_ID");

    thread_lid1 = std::thread(&f_thread_lid1);
    thread_lid2 = std::thread(&f_thread_lid2);
    thread_manager = std::thread(&f_thread_manager);
    thread_cam1 = std::thread(&f_thread_cam1);

    thread_lid1.join();
    thread_lid2.join();
    thread_manager.join();
    thread_cam1.join();
    
    return 0;
}