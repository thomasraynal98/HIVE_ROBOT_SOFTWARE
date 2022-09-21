#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <math.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>

#include <sw/redis++/redis++.h>
#include <sio_client.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

void init_redis_var(sw::redis::Redis* redis);
void set_redis_var(sw::redis::Redis* redis, std::string channel, std::string value);
void pub_redis_var(sw::redis::Redis* redis, std::string channel, std::string value);
std::string get_redis_str(sw::redis::Redis* redis, std::string channel);
int get_redis_multi_str(sw::redis::Redis* redis, std::string channel, std::vector<std::string>& stockage);
int get_multi_str(std::string str, std::vector<std::string>& vec_str);
int64_t get_curr_timestamp();
std::string get_event_str(int ID_event, std::string event_description, std::string event_info);
void read_yaml(sw::redis::Redis* redis, cv::FileStorage* file_mng, std::string channel);
double frequency_to_ms(int frequency);
bool time_is_over(int64_t curr_timestamp, int64_t ref_timestamp, int64_t max_duration_ms);
void print_redis(sw::redis::Redis* redis, std::string channel_str);
std::string get_standard_robot_id_str(sw::redis::Redis* redis);
bool compare_redis_var(sw::redis::Redis* redis, std::string channel, std::string compare);
