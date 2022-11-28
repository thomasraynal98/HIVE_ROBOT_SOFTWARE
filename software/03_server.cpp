#include "00_function.h"
#include "00_communication.h"
#include "00_navigation.h"

//[!!] OPTION FOR HTTP SERVER.
#include <sio_client.h>

//[!!] OPTION FOR HTTPS SERVER.
// #define ASIO_STANDALONE 
// #define SIO_TLS

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_event;
std::thread thread_server;
std::thread thread_telemetry;
std::thread thread_stream_video;

sio::client h;

int64_t timer_end         = get_curr_timestamp();
bool timer_active         = false;
int64_t timer_return_home = get_curr_timestamp();
bool timer_rh_active      = false;

//===================================================================
// CALLBACK WATCHER:
// Permet le traitement centraliser de l'ensemble des êvenements 
// interne du robot.
//===================================================================

void callback_command(std::string channel, std::string msg)
{
    set_redis_var(&redis, channel, msg);
    std::cout << "EVENT SUB : " << msg << std::endl;

    std::vector<std::string> vect_str;
    get_multi_str(msg, vect_str);

    /**
     * NOTE:
     * Un evenement est un object standardisé dans le robot :
     * A|B|C|D| :
     * A = le timestamp (en ms).
     * B = le numero qui indique d'ou l'event à était émis.
     * C = le titre de l'evenement.
     * D = des informations complémentaires de l'évenement.
     */

    if(vect_str[2].compare("MISSION_PAUSE")          == 0 && vect_str[3].compare("START") == 0)
    {
        send_event_server(h.socket(), "WAIT"          , "START");
    }
    if(vect_str[2].compare("MISSION_PAUSE")          == 0 && vect_str[3].compare("COMPLETED") == 0)
    {
        send_event_server(h.socket(), "WAIT"          , "SUCCESS");
    }
    if(vect_str[2].compare("MISSION_PAUSE")          == 0 && vect_str[3].compare("TIMER_END") == 0)
    {
        send_event_server(h.socket(), "TIMER"         , "END");
    }
    if(vect_str[2].compare("MISSION_AUTO_GOTO")      == 0 && vect_str[3].compare("START") == 0)
    {
        send_event_server(h.socket(), "GOTO"          , "START");
    }
    if(vect_str[2].compare("MISSION_AUTO_GOTO")      == 0 && vect_str[3].compare("SUCCESS") == 0)
    {
        send_event_server(h.socket(), "GOTO"          , "SUCCESS");
    }
    if(vect_str[2].compare("MISSION_AUTO_GOTO")      == 0 && vect_str[3].compare("NEED_OPERATOR_SUPERVISION") == 0)
    {
        send_event_server(h.socket(), "GOTO"          , "OPERATOR_SUPERVISION_REQUIRED");
    }
    if(vect_str[2].compare("MISSION_AUTO_GOTO")      == 0 && vect_str[3].compare("NEED_OPERATOR_DRIVE") == 0)
    {
        send_event_server(h.socket(), "GOTO"          , "OPERATOR_DRIVE_REQUIRED");
    }
    if(vect_str[2].compare("MISSION_AUTO_GOTO")      == 0 && vect_str[3].compare("NEED_OPERATOR_PARKING") == 0)
    {
        send_event_server(h.socket(), "GOTO"          , "OPERATOR_PARKING_REQUIRED");
    }
    if(vect_str[2].compare("MISSION_CANCEL_MISSION") == 0 && vect_str[3].compare("SUCCESS") == 0)
    {
        send_event_server(h.socket(), "CANCEL_MISSION", "SUCCESS");
    }
    if(vect_str[2].compare("BOX_OPEN")               == 0)
    {
        send_event_server(h.socket(), "BOX_OPEN"      , vect_str[3]);
    }
    if(vect_str[2].compare("BOX_CLOSE")              == 0)
    {
        send_event_server(h.socket(), "BOX_CLOSE"     , vect_str[3]);
    }
    if(vect_str[2].compare("BOX_TIME_OUT")           == 0)
    {
        send_event_server(h.socket(), "BOX_TIME_OUT"  , vect_str[3]);
    }
    if(vect_str[2].compare("ERR")                    == 0)
    {
        send_event_server(h.socket(), "ERR", vect_str[3]);
    }
}

void f_thread_event()
{
    auto sub = redis.subscriber();
    sub.on_message(callback_command);
    sub.subscribe("EVENT");
    while(true) {sub.consume();}
}

//===================================================================
// SERVER WATCHER:
// Permet le management du server, la connection et la deconnection.
//===================================================================

void f_thread_server()
{
    set_redis_var(&redis, "SERVER_COM_STATE", "DISCONNECTED");
    while(true)
    {
        if(!h.opened())
        {
            usleep(500000);

            if(get_redis_str(&redis, "SERVER_COM_STATE").compare("CONNECTED") == 0)
            {
                set_redis_var(&redis, "SERVER_COM_STATE", "DISCONNECTED");
                pub_redis_var(&redis, "EVENT", get_event_str(3, "DISCONNECTION_SERVER", "SUCCESS"));
                pub_redis_var(&redis, "EVENT", get_event_str(3, "CONNECTION_SERVER", "FAIL"));

                /**
                 * NOTE: 
                 * timer_rh_active est une fonction qui s'active lorsque le robot est déconnecté du
                 * server et permet de lancer un timer qui lancera automatiquement le retour à la maison
                 * du robot en cas de persistance du probleme de connection.
                 */
                timer_rh_active   = true;
                timer_return_home = get_curr_timestamp() + std::stoi(get_redis_str(&redis, "SERVER_MAX_TIME"));
            }

            try
            {
                h.connect(get_redis_str(&redis, "ROBOT_INFO_SERVER_ADRESS"));
                usleep(10000);
                
                // UNCOMMENT FOR HTTPS
                // std::vector<Server_var> vect_msg_server;
                // vect_msg_server.push_back(Server_var("s", "NAME" , get_redis_str(&redis, "ROBOT_INFO_PSEUDO")));
                // vect_msg_server.push_back(Server_var("i", "ID"   , get_redis_str(&redis, "ROBOT_INFO_ID"    )));
                // vect_msg_server.push_back(Server_var("s", "MODEL", get_redis_str(&redis, "ROBOT_INFO_MODEL" )));
                // send_msg_server(h.socket(), "ROBOT_ID", vect_msg_server);

                // UNCOMMENT FOR HTTP
                std::string robot_name = "Newt";
                h.socket()->emit("ROBOT_ID", robot_name);

                usleep(10000);
            }
            catch(...)
            {
                pub_redis_var(&redis, "EVENT", get_event_str(3, "CONNECTION_SERVER", "FAIL"));
            }
        }
        if(h.opened())
        {
            if(get_redis_str(&redis, "SERVER_COM_STATE").compare("DISCONNECTED") == 0)
            {
                bind_events(h.socket());
                set_redis_var(&redis, "SERVER_COM_STATE", "CONNECTED");
                pub_redis_var(&redis, "EVENT", get_event_str(3, "CONNECTION_SERVER", "SUCCESS"));

                timer_rh_active = false;
            }
            usleep(10000);
        }
        
        /**
         * NOTE:
         * Cette fonction ne peux être activer uniquement lorsque timer_rh_active est activé
         * CAD lorsque le robot à pu se connecter une fois mais qu'il n'arrive plus à acceder
         * au server.
         * 
         * Il rentre donc à ça position d'origine. Cependant il continue tout de même a tenter
         * de se connecter.
         */
        if(time_is_over(get_curr_timestamp(), timer_return_home) && timer_rh_active)
        {
            timer_rh_active = false;
            std::vector<std::string> vect_redis_str;
            get_redis_multi_str(&redis, "ROBOT_INFO_HOME_POSITION", vect_redis_str);

            std::string destination_str = std::to_string(get_curr_timestamp()) + "|";
            destination_str += vect_redis_str[0] + "|";
            destination_str += vect_redis_str[1] + "|";
            set_redis_var(&redis, "NAV_AUTO_DESTINATION",       destination_str);
            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",        "TRUE");
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "TRUE");
            set_redis_var(&redis, "ROBOT_MODE",                 "AUTO");
            set_redis_var(&redis, "MISSION_AUTO_TYPE",          "GOTO");
            set_redis_var(&redis, "MISSION_AUTO_STATE",         "START");

            pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_AUTO_GOTO_BACK_HOME", "START"));
        }
    }
}

//===================================================================
// TELEMETRY MANAGEMENT:
// Permet le management de la télémétrie et l'envoie d'information
// essentiel au server.
//===================================================================

void f_thread_telemetry()
{
    double ms_for_loop = frequency_to_ms(1);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        if(h.opened() && get_redis_str(&redis, "SERVER_COM_STATE").compare("CONNECTED") == 0)
        {
            std::vector<std::string> vect_str;
            get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_str);

            std::vector<Server_var> vect_telemetry_server;
            vect_telemetry_server.push_back(Server_var("d", "LONGITUDE"          ,                                       vect_str[1]));
            vect_telemetry_server.push_back(Server_var("d", "LATITUDE"           ,                                       vect_str[2]));
            vect_telemetry_server.push_back(Server_var("d", "HDG"                ,                                       vect_str[3]));
            vect_telemetry_server.push_back(Server_var("d", "VOLTAGE"            ,                                           "100.0"));
            vect_telemetry_server.push_back(Server_var("d", "SPEED"              ,                                             "0.0"));
            vect_telemetry_server.push_back(Server_var("s", "MODE"               ,               get_redis_str(&redis, "ROBOT_MODE")));

            if(get_redis_str(&redis, "ROBOT_MODE").compare("AUTO") == 0)
            {
                vect_telemetry_server.push_back(Server_var("s", "MISSION_TYPE"   ,        get_redis_str(&redis, "MISSION_AUTO_TYPE")));
                vect_telemetry_server.push_back(Server_var("s", "MISSION_STATE"  ,       get_redis_str(&redis, "MISSION_AUTO_STATE")));
            }
            if(get_redis_str(&redis, "ROBOT_MODE").compare("MANUAL") == 0)
            {
                vect_telemetry_server.push_back(Server_var("s", "MISSION_TYPE"   ,      get_redis_str(&redis, "MISSION_MANUAL_TYPE")));
                vect_telemetry_server.push_back(Server_var("s", "MISSION_STATE"  ,     get_redis_str(&redis, "MISSION_MANUAL_STATE")));
            }
            if(get_redis_str(&redis, "ROBOT_MODE").compare("INIT") == 0)
            {
                vect_telemetry_server.push_back(Server_var("s", "MISSION_TYPE"   ,                                  "INITIALISATION"));
                vect_telemetry_server.push_back(Server_var("s", "MISSION_STATE"  ,                                  "INITIALISATION"));
            }

            vect_telemetry_server.push_back(Server_var("d", "TEMPERATURE_1"      ,                                             "0.0"));
            vect_telemetry_server.push_back(Server_var("s", "MCU_MOTOR_STATE"    , get_redis_str(&redis, "HARD_MCU_MOTOR_COM_STATE")));
            vect_telemetry_server.push_back(Server_var("s", "MCU_CARGO_STATE"    , get_redis_str(&redis, "HARD_MCU_CARGO_COM_STATE")));
            vect_telemetry_server.push_back(Server_var("s", "MCU_INTER_STATE"    , get_redis_str(&redis, "HARD_MCU_INTER_COM_STATE")));
            vect_telemetry_server.push_back(Server_var("s", "PIXHAWK_STATE"      ,   get_redis_str(&redis, "HARD_PIXHAWK_COM_STATE")));

            vect_telemetry_server.push_back(Server_var("s", "PROC_SYSTEM"        ,    get_redis_str(&redis, "SOFT_PROCESS_ID_SYS_STATUS")));
            vect_telemetry_server.push_back(Server_var("s", "PROC_HARWARE"       ,   get_redis_str(&redis, "SOFT_PROCESS_ID_HARD_STATUS")));
            vect_telemetry_server.push_back(Server_var("s", "PROC_SERVER"        ,   get_redis_str(&redis, "SOFT_PROCESS_ID_SERV_STATUS")));
            vect_telemetry_server.push_back(Server_var("s", "PROC_NAVIGATION"    ,    get_redis_str(&redis, "SOFT_PROCESS_ID_NAV_STATUS")));
            vect_telemetry_server.push_back(Server_var("s", "PROC_PERCEPTION"    , get_redis_str(&redis, "SOFT_PROCESS_ID_PERCEP_STATUS")));

            vect_telemetry_server.push_back(Server_var("s", "GPS_SAT_NUMBER"     ,          get_redis_str(&redis, "HARD_GPS_NUMBER")));
            vect_telemetry_server.push_back(Server_var("s", "GPS_STATE_FIX"      ,       get_redis_str(&redis, "HARD_GPS_FIX_STATE")));

            vect_telemetry_server.push_back(Server_var("i", "TIME_TO_TARGET"     , get_redis_str(&redis, "MISSION_ESTI_TIME_TO_TARGET")));
            vect_telemetry_server.push_back(Server_var("i", "DIST_TO_TARGET"     , get_redis_str(&redis, "MISSION_ESTI_DIST_TO_TARGET")));

            send_msg_server(h.socket(), "ROBOT_TELEM", vect_telemetry_server);
            // std::cout << "TELEM SEND" << std::endl;
        }

        if(timer_active)
        {
            if(time_is_over(get_curr_timestamp(), timer_end))
            {
                timer_active = false;
                pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_PAUSE", "TIMER_END"));
            }
        }
    }
}

//===================================================================
// VIDEO STREAM MANAGEMENT:
// En fonction des options qui vont être choisis par le server, ce
// thread va remplir plus ou moins le msg ROBOT_STREAM.
//===================================================================

void f_thread_stream_video()
{
    auto next = std::chrono::high_resolution_clock::now();
    int _flag_stream = 0;

    int64_t tsp_last_cam1 = 0;
    int64_t tsp_last_cam2 = 0;

    while(true)
    {
        next += std::chrono::milliseconds((int)frequency_to_ms(std::stoi(get_redis_str(&redis, "SERVER_MAX_STREAM_VIDEO_HZ"))));
        std::this_thread::sleep_until(next);

        _flag_stream = std::stoi(get_redis_str(&redis, "NAV_OPT_STREAM"));

        if(_flag_stream == 1)
        {
            std::vector<Server_var> vect_stream_server;
            
            if(!is_same_time(tsp_last_cam1, std::stoul(get_redis_str(&redis, "ENCODED_CAM1_TIMESTAMP"))))
            {
                tsp_last_cam1 = std::stoul(get_redis_str(&redis, "ENCODED_CAM1_TIMESTAMP"));
                vect_stream_server.push_back(Server_var("s", "CHANNEL_1"      ,      get_redis_str(&redis, "ENCODED_CAM1")));
            }
            else
            {
                vect_stream_server.push_back(Server_var("s", "CHANNEL_1"      ,      "0"));
            }

            vect_stream_server.push_back(Server_var("s", "CHANNEL_2"      ,      "0"));
            send_msg_server(h.socket(), "ROBOT_STREAM", vect_stream_server);
        }

        if(_flag_stream == 2)
        {
            std::vector<Server_var> vect_stream_server;
            
            if(!is_same_time(tsp_last_cam1, std::stoul(get_redis_str(&redis, "ENCODED_CAM2_TIMESTAMP"))))
            {
                tsp_last_cam1 = std::stoul(get_redis_str(&redis, "ENCODED_CAM2_TIMESTAMP"));
                vect_stream_server.push_back(Server_var("s", "CHANNEL_2"      ,      get_redis_str(&redis, "ENCODED_CAM2")));
            }
            else
            {
                vect_stream_server.push_back(Server_var("s", "CHANNEL_2"      ,      "0"));
            }

            vect_stream_server.push_back(Server_var("s", "CHANNEL_1"      ,      "0"));
            send_msg_server(h.socket(), "ROBOT_STREAM", vect_stream_server);
        }

        if(_flag_stream == 3)
        {
            std::vector<Server_var> vect_stream_server;

            if(!is_same_time(tsp_last_cam1, std::stoul(get_redis_str(&redis, "ENCODED_CAM1_TIMESTAMP"))))
            {
                tsp_last_cam1 = std::stoul(get_redis_str(&redis, "ENCODED_CAM1_TIMESTAMP"));
                vect_stream_server.push_back(Server_var("s", "CHANNEL_1"      ,      get_redis_str(&redis, "ENCODED_CAM1")));
            }
            else
            {
                vect_stream_server.push_back(Server_var("s", "CHANNEL_1"      ,      "0"));
            }

            if(!is_same_time(tsp_last_cam2, std::stoul(get_redis_str(&redis, "ENCODED_CAM2_TIMESTAMP"))))
            {
                tsp_last_cam2 = std::stoul(get_redis_str(&redis, "ENCODED_CAM2_TIMESTAMP"));
                vect_stream_server.push_back(Server_var("s", "CHANNEL_2"      ,      get_redis_str(&redis, "ENCODED_CAM2")));
            }
            else
            {
                vect_stream_server.push_back(Server_var("s", "CHANNEL_2"      ,      "0"));
            }

            send_msg_server(h.socket(), "ROBOT_STREAM", vect_stream_server);
        }
    }
}

//===================================================================
// MAIN PROGRAM
//===================================================================

int main(int argc, char *argv[])
{
    set_redis_var(&redis, "SOFT_PROCESS_ID_SERV", std::to_string(getpid()));
   
    thread_event        = std::thread(&f_thread_event);
    thread_server       = std::thread(&f_thread_server);
    thread_telemetry    = std::thread(&f_thread_telemetry);
    thread_stream_video = std::thread(&f_thread_stream_video);

    thread_event.join();
    thread_server.join();
    thread_telemetry.join();
    thread_stream_video.join();

    return 0;
}

//===================================================================
// BIND_EVENTS : 
// Réception de données du server.
//===================================================================

void bind_events(sio::socket::ptr current_socket)
{
    // TODO: setup ORDER_GET_HMR
    current_socket->on("ORDER_GET_HMR"       , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        // Recuperer le fichier txt en base64 sur la variable 'HMR'
        // set_redis_var(&redis, "NAV_HMR_DOWNLOAD_ADRESS", data->get_map()["HMR_LINK"]->get_string());
        // set_redis_var(&redis, "NAV_HMR_MAP_UPDATE",      "TRUE");
    }));

    current_socket->on("ORDER_WAITING"       , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        if(data->get_map()["IMPORTANT"]->get_bool())
        {
            if(data->get_map()["END_TIMESTAMP"]->get_int() != 0)
            {
                timer_end    = data->get_map()["END_TIMESTAMP"]->get_int();
                timer_active = true;
            }
        }

        set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");

        set_redis_var(&redis, "MISSION_AUTO_STATE",  "PAUSE");
        set_redis_var(&redis, "MISSION_MANUAL_STATE",  "PAUSE");

        pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_PAUSE", "START"));
    }));

    current_socket->on("ORDER_WAITING_END"   , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");

        set_redis_var(&redis, "MISSION_AUTO_STATE",  "IN_PROGRESS");
        set_redis_var(&redis, "MISSION_MANUAL_STATE",  "IN_PROGRESS");

        pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_PAUSE", "COMPLETED"));
    }));

    current_socket->on("ORDER_AUTONAV"       , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        /**
         * NOTE:
         * la valeur flag est très important est permet d'empecher le fonctionnement
         * du mode automatique lorsque les conditions optimal ne sont pas obtenue.
         * 
         * ex: problemes d'encoder, de com MCU, pas de caméra arrière...
         */
        int flag = auto_mode_available(&redis);
        
        // L'ordre AutoNav, coupe le timer.
        timer_active = false;

        if(flag == 10 || flag == 20)
        {
            set_redis_var(&redis, "NAV_AUTO_MODE_PARKING", data->get_map()["OPT_DEST"]->get_string());

            std::string destination_str = std::to_string(get_curr_timestamp()) + "|";
            destination_str += (data->get_map()["LONGITUDE"]->get_string()).substr(0, 9) + "|";
            destination_str += (data->get_map()["LATITUDE"]->get_string()).substr(0, 9) + "|";
            set_redis_var(&redis, "NAV_AUTO_DESTINATION",       destination_str);
            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",        "TRUE");
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "TRUE");
            set_redis_var(&redis, "ROBOT_MODE",                 "AUTO");
            set_redis_var(&redis, "MISSION_AUTO_TYPE",          "GOTO");
            set_redis_var(&redis, "MISSION_AUTO_STATE",         "START");

            pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_AUTO_GOTO", "START"));
        }
        else 
        {
            std::string destination_str = std::to_string(get_curr_timestamp()) + "|";
            destination_str += std::to_string(0.0) + "|";
            destination_str += std::to_string(0.0) + "|";
            set_redis_var(&redis, "NAV_AUTO_DESTINATION",       destination_str);
            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",        "TRUE");
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "FALSE");
            set_redis_var(&redis, "ROBOT_MODE",                 "MANUAL");
            set_redis_var(&redis, "MISSION_AUTO_TYPE",          "NO_VAL");
            set_redis_var(&redis, "MISSION_AUTO_STATE",         "NO_VAL");

            if(flag == -1)
            {
                pub_redis_var(&redis, "EVENT", get_event_str(3, "ERR", "MISSION_PARAM_AUTO_ENABLE_INVALID " + std::to_string(flag)));
            }
            else
            {
                pub_redis_var(&redis, "EVENT", get_event_str(3, "ERR", "AUTO_NOT_AVAILABLE " + std::to_string(flag)));
            }
        }
    }));

    current_socket->on("ORDER_MANUALNAV"     , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        /**
         * NOTE:
         * la valeur flag est très important est permet d'empecher le fonctionnement
         * du mode manuel lorsque les conditions optimal ne sont pas obtenue.
         * 
         * ex: problemes d'encoder, de com MCU, pas de caméra arrière...
         */
        int flag = manual_mode_available(&redis);

        if(flag == 10)
        {
            if(get_redis_str(&redis, "ROBOT_MODE").compare("AUTO") == 0) 
            {
                pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_MANUAL_MOVE", "START"));
            }

            std::string event_manual_controler_data_str = std::to_string(get_curr_timestamp()) + "|";
            event_manual_controler_data_str += std::to_string(data->get_vector()[0]->get_double()) + "|";
            event_manual_controler_data_str += std::to_string(data->get_vector()[1]->get_double()) + "|";
            event_manual_controler_data_str += std::to_string(data->get_vector()[2]->get_double()) + "|";
            set_redis_var(&redis, "EVENT_MANUAL_CONTROLER_DATA", event_manual_controler_data_str);

            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",  "FALSE");
            set_redis_var(&redis, "ROBOT_MODE",           "MANUAL");
            set_redis_var(&redis, "MISSION_MANUAL_TYPE",  "MANUAL_MOVE");
            set_redis_var(&redis, "MISSION_MANUAL_STATE", "IN_PROGRESS");
        }
        else
        {
            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",  "TRUE");
            set_redis_var(&redis, "ROBOT_MODE",           "MANUAL");
            set_redis_var(&redis, "MISSION_MANUAL_TYPE",  "MANUAL_MOVE");
            set_redis_var(&redis, "MISSION_MANUAL_STATE", "INTERRUPTED");
            pub_redis_var(&redis, "EVENT", get_event_str(3, "ERR", "MANUAL_NOT_AVAILABLE " + std::to_string(flag)));
        }
    }));

    current_socket->on("ORDER_HARDWARE"      , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        std::vector<std::string> vect_cargo_state;
        get_redis_multi_str(&redis, "HARD_CARGO_STATE", vect_cargo_state);

        std::vector<std::string> vect_cargo_mission;
        get_redis_multi_str(&redis, "MISSION_HARD_CARGO", vect_cargo_mission);

        int index = data->get_map()["ID_BOX"]->get_int();

        if(vect_cargo_state[index].compare("OPEN") == 0)
        {
            pub_redis_var(&redis, "EVENT", get_event_str(3, "ERR"      , "BOX " + std::to_string(index) + " ALREADY_OPEN"));
            pub_redis_var(&redis, "EVENT", get_event_str(3, "BOX_OPEN" , std::to_string(index)));
        }
        else
        {
            pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_OPEN_BOX" + std::to_string(index), "START"));
        }

        std::string new_mission_cargo_str = std::to_string(get_curr_timestamp()) + "|";

        for(int i = 1; i < 4; i++)
        {
            if(i == index) new_mission_cargo_str += "OPEN|";
            else
            {
                new_mission_cargo_str += vect_cargo_mission[i] + "|";
            }
        }

        set_redis_var(&redis, "MISSION_HARD_CARGO", new_mission_cargo_str);

    }));

    current_socket->on("ORDER_STREAM"        , sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "NAV_OPT_STREAM", std::to_string(data->get_map()["OPT_STREAM"]->get_int()));

        pub_redis_var(&redis, "EVENT", get_event_str(3, "STREAM_CHANGE_MODE", std::to_string(data->get_map()["OPT_STREAM"]->get_int())));
    }));

    current_socket->on("ORDER_CANCEL_MISSION", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "ROBOT_MODE",           "MANUAL");
        set_redis_var(&redis, "MISSION_MOTOR_BRAKE",  "TRUE");
        set_redis_var(&redis, "MISSION_AUTO_TYPE",    "NO_VAL");
        set_redis_var(&redis, "MISSION_AUTO_STATE",   "NO_VAL");
        set_redis_var(&redis, "MISSION_MANUAL_TYPE",  "WAITING");
        set_redis_var(&redis, "MISSION_MANUAL_STATE", "IN_PROGRESS");

        pub_redis_var(&redis, "EVENT", get_event_str(3, "MISSION_CANCEL_MISSION", "SUCCESS"));
    }));

    // TODO: setup ORDER_RESET_SOFTWARE
    current_socket->on("ORDER_RESET_SOFTWARE", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {}));
}