#include "00_function.h"
#include "00_communication.h"
#include "00_navigation.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

sio::client h;

//===================================================================
// DESCISION : Machine à état et gestion des événements.
//===================================================================

//===================================================================
// MAIN PROGRAM
//===================================================================

int main(int argc, char *argv[])
{
    try
    {
        h.connect(get_redis_str(&redis, "ROBOT_INFO_SERVER_ADRESS"));
        // bind_events(h.socket());

        // std::string name_robot = "Newt";
        // h.socket()->emit("robot", name_robot);
    }
    catch(...)
    {
        std::cout << "REMOVE: Impossible de se connecter au server.\nFermeture du programme 03." << std::endl;
    }

    return 0;
}

//===================================================================
// BIND_EVENTS : Réception de données du server.
//===================================================================

void bind_events(sio::socket::ptr current_socket)
{
    current_socket->on("ORDER_GET_HMR", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "NAV_HMR_DOWNLOAD_ADRESS", data->get_map()["HMR_LINK"]->get_string());
        set_redis_var(&redis, "NAV_HMR_MAP_UPDATE",      "TRUE");
    }));

    current_socket->on("ORDER_WAITING", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");

        set_redis_var(&redis, "MISSION_AUTO_STATE",  "PAUSE");
        set_redis_var(&redis, "MISSION_MANUAL_STATE",  "PAUSE");

        set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_PAUSE", "START"));
        send_mission_update_server(current_socket, "MISSION_PAUSE", "START", 0);
    }));

    current_socket->on("ORDER_WAITING_END", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "FALSE");

        set_redis_var(&redis, "MISSION_AUTO_STATE",  "IN_PROGESS");
        set_redis_var(&redis, "MISSION_MANUAL_STATE",  "IN_PROGESS");

        set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_PAUSE", "COMPLETED"));
        send_mission_update_server(current_socket, "MISSION_PAUSE", "COMPLETED", 0);
    }));

    // (OK) Ordre pour se rendre à un endroit. (Position, option d'approche)
    current_socket->on("ORDER_AUTONAV", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        int flag = auto_mode_available(&redis);
        
        if(flag == 10 || flag == 20)
        {
            set_redis_var(&redis, "NAV_AUTO_MODE_PARKING", data->get_map()["OPT_DEST"]->get_string());
            
            std::string destination_str = std::to_string(get_curr_timestamp());
            destination_str += std::to_string(data->get_map()["LONGITUDE"]->get_double());
            destination_str += std::to_string(data->get_map()["LATITUDE"]->get_double());
            set_redis_var(&redis, "NAV_AUTO_DESTINATION",       destination_str);
            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",        "TRUE");
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "TRUE");
            set_redis_var(&redis, "ROBOT_MODE",                 "AUTO");
            set_redis_var(&redis, "MISSION_AUTO_TYPE",          "GOTO");
            set_redis_var(&redis, "MISSION_AUTO_STATE",         "START");

            set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_AUTO_GOTO", "START"));
            send_mission_update_server(current_socket, "MISSION_AUTO_GOTO", "START", 0);
        }
        else 
        {
            std::string destination_str = std::to_string(get_curr_timestamp());
            destination_str += std::to_string(0.0);
            destination_str += std::to_string(0.0);
            set_redis_var(&redis, "NAV_AUTO_DESTINATION",       destination_str);
            set_redis_var(&redis, "MISSION_MOTOR_BRAKE",        "TRUE");
            set_redis_var(&redis, "MISSION_UPDATE_GLOBAL_PATH", "FALSE");
            set_redis_var(&redis, "ROBOT_MODE",                 "MANUAL");
            set_redis_var(&redis, "MISSION_AUTO_TYPE",          "NO_VAL");
            set_redis_var(&redis, "MISSION_AUTO_STATE",         "NO_VAL");

            if(flag == -1)
            {
                set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_AUTO_GOTO", "MISSION_PARAM_AUTO_ENABLE_INVALID " + std::to_string(flag)));
            }
            else
            {
                set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_AUTO_GOTO", "AUTO_NOT_AVAILABLE " + std::to_string(flag)));
            }

            send_mission_update_server(current_socket, "MISSION_AUTO_GOTO", "INTERRUPTED", flag);
        }
    }));

    current_socket->on("ORDER_MANUALNAV", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        int flag = manual_mode_available(&redis);

        if(flag == 10)
        {
            if(get_redis_str(&redis, "ROBOT_MODE").compare("AUTO") == 0) 
            {
                set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_MANUAL_MOVE", "START"));
                send_mission_update_server(current_socket, "MISSION_MANUAL_MOVE", "START", 0);
            }

            std::string event_manual_controler_data_str = std::to_string(get_curr_timestamp());
            event_manual_controler_data_str += std::to_string(data->get_vector()[1]->get_double());
            event_manual_controler_data_str += std::to_string(data->get_vector()[2]->get_double());
            event_manual_controler_data_str += std::to_string(data->get_vector()[3]->get_double());
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

            set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_MANUAL_MOVE", "MANUAL_NOT_AVAILABLE " + std::to_string(flag)));
            send_mission_update_server(current_socket, "MISSION_MANUAL_MOVE", "INTERRUPTED", 0);
        }
    }));

    // (OK) Ordre de deveroullage d'une trappe. (Casier ID, avec/sans code)
    current_socket->on("ORDER_HARDWARE", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        std::vector<std::string> vect_cargo_state;
        get_redis_multi_str(&redis, "HARD_CARGO_STATE", vect_cargo_state);

        std::vector<std::string> vect_cargo_mission;
        get_redis_multi_str(&redis, "MISSION_HARD_CARGO", vect_cargo_mission);

        std::string box_id_str = data->get_map()["ID_BOX"]->get_string();

        int index;
        if(box_id_str.compare("A") == 0) index = 1;
        if(box_id_str.compare("B") == 0) index = 2;
        if(box_id_str.compare("C") == 0) index = 3;

        if(vect_cargo_state[index].compare("OPEN") == 0)
        {
            set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_OPEN_BOX" + box_id_str, "ALREADY_COMPLETED"));
            send_mission_update_server(current_socket, "MISSION_OPEN_BOX" + box_id_str, "ALREADY_COMPLETED", 0);
        }
        else
        {
            set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_OPEN_BOX" + box_id_str, "START"));
            send_mission_update_server(current_socket, "MISSION_OPEN_BOX" + box_id_str, "START", 0);
        }

        std::string new_mission_cargo_str = std::to_string(get_curr_timestamp()) + "|";

        for(int i = 1; i < 4; i++)
        {
            if(i == index) new_mission_cargo_str += "OPEN|";
            else
            {
                vect_cargo_state[i] + "|";
            }
        }

        set_redis_var(&redis, "MISSION_HARD_CARGO", new_mission_cargo_str);

    }));


    // // (OK) Ordre lancement stream robot (Option de stream)
    // current_socket->on("ORDER_STREAM_ON", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    // {
    //     redis.set("option_stream", std::to_string(data->get_map()["OPT_STREAM"]->get_int()));
    // }));

    // // (OK) Ordre arret stream robot (Option de stream)
    // current_socket->on("ORDER_STREAM_OFF", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    // {
    //     redis.set("option_stream", "0");
    // }));

    current_socket->on("ORDER_CANCEL_MISSION", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        set_redis_var(&redis, "ROBOT_MODE",           "MANUAL");
        set_redis_var(&redis, "MISSION_MOTOR_BRAKE",  "TRUE");
        set_redis_var(&redis, "MISSION_AUTO_TYPE",    "NO_VAL");
        set_redis_var(&redis, "MISSION_AUTO_STATE",   "NO_VAL");
        set_redis_var(&redis, "MISSION_MANUAL_TYPE",  "WAITING");
        set_redis_var(&redis, "MISSION_MANUAL_STATE", "IN_PROGRESS");

        set_redis_var(&redis, "EVENT", get_event_str(1, "MISSION_CANCEL_MISSION", "SUCESS"));
        send_mission_update_server(current_socket, "MISSION_CANCEL_MISSION", "COMPLETED", 0);
    }));

    // // Ordre de reset du software.
    // current_socket->on("ORDER_RESET_SOFTWARE", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    // {}));
}