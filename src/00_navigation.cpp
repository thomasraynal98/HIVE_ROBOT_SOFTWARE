#include "00_navigation.h"
#include "00_function.h"

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

    std::string opt_str = get_redis_str(redis, "MISSION_PARAM_AUTO_ENABLE");
    
    if(opt_str.compare("AUTO_SIMPLE"))
    {
        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 10;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   == 0) return 11;

        if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") != 0 && \
           get_redis_str(redis, "HARD_PIXHAWK_COM_STATE").compare("CONNECTED")   != 0) return 12;

    }
    if(opt_str.compare("AUTO_STANDARD"))
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

    if(get_redis_str(redis, "HARD_MCU_MOTOR_COM_STATE").compare("CONNECTED") == 0) return 10;
    return 11;
}

void map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle)
{
    //TODO: Cette fonction va mapper les commandes brutes de la manelle en commande 
    //TODO: utilisable par le robot.

    // std::string msg = std::to_string(get_curr_timestamp()) "|";

    // if(back_value < 0.05 && front_value < 0.05)
    // {
    //     //! break.
    //     msg += "0|0|0|0|0|0|";
    //     set_redis_var(redis, "HARD_MOTOR_COMMAND", msg)
    //     return;
    // }
    // if(back_value > 0.05 && front_value > 0.05)
    // {
    //     //! break.
    //     msg += "0|0|0|0|0|0|";
    //     set_redis_var(redis, "HARD_MOTOR_COMMAND", msg)
    //     return;
    // }

    // double vitesse_max = 3.0; // m/s

    // if(front_value > 0.05)
    // {
    //     // 0   - 90  = plus vers la gauche
    //     // 90  - 180 = plus vers la droite
    //     // 180 - 270 = rotation droite
    //     // 270 - 360 = rotation gauche

    //     double current_speed = front_value * vitesse_max; 

    //     if(angle > 0   && angle <= 90)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i >= 3) msg += std::to_string(current_speed) + "|";
    //             if(i < 3) msg += std::to_string(current_speed - current_speed*(90-angle)/90)  + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    //     if(angle > 90  && angle <= 180)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i >= 3) msg += std::to_string(current_speed - current_speed*(angle-90)/90) + "|";
    //             if(i < 3) msg += std::to_string(current_speed)  + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    //     if(angle > 180 && angle <= 270)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i < 3) msg += std::to_string(current_speed) + "|";
    //             if(i >= 3) msg += std::to_string(-1*current_speed) + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    //     if(angle > 270 && angle <= 360)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i < 3) msg += std::to_string(-1*current_speed) + "|";
    //             if(i >= 3) msg += std::to_string(current_speed) + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    // }

    // if(back_value > 0.05)
    // {
    //     double current_speed = back_value * vitesse_max; 

    //     if(angle == 90)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             msg += std::to_string(-1*current_speed) + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    //     if(angle > 180 && angle <= 270)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i < 3) msg += std::to_string(-1*current_speed) + "|";
    //             if(i >= 3) msg += std::to_string(-1*(current_speed - current_speed*(90-(angle-180))/90)) + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    //     if(angle > 270 && angle <= 360)
    //     {
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i < 3) msg += std::to_string(-1*(current_speed - current_speed*(angle-270)/90)) + "|";
    //             if(i >= 3) msg += std::to_string(-1*current_speed) + "|";
    //         }
    //         redis->publish("command_micro", msg);
    //     }
    // }
}