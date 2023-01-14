from enum import Enum

class RobotRedisElement:
    ROBOT_AUTHENTICATION          = "hello" # FIXME

    ROBOT_POSITION                = "NAV_GLOBAL_POSITION"
    ROBOT_SPEED                   = "NAV_CURR_SPEED"
    ROBOT_ROUTE_ID                = "NAV_ROAD_CURRENT_ID"
    NAV_BATTERY_VOLTAGE           = "NAV_BATTERY_VOLTAGE"
    NAV_BATTERY_PERCENTAGE        = "NAV_BATTERY_PERCENTAGE"

    HARD_TEMPERATURE_RC_INFO      = "HARD_TEMPERATURE_INFO"

    HARD_MCU_MOTOR_COM_STATE      = "HARD_MCU_MOTOR_COM_STATE"
    HARD_MCU_CARGO_COM_STATE      = "HARD_MCU_CARGO_COM_STATE"
    HARD_MCU_INTER_COM_STATE      = "HARD_MCU_INTER_COM_STATE"
    HARD_PIXHAWK_COM_STATE        = "HARD_PIXHAWK_COM_STATE"
    HARD_LOCAL_JS_COM_STATE       = "HARD_LOCAL_JS_COM_STATE"
    HARD_CARGO_STATE              = "HARD_CARGO_STATE"

    SOFT_PROCESS_ID_SYS_STATUS    = "SOFT_PROCESS_ID_SYS_STATUS"
    SOFT_PROCESS_ID_HARD_STATUS   = "SOFT_PROCESS_ID_HARD_STATUS"
    SOFT_PROCESS_ID_NAV_STATUS    = "SOFT_PROCESS_ID_NAV_STATUS"
    SOFT_PROCESS_ID_PERCEP_STATUS = "SOFT_PROCESS_ID_PERCEP_STATUS"
    SOFT_PROCESS_ID_SERV_STATUS   = "SOFT_PROCESS_ID_SERV_STATUS"
    SOFT_STATE_LIDAR_0            = "SOFT_STATE_LIDAR_0"
    SOFT_STATE_LIDAR_1            = "SOFT_STATE_LIDAR_1"
    SOFT_STATE_FRONT_CAMERA       = "SOFT_STATE_FRONT_CAMERA"
    SOFT_STATE_BACK_CAMERA        = "SOFT_STATE_BACK_CAMERA"

    ROBOT_MODE                    = "ROBOT_MODE"
    NAV_AUTO_DESTINATION          = "NAV_AUTO_DESTINATION"
    MISSION_AUTO_TYPE             = "MISSION_AUTO_TYPE"
    MISSION_AUTO_STATE            = "MISSION_AUTO_STATE"
    MISSION_MANUAL_TYPE           = "MISSION_MANUAL_TYPE"
    MISSION_MANUAL_STATE          = "MISSION_MANUAL_STATE"