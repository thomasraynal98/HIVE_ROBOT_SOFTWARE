from enum import Enum
from robot_management.redis_connector.redis_communicator import RedisConnector
from robot_management.redis_connector.redis_constants import RobotRedisElement
from robot_management.server_communication.server_connector import ServerConnector, SocketDataStream
import time

PROTOCOL_VERSION = 1 #! Upgrade on each change 

class AlertType(Enum):
    # Collision, Stuck, No Path, Hardware, Process Error
    COLLISION: 0
    STUCK: 1
    NO_PATH: 2
    HARDWARE_ERROR: 3
    PROCESS_ERROR: 4

class EventType(Enum):
    #Event: ReachedWaypoint, ModeChange, CargoOpen, CargoCLose, PathComputed
    WAYPOINT_REACHED: 0
    OPERATING_MODE_CHANGED: 1
    CARGO_OPEN: 2
    CARGO_CLOSE: 3
    EN_ROUTE: 4


class Packet:
    def __init__(self, packet_id: int) -> None:
        self.packet_id: int = packet_id

    def send(self, stream: SocketDataStream):
        raise NotImplementedError()

    def receive(self, stream: SocketDataStream):
        raise NotImplementedError()

    def handle(self):
        raise NotImplementedError()


class Packet10RobotAuthentication(Packet):
    def __init__(self) -> None:
        super().__init__(10)

    def send(self, stream: SocketDataStream):
        stream.send_string(RedisConnector().get(RobotRedisElement.ROBOT_AUTHENTICATION))
        stream.send_int(PROTOCOL_VERSION)


class Packet20RobotPosition(Packet):
    def __init__(self) -> None:
        super().__init__(20)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()
        lon, lat, hdg = redis_cnt.get(RobotRedisElement.ROBOT_POSITION).split('|')[:3]
        spd = redis_cnt.get_strip_timestamp(RobotRedisElement.ROBOT_SPEED)
        route = redis_cnt.get_strip_timestamp(RobotRedisElement.ROBOT_ROUTE_ID)

        stream.send_float(lon)
        stream.send_float(lat)
        stream.send_float(hdg)
        stream.send_float(spd)
        stream.send_int(route)

class Packet21PrimarySystemStatus(Packet):
    def __init__(self) -> None:
        super().__init__(21)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()
        volt = redis_cnt.get(RobotRedisElement.NAV_BATTERY_VOLTAGE)
        voltPercentage = redis_cnt.get(RobotRedisElement.NAV_BATTERY_PERCENTAGE)
        tempRc1, tempRc2, tempRc3 = redis_cnt.get(RobotRedisElement.HARD_TEMPERATURE_RC_INFO).split('|')[:3]

        statusMcuControlMotor = 1 if redis_cnt.get(RobotRedisElement.HARD_MCU_MOTOR_COM_STATE) == 'CONNECTED' else (-1 if redis_cnt.get(RobotRedisElement.HARD_MCU_MOTOR_COM_STATE) == 'DISCONNECTED' else 2)
        statusMcuControlCargo = 1 if redis_cnt.get(RobotRedisElement.HARD_MCU_CARGO_COM_STATE) == 'CONNECTED' else (-1 if redis_cnt.get(RobotRedisElement.HARD_MCU_CARGO_COM_STATE) == 'DISCONNECTED' else 2)
        statusMcuControlInter = 1 if redis_cnt.get(RobotRedisElement.HARD_MCU_INTER_COM_STATE) == 'CONNECTED' else (-1 if redis_cnt.get(RobotRedisElement.HARD_MCU_INTER_COM_STATE) == 'DISCONNECTED' else 2)
        statusPixhawk         = 1 if redis_cnt.get(RobotRedisElement.HARD_PIXHAWK_COM_STATE)   == 'CONNECTED' else (-1 if redis_cnt.get(RobotRedisElement.HARD_PIXHAWK_COM_STATE)   == 'DISCONNECTED' else 2)
        statusLocalJoystick   = 1 if redis_cnt.get(RobotRedisElement.HARD_LOCAL_JS_COM_STATE)  == 'CONNECTED' else (-1 if redis_cnt.get(RobotRedisElement.HARD_LOCAL_JS_COM_STATE)  == 'DISCONNECTED' else 2)

        statusBox1, statusBox2, statusBox3 = [1 if x == "OPEN" else 0 for x in redis_cnt.get_strip_timestamp(RobotRedisElement.HARD_CARGO_STATE).split('|')]
        
        stream.send_float(tempRc1)
        stream.send_float(tempRc2)
        stream.send_float(tempRc3)
        stream.send_float(volt)
        stream.send_float(voltPercentage)
        stream.send_int(statusMcuControlMotor)
        stream.send_int(statusMcuControlCargo)
        stream.send_int(statusMcuControlInter)
        stream.send_int(statusPixhawk)
        stream.send_int(statusLocalJoystick)
        stream.send_int(statusBox1)
        stream.send_int(statusBox2)
        stream.send_int(statusBox3)

class Packet22ProcessStatus(Packet):
    def __init__(self) -> None:
        super().__init__(22)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

        statusProcess1Sys  = 1 if redis_cnt.get(RobotRedisElement.SOFT_PROCESS_ID_SYS_STATUS)    == 'CONNECTED' else 0
        statusProcess2Hrd  = 1 if redis_cnt.get(RobotRedisElement.SOFT_PROCESS_ID_HARD_STATUS)   == 'CONNECTED' else 0
        statusProcess3Srv  = 1
        statusProcess4Nav  = 1 if redis_cnt.get(RobotRedisElement.SOFT_PROCESS_ID_NAV_STATUS)    == 'CONNECTED' else 0
        statusProcess5Per  = 1 if redis_cnt.get(RobotRedisElement.SOFT_PROCESS_ID_PERCEP_STATUS) == 'CONNECTED' else 0

        stream.send_int(statusProcess1Sys)
        stream.send_int(statusProcess2Hrd)
        stream.send_int(statusProcess3Srv)
        stream.send_int(statusProcess4Nav)
        stream.send_int(statusProcess5Per)

class Packet23RobotStatus(Packet):
    def __init__(self) -> None:
        super().__init__(23)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

        destLon, destLat = redis_cnt.get(RobotRedisElement.NAV_AUTO_DESTINATION).split('|')[:2]
        mode = redis_cnt.get(RobotRedisElement.ROBOT_MODE)
        missionTitle  = redis_cnt.get(RobotRedisElement.MISSION_AUTO_TYPE) if mode == "AUTO" else redis_cnt.get(RobotRedisElement.MISSION_MANUAL_TYPE)
        missionStatus = redis_cnt.get(RobotRedisElement.MISSION_AUTO_STATE) if mode == "AUTO" else redis_cnt.get(RobotRedisElement.MISSION_MANUAL_STATE)

        stream.send_float(destLon)
        stream.send_float(destLat)
        stream.send_string(mode)
        stream.send_string(missionTitle)
        stream.send_string(missionStatus)

class Packet30Alert(Packet):
    def __init__(self, alert_type: AlertType) -> None:
        super().__init__(30)
        self.alert_type = alert_type

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()
        stream.send_int(self.alert_type.value)

class Packet31AlertInfo(Packet):
    def __init__(self, txt: str) -> None:
        super().__init__(31)
        self.txt = txt

    def send(self, stream: SocketDataStream):
        stream.send_string(self.txt)

class Packet32Event(Packet):
    def __init__(self, evt_type: int) -> None:
        super().__init__(32)
        self.evt_type = evt_type

    def send(self, stream: SocketDataStream):
        stream.send_int(self.evt_type)

class Packet40SetDestination(Packet):
    def __init__(self) -> None:
        super().__init__(40)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

    def receive(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

        redisStr = str(time.time() * 1000) + "|" + stream.receive_float() + "|" + stream.receive_float() + "|"
        

        redis_cnt.set(RobotRedisElement.MISSION_MOTOR_BRAKE,        "TRUE")
        redis_cnt.set(RobotRedisElement.MISSION_AUTO_TYPE,          "GOTO")
        redis_cnt.set(RobotRedisElement.MISSION_AUTO_STATE,         "START")
        redis_cnt.set(RobotRedisElement.NAV_AUTO_DESTINATION,       redisStr)
        redis_cnt.set(RobotRedisElement.MISSION_UPDATE_GLOBAL_PATH, "TRUE")
        redis_cnt.set(RobotRedisElement.ROBOT_MODE,                 "AUTO")

        # reception msg publish


class Packet41ManualControlInput(Packet):
    def __init__(self) -> None:
        super().__init__(41)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

    def receive(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

        redis_cnt.set(RobotRedisElement.MISSION_MOTOR_BRAKE,         "FALSE")
        redis_cnt.set(RobotRedisElement.MISSION_AUTO_TYPE,           "MANUAL_MOVE")
        redis_cnt.set(RobotRedisElement.MISSION_AUTO_STATE,          "IN_PROGRESS")
        redis_cnt.set(RobotRedisElement.MISSION_UPDATE_GLOBAL_PATH,  "TRUE")
        redis_cnt.set(RobotRedisElement.ROBOT_MODE,                  "MANUAL")

        redisControlerCommandStr = str(time.time() * 1000) + "|" + stream.receive_float() + "|" + stream.receive_float() + "|" + stream.receive_float() + "|";
        redis_cnt.set(RobotRedisElement.EVENT_MANUAL_CONTROLLER_DATA, redisControlerCommandStr)

class Packet42SetHardwareState(Packet):
    def __init__(self) -> None:
        super().__init__(42)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

    def receive(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

        #hardware type
        hw_type = stream.receive_int()
        if hw_type == 0:
            idBox_toOpen = stream.receive_int()
            statusBox = redis_cnt.get_strip_timestamp(RobotRedisElement.MISSION_HARD_CARGO).split('|')

            redisCommandCargoStr = str(time.time() * 1000) + "|"
            for i in range(3):
                redisCommandCargoStr += 'OPEN|' if idBox_toOpen == i+1 else statusBox[i] + "|"
            redis_cnt.set(RobotRedisElement.MISSION_HARD_CARGO,          redisCommandCargoStr)

class Packet43UpdateData(Packet):
    def __init__(self) -> None:
        super().__init__(43)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

    def receive(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()
        # Tu dois écrire dans un fichier texte.
        map = stream.receive_string()
        # set_redis_var(&redis, "NAV_HMR_DOWNLOAD_ADRESS", data->get_map()["HMR_LINK"]->get_string());
        # Write_TXT_file("../data/HMD_TEST_VESINET.txt", data->get_map()["FILE_DATA"]->get_string());
        # set_redis_var(&redis, "NAV_HMR_MAP_UPDATE",      "TRUE");
        # pub_redis_var(&redis, "EVENT", get_event_str(3, "NEW HMD DOWLOAD", "COMPLETED"));

class Packet44Limit(Packet):
    def __init__(self) -> None:
        super().__init__(44)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

    def receive(self, stream: SocketDataStream):
        limit_id = stream.receive_int()
        limit_value = stream.receive_float()


"""
Auth
    10 Token
Telem
    20 Position, speed, heading: Real pos
    21 PrimarySystemStatus: Voltage, Tempratures, Hardware Connection, Trapes
    22 ProcessStatus: Process Status
    23 RobotStatus: Robot Mode, Destination, status (Going, Idle, Waiting)
Evt
    30 Alert: Collision, Stuck, No Path, Hardware, Process Error
    31 AlertInfo: str
    32 Event: ReachedWaypoint, ModeChange, CargoOpen, CargoCLose, PathComputed

Actions
    40 SetDestination: Destination, OffsetTime (ms)
    41 ManualControlInput
    42 SetHardwareState
    43 UpdateData: map
    44 Limit

"""