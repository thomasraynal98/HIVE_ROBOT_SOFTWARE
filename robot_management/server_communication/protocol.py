from robot_management.redis_connector.redis_communicator import RedisConnector
from robot_management.redis_connector.redis_constants import RobotRedisElement
from robot_management.server_communication.server_connector import ServerConnector, SocketDataStream


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
    def __init__(self) -> None:
        super().__init__(30)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet31AlertInfo(Packet):
    def __init__(self) -> None:
        super().__init__(31)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet32Event(Packet):
    def __init__(self) -> None:
        super().__init__(32)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet40ChangeMode(Packet):
    def __init__(self) -> None:
        super().__init__(40)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet41ManualControlInput(Packet):
    def __init__(self) -> None:
        super().__init__(41)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet42SetHardwareState(Packet):
    def __init__(self) -> None:
        super().__init__(42)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet43UpdateData(Packet):
    def __init__(self) -> None:
        super().__init__(43)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet44ServiceManualControl(Packet):
    def __init__(self) -> None:
        super().__init__(44)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet45Limit(Packet):
    def __init__(self) -> None:
        super().__init__(45)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()


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
    40 ChangeMode: SetDestination, SetMode (auto, manual)
    41 ManualControlInput
    42 SetHardwareState
    43 UpdateData: map
    44 ServiceManualControl: Start, Stop, Restart
    45 Limit

"""