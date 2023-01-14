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

    # def send(self, stream: SocketDataStream):
    #     redis_cnt = RedisConnector()
    #     volt = redis_cnt.get(RobotRedisElement.NAV_BATTERY_VOLTAGE)
    #     voltPercentage = redis_cnt.get(RobotRedisElement.NAV_BATTERY_PERCENTAGE)
    #     tempRc1, tempRc2, tempRc3 = redis_cnt.get(RobotRedisElement.HARD_TEMPERATURE_RC_INFO).split('|')[:3]

    #     statusMcuControlMotor = redis_cnt.get(RobotRedisElement.HARD_MCU_MOTOR_COM_STATE)
    #     statusMcuControlCargo = redis_cnt.get(RobotRedisElement.HARD_MCU_CARGO_COM_STATE)
    #     statusMcuControlInter = redis_cnt.get(RobotRedisElement.HARD_MCU_INTER_COM_STATE)

    #     stream.send_float(tempRc1)
    #     stream.send_float(tempRc2)
    #     stream.send_float(tempRc3)
    #     stream.send_float(volt)
    #     stream.send_float(voltPercentage)

class Packet22HardwareState(Packet):
    def __init__(self) -> None:
        super().__init__(22)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet23ProcessStatus(Packet):
    def __init__(self) -> None:
        super().__init__(23)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

class Packet24RobotStatus(Packet):
    def __init__(self) -> None:
        super().__init__(24)

    def send(self, stream: SocketDataStream):
        redis_cnt = RedisConnector()

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
    21 PrimarySystemStatus: Voltage, Tempratures, Harware Connection
    22 HardwareState: MCU, Trapes
    23 ProcessStatus: Process Status
    24 RobotStatus: Robot Mode, Destination, status (Going, Idle, Waiting)
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