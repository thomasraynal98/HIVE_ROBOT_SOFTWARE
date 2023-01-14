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


class Packet1RobotAuthentication(Packet):
    def __init__(self) -> None:
        super().__init__(1)

    def send(self, stream: SocketDataStream):
        stream.send_string(RedisConnector().get(RobotRedisElement.ROBOT_AUTHENTICATION))


class Packet20RobotPosition(Packet):
    def __init__(self) -> None:
        super().__init__(2)

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


# class Packet2

"""
Auth
    Token
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

"""