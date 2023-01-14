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

    def receive(self, stream: SocketDataStream):
        raise NotImplementedError()