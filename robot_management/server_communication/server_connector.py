import socket
import logging
import traceback
import struct

from robot_management.utils import singleton

@singleton
class ServerConnector():
    def __init__(self, endpoint: tuple[str, int]) -> None:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.endpoint = endpoint

    def connect(self) -> bool:
        try:
            self.server_socket.connect(self.endpoint)
            return True
        except:
            traceback.print_exc()
            logging.error("[SERVER_CONNECTOR] Failed to connect to server")
            return False

    def _is_socket_closed(self) -> bool:
        try:
            data = self.server_socket.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                return True
        except BlockingIOError:
            return False  # socket is open and reading from it would block
        except ConnectionResetError:
            return True  # socket was closed for some other reason
        except Exception as e:
            logging.exception("unexpected exception when checking if a socket is closed")
            return False
        return False

    def is_open(self) -> bool:
        return not self._is_socket_closed()

    def get_socket_data_stream(self) -> "SocketDataStream":
        return SocketDataStream(self)

    def send(self, data: bytes) -> None:
        if not self.is_open():
            raise RuntimeError("Socket is closed while sending")
        self.server_socket.send(data)

    def receive(self, size: int) -> bytes:
        if not self.is_open():
            raise RuntimeError("Socket is closed while writing")
        return self.server_socket.recv(size)

class SocketDataStream():
    def __init__(self, server_connector: ServerConnector) -> None:
        self.server_connector = server_connector

    def send_int(self, int: int) -> None:
        self.server_connector.send(struct.pack('i', int))

    def receive_int(self) -> int:
        struct.unpack(self.server_connector.receive(struct.calcsize('i')))[0]

    def send_float(self, e: float) -> None:
        self.server_connector.send(struct.pack('f', e))

    def receive_float(self) -> float:
        struct.unpack(self.server_connector.receive(struct.calcsize('f')))[0]

    def send_string(self, str: str) -> None:
        str_bytes = str.encode('utf-8')
        self.send_int(len(str_bytes))
        self.server_connector.send(str_bytes)

    def receive_string(self) -> str:
        len = self.receive_int()
        return self.server_connector.receive(len).decode('utf-8')