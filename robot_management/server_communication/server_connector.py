import socket
import logging
import traceback
import struct

from multiprocessing import Queue
from threading import Thread
from robot_management.utils import singleton
from robot_management.server_communication.protocol import *

RECEIVE_PACKET_LIST = {
    40: Packet40SetDestination,
    41: Packet41ManualControlInput,
    42: Packet42SetHardwareState,
    43: Packet43UpdateData,
    44: Packet44Limit
}

@singleton
class ServerConnector():
    def __init__(self) -> None:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.endpoint: tuple[str, int] = ("127.0.0.1", 6799) # TODO put in config
        self.socket_data_stream = SocketDataStream(self)
        self.send_queue = Queue()
        self.start_threads()


    def start_threads(self):
        Thread(target = self._recv_thread_target).start()
        Thread(target = self._send_thread_target).start()
        
    def add_to_send_queue(self, p: Packet):
        self.send_queue.put(p)

    def _recv_thread_target(self):
        try:
            while True:
                pck_id: int = self.socket_data_stream.receive_int()
                if pck_id not in RECEIVE_PACKET_LIST:
                    logging.fatal(f'[RECV] Packet id {pck_id} is not in the list of receivable packets, this is a protocol error. Failing early.')
                    exit(1)
                
                pck: Packet = RECEIVE_PACKET_LIST[pck_id]()
                try:
                    pck.receive(self.socket_data_stream)

                except:
                    traceback.print_exc()
                    logging.error(f'Error in handling packet with id {pck_id}')
                    continue
        except:
            traceback.print_exc()
            exit(1)

    def _send_thread_target(self):
        try:
            while True:
                p: Packet = self.send_queue.get()
                self.socket_data_stream.send_int(p.packet_id)
                p.send(self.socket_data_stream)
        except:
            traceback.print_exc()
            exit(1)

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

    def authenticate(self, uuid: str):
        self.send(Packet10RobotAuthentication())


    def is_open(self) -> bool:
        return not self._is_socket_closed()

    def get_socket_data_stream(self) -> "SocketDataStream":
        return self.socket_data_stream

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
        return struct.unpack(self.server_connector.receive(struct.calcsize('i')))[0]

    def send_float(self, e: float) -> None:
        self.server_connector.send(struct.pack('f', e))

    def receive_float(self) -> float:
        return struct.unpack(self.server_connector.receive(struct.calcsize('f')))[0]

    def send_string(self, str: str) -> None:
        str_bytes = str.encode('utf-8')
        self.send_int(len(str_bytes))
        self.server_connector.send(str_bytes)

    def receive_string(self) -> str:
        len = self.receive_int()
        return self.server_connector.receive(len).decode('utf-8')