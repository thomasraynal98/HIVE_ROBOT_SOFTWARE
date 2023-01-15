import redis

from typing import Any
from robot_management.redis_connector.redis_communicator import RobotRedisElement
from robot_management.utils import singleton


@singleton
class RedisConnector():

    def __init__(self) -> None:
        self.redis = redis.Redis(host="127.0.0.1", port = 6379)

    def get(self, element: RobotRedisElement) -> Any:
        return self.redis.get(element.value)

    def get_strip_timestamp(self, element: RobotRedisElement) -> Any:
        return self.get(element).split('|')[1]

    def set(self, element: RobotRedisElement, value: Any) -> None:
        self.redis.set(element.value, value)

    #TODO subcriber to event channeland send