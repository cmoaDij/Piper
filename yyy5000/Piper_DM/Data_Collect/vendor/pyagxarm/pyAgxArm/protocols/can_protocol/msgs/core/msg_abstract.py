from typing import Generic, TypeVar
from .msg_interface import MessageInterface, StrMixin

T = TypeVar("T")

class MessageAbstract(Generic[T], MessageInterface, StrMixin):
    def __init__(self, 
                 msg_type: str="none",
                 msg: T=None, 
                 *, 
                 timestamp: float=0, 
                 hz: float=0):
        self._msg_type = msg_type
        self._timestamp = timestamp
        self._hz = hz
        self._msg = msg
    
    @property
    def msg_type(self) -> str:
        return self._msg_type

    @msg_type.setter
    def msg_type(self, msg_type: str):
        self._msg_type = msg_type

    @property
    def timestamp(self) -> float:
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value: float):
        self._timestamp = value

    @property
    def hz(self) -> float:
        return self._hz

    @hz.setter
    def hz(self, value: float):
        self._hz = value

    @property
    def msg(self) -> T:
        return self._msg

    @msg.setter
    def msg(self, msg: T):
        self._msg = msg
