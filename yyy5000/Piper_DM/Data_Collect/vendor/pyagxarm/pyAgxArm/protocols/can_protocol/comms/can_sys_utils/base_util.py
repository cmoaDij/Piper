from abc import ABC, abstractmethod
from typing import Optional

class CanSystemInfoBase(ABC):

    @abstractmethod
    def is_exists(channel: str) -> bool: ...

    @abstractmethod
    def is_up(channel: str) -> bool: ...

    @abstractmethod
    def get_bitrate(channel: str) -> Optional[int]: ...
