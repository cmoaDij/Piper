from abc import ABC, abstractmethod

class MessageInterface(ABC):
    @property
    @abstractmethod
    def msg_type(self) -> float:
        ...

    @property
    @abstractmethod
    def timestamp(self) -> float:
        ...

    @property
    @abstractmethod
    def hz(self) -> float:
        ...

    @property
    @abstractmethod
    def msg(self):
        ...

class StrMixin:
    def __str__(self):
        # 只打印 _ 开头的属性
        attrs = [f"{k[1:]}: {v}" for k, v in self.__dict__.items() if k.startswith("_")]
        return "\n".join(attrs)
