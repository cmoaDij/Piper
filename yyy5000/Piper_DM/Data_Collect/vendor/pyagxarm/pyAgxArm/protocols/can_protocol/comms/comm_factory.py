from typing import Type, Dict, TypeVar
from .can_comm import CanComm, create_can_comm_config

T = TypeVar("T")

def create_comm_config(comm:str="can", **kwargs) -> dict:
    if(comm == "can"):
        return create_can_comm_config(**kwargs)
    raise ValueError(f"Unsupported comm type: {comm}")

class CommsFactory:

    _registry: Dict[str, Dict[str, Type]] = {
        "can": {
            "impl": CanComm,
        }
    }

    @classmethod
    def register_comm(cls, type: str, version: str, comm_cls: Type):
        """
        注册通信类
        :param comm_type: 例如 "can", "socket"
        :param version: 例如 "base", "impl", "v1"
        :param comm_cls: 实现类
        """
        # 如果该通信类型不存在，则创建一个空字典
        if type not in cls._registry:
            cls._registry[type] = {}
        # 注册版本
        cls._registry[type][version] = comm_cls

    @classmethod
    def load_class(cls, type: str, version: str = "base") -> Type:
        if type not in cls._registry:
            raise KeyError(f"Comm type '{type}' not registered.")
        if version not in cls._registry[type]:
            raise KeyError(f"Version '{version}' not registered for '{type}'.")

        return cls._registry[type][version]

    @classmethod
    def create_comm(cls, type: str, version: str = "base", **kwargs) -> T:
        if type not in cls._registry:
            raise KeyError(f"Comm type '{type}' not registered.")
        if version not in cls._registry[type]:
            raise KeyError(f"Version '{version}' not registered for comm '{type}'.")

        cls_type: Type[T] = cls._registry[type][version]
        return cls_type(**kwargs)
