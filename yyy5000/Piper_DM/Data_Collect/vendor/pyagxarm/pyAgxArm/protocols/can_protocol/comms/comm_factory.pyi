from typing import overload
from typing_extensions import Literal
from .core.can_comm_base import CanCommBase

def create_comm_config(
        comm:str="can", 
        **kwargs
    ) -> dict:...

class CommsFactory:

    @overload
    @classmethod
    def create_comm(
        cls,
        type: Literal["can"],
        version: str = ...,
        **kwargs
    ) -> CanCommBase: ...

    @classmethod
    def create_comm(
        cls,
        type: str,
        version: str = ...,
        **kwargs
    ) -> CanCommBase: ...
