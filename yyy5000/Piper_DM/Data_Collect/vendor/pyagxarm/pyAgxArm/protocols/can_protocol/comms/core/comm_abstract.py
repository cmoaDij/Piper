from typing import Any, Callable, Optional

from .comm_interface import CommInterface


class CommAbstract(CommInterface):

    def __init__(self):
        super().__init__()
        self._cb: Optional[Callable[[Any], None]] = None

    def set_callback(self, cb: Callable[[Any], None]) -> None:
        self._cb = cb

    def get_callback(self) -> Optional[Callable[[Any], None]]:
        return self._cb

    def clear_callback(self) -> None:
        self._cb = None

    def has_callback(self) -> bool:
        return self._cb is not None

    def _trigger_callback(self, data: Any, **kwargs) -> None:
        """Trigger the registered callback if one is available."""
        if self._cb:
            self._cb(data)
