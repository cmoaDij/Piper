"""Dependency-free button edge detection for Quest controls."""


class RisingEdge:
    """Report one event when a Boolean input changes from released to pressed."""

    def __init__(self) -> None:
        self._pressed = False

    def update(self, pressed: object) -> bool:
        """Return true once per press, regardless of how long the button is held."""
        current = bool(pressed)
        rising = current and not self._pressed
        self._pressed = current
        return rising
