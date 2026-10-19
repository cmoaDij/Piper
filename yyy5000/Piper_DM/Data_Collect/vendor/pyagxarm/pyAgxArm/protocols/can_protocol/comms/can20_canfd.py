"""
CAN 2.0 transparent tunnel for CAN FD payloads (0x711 request / 0x712 response).

This module implements **only the Agilex wrist conversion-board framing**:
classic-CAN fragments (``55 AA`` header, sequence nibble, tail CRC16 over the
CAN FD *data* field padded to ISO 11898-1 DLC lengths). It does **not**
interpret ``arbitration_id`` or payload bytes.

On TX, payloads are zero-padded to the next CAN FD DLC slot before
fragmentation and CRC. On RX, the conversion board may omit trailing ``0x00``
fill bytes in fragments, but the tail CRC is computed over the full padded
slot padding is applied before CRC verification.

Layering::

    0x711/0x712 tunnel (this module)  →  logical CAN FD (id + data)
                                         →  vendor SDK / driver
"""

import threading
import time
from collections import deque
from typing import Callable, Deque, List, Optional

from can.message import Message

DEFAULT_REQUEST_ID = 0x711
DEFAULT_RESPONSE_ID = 0x712
FIRST_FRAME_MARKER = bytes((0x55, 0xAA))
CRC16_BYTEORDER = "little"
PAYLOAD_CHUNK_SIZE = 7
MAX_CANFD_DATA_LEN = 64
# ISO 11898-1 CAN FD valid data-field byte counts (DLC mapping).
CANFD_DLC_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def _canfd_dlc_slot(length: int) -> int:
    """Return the smallest CAN FD DLC byte count that fits *length*."""
    if length < 0:
        raise ValueError(f"length must be >= 0, got {length}")
    if length > MAX_CANFD_DATA_LEN:
        raise ValueError(
            f"length must be <= {MAX_CANFD_DATA_LEN}, got {length}"
        )
    for slot in CANFD_DLC_LENGTHS:
        if slot >= length:
            return slot
    return MAX_CANFD_DATA_LEN


def _pad_to_canfd_dlc(data: bytes) -> bytes:
    """Pad *data* with trailing ``0x00`` bytes to the next CAN FD DLC slot."""
    slot = _canfd_dlc_slot(len(data))
    if len(data) == slot:
        return data
    return data + bytes(slot - len(data))


def _crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _encode_sequence_byte(frame_group: int, sequence: int) -> int:
    if not 1 <= frame_group <= 15:
        raise ValueError("frame_group must be in [1, 15]")
    if not 1 <= sequence <= frame_group:
        raise ValueError("sequence must be in [1, frame_group]")
    return ((frame_group << 4) | sequence) & 0xFF


class CanFdMessage:
    """
    One logical CAN FD frame carried over classic CAN 2.0 tunnel fragments.

    Parameters
    ----------
    arbitration_id
        CAN FD frame id (placed in the first fragment ``data[3:7]``).
    data
        CAN FD payload (0..64 bytes). Total frame count and tail CRC are derived
        automatically on construction.
    tunnel_id
        Classic CAN arbitration id used on the host bus (default ``0x711``).
    """

    def __init__(
        self,
        arbitration_id: int,
        data: bytes = b"",
        *,
        tunnel_id: int = DEFAULT_REQUEST_ID,
    ) -> None:
        if not 0 <= arbitration_id <= 0xFFFFFFFF:
            raise ValueError("arbitration_id must fit in 32 bits")
        payload = bytes(data)
        if len(payload) > MAX_CANFD_DATA_LEN:
            raise ValueError(
                f"CAN FD data length must be <= {MAX_CANFD_DATA_LEN}, got {len(payload)}"
            )

        self.arbitration_id = arbitration_id
        self.data = payload
        self._tunnel_id = tunnel_id
        self._fragments = self._build_fragments()

    @property
    def msgs(self) -> List[Message]:
        """Classic-CAN fragments ready to send (``is_extended_id`` is always False)."""
        return [
            Message(
                arbitration_id=self._tunnel_id,
                is_extended_id=False,
                data=fragment,
            )
            for fragment in self._fragments
        ]

    def _build_fragments(self) -> List[bytes]:
        framed_data = _pad_to_canfd_dlc(self.data)
        chunks = [
            framed_data[i : i + PAYLOAD_CHUNK_SIZE]
            for i in range(0, len(framed_data), PAYLOAD_CHUNK_SIZE)
        ]
        frame_group = 1 + len(chunks) + 1
        if frame_group > 15:
            raise ValueError(
                f"too many fragments: {frame_group}, protocol nibble limit is 15"
            )

        id_bytes = self.arbitration_id.to_bytes(4, "big")
        frames: List[bytes] = [
            bytes(
                (
                    _encode_sequence_byte(frame_group, 1),
                    FIRST_FRAME_MARKER[0],
                    FIRST_FRAME_MARKER[1],
                )
            )
            + id_bytes
        ]
        for index, chunk in enumerate(chunks, start=2):
            frames.append(
                bytes((_encode_sequence_byte(frame_group, index),)) + chunk
            )

        crc_bytes = _crc16_ccitt_false(framed_data).to_bytes(2, CRC16_BYTEORDER)
        frames.append(
            bytes((_encode_sequence_byte(frame_group, frame_group),)) + crc_bytes
        )
        return frames


class Can20CanfdRx:
    """
    Collect and reassemble one tunneled CAN FD reply from 0x712 fragments.

    Intended for use with the arm read-loop callback (``can_comm.recv`` →
    ``_trigger_callback``): register :meth:`feed` on the callback path, send
    request fragments via ``comm.send`` yourself, then :meth:`wait`.
    """

    class _Reassembler:
        """Incrementally reassemble one tunneled transaction from tunnel fragments."""

        def __init__(self) -> None:
            self.reset()

        def reset(self) -> None:
            self._frame_group: Optional[int] = None
            self._arbitration_id: Optional[int] = None
            self._next_sequence: Optional[int] = None
            self._payload = bytearray()

        def feed(self, _can_id: int, data: bytes) -> Optional[CanFdMessage]:
            if not data:
                return None

            encoded = data[0]
            encoded_group = encoded >> 4
            sequence = encoded & 0x0F
            if encoded_group < 1 or sequence == 0 or sequence > encoded_group:
                return None

            if sequence == 1:
                if len(data) != 7 or data[1:3] != FIRST_FRAME_MARKER:
                    return None

                self._frame_group = encoded_group
                self._arbitration_id = int.from_bytes(data[3:7], "big")
                self._payload.clear()
                self._next_sequence = 2
                return None

            if (
                self._frame_group is None
                or encoded_group != self._frame_group
                or sequence != self._next_sequence
                or self._arbitration_id is None
            ):
                return None

            if sequence == self._frame_group:
                body = bytes(self._payload)
                raw_crc = data[1:3]
                if len(raw_crc) == 2:
                    expected_crc = int.from_bytes(raw_crc, CRC16_BYTEORDER)
                    actual_crc = _crc16_ccitt_false(_pad_to_canfd_dlc(body))
                    if expected_crc != actual_crc:
                        print(
                            "[WARNING] tunnel CRC mismatch: "
                            f"received=0x{expected_crc:04X}, "
                            f"calculated=0x{actual_crc:04X}"
                        )
                return CanFdMessage(self._arbitration_id, body)

            self._payload.extend(data[1:])
            self._next_sequence = (self._next_sequence or 0) + 1
            return None

    def __init__(self, response_id: int = DEFAULT_RESPONSE_ID) -> None:
        self.response_id = response_id
        self._lock = threading.Lock()
        self._reassembler = self._Reassembler()
        self._pending: Optional[CanFdMessage] = None
        self._spare: Deque[CanFdMessage] = deque()
        self._ready = threading.Event()
        self._error: Optional[BaseException] = None

    def reset(self) -> None:
        with self._lock:
            self._reassembler.reset()
            self._pending = None
            self._spare.clear()
            self._error = None
            self._ready.clear()

    def feed(self, message: Message) -> None:
        if message.arbitration_id != self.response_id:
            return

        data = bytes(message.data)
        if not data:
            return

        with self._lock:
            sequence = data[0] & 0x0F
            if sequence == 1:
                if self._pending is not None:
                    self._spare.append(self._pending)
                    self._pending = None
                self._reassembler.reset()
                self._error = None
                self._ready.clear()
            elif self._ready.is_set():
                return

            try:
                result = self._reassembler.feed(
                    message.arbitration_id, data
                )
                if result is not None:
                    self._pending = CanFdMessage(
                        result.arbitration_id,
                        result.data,
                        tunnel_id=self.response_id,
                    )
                    self._ready.set()
            except BaseException as exc:
                self._error = exc
                self._ready.set()

    def wait(self, timeout: Optional[float] = None) -> Optional[CanFdMessage]:
        """Block until one complete reply arrives via :meth:`feed`."""
        return self.wait_matching(timeout, lambda _msg: True)

    def wait_matching(
        self,
        timeout: Optional[float],
        predicate: Callable[[CanFdMessage], bool],
    ) -> Optional[CanFdMessage]:
        """Block until a complete reply satisfying *predicate* arrives."""
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            with self._lock:
                for index, msg in enumerate(self._spare):
                    if predicate(msg):
                        del self._spare[index]
                        return msg

            complete = self._take_complete()
            if complete is not None:
                if predicate(complete):
                    return complete
                with self._lock:
                    self._spare.append(complete)
                continue

            if deadline is not None and time.monotonic() >= deadline:
                return None

            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return None

            wait_s = remaining if remaining is not None else 0.01
            self._ready.wait(timeout=min(wait_s, 0.01) if wait_s else 0.01)

    def _take_complete(self) -> Optional[CanFdMessage]:
        with self._lock:
            if self._error is not None:
                raise self._error
            if self._pending is None:
                return None
            msg = self._pending
            self._pending = None
            self._ready.clear()
            self._reassembler.reset()
            return msg


if __name__ == "__main__":
    print("can20_canfd demos (no hardware)")

    # ------------------------------------------------------------------
    # Example 1: fragment a short CAN FD message onto classic CAN 0x711
    # ------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("Example 1: short payload (single data frame)")
    short = CanFdMessage(0x007E0806, bytes.fromhex("7e0400000006"))
    print(f"  CAN FD id = 0x{short.arbitration_id:08X}")
    print(f"  payload   = {short.data.hex()} ({len(short.data)} bytes)")
    print(f"  fragments on 0x{short.msgs[0].arbitration_id:03X} x {len(short.msgs)}:")
    for index, msg in enumerate(short.msgs, start=1):
        print(f"    [{index}] data = {bytes(msg.data).hex()}")

    # ------------------------------------------------------------------
    # Example 2: fragment a long payload into multiple data frames
    # ------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("Example 2: long payload (multiple data frames)")
    long_payload = b"Revo2 touch tunnel demo: " + bytes(range(16))
    long_msg = CanFdMessage(0x007E081F, long_payload)
    print(f"  CAN FD id = 0x{long_msg.arbitration_id:08X}")
    print(f"  payload   = {long_msg.data.hex()} ({len(long_msg.data)} bytes)")
    print(f"  fragments on 0x{long_msg.msgs[0].arbitration_id:03X} x {len(long_msg.msgs)}:")
    for index, msg in enumerate(long_msg.msgs, start=1):
        print(f"    [{index}] data = {bytes(msg.data).hex()}")

    # ------------------------------------------------------------------
    # Example 3: reassemble a synthetic 0x712 reply (feed + wait)
    # Tunnel layer returns full (id, data); id semantics are vendor-specific.
    # ------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("Example 3: rx roundtrip via feed() + wait()")
    payload = bytes.fromhex("7e040000000c010203040506")  # 12 bytes
    logical = CanFdMessage(0x007E0806, payload)  # id low byte need not match len
    response = CanFdMessage(
        logical.arbitration_id,
        logical.data,
        tunnel_id=DEFAULT_RESPONSE_ID,
    )
    rx = Can20CanfdRx()
    rx.reset()
    for fragment in response.msgs:
        rx.feed(fragment)
    got = rx.wait(timeout=1.0)
    assert got is not None
    assert got.arbitration_id == logical.arbitration_id
    assert got.data == logical.data
    print(f"  reassembled id      = 0x{got.arbitration_id:08X}")
    print(f"  reassembled payload = {got.data.hex()}")

    # ------------------------------------------------------------------
    # Example 4: async callback delivery while wait() blocks
    # ------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("Example 4: rx async feed (read-loop callback simulation)")
    payload = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09"
    arb_id = 0x007F0A0B  # opaque to tunnel; only framing + tail CRC matter
    rx = Can20CanfdRx()
    rx.reset()

    def _late_reply() -> None:
        time.sleep(0.05)
        late_response = CanFdMessage(arb_id, payload, tunnel_id=DEFAULT_RESPONSE_ID)
        for fragment in late_response.msgs:
            rx.feed(fragment)

    threading.Thread(target=_late_reply, daemon=True).start()
    t0 = time.monotonic()
    got = rx.wait(timeout=1.0)
    elapsed_ms = (time.monotonic() - t0) * 1000.0
    assert got is not None
    print(f"  waited {elapsed_ms:.1f} ms for callback delivery")
    print(f"  reassembled payload = {got.data.hex()}")

    print("\n" + "=" * 60)
    print("All demos OK.")
