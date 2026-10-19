"""Short-lived ZeroMQ requests with strict timeout handling."""

from __future__ import annotations

import json
import time

import numpy as np

from .contract import ACTION_DIM, STATE_DIM


MAGIC = b"PIPER_LEROBOT_V3"


class PolicyClient:
    """Call a local inference server with strict dimensions and request IDs."""

    def __init__(
        self,
        endpoint: str,
        timeout_ms: int = 200,
        *,
        state_dim: int = STATE_DIM,
        image_shape: tuple[int, int, int] = (240, 320, 3),
    ) -> None:
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        self.endpoint = endpoint
        self.timeout_ms = timeout_ms
        if state_dim < 1 or len(image_shape) != 3 or image_shape[2] != 3:
            raise ValueError("invalid policy observation contract")
        self.state_dim = int(state_dim)
        self.image_shape = tuple(int(value) for value in image_shape)

    def _exchange(self, parts: list[bytes]) -> list[bytes]:
        import zmq

        context = zmq.Context.instance()
        socket = context.socket(zmq.REQ)
        socket.linger = 0
        try:
            socket.connect(self.endpoint)
            socket.send_multipart(parts)
            if not socket.poll(self.timeout_ms, zmq.POLLIN):
                raise TimeoutError(
                    "policy server did not respond within "
                    f"{self.timeout_ms} ms"
                )
            return socket.recv_multipart()
        finally:
            socket.close()

    def infer(
        self,
        request_id: int,
        state,
        wrist_rgb,
        global_rgb,
        wrist_timestamp: float,
        global_timestamp: float,
        state_timestamp: float,
    ) -> tuple[np.ndarray, dict]:
        state = np.asarray(state, dtype=np.float32)
        wrist_rgb = np.asarray(wrist_rgb, dtype=np.uint8)
        global_rgb = np.asarray(global_rgb, dtype=np.uint8)
        if state.shape != (self.state_dim,) or not np.isfinite(state).all():
            raise ValueError("invalid policy state")
        expected_rgb_shape = self.image_shape
        if (
            wrist_rgb.shape != expected_rgb_shape
            or global_rgb.shape != expected_rgb_shape
        ):
            raise ValueError("invalid multi-view policy RGB frames")
        observation_timestamp = (
            float(wrist_timestamp) + float(global_timestamp)
        ) * 0.5
        if (
            not np.isfinite(wrist_timestamp)
            or not np.isfinite(global_timestamp)
            or not np.isfinite(state_timestamp)
            or not np.isfinite(observation_timestamp)
        ):
            raise ValueError("invalid observation timestamps")
        header = {
            "request_id": int(request_id),
            "client_monotonic": time.monotonic(),
            "wrist_timestamp": float(wrist_timestamp),
            "global_timestamp": float(global_timestamp),
            "observation_timestamp": observation_timestamp,
            "state_timestamp": float(state_timestamp),
            "state_dim": self.state_dim,
            "image_shape": list(self.image_shape),
        }
        response = self._exchange(
            [
                MAGIC,
                json.dumps(header, separators=(",", ":")).encode(),
                state.tobytes(),
                wrist_rgb.tobytes(),
                global_rgb.tobytes(),
            ]
        )
        if len(response) != 3 or response[0] != MAGIC:
            raise ValueError("invalid policy server response")
        metadata = json.loads(response[1].decode())
        if "error" in metadata:
            raise RuntimeError(metadata["error"])
        if int(metadata.get("request_id", -1)) != request_id:
            raise ValueError("out-of-order policy response")
        try:
            shape = tuple(int(value) for value in metadata["shape"])
            response_observation_timestamp = float(
                metadata["observation_timestamp"]
            )
            execution_steps = int(metadata["execution_steps"])
            period_s = float(metadata["period_s"])
            policy_type = str(metadata["policy_type"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                "policy response is missing scheduling metadata"
            ) from error
        if (
            len(shape) != 2
            or shape[0] < 1
            or shape[1] != ACTION_DIM
            or not 1 <= execution_steps <= shape[0]
            or policy_type not in {"diffusion", "act"}
            or not np.isfinite(period_s)
            or period_s <= 0.0
            or abs(
                response_observation_timestamp - observation_timestamp
            ) > 1e-6
        ):
            raise ValueError("invalid policy scheduling metadata")
        actions = (
            np.frombuffer(response[2], dtype=np.float32)
            .copy()
            .reshape(shape)
        )
        if not np.isfinite(actions).all():
            raise ValueError("invalid policy action chunk")
        return actions, metadata

    def reset(self, request_id: int) -> None:
        header = {"request_id": int(request_id), "reset": True}
        response = self._exchange([MAGIC, json.dumps(header).encode()])
        if len(response) < 2 or response[0] != MAGIC:
            raise ValueError("invalid reset response")
        metadata = json.loads(response[1].decode())
        if "error" in metadata:
            raise RuntimeError(metadata["error"])
        if metadata.get("reset") is not True:
            raise ValueError("policy server did not acknowledge reset")
