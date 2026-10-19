#!/usr/bin/python3
"""Record the color stream from a RealSense camera to an MP4 video.

This node deliberately uses the system Python interpreter. The existing Quest
nodes can keep using a Conda environment, while Ubuntu's ROS/OpenCV packages
perform video encoding without sharing that environment's NumPy stack.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS image-header timestamp to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RealSenseVideoRecorder(Node):
    """Subscribe to a D435 color topic and write real-time-compatible video."""

    def __init__(self) -> None:
        super().__init__("realsense_video_recorder")
        self.declare_parameter("color_topic", "/camera/wrist_camera/color/image_raw")
        self.declare_parameter("record_topic", "/realsense/record_video")
        self.declare_parameter("output_dir", "~/QuestArmTeleop/recordings")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("codec", "avc1")
        self.declare_parameter("record_on_start", False)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.fps = float(self.get_parameter("fps").value)
        self.codec = str(self.get_parameter("codec").value)
        if self.fps <= 0.0:
            raise ValueError("fps must be greater than zero.")
        if len(self.codec) != 4:
            raise ValueError("codec must be a four-character FourCC value, such as avc1.")

        self.recording = bool(self.get_parameter("record_on_start").value)
        self.writer: cv2.VideoWriter | None = None
        self.output_path: Path | None = None
        self.frame_size: tuple[int, int] | None = None
        self._reset_timing()
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("record_topic").value),
            self._on_record_command,
            10,
        )
        self.get_logger().info(
            "RealSense recorder ready. "
            f"topic={self.get_parameter('color_topic').value}, "
            f"control={self.get_parameter('record_topic').value}, output={self.output_dir}"
        )
        if self.recording:
            self.get_logger().info("Recording will start when the first color frame arrives.")

    def _on_record_command(self, message: Bool) -> None:
        if message.data and not self.recording:
            self.recording = True
            self._reset_timing()
            self.get_logger().info("Video recording armed; waiting for the next color frame.")
        elif not message.data and self.recording:
            self.recording = False
            self._close_writer()

    def _on_image(self, message: Image) -> None:
        if not self.recording:
            return
        try:
            frame = self._to_bgr(message)
            if self.writer is None:
                self._open_writer(frame)
            if self.frame_size != (frame.shape[1], frame.shape[0]):
                raise ValueError("Camera frame size changed during recording.")
            self._write_timed_frame(frame, stamp_to_seconds(message.header.stamp))
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f"Stopped video recording: {error}")
            self.recording = False
            self._close_writer()

    @staticmethod
    def _to_bgr(message: Image) -> np.ndarray:
        """Convert common ROS 8-bit colour encodings without cv_bridge."""
        encoding = message.encoding.lower()
        channel_count = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
        if channel_count is None:
            raise ValueError(f"Unsupported color encoding '{message.encoding}'; expected RGB8 or BGR8.")
        row_width = message.width * channel_count
        if message.step < row_width or len(message.data) < message.height * message.step:
            raise ValueError("Image data is shorter than its declared dimensions.")
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, :row_width].reshape(message.height, message.width, channel_count)
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    def _open_writer(self, frame: np.ndarray) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output_path = self.output_dir / f"d435_{timestamp}.mp4"
        self.frame_size = (frame.shape[1], frame.shape[0])
        self.writer = cv2.VideoWriter(
            str(self.output_path),
            cv2.VideoWriter_fourcc(*self.codec),
            self.fps,
            self.frame_size,
        )
        if not self.writer.isOpened():
            self.writer.release()
            self.writer = None
            raise ValueError(f"Could not open video file '{self.output_path}' with codec {self.codec}.")
        self.get_logger().info(f"Writing video to {self.output_path} at {self.fps:g} FPS.")

    def _reset_timing(self) -> None:
        self.frame_count = 0
        self.timestamp_fill_frames = 0
        self.first_image_stamp: float | None = None
        self.last_image_stamp: float | None = None
        self.last_frame: np.ndarray | None = None

    def _write_timed_frame(self, frame: np.ndarray, image_stamp: float) -> None:
        """Write at the slot selected by ``Image.header.stamp``.

        OpenCV's MP4 writer has a fixed frame rate. Missing source frames are
        therefore filled with the previous frame, preserving wall-clock
        duration without relying on poorly supported variable-frame-rate MP4.
        """
        if not np.isfinite(image_stamp) or image_stamp <= 0.0:
            raise ValueError("Image.header.stamp must be a positive finite timestamp.")
        if self.writer is None:
            raise RuntimeError("Video writer is not open.")
        if self.first_image_stamp is None:
            self.first_image_stamp = image_stamp
            target_index = 0
        else:
            target_index = int(round((image_stamp - self.first_image_stamp) * self.fps))
        self.last_image_stamp = image_stamp

        if target_index < self.frame_count:
            # Multiple camera frames can map to one 1/FPS output slot. Keep
            # the newest frame for the next slot instead of speeding up video.
            self.last_frame = frame
            return
        while self.frame_count < target_index:
            if self.last_frame is None:
                raise RuntimeError("No previous frame is available for timestamp fill.")
            self.writer.write(self.last_frame)
            self.frame_count += 1
            self.timestamp_fill_frames += 1
        self.writer.write(frame)
        self.frame_count += 1
        self.last_frame = frame

    def _close_writer(self) -> None:
        if self.writer is None:
            return
        self.writer.release()
        source_duration = 0.0
        if self.first_image_stamp is not None and self.last_image_stamp is not None:
            source_duration = max(0.0, self.last_image_stamp - self.first_image_stamp) + 1.0 / self.fps
        self.get_logger().info(
            f"Saved {self.frame_count} frames to {self.output_path} "
            f"(source duration={source_duration:.3f}s, "
            f"timestamp-fill frames={self.timestamp_fill_frames})."
        )
        self.writer = None
        self.output_path = None
        self.frame_size = None
        self.last_frame = None

    def destroy_node(self):
        self._close_writer()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
