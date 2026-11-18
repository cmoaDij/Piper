#!/usr/bin/env python3
"""Publish atomic world grip/aim/button frames from one Quest ADB reader."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from data_collect.sensors.quest_world import QuestWorldReader


class QuestWorldPublisher(Node):
    def __init__(self):
        super().__init__("pub_pose_node")
        self.reader = QuestWorldReader()
        self.publisher = self.create_publisher(String, "/quest/world_input", 1)
        self.last_sequence = None
        self.last_error = None
        self.timer = self.create_timer(0.01, self.tick)
        self.get_logger().info("[设备] Quest 世界模式：OpenXR LOCAL；首次按 A 时将手柄水平指向基座 +X")

    def tick(self):
        if self.reader.error and self.reader.error != self.last_error:
            self.get_logger().error(self.reader.error)
            self.last_error = self.reader.error
        packet = self.reader.latest()
        if packet is None:
            return
        sequence = (packet["session"], packet["generation"], packet["seq"])
        if sequence == self.last_sequence:
            return
        self.last_sequence = sequence
        self.publisher.publish(String(data=json.dumps(packet, separators=(",", ":"), allow_nan=False)))

    def destroy_node(self):
        self.reader.stop()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = QuestWorldPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
