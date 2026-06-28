#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class GuiPlaceholderNode(Node):
    """Placeholder for future PySide6/rqt GUI integration."""

    def __init__(self):
        super().__init__('gui_placeholder_node')
        self.get_logger().info('KRAI GUI placeholder started')
        self.get_logger().info('Future GUI panels: connection, pose, base, primitive, mission, safety, Slave B placeholder')


def main(args=None):
    rclpy.init(args=args)
    node = GuiPlaceholderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
