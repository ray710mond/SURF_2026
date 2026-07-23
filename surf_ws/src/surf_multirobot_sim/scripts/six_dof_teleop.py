#!/usr/bin/env python3
"""Keyboard teleoperation for all six components of geometry_msgs/Twist."""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist


BINDINGS = {
    'w': ('linear', 'x', 1.0), 's': ('linear', 'x', -1.0),
    'a': ('linear', 'y', 1.0), 'd': ('linear', 'y', -1.0),
    'r': ('linear', 'z', 1.0), 'f': ('linear', 'z', -1.0),
    'u': ('angular', 'x', 1.0), 'j': ('angular', 'x', -1.0),
    'i': ('angular', 'y', 1.0), 'k': ('angular', 'y', -1.0),
    'o': ('angular', 'z', 1.0), 'l': ('angular', 'z', -1.0),
}


def main():
    rclpy.init()
    node = rclpy.create_node('six_dof_teleop')
    topic = node.declare_parameter('topic', '/drone/cmd_vel').value
    linear_speed = node.declare_parameter('linear_speed', 1.0).value
    angular_speed = node.declare_parameter('angular_speed', 0.8).value
    publisher = node.create_publisher(Twist, topic, 10)
    old_settings = termios.tcgetattr(sys.stdin)
    print(
        '6-DOF teleop (body frame)\n'
        '  W/S: +X/-X    A/D: +Y/-Y    R/F: +Z/-Z\n'
        '  U/J: roll     I/K: pitch    O/L: yaw\n'
        '  Space: stop   Ctrl-C: quit'
    )
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            key = sys.stdin.read(1).lower()
            if key == '\x03':
                break
            command = Twist()
            if key in BINDINGS:
                group, axis, direction = BINDINGS[key]
                speed = linear_speed if group == 'linear' else angular_speed
                setattr(getattr(command, group), axis, direction * speed)
            publisher.publish(command)
    finally:
        publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
