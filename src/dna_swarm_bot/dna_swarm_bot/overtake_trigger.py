#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys
import select
import termios
import tty

class OvertakeTrigger(Node):
    def __init__(self):
        super().__init__('overtake_trigger')
        self.overtake_pub = self.create_publisher(Empty, '/overtake_trigger', 10)
        self.resume_pub = self.create_publisher(Empty, '/resume_trigger', 10)
        self.timer = self.create_timer(0.1, self.check_input)
        self.get_logger().info("=== Overtake Trigger ===")
        self.get_logger().info("Press [o] to trigger overtake")
        self.get_logger().info("Press [r] to resume from wall stop")
        self.get_logger().info("Press [q] to quit")
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def check_input(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'o':
                self.overtake_pub.publish(Empty())
                self.get_logger().info("Overtake triggered!")
            elif key == 'r':
                self.resume_pub.publish(Empty())
                self.get_logger().info("Resume triggered!")
            elif key == 'q':
                self.get_logger().info("Quitting...")
                rclpy.shutdown()

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OvertakeTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
