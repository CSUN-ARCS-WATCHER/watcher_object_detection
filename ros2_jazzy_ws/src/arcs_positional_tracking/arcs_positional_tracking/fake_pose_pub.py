import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakePosePublisher(Node):
    def __init__(self):
        super().__init__('fake_pose_pub')
        self.pub = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.get_logger().info('FakePosePublisher started, publishing on /zed/zed_node/pose')

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Simple circle path: radius 1.0 m
        msg.pose.position.x = math.cos(t) * 1.0
        msg.pose.position.y = math.sin(t) * 1.0
        msg.pose.position.z = 0.0

        # Just identity orientation for now
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().debug(f'Published fake pose at t={t:.2f}s')


def main(args=None):
    rclpy.init(args=args)
    node = FakePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
