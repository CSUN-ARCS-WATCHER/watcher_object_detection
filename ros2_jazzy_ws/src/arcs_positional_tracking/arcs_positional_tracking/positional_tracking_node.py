import os
import csv
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

class PositionalTrackingNode(Node):
    def __init__(self):
        super().__init__('positional_tracking_node')

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.pose_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/watcher/pose_path', 10)
        self.yaw_pub = self.create_publisher(Float32, '/watcher/yaw', 10)
        self.speed_pub = self.create_publisher(Float32, '/watcher/speed', 10)

        # Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # For speed computation
        self.last_time_sec = None
        self.last_x = None
        self.last_y = None

        # CSV Logging setup
        log_dir = '/ros2_ws/pose_logs'
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, 'pose_log.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # header
        self.csv_writer.writerow(['time_sec', 'x', 'y', 'z', 'yaw_rad', 'speed_m_s'])

        self.get_logger().info(
            f'PositionalTrackingNode started. Logging poses to {self.csv_path}'
        )

    # Callbacks
    def pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation

        # Time (seconds)
        t_sec = self.get_clock().now().nanoseconds / 1e9

        # Orientation to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Speed computation
        speed = 0.0
        if self.last_time_sec is not None and self.last_x is not None and self.last_y is not None:
            dt = t_sec - self.last_time_sec
            if dt > 1e-6:
                dx = p.x - self.last_x
                dy = p.y - self.last_y
                dist = math.sqrt(dx * dx + dy * dy)
                speed = dist / dt

        # Update stored previous pose/time
        self.last_time_sec = t_sec
        self.last_x = p.x
        self.last_y = p.y

        # Console log
        self.get_logger().info(
            f"Pose → x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}, yaw={yaw:.2f} rad, speed={speed:.2f} m/s"
        )

        # Publishers
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        pose_copy = PoseStamped()
        pose_copy.header = msg.header
        pose_copy.pose = msg.pose

        self.path_msg.poses.append(pose_copy)
        self.path_pub.publish(self.path_msg)

        yaw_msg = Float32()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)

        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)

        #Write to CSV log
        self.csv_writer.writerow([t_sec, p.x, p.y, p.z, yaw, speed])

    def odom_callback(self, msg: Odometry):
        # keeping this for future odom-based logic if needed
        p = msg.pose.pose.position
        self.get_logger().debug(
            f"Odom → x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
        )

    # Cleanup on shutdown
    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            self.get_logger().info(f'Closing CSV log {self.csv_path}')
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PositionalTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down node')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Ignore double-shutdown or other shutdown-related errors
            pass



if __name__ == '__main__':
    main()
