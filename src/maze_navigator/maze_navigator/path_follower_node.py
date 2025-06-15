import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        self.path = []
        self.current_index = 0
        self.safe_distance = 0.5  # meters

        self.path_sub = self.create_subscription(Path, 'robot_path', self.path_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.obstacle_close = False

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.current_index = 0

    def scan_callback(self, msg: LaserScan):
        self.obstacle_close = any(r < self.safe_distance for r in msg.ranges if r > 0)
        self.follow_path()

    def follow_path(self):
        twist = Twist()
        if not self.path or self.current_index >= len(self.path):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        if self.obstacle_close:
            # Stop and turn
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            return

        # Move toward current waypoint
        current_pose = self.path[self.current_index].pose.position
        # For simplicity, assume robot at origin facing x+
        dx = current_pose.x
        dy = current_pose.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        if distance < 0.1:
            self.current_index += 1
        else:
            twist.linear.x = 0.2
            twist.angular.z = angle_to_goal
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

