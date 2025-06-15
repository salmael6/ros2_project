import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class RealTimePathPublisher(Node):
    def __init__(self):
        super().__init__('real_time_path_publisher')
        self.path = Path()
        self.path.header.frame_id = 'map'  

        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.last_pose = None
        self.min_distance = 0.05  # seuil pour ajouter un nouveau point (en mètres)

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Ajouter un point seulement si le robot a bougé suffisamment
        if self.last_pose is None or self.distance(pose.pose.position, self.last_pose.position) > self.min_distance:
            self.path.poses.append(pose)
            self.last_pose = pose.pose
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)

    def distance(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return (dx*dx + dy*dy + dz*dz)**0.5

def main(args=None):
    rclpy.init(args=args)
    node = RealTimePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

