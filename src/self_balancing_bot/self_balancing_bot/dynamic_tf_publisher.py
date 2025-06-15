import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class DynamicTfPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Abonnement au topic /odom pour récupérer la pose du robot
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def odom_callback(self, msg: Odometry):
        # Récupération de la position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Récupération de l'orientation quaternion
        q = msg.pose.pose.orientation
        # Conversion quaternion -> yaw (Euler)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Publier le transform avec la pose mise à jour
        self.publish_transform()

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'      # Frame parent
        t.child_frame_id = 'chassis'   # Frame enfant

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Conversion Euler yaw en quaternion
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f'Published transform map->chassis: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

