import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        self.get_logger().info('Initialisation de static_map_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_map)
        try:
            self.map_msg = self.load_map('/home/salma/ros2_sb/src/self_balancing_bot/worlds/map.yaml')
        except Exception as e:
            self.get_logger().error(f'Erreur lors du chargement de la carte : {e}')
            self.map_msg = None


    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)
        img = Image.open(map_yaml['image'])
        img = img.convert('L')  # niveaux de gris
        data = np.array(img)
        occupancy_data = []
        for val in data.flatten():
            if val > 250:
                occupancy_data.append(0)    # libre
            elif val < 10:
                occupancy_data.append(100)  # occupé
            else:
                occupancy_data.append(-1)   # inconnu

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = map_yaml['resolution']
        msg.info.width = data.shape[1]
        msg.info.height = data.shape[0]
        msg.info.origin.position.x = map_yaml['origin'][0]
        msg.info.origin.position.y = map_yaml['origin'][1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy_data
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.map_msg)
        self.get_logger().info('Carte statique publiée')

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

