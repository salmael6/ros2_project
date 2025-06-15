import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
from PIL import Image
import numpy as np

class SimpleMapPublisher(Node):
    def __init__(self):
        super().__init__('simple_map_publisher')

        self.publisher = self.create_publisher(OccupancyGrid, 'map_grid', 10)

        # Publier la carte périodiquement (chaque 1 seconde)
        self.timer = self.create_timer(1.0, self.publish_map)

        # Charger le fichier YAML
        map_yaml_path = '/home/salma/ros2_sb/src/self_balancing_bot/worlds/map.yaml'  
        with open(map_yaml_path, 'r') as file:
            self.map_metadata = yaml.safe_load(file)

        # harger l'image PGM
        image_path = self.map_metadata['image']
        image = Image.open(image_path)
        image = image.transpose(Image.FLIP_TOP_BOTTOM)  # RViz convention
        image_data = np.array(image)

        # Convertir l'image en données de grille
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.info.resolution = self.map_metadata['resolution']
        self.grid.info.width = image_data.shape[1]
        self.grid.info.height = image_data.shape[0]
        self.grid.info.origin.position.x = self.map_metadata['origin'][0]
        self.grid.info.origin.position.y = self.map_metadata['origin'][1]
        self.grid.info.origin.position.z = 0.0
        self.grid.info.origin.orientation.w = 1.0

        # Convertir en tableau 1D
        flat_data = 100 - image_data.flatten()
        flat_data = np.clip(flat_data, 0, 100)
        self.grid.data = flat_data.tolist()
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.grid)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

