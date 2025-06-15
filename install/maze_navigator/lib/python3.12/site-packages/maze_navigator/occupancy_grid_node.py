import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped, PoseStamped # For transforming points
from tf_transformations import euler_from_quaternion # For converting quaternion to Euler

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # Map parameters
        self.grid_size = 200  # cells (e.g., 200x200)
        self.resolution = 0.05  # 5cm per cell (0.05m)
        self.map_width_m = self.grid_size * self.resolution # 10 meters
        self.map_height_m = self.grid_size * self.resolution # 10 meters

        # Initialize the grid: -1 = unknown, 0 = free, 100 = occupied
        # The grid is centered at (0,0) in the 'map' frame
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Map origin in 'map' frame (bottom-left corner of the grid)
        # This defines where (0,0) of the grid array corresponds to in the 'map' frame
        self.origin_x = -self.map_width_m / 2.0
        self.origin_y = -self.map_height_m / 2.0

        # Publisher for the OccupancyGrid
        # QoS history is set to 'keep all' for transient local durability to ensure RViz receives it
        # even if it starts later, and reliability to 'reliable'.
        self.publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid', rclpy.qos.qos_profile_default)

        # Subscriber for LaserScan messages
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)

        # TF2 buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Occupancy Grid Node has been initialized.')

    def _world_to_grid(self, x_world, y_world):
        """Converts world coordinates (m) to grid coordinates (cells)."""
        ix = int((x_world - self.origin_x) / self.resolution)
        iy = int((y_world - self.origin_y) / self.resolution)
        return ix, iy

    def scan_callback(self, msg: LaserScan):
        frame_id = msg.header.frame_id
        if frame_id.endswith('/gpu_lidar'):
            frame_id = frame_id.rsplit('/gpu_lidar', 1)[0]
        if frame_id.startswith('self_balancing_bot/'):
            frame_id = frame_id.replace('self_balancing_bot/', '', 1)
        # Look up the transform from the scan's frame_id (chassis) to the map frame
        try:
            # Get the transform at the time the scan was taken
            transform = self.tf_buffer.lookup_transform(
                'map', frame_id, msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {frame_id} to map: {ex}')
            return

        # Extract robot's pose in map frame (for transforming scan points)
        robot_x_map = transform.transform.translation.x
        robot_y_map = transform.transform.translation.y
        
        # Convert quaternion to yaw (robot's orientation in map frame)
        quat = transform.transform.rotation
        _, _, robot_yaw_map = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # Process each laser scan point
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Convert polar coordinates to Cartesian in the robot's local frame
                x_robot_local = r * math.cos(angle)
                y_robot_local = r * math.sin(angle)

                # Transform local point to map frame using robot's pose
                x_map = robot_x_map + x_robot_local * math.cos(robot_yaw_map) - y_robot_local * math.sin(robot_yaw_map)
                y_map = robot_y_map + x_robot_local * math.sin(robot_yaw_map) + y_robot_local * math.cos(robot_yaw_map)

                # Convert map coordinates to grid indices
                ix, iy = self._world_to_grid(x_map, y_map)

                # Mark the cell as occupied if within grid bounds
                if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
                    self.grid[iy, ix] = 100 # Mark occupied

            # Increment angle for the next scan point
            angle += msg.angle_increment
        
        # If you want to mark free space, you would implement ray tracing (e.g., Bresenham's)
        # from the robot's position to each occupied cell, marking cells along the ray as free (0).
        # For simplicity, this example only marks occupied cells.

        # Publish the updated occupancy grid
        self._publish_map()

    def _publish_map(self):
        """Publishes the current state of the occupancy grid."""
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map' # This grid is in the 'map' frame

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        
        # Origin of the map (bottom-left corner of the grid) in the 'map' frame
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0 # No rotation for the map itself

        grid_msg.data = self.grid.flatten().tolist()
        self.publisher.publish(grid_msg)
        self.get_logger().info(f"Published map with {np.count_nonzero(self.grid == 100)} occupied cells.")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

