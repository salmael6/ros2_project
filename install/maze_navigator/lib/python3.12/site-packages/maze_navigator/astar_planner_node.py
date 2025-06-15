import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        self.subscription = self.create_subscription(OccupancyGrid, 'occupancy_grid', self.grid_callback, 10)
        self.publisher = self.create_publisher(Path, 'planned_path', 10)

        # Define start and goal in grid coordinates (adjust as needed)
        self.start = (10, 10)
        self.goal = (80, 80)
        self.grid = None

    def grid_callback(self, msg: OccupancyGrid):
        self.grid = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        path = self.astar(self.start, self.goal)
        if path:
            self.publish_path(path)
        else:
            self.get_logger().info('No path found')

    def neighbors(self, node):
        x, y = node
        nbrs = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                idx = ny * self.width + nx
                if self.grid[idx] == 0:  # free cell
                    nbrs.append((nx, ny))
        return nbrs

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, cost, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x * self.resolution + self.origin_x + self.resolution/2
            pose.pose.position.y = y * self.resolution + self.origin_y + self.resolution/2
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

