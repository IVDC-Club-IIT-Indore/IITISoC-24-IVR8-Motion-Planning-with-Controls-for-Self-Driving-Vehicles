import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, MarkerArray
import numpy as np
import heapq
import cv2
from PIL import Image

class NodeAStar:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

def get_neighbors(node, grid, width, height):
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]
    for direction in directions:
        nx, ny = node.x + direction[0], node.y + direction[1]
        if 0 <= nx < width and 0 <= ny < height and grid[ny * width + nx] == 0:
            neighbors.append(NodeAStar(nx, ny, 0))
    return neighbors

def a_star(start, goal, grid, width, height):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        current = heapq.heappop(open_list)[1]

        if (current.x, current.y) == (goal.x, goal.y):
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]

        for neighbor in get_neighbors(current, grid, width, height):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                neighbor.cost = priority
                neighbor.parent = current
                heapq.heappush(open_list, (priority, neighbor))

    return []

def pgm_to_occupancy_grid(pgm_path, inflation_radius):
    try:
        image = Image.open(pgm_path)
        image = image.convert('L')
        width, height = image.size
        data = np.array(image)
        binary_grid = np.where(data > 127, 0, 1)
        kernel = np.ones((inflation_radius, inflation_radius), np.uint8)
        inflated_grid = cv2.dilate(binary_grid.astype(np.uint8), kernel, iterations=1)
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.resolution = 1.0
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.data = [100 if cell == 1 else 0 for cell in inflated_grid.flatten()]
        return occupancy_grid
    except Exception as e:
        print(f"Error converting PGM to OccupancyGrid: {e}")
        return None

class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.grid_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.marker_publisher_ = self.create_publisher(InteractiveMarker, 'interactive_marker', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every 1 second
        self.get_logger().info('AStarPathPlanner Node Initialized')
        self.get_logger().info('MarkerPublisher Node Initialized')

        # Set initial start and goal positions
        self.start = None
        self.goal = None

        self.grid = None

        # Create an interactive marker server
        self.server = InteractiveMarkerServer(self, 'marker_server')
        self.create_interactive_markers()

        # Subscribe to occupancy grid and trigger the planning process when updated
        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.occupancy_grid_callback, 10
        )

    def timer_callback(self):
        
        marker = InteractiveMarker()
        marker.header.frame_id = 'map'
        marker.name = 'example_marker'
        marker.description = 'An example interactive marker'
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale = 1.0

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.name = 'move_control'
        control.orientation.w = 1.0
        marker.controls.append(control)

        self.marker_publisher_.publish(marker)
        self.get_logger().info('Publishing Marker message to /interactive_marker')

        inflation_radius = 5  # Adjust this value based on your robot's size
        occupancy_grid = pgm_to_occupancy_grid('/home/manan/IITISOC/IITISoC-24-IVR8-Motion-Planning-with-Controls-for-Self-Driving-Vehicles/bot_ws/src/maps/housemap2.pgm', inflation_radius)
        if occupancy_grid:
            self.grid_publisher_.publish(occupancy_grid)
            self.get_logger().info('Publishing OccupancyGrid message to /map')
        else:
            self.get_logger().error('Failed to create OccupancyGrid from PGM')

        self.plan_path()

    def create_interactive_markers(self):
        # Create start marker
        self.start_marker = self.create_marker('start', 'Start Position', Point(x=0.0, y=0.0, z=0.0))
        self.server.insert(self.start_marker)
        self.server.setCallback(self.start_marker.name, self.process_feedback)

        # Create goal marker
        self.goal_marker = self.create_marker('goal', 'Goal Position', Point(x=1.0, y=1.0, z=0.0))
        self.server.insert(self.goal_marker)
        self.server.setCallback(self.goal_marker.name, self.process_feedback)

        self.server.applyChanges()

    def create_marker(self, name, description, position):
        marker = InteractiveMarker()
        marker.header.frame_id = 'map'
        marker.name = name
        marker.description = description
        marker.pose.position = position

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append(self.make_box(marker))
        marker.controls.append(control)

        return marker

    def make_box(self, msg):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def process_feedback(self, feedback):
        if feedback.marker_name == 'start':
            self.start = feedback.pose.position
        elif feedback.marker_name == 'goal':
            self.goal = feedback.pose.position

        if self.start and self.goal:
            self.plan_path()

    def occupancy_grid_callback(self, msg):
        self.grid = msg

    def plan_path(self):
        if not self.grid:
            return

        width = self.grid.info.width
        height = self.grid.info.height
        resolution = self.grid.info.resolution
        grid = self.grid.data

        start_x = int(self.start.x / resolution)
        start_y = int(self.start.y / resolution)
        goal_x = int(self.goal.x / resolution)
        goal_y = int(self.goal.y / resolution)

        start_node = NodeAStar(start_x, start_y, 0)
        goal_node = NodeAStar(goal_x, goal_y, 0)

        path = a_star(start_node, goal_node, grid, width, height)

        # Publish the path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = node.x * resolution
            pose.pose.position.y = node.y * resolution
            path_msg.poses.append(pose)
        self.publisher_.publish(path_msg)
        self.get_logger().info("Publishing Path")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
