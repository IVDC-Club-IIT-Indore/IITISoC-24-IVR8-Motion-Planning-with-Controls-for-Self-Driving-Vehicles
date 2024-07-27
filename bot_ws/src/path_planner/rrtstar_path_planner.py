import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np
import random
import math

class RRTStarPathPlanner(Node):
    def __init__(self):
        super().__init__('rrtstar_path_planner')
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.marker_publisher = self.create_publisher(Marker, 'path_marker', 10)

        self.declare_parameter('start', [0, 0])
        self.declare_parameter('goal', [0, 0])
        self.grid = None
        self.resolution = 0.05  # Placeholder resolution, update with actual map resolution

        self.max_iter = 1000
        self.goal_sample_rate = 0.1
        self.expand_dis = 1.0
        self.max_nodes = 10000
        self.node_list = []

        self.get_logger().info('RRTStarPathPlanner Node Initialized')

    def map_callback(self, msg):
        self.get_logger().info('Map received')
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.grid = np.array(msg.data).reshape((height, width))

        self.get_logger().info('Occupancy grid updated')

    def run_rrtstar(self, start, goal):
        if self.grid is None:
            self.get_logger().warn('Grid is not initialized yet')
            return None

        start = self.world_to_grid(start)
        goal = self.world_to_grid(goal)

        self.node_list = [Node(start)]
        for i in range(self.max_iter):
            rnd = self.get_random_point(goal)
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd, self.expand_dis)
            if self.check_collision(new_node, self.grid):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

            if self.node_list[-1].cost > self.max_nodes:
                self.get_logger().warn('Reached maximum node limit')
                break

            if i % 100 == 0:
                self.get_logger().info(f'Iteration: {i}')

        last_index = self.search_best_goal_node(goal)
        if last_index is None:
            self.get_logger().warn('No path found')
            return None

        path = self.get_final_path(last_index)
        return path

    def get_random_point(self, goal):
        if random.random() > self.goal_sample_rate:
            rnd = [random.uniform(0, self.grid.shape[1]), random.uniform(0, self.grid.shape[0])]
        else:
            rnd = goal
        return rnd

    def get_nearest_node_index(self, node_list, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def steer(self, from_node, to_point, extend_length=float('inf')):
        new_node = Node(from_node)
        d, theta = self.calc_distance_and_angle(from_node, to_point)

        new_node.x += min(extend_length, d) * math.cos(theta)
        new_node.y += min(extend_length, d) * math.sin(theta)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        new_node.cost += d

        return new_node

    def calc_distance_and_angle(self, from_node, to_point):
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def check_collision(self, node, grid):
        if 0 <= node.x < grid.shape[1] and 0 <= node.y < grid.shape[0]:
            if grid[int(node.y)][int(node.x)] == 0:
                return True
        return False

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))  # radius
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return near_inds

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, [new_node.x, new_node.y])
            if self.check_collision(t_node, self.grid):
                costs.append(near_node.cost + self.calc_distance_and_angle(near_node, [new_node.x, new_node.y])[0])
            else:
                costs.append(float('inf'))

        min_cost = min(costs)
        min_ind = near_inds[costs.index(min_cost)]

        if min_cost == float('inf'):
            return None

        new_node = self.steer(self.node_list[min_ind], [new_node.x, new_node.y])
        new_node.cost = min_cost

        return new_node

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, [near_node.x, near_node.y])

            no_collision = self.check_collision(edge_node, self.grid)
            improved_cost = new_node.cost + self.calc_distance_and_angle(new_node, [near_node.x, near_node.y])[0]

            if no_collision and near_node.cost > improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = improved_cost
                near_node.parent = new_node

    def search_best_goal_node(self, goal):
        dist_to_goal = lambda node: self.calc_distance_and_angle(node, goal)[0]
        goal_inds = [i for i in range(len(self.node_list)) if dist_to_goal(self.node_list[i]) <= self.expand_dis]
        if not goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in goal_inds])
        for i in goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def get_final_path(self, goal_ind):
        path = [[self.node_list[goal_ind].x, self.node_list[goal_ind].y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            node = node.parent
            path.append([node.x, node.y])
        path.reverse()
        return path

    def world_to_grid(self, point):
        x = int((point[0] - self.origin_x) / self.resolution)
        y = int((point[1] - self.origin_y) / self.resolution)
        return (x, y)

    def grid_to_world(self, point):
        x = point[0] * self.resolution + self.origin_x
        y = point[1] * self.resolution + self.origin_y
        return (x, y)

    def publish_path(self, path):
        if path is None:
            self.get_logger().warn('No path to publish')
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for grid_point in path:
            if grid_point is None:
                self.get_logger().warn('Path contains None entries')
                return
            
            world_point = self.grid_to_world(grid_point)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = world_point[0]
            pose.pose.position.y = world_point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Path published')

    def publish_marker(self, path):
        if path is None:
            self.get_logger().warn('No path to publish as marker')
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        for grid_point in path:
            if grid_point is None:
                self.get_logger().warn('Path contains None entries')
                return

            world_point = self.grid_to_world(grid_point)
            p = Point()
            p.x = world_point[0]
            p.y = world_point[1]
            p.z = 0.0
            marker.points.append(p)

        self.marker_publisher.publish(marker)
        self.get_logger().info('Path marker published')

class Node:
    def __init__(self, point):
        self.x = point[0]
        self.y = point[1]
        self.cost = 0.0
        self.path_x = []
        self.path_y = []
        self.parent = None

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPathPlanner()
    
    # Simulated start and goal positions for testing
    start = [150, 100]
    goal = [250, 120]

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
        if node.grid is not None:
            path = node.run_rrtstar(start, goal)
            node.publish_path(path)
            node.publish_marker(path)
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
