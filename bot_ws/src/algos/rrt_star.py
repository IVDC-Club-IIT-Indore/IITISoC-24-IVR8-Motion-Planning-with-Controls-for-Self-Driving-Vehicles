import random
import math
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, occupancy_grid, expand_dist = 1.0, goal_sampling_chance = 5, max_iter = 500):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.expand_dist = expand_dist
        self.goal_sampling_chance = goal_sampling_chance
        self.max_iter = max_iter
        self.occupancy_grid = occupancy_grid
        self.node_list = [self.start]
    
    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(0, self.occupancy_grid.shape[1]), random.uniform(0, self.occupancy_grid.shape[0])]
        else:
            rnd = [self.goal.x, self.goal.y]
        return Node(rnd[0], rnd[1])
    
    def get_nearest_node_index(self, node_list, random_node):
        dist_list = [(node.x - random_node.x) ** 2 + (node.y - random_node.y) ** 2 for node in node_list]
        nearest_node_index = dist_list.index(min(dist_list))
        return nearest_node_index
    
    def dist_angle_between_nodes(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return dist, theta
    
    def generate_new_node(self, from_node, to_node, expand_dist):
        new_node = Node(from_node.x, from_node.y)
        dist, theta = self.dist_angle_between_nodes(from_node, to_node)
        new_node.x += min(expand_dist, dist) * math.cos(theta)
        new_node.y += min(expand_dist, dist) * math.sin(theta)
        new_node.cost = from_node.cost + self.dist_angle_between_nodes(from_node, new_node)[0]
        new_node.parent = from_node
        return new_node
    
    def is_colliding(self, node):
        if int(node.x) < 0 or int(node.x) >= self.occupancy_grid.shape[1] or int(node.y) < 0 or int(node.y) >= self.occupancy_grid.shape[0]:
            return True
        return self.occupancy_grid[int(node.y)][int(node.x)] == 1
    
    def get_nearby_nodes(self, new_node):
        num_of_nodes = len(self.node_list)
        r = self.expand_dist * math.sqrt(math.log(num_of_nodes) / num_of_nodes)
        dist_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
        near_index_list = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_index_list
    
    def choose_parent(self, new_node, near_index_list):
        if not near_index_list:
            return None
        
        cost_list = []
        for i in near_index_list:
            near_node = self.node_list[i]
            if self.is_colliding(self.generate_new_node(near_node, new_node), self.obstacle_list):
                cost_list.append(float('inf'))
            else:
                cost_list.append(near_node.cost + self.dist_angle_between_nodes(near_node, new_node)[0])
        
        min_cost = min(cost_list)
        min_index = near_index_list[cost_list.index(min_cost)]
        if min_cost == float('inf'):
            return None
        new_node = self.generate_new_node(self.node_list[min_index], new_node)
        return new_node
    
    def propagate_cost_updates(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = parent_node.cost + self.dist_angle_between_nodes(parent_node, node)[0]
                self.propagate_cost_updates(node)

    def rewire_tree(self, new_node, near_index_list):
        for i in near_index_list:
            near_node = self.node_list[i]
            edge_node = self.generate_new_node(new_node, near_node)
            edge_node.cost = new_node.cost + self.dist_angle_between_nodes(new_node, near_node)[0]
            if self.is_colliding(edge_node, self.obstacle_list):
                continue

            if near_node.cost > edge_node.cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.parent = new_node
                self.propagate_cost_updates(new_node)
    
    def search_best_goal_node(self):
        dist_to_goal_list = [self.dist_angle_between_nodes(node, self.goal)[0] for node in self.node_list]
        goal_index_list = [dist_to_goal_list.index[i] for i in dist_to_goal_list if i <= self.expand_dist]
        if not goal_index_list:
            return None
        min_cost = min([self.node_list[i].cost for i in goal_index_list])
        for i in goal_index_list:
            if self.node_list[i].cost == min_cost:
                return i
        return None
    
    def generate_final_course(self, goal_index):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path
    
    def rrt_srat(self):
        for i in range(self.max_iter):
            random_node = self.get_random_node()
            nearest_index = self.get_nearest_node_index(self.node_list, random_node)
            nearest_node = self.node_list[nearest_index]
            new_node = self.generate_new_node(nearest_node, random_node, self.expand_dist)
            if not self.is_colliding(new_node, self.obstacle_list):
                near_index_list = self.get_nearby_nodes(new_node)
                new_node = self.choose_parent(new_node, near_index_list)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire_tree(new_node, near_index_list)
            
            if i % 10 == 0:
                print(f"Iter: {i}, number of nodes: {len(self.node_list)}")
            
            goal_index = self.search_best_goal_node()
            if goal_index:
                return self.generate_final_course(goal_index)
            return None
