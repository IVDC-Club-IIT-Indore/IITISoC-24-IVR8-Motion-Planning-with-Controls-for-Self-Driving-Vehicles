# IITISoC-24-IVR8-Motion-Planning-with-Controls-for-Self-Driving-Vehicles

## Goal:
To develop a reliable Motion Planner with Control algorithms for collision-free path planning and efficient navigation in a simulated environment.

People Involved : 

Mentors:
- [Arjun S Nair](https://github.com/arjun-593)
- [Ampady B R](https://github.com/ampady06)

Members:
- [Zaryan Ali Ansari](https://github.com/ghostzaryan)
- [Manan Nigam](https://github.com/MananNigam)
- [Mohd Sharik Mansoori](https://github.com/Sharik-18)
<br>

## Intro
This repository contains the implementation of a path planning system using the A* algorithm and a PID based control system for self driving vehicals.The system is developed using ROS2, and the planned path is visualised using RViz.

## Overview
The project consists of two main components:

- Path Planning Node: Implements the A* and RRT* algorithm to generate a path from a start point to a goal point and publishes the path as a planned_path/Path message.
- Control System Node: Subscribes to the planned path and generates velocity commands (cmd_vel/Twist) to follow the path. It uses PID control to maintain a specific speed and stabalise the movement of vehical.

## Prerequisites
- ROS2 Humble
- Python3
- Gazebo
- RViz

## Appraoch
- ROS2 humble is installed and the environment is setup to make the files and run packages.
- The worlds are mapped using slam toolbox and turtlebot3.
- The ymal and pgm file are then used for path planning.
- The pgm file is converted to binary occupancy grid and then feed to the path planning algorith( A* and RRT*).
- A node is build to publish the planned path.
- The control node is then build which subscribes to the planned_path topic and publishes it to com_vel

## Algorithms Used

### A* Algorithm
The A* Algorithm is an informed search algorithm widely used for pathfinding and graph traversal. It finds the shortest path from a start node to a goal node by combining the strengths of Dijkstra's Algorithm and Greedy Best-First Search.

It works in the following basic steps:
 - **Initialization:** A* starts with a priority queue (usually a min-heap) that initially contains the starting node. Each node has a cost associated with it, which is the sum of two components:
    * g(n): The cost from the start node to the current node n.\
    * h(n): A heuristic estimate of the cost from the current node n to the goal node.
- **Priority Queue:** Nodes are processed based on their total estimated cost, f(n) = g(n) + h(n). The node with the lowest f(n) value is selected for expansion.
- **Expansion:** The selected node is removed from the priority queue and added to a "closed set" to prevent reprocessing. The algorithm then examines its neighbors (adjacent nodes).
- **Neighbor Evaluation:** For each neighbor, the algorithm calculates the tentative cost g(n) to reach that neighbor. If this cost is lower than any previously recorded cost for that neighbor, the neighbor is updated with this lower cost, and its new total estimated cost f(n) is calculated.
- **Update Queue:** If the neighbor node is not in the priority queue, it is added; if it is already in the queue with a higher cost, its position is updated.
- **Termination:** This process continues until the goal node is selected for expansion (indicating the path has been found) or the priority queue is empty (indicating no path exists).

### RRT* Algorithm
The RRT* (Rapidly-exploring Random Tree Star) algorithm is an extension of the RRT (Rapidly-exploring Random Tree) algorithm, designed to find an optimal path from a start node to a goal node in a continuous space.

RRT* improves upon RRT by not just finding a feasible path, but iteratively refining the tree to find an optimal path. It achieves this through the addition of the rewiring step, which optimizes the path cost for each newly added node by considering alternative routes through the new node.

It works in the following basic steps:
- **Initialization:** Start with an initial tree containing the start node as the root.
- **Sampling:** Randomly sample a point in the search space.
- **Nearest Neighbour:** Find the node in the tree that closest to the sampled point.
- **Steering:** Move from the tree node towards the sampled node by a small distance to producce a new node
- **Rewiring:** Identify a set of nodes within a radius of the new node. Choose the node form this set that minimizes the cost to reach the newly sampled node when connnected.
- **Optimization:** For each node in the tree check if connecting the nodes in the nearby nodes set to the newly sampled node would reduce the cost to reach them. If so, rewire the tree by changing its parent to new node.
- **Iteration:** Repeat this process until a stopping condition is met.
