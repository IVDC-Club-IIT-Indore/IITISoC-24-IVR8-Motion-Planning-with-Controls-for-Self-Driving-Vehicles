import math

def dist(pos, goal_pos):
    return math.sqrt((pos[0] - goal_pos[0]) ** 2 + (pos[1] - goal_pos[1]) ** 2)

def get_neighbors(cell, width, height):
    neighbors = []

    up = [cell[0] - 1, cell[1]]
    up_right = [cell[0] - 1, cell[1] + 1]
    right = [cell[0], cell[1] + 1]
    down_right = [cell[0] + 1, cell[1] + 1]
    down = [cell[0] + 1, cell[1]]
    down_left = [cell[0] + 1, cell[1] - 1]
    left = [cell[0], cell[1] - 1]
    up_left = [cell[0] - 1, cell[1] - 1]

    if up[0] >= 0:
        neighbors.append([up, 10])
    if up_right[0] >= 0 and up_right[1] < width:
        neighbors.append([up_right, 14])
    if up_left[0] >= 0 and up_left[1] >= 0:
        neighbors.append([up_left, 14])
    if right[1] < width:
        neighbors.append([right, 10])
    if down_right[0] < height and down_right[1] < width:
        neighbors.append([down_right, 14])
    if down[0] < height:
        neighbors.append([down, 10])
    if down_left[0] < height and down_left[1] >= 0:
        neighbors.append([down_left, 14])
    if left[1] >= 0:
        neighbors.append([left, 10])
    
    return neighbors

def astar(start_pos, goal_pos, width, height, occupancy_grid):
    open_cells = [] # set of open cells
    closed_cells = set() # set of evaluated cells
    f_costs_map = dict()
    g_costs_map = dict()
    parent_cells = dict()
    path_found = False
    path = []

    open_cells.append([start_pos, dist(start_pos, goal_pos), dist(start_pos, goal_pos)]) # [pos, f-cost, h-cost]
    f_costs_map[tuple(start_pos)] = dist(start_pos, goal_pos)
    g_costs_map[tuple(start_pos)] = 0

    while open_cells:
        open_cells.sort(key = lambda x: x[1] and x[2]) # sort by lowest f-cost and h-cost
        current_cell = open_cells.pop(0)[0]
        closed_cells.update(tuple(current_cell))

        if current_cell == goal_pos:
            path_found = True
            break

        neighbor_cells = get_neighbors(current_cell, width, height)

        for neighbor, step_cost in neighbor_cells:
            if occupancy_grid[neighbor[1]][neighbor[0]] != 1.0:
                continue

            g_cost = g_costs_map[tuple(current_cell)] + step_cost
            h_cost = dist(neighbor, goal_pos)
            f_cost = g_cost + h_cost
            if neighbor in open_cells:
                if f_cost < f_costs_map[tuple(neighbor)]:
                    g_costs_map[tuple(neighbor)] = g_cost
                    f_costs_map[tuple(neighbor)] = f_cost
                    parent_cells[tuple(neighbor)] = current_cell
                    open_cells[open_cells.index(neighbor)] = [neighbor, f_cost, h_cost]
            else:
                g_costs_map[tuple(neighbor)] = g_cost
                f_costs_map[tuple(neighbor)] = f_cost
                parent_cells[tuple(neighbor)] = current_cell
                open_cells.append([neighbor, f_cost, h_cost])
    
    if path_found:
        step = goal_pos
        while step != start_pos:
            path.append(step)
            step = parent_cells[tuple(step)]
    
    return path
