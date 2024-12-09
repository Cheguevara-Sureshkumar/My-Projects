import numpy as np
import random
import math
from queue import PriorityQueue

class AStarNavigator:
    def __init__(self):
        self.DIRECTIONS = [
            (0, 1),   # up
            (0, -1),  # down
            (1, 0),   # right
            (-1, 0),  # left
            (1, 1),   # down-right
            (-1, -1), # up-left
            (1, -1),  # down-left
            (-1, 1)   # up-right
        ]

    def create_path(self, start_coord, goal_coord, width=10, height=10, obstacle_ratio=0.0):
        grid = self.create_grid_with_coordinates_and_obstacles(
            start_coord, goal_coord, width, height, obstacle_ratio
        )
        path = self.astar(grid, start_coord, goal_coord)
        if path:
            # Convert path coordinates to meters (assuming each grid cell is 1x1 meter)
            return [(float(x), float(y)) for x, y in path]
        return None

    def create_grid_with_coordinates_and_obstacles(self, start_coord, goal_coord, width, height, obstacle_ratio=0.2):
        while True:
            grid = np.ones((height, width), dtype=int)
            start_x, start_y = start_coord[0] - 1, height - start_coord[1]
            goal_x, goal_y = goal_coord[0] - 1, height - goal_coord[1]
           
            num_obstacles = int(width * height * obstacle_ratio)
            for _ in range(num_obstacles):
                x, y = random.randint(0, width-1), random.randint(0, height-1)
                if (x, y) != (start_x, start_y) and (x, y) != (goal_x, goal_y):
                    grid[y][x] = 0
           
            grid[start_y][start_x] = 2
            grid[goal_y][goal_x] = 3
           
            if self.astar(grid, start_coord, goal_coord):
                return grid

    def heuristic(self, a, b):
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def get_neighbors(self, grid, node):
        neighbors = []
        for dx, dy in self.DIRECTIONS:
            new_x, new_y = node[0] + dx, node[1] + dy
            if 0 <= new_x < grid.shape[1] and 0 <= new_y < grid.shape[0] and grid[new_y][new_x] != 0:
                neighbors.append((new_x, new_y))
        return neighbors

    def astar(self, grid, start, goal):
        height, width = grid.shape
        start = (start[0] - 1, height - start[1])
        goal = (goal[0] - 1, height - goal[1])
        
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                path = []
                while current:
                    path.append((current[0] + 1, height - current[1]))
                    current = came_from[current]
                return path[::-1]

            for next_node in self.get_neighbors(grid, current):
                if abs(next_node[0] - current[0]) + abs(next_node[1] - current[1]) == 1:
                    new_cost = cost_so_far[current] + 1
                else:
                    new_cost = cost_so_far[current] + math.sqrt(2)

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    frontier.put((priority, next_node))
                    came_from[next_node] = current

        return None