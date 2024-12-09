import numpy as np
from queue import PriorityQueue
import math

class DijkstraPlanner:
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

    def create_path(self, start_coord, goal_coord, grid):
        self.grid = grid
        height, width = grid.shape
        
        # Convert to grid coordinates
        start = (start_coord[0] - 1, height - start_coord[1])
        goal = (goal_coord[0] - 1, height - goal_coord[1])
        
        # Run Dijkstra's algorithm
        path = self.dijkstra(start, goal)
        
        if path:
            # Convert back to world coordinates
            return [(x + 1, height - y) for x, y in path]
        return None

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in self.DIRECTIONS:
            new_x, new_y = node[0] + dx, node[1] + dy
            if (0 <= new_x < self.grid.shape[1] and 
                0 <= new_y < self.grid.shape[0] and 
                self.grid[new_y][new_x] != 0):
                neighbors.append((new_x, new_y))
        return neighbors

    def cost(self, current, neighbor):
        dx = abs(neighbor[0] - current[0])
        dy = abs(neighbor[1] - current[1])
        return math.sqrt(2) if dx + dy == 2 else 1

    def dijkstra(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.cost(current, next_node)

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    frontier.put((new_cost, next_node))
                    came_from[next_node] = current

        return None