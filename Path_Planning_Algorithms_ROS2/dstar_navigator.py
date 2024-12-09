import numpy as np
from queue import PriorityQueue
import math

class DStarPlanner:
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
        
        # D* specific variables
        self.OPEN = PriorityQueue()
        self.CLOSED = set()
        self.t = {}  # tag (NEW, OPEN, CLOSED)
        self.h = {}  # path cost
        self.k = {}  # path cost estimate
        self.b = {}  # backpointers

    def create_path(self, start_coord, goal_coord, grid):
        self.grid = grid
        self.height, self.width = grid.shape
        
        # Convert to grid coordinates
        self.start = (start_coord[0] - 1, self.height - start_coord[1])
        self.goal = (goal_coord[0] - 1, self.height - goal_coord[1])
        
        # Initialize D*
        self.initialize()
        
        # Run D* algorithm
        path = self.run()
        
        if path:
            # Convert back to world coordinates
            return [(x + 1, self.height - y) for x, y in path]
        return None

    def initialize(self):
        self.t.clear()
        self.h.clear()
        self.k.clear()
        self.b.clear()
        
        # Initialize goal state
        self.h[self.goal] = 0
        self.k[self.goal] = 0
        self.t[self.goal] = 'OPEN'
        self.OPEN.put((0, self.goal))

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in self.DIRECTIONS:
            new_x, new_y = node[0] + dx, node[1] + dy
            if (0 <= new_x < self.width and 
                0 <= new_y < self.height and 
                self.grid[new_y][new_x] != 0):
                neighbors.append((new_x, new_y))
        return neighbors

    def cost(self, current, neighbor):
        dx = abs(neighbor[0] - current[0])
        dy = abs(neighbor[1] - current[1])
        return math.sqrt(2) if dx + dy == 2 else 1

    def process_state(self):
        if self.OPEN.empty():
            return None
            
        k_old = self.OPEN.get()[0]
        current = self.OPEN.get()[1]
        
        if current == self.start:
            return 'PATH FOUND'
            
        self.t[current] = 'CLOSED'
        
        # Get neighbors
        neighbors = self.get_neighbors(current)
        
        for neighbor in neighbors:
            if neighbor not in self.t:
                self.h[neighbor] = float('inf')
                self.t[neighbor] = 'NEW'
                
            if self.t[neighbor] != 'CLOSED':
                if self.h[neighbor] > self.h[current] + self.cost(current, neighbor):
                    self.h[neighbor] = self.h[current] + self.cost(current, neighbor)
                    self.b[neighbor] = current
                    if self.t[neighbor] == 'OPEN':
                        self.OPEN.put((self.h[neighbor], neighbor))
                    else:
                        self.t[neighbor] = 'OPEN'
                        self.OPEN.put((self.h[neighbor], neighbor))
        
        return k_old

    def run(self):
        while True:
            result = self.process_state()
            if result == 'PATH FOUND':
                path = []
                current = self.start
                while current != self.goal:
                    path.append(current)
                    current = self.b[current]
                path.append(self.goal)
                return path
            elif result is None:
                return None