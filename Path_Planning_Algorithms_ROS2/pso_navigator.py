import numpy as np
import random
import math

class Particle:
    def __init__(self, num_points, bounds):
        self.position = np.array([
            [random.uniform(bounds[0], bounds[1]) for _ in range(2)]
            for _ in range(num_points)
        ])
        self.velocity = np.zeros_like(self.position)
        self.best_position = self.position.copy()
        self.best_score = float('inf')

class PSOPlanner:
    def __init__(self):
        # PSO parameters
        self.num_particles = 50
        self.num_points = 10  # Number of points in the path
        self.max_iterations = 100
        self.w = 0.729  # Inertia weight
        self.c1 = 1.49445  # Cognitive parameter
        self.c2 = 1.49445  # Social parameter

    def create_path(self, start_coord, goal_coord, grid):
        self.grid = grid
        self.height, self.width = grid.shape
        self.start = np.array([start_coord[0] - 1, self.height - start_coord[1]])
        self.goal = np.array([goal_coord[0] - 1, self.height - goal_coord[1]])
        
        bounds = (0, max(self.width, self.height))
        
        # Initialize particles
        particles = [Particle(self.num_points, bounds) for _ in range(self.num_particles)]
        global_best_position = None
        global_best_score = float('inf')
        
        # PSO main loop
        for _ in range(self.max_iterations):
            for particle in particles:
                # Evaluate fitness
                score = self.evaluate_path(particle.position)
                
                # Update particle's best
                if score < particle.best_score:
                    particle.best_score = score
                    particle.best_position = particle.position.copy()
                
                # Update global best
                if score < global_best_score:
                    global_best_score = score
                    global_best_position = particle.position.copy()
                
                # Update velocity and position
                r1, r2 = random.random(), random.random()
                particle.velocity = (self.w * particle.velocity + 
                                  self.c1 * r1 * (particle.best_position - particle.position) +
                                  self.c2 * r2 * (global_best_position - particle.position))
                
                particle.position += particle.velocity
                
                # Bound checking
                particle.position = np.clip(particle.position, 0, max(self.width, self.height))
        
        if global_best_position is not None:
            # Create final path including start and goal
            path = np.vstack([self.start, global_best_position, self.goal])
            
            # Convert to list of tuples and world coordinates
            return [(int(x + 1), int(self.height - y)) for x, y in path]
        return None

    def evaluate_path(self, path):
        # Calculate path length
        length = self.path_length(np.vstack([self.start, path, self.goal]))
        
        # Calculate collision penalty
        collision_penalty = self.collision_penalty(path)
        
        # Calculate smoothness penalty
        smoothness_penalty = self.smoothness_penalty(path)
        
        return length + 1000 * collision_penalty + 10 * smoothness_penalty

    def path_length(self, path):
        return sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))

    def collision_penalty(self, path):
        penalty = 0
        for point in path:
            x, y = int(point[0]), int(point[1])
            if (x < 0 or x >= self.width or 
                y < 0 or y >= self.height or 
                self.grid[y][x] == 0):
                penalty += 1
        return penalty

    def smoothness_penalty(self, path):
        if len(path) < 3:
            return 0
            
        penalty = 0
        for i in range(len(path)-2):
            v1 = path[i+1] - path[i]
            v2 = path[i+2] - path[i+1]
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            penalty += 1 - cos_angle
        return penalty