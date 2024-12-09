import numpy as np
import random
import math
import time
import matplotlib.pyplot as plt
from queue import PriorityQueue


DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

ACTION_MAP = {
    0: "up",
    1: "right",
    2: "down",
    3: "left"
}

RAISED = 0
LOWER = 1
NEW = 2
CLOSED = 3
OPEN = 4

class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.tag = NEW
        self.h = float('inf')
        self.k = float('inf')
        self.neighbors = []
        self.back_pointer = None

    def __lt__(self, other):
        return self.k < other.k

    def __eq__(self, other):
        if isinstance(other, State):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))

def get_neighbors(grid, node):
    neighbors = []
    for dx, dy in DIRECTIONS:
        new_x, new_y = node[0] + dx, node[1] + dy
        if 0 <= new_x < grid.shape[1] and 0 <= new_y < grid.shape[0] and grid[new_y][new_x] != 0:
            neighbors.append((new_x, new_y))
    return neighbors

def heuristic(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def get_valid_actions(grid, visited_positions, width, height):
    current_pos = get_current_position(grid, height)
    if current_pos is None:
        return []
   
    x, y = current_pos
    valid_actions = []
    moves = [
        ("up", y + 1, x),
        ("right", y, x + 1),
        ("down", y - 1, x),
        ("left", y, x - 1)
    ]
   
    for action, new_y, new_x in moves:
        if (0 < new_x <= width and 0 < new_y <= height):
            if (grid[height - new_y][new_x - 1] != 0 and
                (visited_positions is None or (new_x, new_y) not in visited_positions)):
                valid_actions.append(action)
    return valid_actions

def take_action(action, grid, width, height):
    current_pos = get_current_position(grid, height)
    if current_pos is None:
        return grid
   
    x, y = current_pos
    new_grid = grid.copy()
    new_grid[height - y][x - 1] = 1

    action_deltas = {
        "up": (0, 1),
        "right": (1, 0),
        "down": (0, -1),
        "left": (-1, 0)
    }

    if action in action_deltas:
        dx, dy = action_deltas[action]
        new_x, new_y = x + dx, y + dy
       
        if (0 < new_x <= width and 0 < new_y <= height and
            new_grid[height - new_y][new_x - 1] != 0):
            new_grid[height - new_y][new_x - 1] = 2
   
    return new_grid

def calculate_step_distance(current_pos, next_pos):
    return 1.0

def create_grid(start_coord, goal_coord, width, height, obstacle_ratio=0.2):
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
    return grid

def get_current_position(grid, height):
    current_pos = np.where(grid == 2)
    if len(current_pos[0]) == 0:
        return None
    return (current_pos[1][0] + 1, height - current_pos[0][0])

def dijkstra(grid, start, goal):
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

        for next_node in get_neighbors(grid, current):
            new_cost = cost_so_far[current] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost
                frontier.put((priority, next_node))
                came_from[next_node] = current

    return None

def astar(grid, start, goal):
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

        for next_node in get_neighbors(grid, current):
            new_cost = cost_so_far[current] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(goal, next_node)
                frontier.put((priority, next_node))
                came_from[next_node] = current

    return None

def dstar(grid, start, goal):
    height, width = grid.shape
    start = (start[0] - 1, height - start[1])
    goal = (goal[0] - 1, height - goal[1])

    states = {}
    for y in range(height):
        for x in range(width):
            if grid[y][x] != 0:
                state = State(x, y)
                states[(x, y)] = state
                for dx, dy in DIRECTIONS:
                    new_x, new_y = x + dx, y + dy
                    if (0 <= new_x < width and 0 <= new_y < height and
                        grid[new_y][new_x] != 0):
                        state.neighbors.append((new_x, new_y))

    # Initialize goal state
    goal_state = states[goal]
    goal_state.h = 0
    goal_state.k = 0
    goal_state.tag = OPEN

    open_states = PriorityQueue()
    open_states.put((goal_state.k, goal_state))

    while not open_states.empty():
        current_state = open_states.get()[1]
        current_state.tag = CLOSED

        if (current_state.x, current_state.y) == start:
            break

     
        for next_x, next_y in current_state.neighbors:
            next_state = states[(next_x, next_y)]
            if next_state.tag == NEW:
                next_state.h = float('inf')
                next_state.tag = OPEN
           
            if next_state.h > current_state.h + 1:
                next_state.h = current_state.h + 1
                next_state.back_pointer = current_state
                if next_state.tag == OPEN:
                    open_states.put((next_state.h, next_state))
                else:
                    next_state.tag = OPEN
                    next_state.k = next_state.h
                    open_states.put((next_state.k, next_state))
    path = []
    current = states[start]
    while current:
        path.append((current.x + 1, height - current.y))
        current = current.back_pointer
        if current and (current.x, current.y) == goal:
            path.append((current.x + 1, height - current.y))
            break

    return path if path else None

def simulate_path(particle, grid, start_coord, goal_coord, width, height):
    temp_grid = grid.copy()
    path = [start_coord]
    visited_positions = {start_coord}
   
    for action_idx in particle:
        current_pos = get_current_position(temp_grid, height)
        if current_pos is None:
            break
       
        action = ACTION_MAP[action_idx]
        valid_actions = get_valid_actions(temp_grid, visited_positions, width, height)
       
        if action not in valid_actions:
            continue
       
        temp_grid = take_action(action, temp_grid, width, height)
        new_pos = get_current_position(temp_grid, height)
       
        if new_pos is None:
            break
       
        if new_pos not in visited_positions:
            path.append(new_pos)
            visited_positions.add(new_pos)
       
        if temp_grid[height - goal_coord[1]][goal_coord[0] - 1] == 2:
            path.append(goal_coord)
            return path, True
   
    return path, False

def evaluate_fitness(particle, grid, start_coord, goal_coord, width, height):
    path, reached_goal = simulate_path(particle, grid, start_coord, goal_coord, width, height)
   
    if reached_goal:
        return 1000 - len(path)
    else:
        final_pos = path[-1]
        distance = math.sqrt((goal_coord[0] - final_pos[0])**2 + (goal_coord[1] - final_pos[1])**2)
        return -distance - len(path) * 0.1

def pso_navigation(grid, start_coord, goal_coord, num_particles=150, num_iterations=300, path_length=50):
    width, height = grid.shape[1], grid.shape[0]
    particles = [np.random.randint(0, 4, path_length) for _ in range(num_particles)]
    velocities = [np.zeros(path_length) for _ in range(num_particles)]
    personal_best = particles.copy()
    personal_best_fitness = [evaluate_fitness(p, grid, start_coord, goal_coord, width, height) for p in particles]
    global_best = max(range(num_particles), key=lambda i: personal_best_fitness[i])
    global_best_particle = personal_best[global_best].copy()
    global_best_fitness = personal_best_fitness[global_best]

    w, c1, c2 = 0.729, 1.699, 1.699

    for _ in range(num_iterations):
        for i in range(num_particles):
            fitness = evaluate_fitness(particles[i], grid, start_coord, goal_coord, width, height)
           
            if fitness > personal_best_fitness[i]:
                personal_best[i] = particles[i].copy()
                personal_best_fitness[i] = fitness
           
            if fitness > global_best_fitness:
                global_best = i
                global_best_particle = particles[i].copy()
                global_best_fitness = fitness

            r1, r2 = np.random.rand(path_length), np.random.rand(path_length)
            velocities[i] = (w * velocities[i] +
                           c1 * r1 * (personal_best[i] - particles[i]) +
                           c2 * r2 * (global_best_particle - particles[i]))
            particles[i] = np.clip(particles[i] + velocities[i], 0, 3).astype(int)

        if global_best_fitness > 0:
            break

    final_path, _ = simulate_path(global_best_particle, grid, start_coord, goal_coord, width, height)
    return final_path

def calculate_step_distance(current_pos, next_pos):
    dx = abs(next_pos[0] - current_pos[0])
    dy = abs(next_pos[1] - current_pos[1])
   

    if dx == 1 and dy == 1:
        return math.sqrt(2)  
       
    return 1.0

def calculate_path_length_meters(path):
    if isinstance(path, np.ndarray):
        path = path.tolist()
   
    if not path or len(path) < 2:
        return 0.0
   
    total_length = 0.0
    for i in range(len(path) - 1):
        total_length += calculate_step_distance(path[i], path[i+1])
   
    return round(total_length, 2)

def visualize_paths(grid, paths, algorithms):
    fig, ax = plt.subplots(figsize=(10, 10))
   
    plt.imshow(grid, cmap='binary')
   
    colors = ['blue', 'red', 'green', 'purple']
   
    for path, color, algo in zip(paths, colors, algorithms):
        if path is not None:
            if isinstance(path, np.ndarray):
                path = path.tolist()
           
            x_coords = []
            y_coords = []
           
            for x, y in path:
                x_coords.append(x - 1)  
                y_coords.append(grid.shape[0] - y)
           
            path_length_m = calculate_path_length_meters(path)
            plt.plot(x_coords, y_coords, color=color, linewidth=2,
                    label=f'{algo} (length: {path_length_m}m)')
   
    plt.grid(True)
    plt.legend()
    plt.title('Path Comparison (1 unit = 1 meter)')
    plt.show()

def run_single_comparison(start_coord, goal_coord, width, height, obstacle_ratio=0.2):
    grid = create_grid(start_coord, goal_coord, width, height, obstacle_ratio)
    paths = []
    algorithms = ['Dijkstra', 'A*', 'D*', 'PSO']
    times = []
   
    print("\nRunning algorithms:")
   
    # Dijkstra
    start_time = time.time()
    dijkstra_path = dijkstra(grid.copy(), start_coord, goal_coord)
    time_taken = (time.time() - start_time)
    if dijkstra_path:
        path_length_m = calculate_path_length_meters(dijkstra_path)
        print(f"Dijkstra - Time: {(time_taken + (path_length_m)/0.16):.6f}seconds, Path Length: {path_length_m}m")
    else:
        print(f"Dijkstra - Time: {time_taken:.3f}ms, No path found")
    paths.append(dijkstra_path)
    times.append(time_taken)
   
    # A*
    start_time = time.time()
    astar_path = astar(grid.copy(), start_coord, goal_coord)
    time_taken = (time.time() - start_time)
    if astar_path:
        path_length_m = calculate_path_length_meters(astar_path)
        print(f"A* - Time: {(time_taken + (path_length_m)/0.16):.6f}seconds, Path Length: {path_length_m}m")
    else:
        print(f"A* - Time: {time_taken:.3f}ms, No path found")
    paths.append(astar_path)
    times.append(time_taken)
   
    # D*
    start_time = time.time()
    dstar_path = dstar(grid.copy(), start_coord, goal_coord)
    time_taken = (time.time() - start_time)
    if dstar_path:
        path_length_m = calculate_path_length_meters(dstar_path)
        print(f"D* - Time: {(time_taken + (path_length_m)/0.16):.6f}seconds, Path Length: {path_length_m}m")
    else:
        print(f"D* - Time: {time_taken:.3f}ms, No path found")
    paths.append(dstar_path)
    times.append(time_taken)
   
    # PSO
    start_time = time.time()
    pso_path = pso_navigation(grid.copy(), start_coord, goal_coord)
    time_taken = (time.time() - start_time)
    if pso_path:
        path_length_m = calculate_path_length_meters(pso_path)
        print(f"PSO - Time: {(time_taken + (path_length_m)/0.16):.6f}seconds, Path Length: {path_length_m}m")
    else:
        print(f"PSO - Time: {time_taken:.3f}ms, No path found")
    paths.append(pso_path)
    times.append(time_taken)
   
    # Visualize the paths
    visualize_paths(grid, paths, algorithms)
   
    return grid, paths, times

if __name__ == "__main__":
    # Test parameters
    start_coord = (6, 5)
    goal_coord = (13, 15)
    width, height = 20, 20
    obstacle_ratio = 0.2        # Increase the complexity here
   
    run_single_comparison(start_coord, goal_coord, width, height, obstacle_ratio)
