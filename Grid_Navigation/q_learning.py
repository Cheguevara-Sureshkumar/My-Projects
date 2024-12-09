import numpy as np
import random
import math
import time
import matplotlib.pyplot as plt

def create_grid_with_coordinates_and_obstacles(width, height, obstacle_ratio=0.2):
    grid = np.ones((height, width), dtype=int)
    
    # Add obstacles
    num_obstacles = int(width * height * obstacle_ratio)
    for _ in range(num_obstacles):
        x, y = random.randint(0, width-1), random.randint(0, height-1)
        grid[y][x] = 0  # 0 represents an obstacle
    
    return grid

def set_start_goal(grid, start_coord, goal_coord):
    height, width = grid.shape
    start_x, start_y = start_coord[0] - 1, height - start_coord[1]
    goal_x, goal_y = goal_coord[0] - 1, height - goal_coord[1]
    
    new_grid = grid.copy()
    new_grid[start_y][start_x] = 2  # Start position
    new_grid[goal_y][goal_x] = 3    # Goal position
    
    return new_grid

def check_reached(grid, goal_x, goal_y):
    current_pos = np.where(grid == 2)
    current_x = current_pos[1][0] + 1
    current_y = len(grid) - current_pos[0][0]
    return current_x == goal_x and current_y == goal_y

def initialize_q_table(grid):
    return np.zeros((4, grid.shape[0], grid.shape[1]))

def get_current_position(grid):
    current_pos = np.where(grid == 2)
    if len(current_pos[0]) == 0:
        return None
    return (current_pos[1][0] + 1, len(grid) - current_pos[0][0])

def get_valid_actions(grid, visited_positions=None):
    current_pos = get_current_position(grid)
    if current_pos is None:
        return []
    
    x, y = current_pos
    height, width = grid.shape
    valid_actions = []
    
    moves = [
        ("up", y + 1, x),
        ("left", y, x - 1),
        ("down", y - 1, x),
        ("right", y, x + 1)
    ]
    
    for action, new_y, new_x in moves:
        if (0 < new_x <= width and 0 < new_y <= height):
            if (grid[height - new_y][new_x - 1] != 0 and 
                (visited_positions is None or (new_x, new_y) not in visited_positions)):
                valid_actions.append(action)
    
    return valid_actions

def choose_action(grid, q_table, epsilon, visited_positions):
    current_pos = get_current_position(grid)
    if current_pos is None:
        return None
    
    x, y = current_pos
    height = len(grid)
    valid_actions = get_valid_actions(grid, visited_positions)
   
    if not valid_actions:
        return None
   
    if np.random.uniform(0, 1) < epsilon:
        action = np.random.choice(valid_actions)
    else:
        q_values = [q_table[n][height - y][x - 1] for n in range(4)]
        max_value = max(q_values)
        possible_actions = [action_index(i) for i in range(4) if q_values[i] == max_value]
        possible_actions = [a for a in possible_actions if a in valid_actions]
        if possible_actions:
            action = np.random.choice(possible_actions)
        else:
            action = np.random.choice(valid_actions)
    return action

def action_index(action):
    if isinstance(action, str):
        return {"up": 0, "left": 1, "down": 2, "right": 3}[action]
    elif isinstance(action, int):
        return ["up", "left", "down", "right"][action]
    raise ValueError("Invalid action")

def calc_reward(new_grid, goal_coord, visited_positions):
    current_pos = get_current_position(new_grid)
    if current_pos is None:
        return -10
        
    goal_x, goal_y = goal_coord
    current_x, current_y = current_pos
    distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    
    if check_reached(new_grid, goal_x, goal_y):
        reward = 100
    else:
        reward = -1
        
        if current_pos in visited_positions:
            reward -= 5
            
        reward += 1 / (distance + 1)
    
    return reward

def take_action(action, grid, valid_actions):
    current_pos = get_current_position(grid)
    if current_pos is None or action not in valid_actions:
        return grid
        
    x, y = current_pos
    height = len(grid)
    new_grid = grid.copy()
    new_grid[height - y][x - 1] = 1  # Mark current position as visited

    if action == "up" and y + 1 <= height:
        new_y, new_x = y + 1, x
    elif action == "down" and y - 1 > 0:
        new_y, new_x = y - 1, x
    elif action == "left" and x - 1 > 0:
        new_y, new_x = y, x - 1
    elif action == "right" and x + 1 <= len(grid[0]):
        new_y, new_x = y, x + 1
    else:
        return new_grid

    if (0 <= new_x - 1 < len(grid[0]) and 0 <= height - new_y < height and 
        new_grid[height - new_y][new_x - 1] != 0):
        new_grid[height - new_y][new_x - 1] = 2
    
    return new_grid

def train_q_table_multi_points(base_grid, episodes, max_steps, epsilon, alpha, gamma, training_points):
    q_table = initialize_q_table(base_grid)
    
    for episode in range(episodes):
        if episode % 100 == 0:
            print(f"Training episode {episode}/{episodes}")
        
        for start_coord, goal_coord in training_points:
            grid = set_start_goal(base_grid, start_coord, goal_coord)
            visited_positions = {start_coord}
            current_pos = get_current_position(grid)
            
            for step in range(max_steps):
                if current_pos is None:
                    break
                    
                next_action = choose_action(grid, q_table, epsilon, visited_positions)
                if next_action is None:
                    break
                
                new_grid = take_action(next_action, grid, get_valid_actions(grid, visited_positions))
                new_pos = get_current_position(new_grid)
                
                if new_pos is None:
                    break
                    
                visited_positions.add(new_pos)
                reward = calc_reward(new_grid, goal_coord, visited_positions)
                
                a = action_index(next_action)
                current_x, current_y = current_pos
                new_x, new_y = new_pos
                height = len(grid)
                
                current_q = q_table[a][height - current_y][current_x - 1]
                q_values = [q_table[n][height - new_y][new_x - 1] for n in range(4)]
                max_future_q = max(q_values)
                
                new_q = (1 - alpha) * current_q + alpha * (reward + gamma * max_future_q)
                q_table[a][height - current_y][current_x - 1] = new_q
                
                if check_reached(new_grid, goal_coord[0], goal_coord[1]):
                    break
                
                grid = new_grid
                current_pos = new_pos
            
    return q_table

def get_path(base_grid, start_coord, goal_coord, q_table, max_steps=1000):
    grid = set_start_goal(base_grid, start_coord, goal_coord)
    path = [start_coord]
    visited_positions = {start_coord}
    steps = 0
    
    while not check_reached(grid, goal_coord[0], goal_coord[1]) and steps < max_steps:
        current_pos = get_current_position(grid)
        if current_pos is None:
            break
            
        x, y = current_pos
        height = len(grid)
        q_values = [q_table[n][height - y][x - 1] for n in range(4)]
        valid_actions = get_valid_actions(grid, visited_positions)
        
        if not valid_actions:
            break
        
        # Choose the best action among valid actions
        best_action = max(valid_actions, key=lambda a: q_values[action_index(a)])
        
        grid = take_action(best_action, grid, valid_actions)
        new_pos = get_current_position(grid)
        if new_pos is None or new_pos in visited_positions:
            break
            
        path.append(new_pos)
        visited_positions.add(new_pos)
        steps += 1
        
        # Debug information
        print(f"Step {steps}: Moving {best_action} to {new_pos}")
   
    if check_reached(grid, goal_coord[0], goal_coord[1]):
        print("Goal reached!")
    else:
        print("Failed to reach the goal.")
    
    return path, grid

def visualize_grid_and_path(grid, path, start_coord, goal_coord):
    if not path:
        print("No path to visualize")
        return
        
    fig, ax = plt.subplots(figsize=(10, 10))
    height, width = grid.shape

    # Create a custom colormap
    cmap = plt.cm.colors.ListedColormap(['black', 'white', 'green', 'red'])
    bounds = [0, 1, 2, 3, 4]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    ax.imshow(grid, cmap=cmap, norm=norm)

    # Plot the path
    path_x = [p[0] - 1 for p in path]
    path_y = [height - p[1] for p in path]
    ax.plot(path_x, path_y, marker='o', color='blue', linewidth=2, markersize=6, label='Path')

    # Highlight start and goal
    ax.scatter(start_coord[0] - 1, height - start_coord[1], c='lime', s=200, label='Start', zorder=5)
    ax.scatter(goal_coord[0] - 1, height - goal_coord[1], c='red', s=200, label='Goal', zorder=5)

    # Add grid lines
    ax.set_xticks(np.arange(-.5, width, 1), minor=True)
    ax.set_yticks(np.arange(-.5, height, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)

    # Remove axis labels
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    ax.legend()
    plt.title('Grid and Path Visualization')
    plt.show()

if __name__ == "__main__":
    # Parameters
    width = 10
    height = 10
    epsilon = 0.1  # Reduced epsilon for more exploitation
    alpha = 0.5
    gamma = 0.99  # Increased gamma for more future reward consideration
    episodes = 10000  # Increased number of episodes
    total_iterations = 1000
    obstacle_ratio = 0.2

    # Create a single base grid with obstacles
    base_grid = create_grid_with_coordinates_and_obstacles(width, height, obstacle_ratio)

    # Define multiple training points
    training_points = [
        ((1, 1), (10, 10)),
        ((1, 2), (9, 9)),
        ((1, 3), (8, 8)),
        ((1, 4), (7, 7)),
        ((1, 5), (6, 6)),
        ((2, 1), (9, 10)),
        ((2, 2), (8, 9)),
        ((3, 1), (8, 10)),
        ((3, 2), (7, 9)),
        ((4, 1), (7, 10)),
        ((5, 1), (6, 10)),
        ((10, 10), (1, 1)),
        ((9, 9), (2, 2)),
        ((8, 8), (3, 3)),
        ((7, 7), (4, 4)),
        ((6, 6), (5, 5))
    ]

    # Train the Q-table with multiple points
    start_time = time.time()
    q_table = train_q_table_multi_points(base_grid, episodes, total_iterations, 
                                         epsilon, alpha, gamma, training_points)
    end_time = time.time()
    print(f"Training time: {end_time - start_time} seconds")

    # Test with different start and goal points (including some not in training)
    test_scenarios = [
        ((4, 4), (8, 8)),
        ((1, 1), (9, 9)),   # Trained scenario
        ((1, 1), (10, 10))  # Another trained scenario
    ]
    
    for start, goal in test_scenarios:
        print(f"\nFinding path from {start} to {goal}")
        start_time = time.time()
        path, final_grid = get_path(base_grid, start, goal, q_table)
        if path:
            print(f"Path length: {len(path)}")
            end_time = time.time()
            print(f"Finding time: {(end_time - start_time)*1000} milliseconds")
            visualize_grid_and_path(final_grid, path, start, goal)
        else:
            print("No valid path found")

    # Visualize the Q-table
    plt.figure(figsize=(12, 10))
    plt.imshow(np.max(q_table, axis=0), cmap='hot', interpolation='nearest')
    plt.colorbar(label='Max Q-value')
    plt.title('Q-table Visualization')
    plt.show()
