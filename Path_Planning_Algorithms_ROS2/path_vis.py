
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from .astar_navigator import AStarNavigator

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
            

        self.current_x = 0.0
        self.current_y = 0.0
        self.position_history_x = []
        self.position_history_y = []

        navigator = AStarNavigator()            #change here.
        start_coord = (1, 1)
        goal_coord = (8, 7)
        width, height = 10, 10
        obstacle_ratio = 0.0
        
        self.grid = navigator.create_grid_with_coordinates_and_obstacles(
            start_coord, goal_coord, width, height, obstacle_ratio
        )
        self.path = navigator.create_path(start_coord, goal_coord)
        
        if not self.path:
            self.get_logger().error('Failed to generate path!')
            return
            
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.setup_plot()
        
        # Create timer for updating visualization
        self.timer = self.create_timer(0.1, self.update_plot)  # 10Hz update rate
        
        self.get_logger().info('Path visualizer initialized')

    def setup_plot(self):
        height, width = self.grid.shape
        
        # Plot grid
        self.ax.imshow(self.grid, cmap='gray_r')
        
        # Plot start and goal
        if self.path:
            start = self.path[0]
            goal = self.path[-1]
            self.ax.scatter(start[0] - 1, height - start[1], c='green', s=100, label='Start')
            self.ax.scatter(goal[0] - 1, height - goal[1], c='red', s=100, label='Goal')
        
        # Plot planned path
        if self.path:
            path_x = [p[0] - 1 for p in self.path]
            path_y = [height - p[1] for p in self.path]
            self.ax.plot(path_x, path_y, 'b--', linewidth=2, label='Planned Path')
        
        # Setup robot trajectory plot
        self.trajectory_line, = self.ax.plot([], [], 'r-', linewidth=2, label='Robot Trajectory')
        self.robot_point, = self.ax.plot([], [], 'ro', markersize=10, label='Robot')
        
        # Configure grid
        self.ax.set_xticks(np.arange(-.5, width, 1), minor=True)
        self.ax.set_yticks(np.arange(-.5, height, 1), minor=True)
        self.ax.grid(which='minor', color='black', linestyle='-', linewidth=1)
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        
        self.ax.legend()
        plt.title('Path Following Visualization')

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Add to position history
        self.position_history_x.append(self.current_x)
        self.position_history_y.append(self.current_y)

    def update_plot(self):
        if not self.position_history_x:
            return
            
        # Update trajectory line
        self.trajectory_line.set_data(self.position_history_x, self.position_history_y)
        
        # Update robot position
        self.robot_point.set_data([self.current_x], [self.current_y])
        
        # Adjust plot limits if necessary
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Draw changes
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    visualizer = PathVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        plt.ioff()
        plt.close('all')
        rclpy.shutdown()

if __name__ == '__main__':
    main()