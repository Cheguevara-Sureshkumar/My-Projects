import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi
from tf_transformations import euler_from_quaternion
import time
import sys

class PathFollower(Node):
    def __init__(self, planner_type='astar'):
        super().__init__('path_follower')
        self.get_logger().info('NOde has been started !!!')
        # Create publisher for velocity commands
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Initialize position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Select and initialize path planner
        start_coord = (1, 1)
        goal_coord = (8, 7)
        width, height = 10, 10
        
        '''if planner_type == 'astar':
            from .astar_navigator import AStarNavigator
            planner = AStarNavigator()
            self.get_logger().info('Astar algorithm')
        elif planner_type == 'dstar':
            from .dstar_navigator import DStarPlanner
            planner = DStarPlanner()
            self.get_logger().info('Dstar algorithm')
        elif planner_type == 'dijkstra':
            from .dijikstra_navigator import DijkstraPlanner
            planner = DijkstraPlanner()
            self.get_logger().info('Dijikstra algorithm')
        elif planner_type == 'pso':
            from .pso_navigator import PSOPlanner
            planner = PSOPlanner()
            self.get_logger().info('PSO algorithm')
        else:
            self.get_logger().error(f'Unknown planner type: {planner_type}')
            return'''
        from .pso_navigator import PSOPlanner
        planner = PSOPlanner()
        self.start_time = time.time()    
        # Generate path using selected planner
        self.path_coords = planner.create_path(start_coord, goal_coord)
        
        if not self.path_coords:
            self.get_logger().error('Failed to generate path!')
            return
            
        self.current_goal = 0
        
        # Control parameters
        self.distance_threshold = 0.08
        self.angular_velocity = 0.5
        self.linear_velocity = 0.2
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Path follower initialized with {planner_type} planner')
        self.get_logger().info(f'Path to follow: {self.path_coords}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

    def control_loop(self):
        if not self.path_coords or self.current_goal >= len(self.path_coords):
            self.stop_robot()
            self.get_logger().info("Path completed.")
            self.end_time = time.time()
            self.get_logger().info(f"Total time taken : {self.end_time - self.start_time} seconds")
            sys.exit()

        goal_x, goal_y = self.path_coords[self.current_goal]
        distance = sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)
        
        if distance < self.distance_threshold:
            self.current_goal += 1
            self.get_logger().info(f'Reached waypoint {self.current_goal}, moving to next point')
            return
        
        angle_to_goal = atan2(goal_y - self.current_y, goal_x - self.current_x)
    
        heading_error = self.normalize_angle(angle_to_goal - self.current_theta)
        

        vel_msg = Twist()
        vel_msg.angular.z = self.angular_velocity * heading_error
        
        if abs(heading_error) < pi/4:
            vel_msg.linear.x = self.linear_velocity * (1 - abs(heading_error)/(pi/4))
        else:
            vel_msg.linear.x = 0.0
            
        self.vel_publisher.publish(vel_msg)

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()