#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class drawcirclenode(Node):
    def __init__(self):
        super().__init__("Draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5,self.send_velocity)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.8
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = drawcirclenode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__  == "__main__":
    main()
