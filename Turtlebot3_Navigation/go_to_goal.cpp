#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class BasicRobotNavigation : public rclcpp::Node {
public:
    BasicRobotNavigation() : Node("basic_robot_navigation") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/tb0_0/cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/tb0_0/odom", 10, 
            std::bind(&BasicRobotNavigation::odomCallback, this, std::placeholders::_1));

        goal_position_ = std::make_pair(3.0, 3.0);
        distance_tolerance_ = 0.1;
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = *msg;
        navigateToGoal();
    }

    void navigateToGoal() {
        double goal_x = goal_position_.first;
        double goal_y = goal_position_.second;
        double current_x = current_pose_.pose.pose.position.x;
        double current_y = current_pose_.pose.pose.position.y;

        double distance_to_goal = std::sqrt(std::pow(goal_x - current_x, 2) + std::pow(goal_y - current_y, 2));
        
        RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f", distance_to_goal);

        if (distance_to_goal < distance_tolerance_) {
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "Goal reached");
        } else {
            double angle_to_goal = std::atan2(goal_y - current_y, goal_x - current_x);
            double current_theta = std::atan2(
                2 * current_pose_.pose.pose.orientation.z, 
                1 - 2 * std::pow(current_pose_.pose.pose.orientation.z, 2));

            double relative_angle_to_goal = angle_to_goal - current_theta;

            // Adjust linear velocity based on distance to the goal
            double linear_velocity = std::max(0.08, distance_to_goal);
            linear_velocity = std::min(linear_velocity, 0.20);

            // Adjust angular velocity based on relative angle to the goal
            double angular_velocity = relative_angle_to_goal * 0.5;

            // Update Twist message
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = linear_velocity;
            twist_msg.angular.z = angular_velocity;

            // Publish Twist message
            publisher_->publish(twist_msg);
        }
    }

    void stopRobot() {
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        publisher_->publish(stop_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    nav_msgs::msg::Odometry current_pose_;
    std::pair<double, double> goal_position_;
    double distance_tolerance_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicRobotNavigation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
