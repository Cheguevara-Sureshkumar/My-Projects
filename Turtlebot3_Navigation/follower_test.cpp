#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>
#include <fstream>
#include <iomanip>

class LeaderFollowerController : public rclcpp::Node {
public:
    LeaderFollowerController() : Node("follower_controller") {
        // Declare parameters
        declare_parameter("leader_odom_topic", "/tb0_0/odom");
        declare_parameter("follower_odom_topic", "/tb0_1/odom");
        declare_parameter("leader_cmd_topic", "/tb0_0/cmd_vel");
        declare_parameter("follower_cmd_topic", "/tb0_1/cmd_vel");
        declare_parameter("log_file_path", "leader_follower_log.csv");

        // Get parameters
        auto leader_odom_topic = get_parameter("leader_odom_topic").as_string();
        auto follower_odom_topic = get_parameter("follower_odom_topic").as_string();
        auto leader_cmd_topic = get_parameter("leader_cmd_topic").as_string();
        auto follower_cmd_topic = get_parameter("follower_cmd_topic").as_string();
        auto log_file_path = get_parameter("log_file_path").as_string();

        // Configuration parameters
        follower_distance_ = 1.0;
        follower_angle_ = -M_PI;
        k1_ = 0.3;
        k2_ = 0.3;
        damping_factor_ = 0.1;
        MAX_LINEAR_VEL_ = 0.19;
        MAX_ANGULAR_VEL_ = 2.5;
        time_ = 0;

        leader_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            leader_odom_topic, 10, 
            std::bind(&LeaderFollowerController::leaderPoseCallback, this, std::placeholders::_1)
        );

        follower_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            follower_odom_topic, 10, 
            std::bind(&LeaderFollowerController::followerPoseCallback, this, std::placeholders::_1)
        );

        leader_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            leader_cmd_topic, 10,
            std::bind(&LeaderFollowerController::leaderVelCallback, this, std::placeholders::_1)
        );

        follower_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(follower_cmd_topic, 10);

        
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&LeaderFollowerController::controlLoop, this)
        );

        // CSV logging setup
        csv_file_.open(log_file_path, std::ios::out);
        if (!csv_file_) {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV log file at %s", log_file_path.c_str());
        } else {
            csv_file_ << "Time,Leader_X,Leader_Y,Leader_Theta,Linear_Vel_Leader,Angular_Vel_Leader,"
                      << "Follower_X,Follower_Y,Follower_Theta,Linear_Vel_Follower,Angular_Vel_Follower,"
                      << "Distance_Error,Angle_Error\n";
        }
    }

    ~LeaderFollowerController() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void leaderPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        leader_pose_ = msg;
    }

    void followerPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        follower_pose_ = msg;
    }

    void leaderVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        leader_vel_ = *msg;
    }

    void controlLoop() {
        if (!leader_pose_ || !follower_pose_) {
            RCLCPP_WARN(get_logger(), "Waiting for pose data...");
            return;
        }

        // Extract leader pose
        double posxl = leader_pose_->pose.pose.position.x;
        double posyl = leader_pose_->pose.pose.position.y;
        double thetal = tf2::getYaw(leader_pose_->pose.pose.orientation);

        // Extract follower pose
        double posxf = follower_pose_->pose.pose.position.x;
        double posyf = follower_pose_->pose.pose.position.y;
        double thetaf = tf2::getYaw(follower_pose_->pose.pose.orientation);

        // Compute error in global frame
        double dx = posxl - posxf;
        double dy = posyl - posyf;
        double distance_error = std::sqrt(dx*dx + dy*dy) - follower_distance_;
        
        double angle_error = std::atan2(dy, dx) - thetaf;
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        // Compute control inputs with damping
        double v = k1_ * distance_error - damping_factor_ * leader_vel_.linear.x;
        double w = k2_ * angle_error - damping_factor_ * leader_vel_.angular.z;

        // Apply velocity limits
        if (v > MAX_LINEAR_VEL_) {
            RCLCPP_WARN(get_logger(), "Linear velocity clamped: %.2f -> %.2f", v, MAX_LINEAR_VEL_);
            v = MAX_LINEAR_VEL_;
        } else if (v < -MAX_LINEAR_VEL_) {
            RCLCPP_WARN(get_logger(), "Linear velocity clamped: %.2f -> %.2f", v, -MAX_LINEAR_VEL_);
            v = -MAX_LINEAR_VEL_;
        }

        if (w > MAX_ANGULAR_VEL_) {
            RCLCPP_WARN(get_logger(), "Angular velocity clamped: %.2f -> %.2f", w, MAX_ANGULAR_VEL_);
            w = MAX_ANGULAR_VEL_;
        } else if (w < -MAX_ANGULAR_VEL_) {
            RCLCPP_WARN(get_logger(), "Angular velocity clamped: %.2f -> %.2f", w, -MAX_ANGULAR_VEL_);
            w = -MAX_ANGULAR_VEL_;
        }

        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = v;
        cmd_msg.angular.z = w;
        follower_cmd_pub_->publish(cmd_msg);

        // Log data to CSV
        if (csv_file_) {
            csv_file_ << time_ << ","
                      << posxl << "," << posyl << "," << thetal << ","
                      << leader_vel_.linear.x << "," << leader_vel_.angular.z << ","
                      << posxf << "," << posyf << "," << thetaf << ","
                      << v << "," << w << ","
                      << distance_error << "," << angle_error << "\n";
        }
        time_ += 100;
        RCLCPP_INFO(get_logger(), 
            "Control: v=%.2f, w=%.2f, distance_error=%.2f, angle_error=%.2f", 
            v, w, distance_error, angle_error
        );
    }

    double follower_distance_;
    double follower_angle_;
    double k1_, k2_, damping_factor_;
    double MAX_LINEAR_VEL_, MAX_ANGULAR_VEL_;
    int time_;

    nav_msgs::msg::Odometry::SharedPtr leader_pose_;
    nav_msgs::msg::Odometry::SharedPtr follower_pose_;
    geometry_msgs::msg::Twist leader_vel_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr leader_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follower_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::ofstream csv_file_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LeaderFollowerController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
