#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance") {
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ObstacleAvoidance::scanCallback, this, std::placeholders::_1)
        );
        
        count1_ = 0;
        count2_ = 0;
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data) {
        if (data->ranges.size() < 719) {
            RCLCPP_WARN(get_logger(), "Invalid data.ranges length");
            return;
        }

        // Compute averages for different regions
        double sum_centre = 0, sum_left = 0, sum_right = 0;
        
        // Left region
        for (int i = 41; i < 180; ++i) {
            sum_left += data->ranges[i];
        }
        
        // Right region
        for (int j = 540; j < 680; ++j) {
            sum_right += data->ranges[j];
        }
        
        // Centre region
        for (int k = 0; k < 41; ++k) {
            sum_centre += data->ranges[k];
        }
        for (int l = 680; l < 719; ++l) {
            sum_centre += data->ranges[l];
        }

        double forward_safe_distance = sum_centre / 40;
        double left_safe_distance = sum_left / 70;
        double right_safe_distance = sum_right / 70;

        auto msg = geometry_msgs::msg::Twist();

        if (forward_safe_distance > 0.9 && left_safe_distance > 0.9 && right_safe_distance > 0.9) {
            msg.linear.x = 0.15;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
        }
        else if (forward_safe_distance < 0.9 && left_safe_distance > 0.9 && right_safe_distance > 0.9) {
            msg.linear.x = 0.0;
            msg.angular.z = -0.5;
            publisher_->publish(msg);
        }
        else {
            if (left_safe_distance > right_safe_distance) {
                count1_++;
                count2_ = 0;
                
                if (count1_ >= 5) {
                    msg.linear.x = 0.0;
                    msg.angular.z = -0.5;
                    publisher_->publish(msg);

                    if (forward_safe_distance > 0.9 && left_safe_distance > 0.9 && right_safe_distance > 0.9) {
                        count1_ = 0;
                        msg.linear.x = 0.15;
                        msg.angular.z = 0.0;
                    }
                }
            }
            else if (left_safe_distance < right_safe_distance) {
                count2_++;
                count1_ = 0;
                
                if (count2_ >= 5) {
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.5;
                    publisher_->publish(msg);

                    if (forward_safe_distance > 0.9 && left_safe_distance > 0.9 && right_safe_distance > 0.9) {
                        count2_ = 0;
                        msg.linear.x = 0.15;
                        msg.angular.z = 0.0;
                    }
                }
            }
            
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    int count1_, count2_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto obstacle_avoidance = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(obstacle_avoidance);
    rclcpp::shutdown();
    return 0;
}
