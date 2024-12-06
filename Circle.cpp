#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class DrawCircleNode : public rclcpp::Node 
{
public:
    DrawCircleNode() : Node("draw_circle")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&DrawCircleNode::sendVelocity, this)
        );
        RCLCPP_INFO(this->get_logger(), "Draw circle node has been started");
    }

private:
    void sendVelocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;
        msg.angular.z = 0.8;
        cmd_vel_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
