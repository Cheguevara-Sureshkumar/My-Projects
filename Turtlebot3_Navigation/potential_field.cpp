#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>

#define PI 3.14159265

using std::placeholders::_1;

class PotentialField : public rclcpp::Node
{
  public:
    PotentialField(char* x_goal, char* y_goal)
    : Node("potential_field_node")

    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&PotentialField::odom_callback, this, _1));

      sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", default_qos, std::bind(&PotentialField::scan_callback, this, _1));

      cmd_pub  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);

      att_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("attraction_vector",1);

      rep_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("repulsion_vector",1);

      fin_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("Final_Vector",1);

      RCLCPP_INFO(this->get_logger(), "x = %s and y = %s", x_goal,y_goal);

      goal_x = atof(x_goal);
      goal_y = atof(y_goal);


    }

    int controller()
    {

      double x_final = V_attraction[0] + V_repulsion[0];
      double y_final = V_attraction[1] + V_repulsion[1];

      geometry_msgs::msg::PoseStamped finalvector = PublishVector(x_final,y_final);
      fin_pub->publish(finalvector);
      geometry_msgs::msg::Twist direction;
      double tolerance = 0.2 ;
      double angle = atan(y_final/x_final);
      double delta;

      if (x_final < 0)
      {
        angle = PI + atan(y_final/x_final);
      }
      else
      {
        angle = atan(y_final/x_final);
      }

      delta = PI - fabs(fmod(fabs(angle - theta), 2*PI) - PI);

      RCLCPP_INFO(this->get_logger(), "angle to goal: %f", delta);
      
      if(delta < 0 - tolerance)
      {
        direction.angular.z = -0.2;
        direction.linear.x  = 0;
        // v angle +
      }
      else if(delta > 0 + tolerance)
      {
        // v angle -
        direction.angular.z = 0.2;
        direction.linear.x  = 0;
      }
      else
      {
        // v forward +
        direction.linear.x  = 0.1;
        direction.angular.z = 0;
      
      }

      cmd_pub->publish(direction);
      return 0;
    }

    geometry_msgs::msg::PoseStamped PublishVector(float x, float y)
    {
      geometry_msgs::msg::PoseStamped vector;
      
      std::string id_frame = "/odom";
      vector.header.frame_id = id_frame;
      vector.header.stamp = this->get_clock()->now();
      vector.pose.position.x = x_odom ;
      vector.pose.position.y = y_odom ;
      vector.pose.position.z = 0 ;
      float angle;
      if (x < 0)
      {
        angle = PI + atan(y/x);
      }
      else
      {
        angle = atan(y/x);
      }
      
      tf2::Quaternion q;
      q.setRPY(0,0,angle);
      vector.pose.orientation = tf2::toMsg(q);

      return vector;

    }

    void ComputeAttraction(float x_a, float y_a)
    {
      RCLCPP_INFO(this->get_logger(), "GOAL | x : %f | y : %f",x_a,y_a);

      float distance =  sqrt(pow(x_a - x_odom,2) + pow(y_a - y_odom,2));
      x_a = x_a - x_odom;
      y_a = y_a - y_odom;

      int Q_attraction = 100;
      float F_attraction = (Q_attraction )/(4 * PI * pow(distance,2));
      V_attraction = {F_attraction * x_a , F_attraction * y_a};
    
      //RCLCPP_INFO(this->get_logger(), "x : %f | y : %f",x_a,y_a);
      //RCLCPP_INFO(this->get_logger(), "Force: %f",F_attraction);

      //RCLCPP_INFO(this->get_logger(), "angle attraction :%f°",atan(V_attraction[1]/V_attraction[0])*180/PI);
      //RCLCPP_INFO(this->get_logger(), "v_attraction is : x = %f ; y = %f",x,y);

      geometry_msgs::msg::PoseStamped attraction = PublishVector(V_attraction[0],V_attraction[1]);
      att_pub->publish(attraction);

    }
    

  private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // set current x position
      x_odom = msg->pose.pose.position.x;
      // set current y position
      y_odom = msg->pose.pose.position.y;

      tf2::Quaternion q(
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z,
              msg->pose.pose.orientation.w);
      
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      theta = yaw;

      //RCLCPP_INFO(this->get_logger(), "Odometry : x = %f | y = %f | theta = %f" , x , y, theta);

      ComputeAttraction(goal_x,goal_y);  

    }

    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
      float angle_min = _msg->angle_min;
      //float angle_max = _msg->angle_max;
      float step      = _msg->angle_increment; 
      auto scan       = _msg->ranges;
      auto len        = int(float(scan.size()));

      int counter = 0;


      float x_r = 0;
      float y_r = 0;

      for(int i = 0 ; i < len ; i++)
      {
        if(scan[i] < 100 and scan[i] > 0.1)
        { 
          int Q_repulsion = 1;
          //RCLCPP_INFO(this->get_logger(), "Scan n: %d | value: %f",i,scan[i]);
          float Current_Q = (Q_repulsion) / (4 * PI * pow(scan[i],2));
          x_r -= Current_Q * cos(angle_min+theta+step*i);
          y_r -= Current_Q * sin(angle_min+theta+step*i);
        }
        else
        {
          counter += 1;
        }
        
      }
      //RCLCPP_INFO(this->get_logger(), "Counter : %d  ",counter);
      if(counter == 360)
      {
        V_repulsion = {0.0001,0.000000000001};
      }
      else
      {
        //RCLCPP_INFO(this->get_logger(), "x: %f | y: %f",x_r,y_r);
        V_repulsion = {x_r, y_r};
      }

      //RCLCPP_INFO(this->get_logger(), "\n angle repulsion : %f°",atan(V_repulsion[1]/V_repulsion[0])*180/PI);


      geometry_msgs::msg::PoseStamped repulsion = PublishVector(V_repulsion[0],V_repulsion[1]);
      rep_pub->publish(repulsion);
      controller();
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr att_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rep_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fin_pub;

    double x_odom;
    double y_odom;
    double theta;
    std::vector<float> V_attraction ;
    std::vector<float> V_repulsion ;
    float goal_x = 0;
    float goal_y = 0;

};

int main(int argc, char * argv[])
{  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PotentialField>(argv[1],argv[2]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
