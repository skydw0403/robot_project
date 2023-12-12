#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
  int step_;
  bool turn_left_;
  bool turn_right_;
  bool follow_wall_;

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0), turn_left_(false), turn_right_(false), follow_wall_(false)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile, callback);
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist vel;

    if (!turn_left_ && !turn_right_ && !follow_wall_)
    {
      vel.linear.x = 0.15; 
      vel.angular.z = 0.0;


      if (scan->ranges[90] < 0.5)
      {
        vel.linear.x = 0.0;
        vel.angular.z = 0.2;
        turn_left_ = true;
      }
    }
    else
    {

      if (scan->ranges[0] >= 0.25)
      {
        if (turn_left_)
        {
          vel.linear.x = 0.;
          vel.angular.z = -0.2;
          if (scan->ranges[90] < 0.5)
          {
            turn_left_ = false;
            turn_right_ = true;
          }
        }
        else if (turn_right_)
        {
          vel.linear.x = 0.;
          vel.angular.z = 0.2;
          if (scan->ranges[270] < 0.5)
          {
            turn_right_ = false;
            follow_wall_ = true;
          }
        }
        else if (follow_wall_)
        {
    
          vel.linear.x = 0.15; 
          vel.angular.z = 0.0;

          if (scan->ranges[90] < 0.35)
          {
            vel.linear.x = 0.1;
            vel.angular.z = 0.0; 
          }

  
          if (scan->ranges[90] > 0.5)
          {
            vel.linear.x = 0.0;
            vel.angular.z = 0.2;
            follow_wall_ = false;
            turn_right_ = true;
          }
        }
      }
      else
      {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("self_drive"),
                "step=%d, range0=%1.2f, linear=%1.2f, angular=%1.2f", step_, scan->ranges[0],
                vel.linear.x, vel.angular.z);

    pose_pub_->publish(vel);
    step_++;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

