#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

std::string zmienna3 = "";

double rover_pos[3];

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("drive_to_pos_node")
    {
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/a200_0000/platform/odom/filtered", 1, std::bind(&MinimalPublisher::subscribe_message, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("a200_0000/cmd_vel", 1);
      timer_ = this->create_wall_timer(0.05ms, std::bind(&MinimalPublisher::publish_message, this));
      this->declare_parameter("pos_x", double(0.0));
      this->declare_parameter("pos_y", double(0.0));
    }

  private:
    void subscribe_message(const nav_msgs::msg::Odometry::SharedPtr message) const
    {
      nav_msgs::msg::Odometry_<std::allocator<void> >::_pose_type zmienna = (message->pose);
      geometry_msgs::msg::Point pozycja = zmienna.pose.position;
      geometry_msgs::msg::Pose_<std::allocator<void> >::_orientation_type orientacja = zmienna.pose.orientation;
      rover_pos[0] = pozycja.x;
      rover_pos[1] = pozycja.y;
      rover_pos[2] = orientacja.z;
    }


    void publish_message()
    {
      auto message = geometry_msgs::msg::Twist();

      if(time <=2001.0)
      {
        time += 0.05;
      }

      if(time > 2000.0)
      {
        if(rover_pos[2] > 0)
        {
          this->multiplier_x_rot = -1;
        }
        else
        {
          this->multiplier_x_rot = 1;
        }
      }
      

      if((is_0_pos_reached == false) && (is_x_reached == false) && (time > 2000.0))
      {
        message.linear.x = 0.0;
        message.angular.z = (this->multiplier_x_rot)*0.05;
        if((rover_pos[2] <= 0.005) && (rover_pos[2] >= -0.005))
        {
          is_0_pos_reached = true;
          if((this->get_parameter("pos_x").as_double() - rover_pos[0]) < 0.0)
          {
            this->multiplier = -1;
          }
        }
      }

      if((is_0_pos_reached == true) && (is_x_reached == false))
      {
        message.linear.x = (this->multiplier)*0.1;
        message.angular.z = 0.0;
        if(std::abs((this->get_parameter("pos_x").as_double() - rover_pos[0])) < 0.04)
        {
          message.linear.x = 0.0;
          message.angular.z = 0.0;
          is_x_reached = true;
        }
      }


      if((is_y_rot_reached == false) && (is_x_reached == true))
      {
        message.linear.x = 0.0;
        message.angular.z = 0.02;
        if((rover_pos[2] > 0.705) && (rover_pos[2] < 0.72))
        {
          is_y_rot_reached = true;
          if((this->get_parameter("pos_y").as_double()-rover_pos[1]) >= 0.0)
          {
            this->multiplier = 1;
          }
          else
          {
            this->multiplier = -1;
          }
        }
      }

      if((is_y_rot_reached == true) && (is_y_reached == false))
      {
        message.linear.x = (this->multiplier)*0.1;
        message.angular.z = 0.0;
        if(std::abs((this->get_parameter("pos_y").as_double() - rover_pos[1])) < 0.04)
        {
          is_y_reached = true;
        }
      }

      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    bool is_0_pos_reached = false;
    bool is_y_rot_reached = false;
    bool is_x_reached = false;
    bool is_y_reached = false;
    int multiplier = 1;
    int multiplier_x_rot = 1;
    double time = 0.0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
