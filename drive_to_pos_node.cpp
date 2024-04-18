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

double rover_pos[3];

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("drive_to_pos_node")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("a200_0000/cmd_vel", 1);
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/a200_0000/platform/odom/filtered", 1, std::bind(&MinimalPublisher::subscribe_message, this, _1));
      timer_ = this->create_wall_timer(0.05ms, std::bind(&MinimalPublisher::publish_message, this));
      this->declare_parameter("pos_x", double(0.0));
      this->declare_parameter("pos_y", double(0.0));
    }

  private:
    void subscribe_message(const nav_msgs::msg::Odometry::SharedPtr message) const
    {
      nav_msgs::msg::Odometry_<std::allocator<void> >::_pose_type zmienna = (message->pose);
      geometry_msgs::msg::Point pozycja = zmienna.pose.position;
      rover_pos[0] = pozycja.x;
      rover_pos[1] = pozycja.y;
      rover_pos[2] = pozycja.z;
    }


    void publish_message()
    {
      auto message = geometry_msgs::msg::Twist();

      if((this->get_parameter("pos_x").as_double() > rover_pos[0]) && (this->get_parameter("pos_y").as_double() < rover_pos[1]) && (std::abs((this->get_parameter("pos_x").as_double()-rover_pos[0]) > 0.01) || std::abs((this->get_parameter("pos_y").as_double()-rover_pos[1]) > 0.01)))
      {
        message.linear.x = 0.2;
        message.linear.y = -0.2;
        message.angular.z = -0.2;
        publisher_->publish(message);
      }

      if((this->get_parameter("pos_x").as_double() > rover_pos[0]) && (this->get_parameter("pos_y").as_double() > rover_pos[1]) && (std::abs((this->get_parameter("pos_x").as_double()-rover_pos[0]) > 0.01) || std::abs((this->get_parameter("pos_y").as_double()-rover_pos[1]) > 0.01)))
      {
        message.linear.x = 0.2;
        message.linear.y = 0.2;
        message.angular.z = 0.2;
        publisher_->publish(message);
      }

      if((this->get_parameter("pos_x").as_double() < rover_pos[0]) && (this->get_parameter("pos_y").as_double() > rover_pos[1]) && (std::abs((this->get_parameter("pos_x").as_double()-rover_pos[0]) > 0.01) || std::abs((this->get_parameter("pos_y").as_double()-rover_pos[1]) > 0.01)))
      {
        message.linear.x = -0.2;
        message.linear.y = 0.2;
        message.angular.z = 0.2;
        publisher_->publish(message);
      }

      if((this->get_parameter("pos_x").as_double() < rover_pos[0]) && (this->get_parameter("pos_y").as_double() < rover_pos[1]) && (std::abs((this->get_parameter("pos_x").as_double()-rover_pos[0]) > 0.01) || std::abs((this->get_parameter("pos_y").as_double()-rover_pos[1]) > 0.01)))
      {
        message.linear.x = -0.2;
        message.linear.y = -0.2;
        message.angular.z = -0.2;
        publisher_->publish(message);
      }

      if(std::abs((this->get_parameter("pos_x").as_double()-rover_pos[0]) < 0.01) && std::abs((this->get_parameter("pos_y").as_double()-rover_pos[1]) < 0.01))
      {
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
      }



    }
    


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    float i;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
