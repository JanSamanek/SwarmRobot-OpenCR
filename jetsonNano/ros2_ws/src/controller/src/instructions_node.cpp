#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "instructions_node.h"

using namespace std::chrono_literals;

class InstructionsPublisher : public rclcpp::Node
{
  public:
    InstructionsPublisher()
    : Node("movement_data_publisher")
    {
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("microRos/moveInstructions", 10);
      m_timer = this->create_wall_timer(500ms, std::bind(&InstructionsPublisher::timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

   void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1024;
      message.linear.y = 1024;
      message.linear. z = 0;

      message.angular.x = 0;
      message.angular.y = 0;
      message.angular.z = 1024;
      m_publisher->publish(message);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InstructionsPublisher>());
  rclcpp::shutdown();
  return 0;
}