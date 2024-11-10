#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MovementDataPublisher : public rclcpp::Node
{
  public:
    MovementDataPublisher()
    : Node("movement_data_publisher")
    {
      m_publisher = this->create_publisher<std_msgs::msg::String>("movement_data", 10);
      m_timer = this->create_wall_timer(500ms, std::bind(&MovementDataPublisher::timer_callback, this));
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

   void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world!";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      m_publisher->publish(message);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementDataPublisher>());
  rclcpp::shutdown();
  return 0;
}
