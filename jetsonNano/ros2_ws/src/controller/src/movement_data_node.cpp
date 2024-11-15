#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MovementDataPublisher : public rclcpp::Node
{
  public:
    MovementDataPublisher()
    : Node("movement_data_publisher")
    {
      m_publisher = this->create_publisher<std_msgs::msg::Int32>("movement_data", 10);
      m_timer = this->create_wall_timer(500ms, std::bind(&MovementDataPublisher::timer_callback, this));
    }

  private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

   void timer_callback()
    {
      auto message = std_msgs::msg::Int32();
      message.data = 101;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data);
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
