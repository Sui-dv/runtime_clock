#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;

class RuntimeClock : public rclcpp::Node
{
  public:
    RuntimeClock() : Node("runtime_clock")
    {
      // Param init
      declare_parameter("robot_name", "Lasagna");   // defaults to "Lasagna"
      declare_parameter("hz", 1.0f);                // 1.0f -> float type

      // Param load
      get_parameter("hz", hz_);
      get_parameter("robot_name", robot_name_);

      publisher_ = this->create_publisher<std_msgs::msg::String>("clock", 10);
      timer_ = this->create_wall_timer( 1/hz_*1000ms, std::bind(&RuntimeClock::timer_callback, this));
    }


  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    double hz_;
    std::string robot_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RuntimeClock>());
  rclcpp::shutdown();
  return 0;
}

void RuntimeClock::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + robot_name_ + std::to_string(hz_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}