#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

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

      publisher_ = this->create_publisher<builtin_interfaces::msg::Time>(robot_name_ + "/clock", 10);
      timer_ = this->create_wall_timer(1s/hz_, bind(&RuntimeClock::timer_callback, this));

      start_time_ = std::chrono::high_resolution_clock::now();
    }


  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;

    double hz_;
    std::string robot_name_;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    int32_t sec_;
    uint32_t nanosec_;
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
  std::chrono::duration<double> step = std::chrono::high_resolution_clock::now() - start_time_;

  // Cast the step duration to seconds (int32_t) and nanoseconds (uint32_t)
  sec_ = static_cast<int32_t>(step.count());
  nanosec_ = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(step - std::chrono::seconds(sec_)).count()
  );

  auto message = builtin_interfaces::msg::Time();
  message.sec = sec_;
  message.nanosec = nanosec_;
  RCLCPP_INFO(this->get_logger(), "Publishing: %d %d", message.sec, message.nanosec);
  publisher_->publish(message);
}