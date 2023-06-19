#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class RuntimeClock : public rclcpp::Node
{
  public:
    RuntimeClock() : Node("runtime_clock"), start_time_(std::chrono::system_clock::now())
    {
      // Param init
      declare_parameter("frame_id", "");     
      declare_parameter("hz", 1.0f);         // 1.0f -> float type

      // Param load
      get_parameter("hz", hz_);
      get_parameter("frame_id", frame_id_);

      publisher_ = this->create_publisher<builtin_interfaces::msg::Time>(frame_id_ + "/clock", 10);
      timer_ = this->create_wall_timer(1s/hz_, bind(&RuntimeClock::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "Timer %s/clock started with %f hz frequency.", frame_id_.c_str(), hz_);
    }


  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;

    float hz_;
    std::string frame_id_;

    const std::chrono::time_point<std::chrono::system_clock> start_time_;
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
  auto message = builtin_interfaces::msg::Time();
  
  std::chrono::duration<double> step = std::chrono::system_clock::now() - start_time_;

  // Cast the step duration to seconds (int32_t) and nanoseconds (uint32_t)
  message.sec = static_cast<int32_t>(step.count());
  message.nanosec = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(step - std::chrono::seconds(message.sec)).count()
  );

  publisher_->publish(message);
}