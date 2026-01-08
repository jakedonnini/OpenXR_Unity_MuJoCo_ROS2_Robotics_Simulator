#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "kinematics.h"
#include "panda_kinematics/msg/joint_command.hpp"  // Changed this line

using namespace std::chrono_literals;

class ArmPublisher : public rclcpp::Node
{
public:
  ArmPublisher() : Node("arm_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<panda_kinematics::msg::JointCommand>(  // Changed this
      "arm_joint_commands", 10);
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ArmPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Arm publisher node started");
  }

private:
  void timer_callback()
  {
    auto msg = panda_kinematics::msg::JointCommand();  // Changed this
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    msg.joint_angles = {
      3.14, sin(count_ * 0.1) * 0.5, 0.0, -1.57, 
      0.0, cos(count_ * 0.1) * 0.3, 0.0
    };
    
    msg.command_type = "position";
    
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing joint commands");
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<panda_kinematics::msg::JointCommand>::SharedPtr publisher_;  // Changed this
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmPublisher>());
  rclcpp::shutdown();
  return 0;
}