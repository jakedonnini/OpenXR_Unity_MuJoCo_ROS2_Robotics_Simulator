#include "panda_kinematics/panda_arm_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<panda_kinematics::ArmController>());
  rclcpp::shutdown();
  return 0;
}