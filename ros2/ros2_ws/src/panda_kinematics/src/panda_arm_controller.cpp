#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "kinematics.h"
#include "panda_kinematics/msg/joint_command.hpp"  // Changed this line
#include "panda_kinematics/msg/pos_rot.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
  ArmController() : Node("arm_controller"), count_(0)
  {
    joint_cmd_publisher_  = this->create_publisher<panda_kinematics::msg::JointCommand>(  // Changed this
      "arm_joint_commands", 10);

    // Subscriber for target pose (pos_rot)
    pose_subscriber_ = this->create_subscription<panda_kinematics::msg::PosRot>(
      "/pos_rot", 10, 
      std::bind(&ArmController::pose_callback, this, std::placeholders::_1));
    
    // Subscriber for current joint states
    joint_state_subscriber_ = this->create_subscription<panda_kinematics::msg::JointCommand>(
      "/panda_joint_states", 10,
      std::bind(&ArmController::joint_state_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ArmController::timer_callback, this));
    
    // Initialize current joint positions to zero
    current_joint_angles_.setZero();
    target_joint_angles_.setZero();
    T_target_ = Eigen::Matrix4d::Identity();
    
    RCLCPP_INFO(this->get_logger(), "Arm controller node started");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /pos_rot and /panda_joint_states");
  }

private:
  void timer_callback()
  {
    static KinematicsCache cache;
    // IK cache (declared once, reused each loop)
    cache.setConfiguration(current_joint_angles_);
    Vector7d dq_step = inverse_kinematics_step_optimized(cache, T_target_, 0.1, 0.1);

    target_joint_angles_ += dq_step; // scale step size

    publish_joint_command(target_joint_angles_);
    count_++;
  }

  void pose_callback(const panda_kinematics::msg::PosRot::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received target pose: pos(%.2f, %.2f, %.2f)", 
                msg->pos_x, msg->pos_y, msg->pos_z);
    
    // Set position
    T_target_(0, 3) = msg->pos_x;
    T_target_(1, 3) = msg->pos_y;
    T_target_(2, 3) = msg->pos_z;
    
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quat(msg->rot_w, msg->rot_x, msg->rot_y, msg->rot_z);
    T_target_.block<3,3>(0,0) = quat.toRotationMatrix();
  }

  void joint_state_callback(const panda_kinematics::msg::JointCommand::SharedPtr msg)
  {
    // Update current joint state
    if (msg->joint_angles.size() == 7) {
      for (int i = 0; i < 7; i++) {
        current_joint_angles_[i] = msg->joint_angles[i];
      }
      // RCLCPP_INFO(this->get_logger(), "Updated current joint state");
    }
  }

  void publish_joint_command(const Vector7d& joint_angles)
  {
    auto msg = panda_kinematics::msg::JointCommand();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.command_type = "position";
    
    // Direct assignment to std::array (no resize needed)
    for (int i = 0; i < 7; i++) {
      msg.joint_angles[i] = joint_angles[i];
    }
    
    joint_cmd_publisher_->publish(msg);
  }

  rclcpp::Publisher<panda_kinematics::msg::JointCommand>::SharedPtr joint_cmd_publisher_;
  rclcpp::Subscription<panda_kinematics::msg::PosRot>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<panda_kinematics::msg::JointCommand>::SharedPtr joint_state_subscriber_;
  Vector7d current_joint_angles_; // where the arm is currently
  Vector7d target_joint_angles_; // where the arm's next step is
  Eigen::Matrix4d T_target_; // position and rotation target object
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}