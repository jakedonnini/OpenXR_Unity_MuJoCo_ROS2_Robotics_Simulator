#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "kinematics.h"
#include "panda_kinematics/msg/joint_command.hpp"  // Changed this line
#include "unity_robotics_demo_msgs/msg/pos_rot.hpp"  // Use the original

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
  ArmController() : Node("arm_controller"), count_(0)
  {
    joint_cmd_publisher_  = this->create_publisher<panda_kinematics::msg::JointCommand>(  // Changed this
      "arm_joint_commands", 10);

    // Subscriber for target pose (pos_rot)
    pose_subscriber_ = this->create_subscription<unity_robotics_demo_msgs::msg::PosRot>(
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

    if (is_valid_solution(current_joint_angles_, T_target_, 0.05, 0.05)) {
        RCLCPP_INFO(this->get_logger(), "Reached target within tolerance.");
        // make arm stop moving
        return;
    }

    if (T_target_(0,3) == 0.0 && T_target_(1,3) == 0.0 && T_target_(2,3) == 0.0) {
        RCLCPP_WARN(this->get_logger(), "No target pose set yet.");
        Vector7d first_pos;
        first_pos << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
        publish_joint_command(first_pos);
        return; // no target set yet
    }

    Eigen::Matrix4d not_Rot_T = T_target_;
    not_Rot_T.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); // keep orientation fixed for now
    not_Rot_T(1,1) = -1.0; // make pointing down
    not_Rot_T(2,2) = -1.0;

    static KinematicsCache cache;
    // IK cache (declared once, reused each loop)
    cache.setConfiguration(current_joint_angles_);
    Vector7d dq_step = inverse_kinematics_step_optimized(cache, not_Rot_T, 0.05, 0.2);

    target_joint_angles_ = current_joint_angles_ + dq_step; // scale step size

    // print joint angeles
    RCLCPP_INFO(this->get_logger(), "Step %zu: Current Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                count_,
                current_joint_angles_[0], current_joint_angles_[1], current_joint_angles_[2],
                current_joint_angles_[3], current_joint_angles_[4], current_joint_angles_[5],
                current_joint_angles_[6]);
    RCLCPP_INFO(this->get_logger(), "          Target  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                target_joint_angles_[0], target_joint_angles_[1], target_joint_angles_[2],
                target_joint_angles_[3], target_joint_angles_[4], target_joint_angles_[5],
                target_joint_angles_[6]);
    RCLCPP_INFO(this->get_logger(), "          Target Pose: Pos(%.2f, %.2f, %.2f) Rot:\n[%.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f]",
                T_target_(0, 3), T_target_(1, 3), T_target_(2, 3),
                T_target_(0, 0), T_target_(0, 1), T_target_(0, 2),
                T_target_(1, 0), T_target_(1, 1), T_target_(1, 2),
                T_target_(2, 0), T_target_(2, 1), T_target_(2, 2));

    // Debug: print current end-effector pose
    Eigen::Matrix4d CurrentPose;
    Eigen::Matrix<double, 8, 3> jointPositions;
    forwardKinematics(current_joint_angles_, jointPositions, CurrentPose);
    RCLCPP_INFO(this->get_logger(), "Current pose: Pos(%.2f, %.2f, %.2f) Rot:\n[%.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f]",
                CurrentPose(0, 3),
                CurrentPose(1, 3),
                CurrentPose(2, 3),
                CurrentPose(0, 0), CurrentPose(0, 1), CurrentPose(0, 2),
                CurrentPose(1, 0), CurrentPose(1, 1), CurrentPose(1, 2),
                CurrentPose(2, 0), CurrentPose(2, 1), CurrentPose(2, 2));

    // test by rotating to pi one at a time
    // Vector7d test;
    // float num = M_PI/2;
    // if (count_ > 100 && count_ <= 200)
    //     test << num, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // else if (count_ > 200 && count_ <= 300)
    //     test << num, num, 0.0, 0.0, 0.0, 0.0, 0.0;
    // else if (count_ > 300 && count_ <= 400)
    //     test << num, num, num, 0.0, 0.0, 0.0, 0.0;
    // else if (count_ > 400 && count_ <= 500)
    //     test << num, num, num, -num, 0.0, 0.0, 0.0;
    // else if (count_ > 500 && count_ <= 600)
    //     test << num, num, num, -num, num, 0.0, 0.0;
    // else if (count_ > 600 && count_ <= 700)
    //     test << num, num, num, -num, num, num, 0.0;
    // else
    //     test << num, num, num, -num, num, num, num;
    if (count_ > 1000)
        target_joint_angles_ << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
    publish_joint_command(target_joint_angles_);
    count_++;
  }

  void pose_callback(const unity_robotics_demo_msgs::msg::PosRot::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received target pose: pos(%.2f, %.2f, %.2f)", 
    //             msg->pos_x, msg->pos_y, msg->pos_z);
    
    // Set position
    T_target_(0, 3) = msg->pos_x;
    T_target_(1, 3) = msg->pos_y;
    T_target_(2, 3) = msg->pos_z;
    
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quat(msg->rot_w, msg->rot_z, -msg->rot_x, msg->rot_y); // Note the reordering and sign change
    quat.normalize();
    T_target_.block<3,3>(0,0) = quat.toRotationMatrix();
    // Unity → ROS position
    T_target_(0,3) =  msg->pos_z;
    T_target_(1,3) = -msg->pos_x;
    T_target_(2,3) =  msg->pos_y;
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
  rclcpp::Subscription<unity_robotics_demo_msgs::msg::PosRot>::SharedPtr pose_subscriber_;
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