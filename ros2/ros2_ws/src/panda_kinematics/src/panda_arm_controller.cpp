#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "kinematics.h"
#include "panda_kinematics/msg/joint_command.hpp" 
#include "unity_robotics_demo_msgs/msg/pos_rot.hpp"
#include "panda_kinematics/msg/all_joint_pos.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
  ArmController() : Node("arm_controller"), count_(0)
  {
    joint_cmd_publisher_  = this->create_publisher<panda_kinematics::msg::JointCommand>(
      "arm_joint_commands", 10);

    // for debug show all poses
    joint_pos_publisher_  = this->create_publisher<panda_kinematics::msg::AllJointPos>(
      "arm_joint_pos", 10);

    // Subscriber for target pose (pos_rot)
    pose_subscriber_ = this->create_subscription<unity_robotics_demo_msgs::msg::PosRot>(
      "/pos_rot", 10, 
      std::bind(&ArmController::pose_callback, this, std::placeholders::_1));
    
    // Subscriber for current joint states
    joint_state_subscriber_ = this->create_subscription<panda_kinematics::msg::JointCommand>(
      "/panda_joint_states_pos", 10,
      std::bind(&ArmController::joint_state_callback_pos, this, std::placeholders::_1));
    
    joint_vel_subscriber_ = this->create_subscription<panda_kinematics::msg::JointCommand>(
      "/panda_joint_states_vel", 10,
      std::bind(&ArmController::joint_state_callback_vel, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ArmController::timer_callback, this));
    
    // Initialize current joint positions to zero
    current_joint_angles_.setZero();
    target_joint_angles_.setZero();
    T_target_ = Eigen::Matrix4d::Identity();
    
    RCLCPP_INFO(this->get_logger(), "Arm controller node started");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /pos_rot, /panda_joint_states_pos, and /panda_joint_states_vel");
  }

private:
  void timer_callback()
  {
    // create a new cache every step
    static KinematicsCache cache;

    if (T_target_(0,3) == 0.0 && T_target_(1,3) == 0.0 && T_target_(2,3) == 0.0) {
        RCLCPP_WARN(this->get_logger(), "No target pose set yet.");
        return; // no target set yet
    }

    if (count_ < 50) {
        // this is the home position
        if (count_ > 25) { 
          open_gripper();
        } else {
          close_gripper();
          move_arm_home();
        }

        count_++;
        return;
    }

    

    // Increment step count
    count_++;
  }

  bool move_arm_step_vel(
    KinematicsCache& cache, 
    const Eigen::Matrix4d& T_target, 
    double kp_pos=1.0, 
    double kp_rot=1.0, 
    double joint_centering_rate=0.2, 
    double sol_tol_pos=0.1, 
    double sol_tol_angle=0.1)
  {
    // only keep rotation around Z axis for now
    Eigen::Vector3d z_down(0, 0, -1);

    // Project target X onto plane orthogonal to Z
    Eigen::Vector3d x_proj = R_target.col(0);
    x_proj -= x_proj.dot(z_down) * z_down;
    x_proj.normalize();

    Eigen::Vector3d y_axis = z_down.cross(x_proj);

    Eigen::Matrix3d R_down;
    R_down.col(0) = x_proj;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_down;

    Eigen::Matrix4d T_down = T_target;
    T_down.block<3,3>(0,0) = R_down;

    // IK cache (declared once, reused each loop)
    cache.setConfiguration(current_joint_angles_);
    Vector7d dq_step = inverse_kinematics_velocity(cache, T_down, kp_pos, kp_rot, joint_centering_rate);

    // if soultion is vaild, closing gripper happens outsdie this function
    if (is_valid_solution(current_joint_angles_, T_down, sol_tol_pos, sol_tol_angle)) {
      RCLCPP_INFO(this->get_logger(), "Valid solution reached, closing gripper.");
      return true; // switch to a state matchine in real application
    } 

    publish_joint_command_vel(dq_step, gripper_target_pos_);
    const auto& jointPositions = cache.jointPositions();
    publish_joint_positions(jointPositions);
    return false;
  }

  // make a step towards target pose with position, return true when reached
  bool move_arm_step_pos(
    KinematicsCache& cache, 
    const Eigen::Matrix4d& T_target, 
    double step_size=0.8, 
    double joint_centering_rate=0.2, 
    double sol_tol_pos=0.1, 
    double sol_tol_angle=0.1)
  {
    // only keep rotation around Z axis for now
    Eigen::Vector3d z_down(0, 0, -1);

    // Project target X onto plane orthogonal to Z
    Eigen::Vector3d x_proj = R_target.col(0);
    x_proj -= x_proj.dot(z_down) * z_down;
    x_proj.normalize();

    Eigen::Vector3d y_axis = z_down.cross(x_proj);

    Eigen::Matrix3d R_down;
    R_down.col(0) = x_proj;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_down;

    Eigen::Matrix4d T_down = T_target;
    T_down.block<3,3>(0,0) = R_down;

    // IK cache (declared once, reused each loop)
    cache.setConfiguration(current_joint_angles_);
    Vector7d dq_step = inverse_kinematics_step_optimized_position(cache, T_down, step_size, joint_centering_rate);

    target_joint_angles_ = current_joint_angles_ + dq_step; // scale step size

    // print joint angeles
    RCLCPP_INFO(this->get_logger(), "Step %zu: Current Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, Gripper: %.2f, Target Gripper: %.2f]",
                count_,
                current_joint_angles_[0], current_joint_angles_[1], current_joint_angles_[2],
                current_joint_angles_[3], current_joint_angles_[4], current_joint_angles_[5],
                current_joint_angles_[6], gripper_current_pos_, gripper_target_pos_);
    RCLCPP_INFO(this->get_logger(), "          Target  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                target_joint_angles_[0], target_joint_angles_[1], target_joint_angles_[2],
                target_joint_angles_[3], target_joint_angles_[4], target_joint_angles_[5],
                target_joint_angles_[6]);
    RCLCPP_INFO(this->get_logger(), "          Target Pose: Pos(%.2f, %.2f, %.2f) Rot:\n[%.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f]",
                T_down(0, 3), T_down(1, 3), T_down(2, 3),
                T_down(0, 0), T_down(0, 1), T_down(0, 2),
                T_down(1, 0), T_down(1, 1), T_down(1, 2),
                T_down(2, 0), T_down(2, 1), T_down(2, 2));

    // Debug: print current end-effector pose
    const auto& CurrentPose = cache.T0e();
    const auto& jointPositions = cache.jointPositions();
    RCLCPP_INFO(this->get_logger(), "Current pose: Pos(%.2f, %.2f, %.2f) Rot:\n[%.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f;\n %.2f, %.2f, %.2f]",
                CurrentPose(0, 3),
                CurrentPose(1, 3),
                CurrentPose(2, 3),
                CurrentPose(0, 0), CurrentPose(0, 1), CurrentPose(0, 2),
                CurrentPose(1, 0), CurrentPose(1, 1), CurrentPose(1, 2),
                CurrentPose(2, 0), CurrentPose(2, 1), CurrentPose(2, 2));

    // print the distance to target
    Eigen::Vector3d diff = diff_to_target(CurrentPose, T_down);
    RCLCPP_INFO(this->get_logger(), "Distance to target: X: %.4f, Y: %.4f, Z: %.4f",
                diff[0], diff[1], diff[2]);

    // if soultion is vaild, closing gripper happens outsdie this function
    if (is_valid_solution(current_joint_angles_, T_down, sol_tol_pos, sol_tol_angle)) {
      RCLCPP_INFO(this->get_logger(), "Valid solution reached, closing gripper.");
      return true; // switch to a state matchine in real application
    } 

    publish_joint_command_pos(target_joint_angles_, gripper_target_pos_);
    publish_joint_positions(jointPositions);
    return false;
  }

  // sets the arm to home position
  void move_arm_home_pos()
  {
    Vector7d first_pos;
    first_pos << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
    publish_joint_command_pos(first_pos, gripper_target_pos_);
  }

  void close_gripper()
  {
    gripper_target_pos_ = 0.0; // closed
    publish_joint_command_pos(current_joint_angles_, gripper_target_pos_);
  }

  void open_gripper()
  {
    gripper_target_pos_ = 1.0; // open
    publish_joint_command_pos(current_joint_angles_, gripper_target_pos_);
  }

  void pose_callback(const unity_robotics_demo_msgs::msg::PosRot::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received target pose: pos(%.2f, %.2f, %.2f)", 
    //             msg->pos_x, msg->pos_y, msg->pos_z);

    T_target_(0, 3) = msg->pos_x;
    T_target_(1, 3) = msg->pos_y;
    T_target_(2, 3) = msg->pos_z;
    Eigen::Quaterniond quat(msg->rot_w, msg->rot_x, msg->rot_y, msg->rot_z);
    quat.normalize();
    T_target_.block<3,3>(0,0) = quat.toRotationMatrix();
  }

  void joint_state_callback_pos(const panda_kinematics::msg::JointCommand::SharedPtr msg)
  {
    // Update current joint state
    if (msg->joint_angles.size() == 7) {
      for (int i = 0; i < 7; i++) {
        current_joint_angles_[i] = msg->joint_angles[i];
      }
      // RCLCPP_INFO(this->get_logger(), "Updated current joint state");
    }
    gripper_current_pos_ = msg->gripper_pos;
  }

  void joint_state_callback_vel(const panda_kinematics::msg::JointCommand::SharedPtr msg)
  {
    // Update current joint state
    if (msg->joint_angles.size() == 7) {
      for (int i = 0; i < 7; i++) {
        current_joint_velocities_[i] = msg->joint_angles[i];
      }
      // RCLCPP_INFO(this->get_logger(), "Updated current joint state");
    }
    gripper_current_pos_ = msg->gripper_pos;
  }

  void publish_joint_command_pos(const Vector7d& joint_angles, float gripper_pos=0.0)
  {
    // gripper: 1 open, 0 closed
    auto msg = panda_kinematics::msg::JointCommand();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.command_type = "position_joint";
    
    // Direct assignment to std::array (no resize needed)
    for (int i = 0; i < 7; i++) {
      msg.joint_angles[i] = joint_angles[i];
    }
    
    // Set gripper position
    msg.gripper_pos = gripper_pos;

    joint_cmd_publisher_->publish(msg);
  }

  void publish_joint_command_vel(const Vector7d& joint_vels, float gripper_pos=0.0)
  {
    // gripper: 1 open, 0 closed
    auto msg = panda_kinematics::msg::JointCommand();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.command_type = "velocity_joint";
    
    // Direct assignment to std::array (no resize needed)
    for (int i = 0; i < 7; i++) {
      msg.joint_angles[i] = joint_vels[i];
    }
    
    // Set gripper position
    msg.gripper_pos = gripper_pos;

    joint_cmd_publisher_->publish(msg);
  }

  void publish_joint_positions(const Eigen::Matrix<double, 8, 3> jointPositions)
  {
    panda_kinematics::msg::AllJointPos msg;
    for (int r = 0; r < 8; ++r) {
        for (int c = 0; c < 3; ++c) {
            msg.data[r * 3 + c] = jointPositions(r, c);
        }
    }
    joint_pos_publisher_->publish(msg);
  }

  rclcpp::Publisher<panda_kinematics::msg::JointCommand>::SharedPtr joint_cmd_publisher_;
  rclcpp::Publisher<panda_kinematics::msg::AllJointPos>::SharedPtr joint_pos_publisher_;
  rclcpp::Subscription<unity_robotics_demo_msgs::msg::PosRot>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<panda_kinematics::msg::JointCommand>::SharedPtr joint_state_subscriber_;
  Vector7d current_joint_angles_; // where the arm is currently
  Vector7d current_joint_velocities_; // where the arm is currently
  Vector7d target_joint_velocities_;
  Vector7d target_joint_angles_; // where the arm's next step is
  float gripper_current_pos_; // current gripper position
  float gripper_target_pos_; // target gripper position
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