#include "panda_kinematics/panda_arm_controller.hpp"
#include "panda_kinematics/commands/move_to_pose_command.hpp"
#include "panda_kinematics/commands/gripper_command.hpp"
#include "panda_kinematics/commands/wait_command.hpp"
#include "panda_kinematics/commands/callback_command.hpp"
#include "panda_kinematics/commands/move_home_command.hpp"

using namespace std::chrono_literals;

namespace panda_kinematics {

ArmController::ArmController() : Node("arm_controller"), count_(0)
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
  current_joint_velocities_.setZero();
  T_target_ = Eigen::Matrix4d::Identity();
  gripper_current_pos_ = 0.0;
  gripper_target_pos_ = 0.0;
  last_time_ = 0.0;
  
  state_ = State::IDLE;
  current_command_ = nullptr;
  
  RCLCPP_INFO(this->get_logger(), "Arm controller node started");

  // Load demo sequence TODO for test
  load_demo_sequence();
}

// Command Queue Managment

void ArmController::add_command(std::unique_ptr<RobotCommand> cmd) {
  RCLCPP_INFO(this->get_logger(), "Adding command '%s' to queue (queue size: %zu)", 
              cmd->get_name().c_str(), command_queue_.size());
  command_queue_.push(std::move(cmd));
}

void ArmController::clear_queue() {
  while (!command_queue_.empty()) {
    command_queue_.pop();
  }
  RCLCPP_INFO(this->get_logger(), "Command queue cleared");
}

bool ArmController::is_busy() const {
  return state_ != State::IDLE || !command_queue_.empty();
}

size_t ArmController::queue_size() const {
  return command_queue_.size();
}

bool ArmController::move_arm_vel(
  KinematicsCache& cache, 
  const Eigen::Matrix4d& T_target, 
  double kp_pos, 
  double kp_rot, 
  double joint_centering_rate, 
  double sol_tol_pos, 
  double sol_tol_angle,
  int max_iterations
  )
{
  // start from current position and iterate angels until target reached
  Vector7d q = current_joint_angles_;

  // iterate until max iterations or target reached
  int i = 0;
  while (i < max_iterations) {
    RCLCPP_INFO(this->get_logger(), "Iteration %d/%d", i+1, max_iterations);
    // take a step
    bool reached = move_arm_step_vel(
      cache, 
      T_target,
      q,
      kp_pos,
      kp_rot,
      joint_centering_rate,
      0.01, // sol_tol_pos
      0.05  // sol_tol_angle
    );

    RCLCPP_INFO(this->get_logger(), "Q: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                  q[0], q[1], q[2],
                  q[3], q[4], q[5],
                  q[6]);

    Eigen::Matrix4d T_end_effector = cache.T0e();
    RCLCPP_INFO(this->get_logger(), "Pos: (%.3f, %.3f, %.3f)", 
                T_end_effector(0,3), T_end_effector(1,3), T_end_effector(2,3));

    if (reached) {
      RCLCPP_INFO(this->get_logger(), "Target pose reached in %d iterations.", i+1);
      const auto& jointPositions = cache.jointPositions();
      publish_joint_positions(jointPositions);

      // set the target joint angles to last computed
      target_joint_angles_ = q;

      publish_joint_command_pos(target_joint_angles_, gripper_target_pos_);

      return true;
    }
    i++;
  }

  return false;
}

bool ArmController::move_arm_step_vel(
  KinematicsCache& cache, 
  const Eigen::Matrix4d& T_target,
  Vector7d& final_pos, 
  double kp_pos, 
  double kp_rot, 
  double joint_centering_rate, 
  double sol_tol_pos, 
  double sol_tol_angle
  )
{

  // caluate the time step between this step and last
  // double dt = this->now().seconds() - last_time_;
  double dt = 0.01;

  Eigen::Matrix3d R_target = T_target.block<3,3>(0,0);

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

  // clamp min z value
  float min_z = 0.04;
  if (T_down(2,3) < min_z) {
    T_down(2,3) = min_z;
  }

  // IK cache (declared once, reused each loop)
  cache.setConfiguration(final_pos);
  Vector7d dq_step = inverse_kinematics_velocity(cache, T_down, kp_pos, kp_rot, joint_centering_rate);

  final_pos = final_pos + dq_step * static_cast<float>(dt);

  // if soultion is vaild, closing gripper happens outsdie this function
  if (is_valid_solution(final_pos, T_down, sol_tol_pos, sol_tol_angle)) {
    RCLCPP_INFO(this->get_logger(), "Valid solution reached");
    RCLCPP_WARN(this->get_logger(), "Final Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                final_pos[0], final_pos[1], final_pos[2],
                final_pos[3], final_pos[4], final_pos[5],
                final_pos[6]);
    return true; // switch to a state matchine in real application
  } 

  return false;
}

// make a step towards target pose with position, return true when reached
bool ArmController::move_arm_step_pos(
  KinematicsCache& cache, 
  const Eigen::Matrix4d& T_target, 
  double step_size, 
  double joint_centering_rate, 
  double sol_tol_pos, 
  double sol_tol_angle)
{
  Eigen::Matrix3d R_target = T_target.block<3,3>(0,0);

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
void ArmController::move_arm_home_pos()
{
  target_joint_angles_ << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
  publish_joint_command_pos(target_joint_angles_, gripper_target_pos_);
}

bool ArmController::reached_arm_home_position()
{
  target_joint_angles_ << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
  double tol = 0.1; // radians
  for (int i = 0; i < 7; i++) {
    if (std::abs(current_joint_angles_[i] - target_joint_angles_[i]) > tol) {
      return false;
    }
  }
  return true;
}

void ArmController::close_gripper()
{
  gripper_target_pos_ = GRIPPER_CLOSED_POS_; // closed
  publish_joint_command_pos(target_joint_angles_, gripper_target_pos_);
}

void ArmController::open_gripper()
{
  gripper_target_pos_ = GRIPPER_OPEN_POS_; // open
  publish_joint_command_pos(target_joint_angles_, gripper_target_pos_);
}

bool ArmController::gripper_reached_target(double tolerance) const {
  // RCLCPP_INFO(this->get_logger(), "Gripper current pos: %.4f, target pos: %.4f, tolerance: %.4f",
  //             gripper_current_pos_, gripper_target_pos_, tolerance);
  return std::abs(gripper_current_pos_ - gripper_target_pos_) < tolerance;
}

rclcpp::Time ArmController::now() {
  return this->get_clock()->now();
}

// Command Queue State Machine

void ArmController::timer_callback()
{
  // if (T_target_(0,3) == 0.0 && T_target_(1,3) == 0.0 && T_target_(2,3) == 0.0) {
  //     RCLCPP_WARN(this->get_logger(), "No target pose set yet.");
  //     return; // no target set yet
  // }

  count_++;
  
  // State machine for command execution
  switch (state_) {
    case State::IDLE:
      // Start next command if available
      if (!command_queue_.empty()) {
        start_next_command();
      }
      break;
      
    case State::EXECUTING:
      execute_current_command();
      break;
      
    case State::WAITING:
      // External event needed
      break;
  }
}

void ArmController::start_next_command() {
  current_command_ = std::move(command_queue_.front());
  command_queue_.pop();
  
  command_start_time_ = this->now();
  state_ = State::EXECUTING;
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Starting command: %s (queue remaining: %zu) ===", 
              current_command_->get_name().c_str(),
              command_queue_.size());
  
  current_command_->on_start();
}

void ArmController::execute_current_command() {
  if (!current_command_) {
    state_ = State::IDLE;
    return;
  }

  // Check for timeout
  double timeout = current_command_->get_timeout();
  if (timeout > 0.0) {
    auto elapsed = (this->now() - command_start_time_).seconds();
    if (elapsed > timeout) {
      RCLCPP_WARN(this->get_logger(), 
                  "Command '%s' timed out after %.2f seconds",
                  current_command_->get_name().c_str(), elapsed);
      complete_current_command(false);
      return;
    }
  }

  // Execute the command
  bool is_complete = current_command_->execute();
  
  if (is_complete) {
    complete_current_command(true);
  }
}

void ArmController::complete_current_command(bool success) {
  if (!current_command_) return;
  
  auto elapsed = (this->now() - command_start_time_).seconds();
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Command '%s' completed in %.2f seconds [%s] ===",
              current_command_->get_name().c_str(),
              elapsed,
              success ? "SUCCESS" : "FAILED");
  
  current_command_->on_complete(success);
  current_command_.reset();
  state_ = State::IDLE;
}

// ROS Callbacks and Publishers

void ArmController::pose_callback(const unity_robotics_demo_msgs::msg::PosRot::SharedPtr msg)
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

void ArmController::joint_state_callback_pos(const panda_kinematics::msg::JointCommand::SharedPtr msg)
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

void ArmController::joint_state_callback_vel(const panda_kinematics::msg::JointCommand::SharedPtr msg)
{
  // Update current joint state
  if (msg->joint_angles.size() == 7) {
    for (int i = 0; i < 7; i++) {
      current_joint_velocities_[i] = msg->joint_angles[i];
    }
    // RCLCPP_INFO(this->get_logger(), "Updated current joint state");
  }
}

void ArmController::publish_joint_command_pos(const Vector7d& joint_angles, float gripper_pos)
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

void ArmController::publish_joint_command_vel(const Vector7d& joint_vels, float gripper_pos)
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

void ArmController::publish_joint_positions(const Eigen::Matrix<double, 8, 3> jointPositions)
{
  panda_kinematics::msg::AllJointPos msg;
  for (int r = 0; r < 8; ++r) {
      for (int c = 0; c < 3; ++c) {
          msg.data[r * 3 + c] = jointPositions(r, c);
      }
  }
  joint_pos_publisher_->publish(msg);
}

// Sequences
void ArmController::load_demo_sequence() {
  RCLCPP_INFO(this->get_logger(), "Loading demo pick-and-place sequence...");
  
  // 1. Move to home position
  add_command(std::make_unique<MoveHomeCommand>(this));
  
  // 2. Open gripper
  add_command(std::make_unique<OpenGripperCommand>(this));
  
  // 3. Wait a bit
  add_command(std::make_unique<WaitCommand>(this, 1.0));

  add_command(std::make_unique<MoveToPoseCommand>(this, std::ref(T_target_), MoveToPoseCommand::ByReference{}));
  
  add_command(std::make_unique<WaitCommand>(this, 2.0));

  // 5. Close gripper
  add_command(std::make_unique<CloseGripperCommand>(this));

  add_command(std::make_unique<WaitCommand>(this, 2.0)); // wait for cube to settle
  
  // 4. Move to pick position following the cube. lift it in the air
  Eigen::Matrix4d above_target = Eigen::Matrix4d::Identity();
  above_target(0,3) = 0.48;
  above_target(1,3) = 0.0;
  above_target(2,3) = 0.5;
  add_command(std::make_unique<MoveToPoseCommand>(this, above_target));
  
  // 6. Wait
  add_command(std::make_unique<WaitCommand>(this, 2.0));
  
  // 7. Move to place position
  Eigen::Matrix4d place_pose_2 = Eigen::Matrix4d::Identity();
  place_pose_2(0, 3) = 0.4;
  place_pose_2(1, 3) = -0.2;
  place_pose_2(2, 3) = 0.3;
  add_command(std::make_unique<MoveToPoseCommand>(this, place_pose_2));

  // 6. Wait
  add_command(std::make_unique<WaitCommand>(this, 2.0));
  
  // 8. Open gripper
  add_command(std::make_unique<OpenGripperCommand>(this));
  
  // 9. Custom callback
  add_command(std::make_unique<CallbackCommand>(
    "LogMessage",
    [this]() {
      RCLCPP_INFO(this->get_logger(), "Custom callback executed!");
      return true;
    }
  ));
  
  // 10. Return home
  add_command(std::make_unique<MoveHomeCommand>(this));
  
  RCLCPP_INFO(this->get_logger(), "Demo sequence loaded with %zu commands", queue_size());
}


} // namespace