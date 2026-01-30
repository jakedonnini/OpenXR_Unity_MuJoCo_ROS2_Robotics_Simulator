#ifndef PANDA_KINEMATICS_ARM_CONTROLLER_HPP
#define PANDA_KINEMATICS_ARM_CONTROLLER_HPP

#include <chrono>
#include <memory>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "panda_kinematics/kinematics.h"
#include "panda_kinematics/msg/joint_command.hpp" 
#include "unity_robotics_demo_msgs/msg/pos_rot.hpp"
#include "panda_kinematics/msg/all_joint_pos.hpp"
#include "panda_kinematics/commands/robot_command.hpp"


using namespace std::chrono_literals;

namespace panda_kinematics {

/**
 * @brief Main controller for the Panda robot arm with command queue system
 * 
 * Provides a command queue for sequential execution of robot commands.
 * Commands are executed one at a time, with automatic progression to the
 * next command when the current one completes. Includes kineomatic algos
 * for control of the Panda Arm.
 */
class ArmController : public rclcpp::Node
{
public:
    ArmController();

    // Command Queue Management
    /**
     * @brief Add a command to the execution queue
     * @param cmd Unique pointer to the command to add
     */
    void add_command(std::unique_ptr<panda_kinematics::RobotCommand> cmd);

    /**
     * @brief Clear all pending commands from the queue
     */
    void clear_queue();

    /**
     * @brief Check if controller is currently executing or has pending commands
     * @return true if busy, false if idle
     */
    bool is_busy() const;

    /**
     * @brief Get number of pending commands in queue
     * @return queue size
     */
    size_t queue_size() const;

    // Robot control Functions
    /**
     * @brief Execute one velocity step toward target pose
     * @return true if target reached within tolerance
     */
    bool move_arm_step_vel(
        KinematicsCache& cache, 
        const Eigen::Matrix4d& T_target, 
        double kp_pos = 1.0, 
        double kp_rot = 1.0, 
        double joint_centering_rate = 0.2, 
        double sol_tol_pos = 0.1, 
        double sol_tol_angle = 0.1
    );

    /**
     * @brief Older version of algo that uses the pos differentnce.
     * Works decently but doesn't converge fully.
     * @return true if target reached within tolerance
     */
    bool move_arm_step_pos(
        KinematicsCache& cache, 
        const Eigen::Matrix4d& T_target, 
        double step_size=0.8, 
        double joint_centering_rate=0.2, 
        double sol_tol_pos=0.1, 
        double sol_tol_angle=0.1
    );

    /**
     * @brief Command gripper to open
     */
    void open_gripper();

    /**
     * @brief Command gripper to close
     */
    void close_gripper();

    /**
     * @brief Move arm to predefined home position
     */
    void move_arm_home_pos();

    /**
     * @brief Check if gripper has reached its target position
     * @param tolerance Position tolerance (default: 0.05)
     * @return true if within tolerance
     */
    bool gripper_reached_target(double tolerance = 0.05) const;

    /**
     * @brief Get current ROS time
     */
    rclcpp::Time now();

    /**
     * @brief Get kinematics cache
     */
    KinematicsCache& get_cache() { return cache_; }
    
    /**
     * @brief Get current joint angles
     */
    const Vector7d& get_current_joint_angles() const { return current_joint_angles_; }
    
    /**
     * @brief Get current gripper position
     */
    double get_gripper_position() const { return gripper_current_pos_; }

private:

    // Command Queue State Machine

    enum class State {
        IDLE,       // No command executing
        EXECUTING,  // Command in progress
        WAITING     // Waiting for external event
    };

    void timer_callback();
    void start_next_command();
    void execute_current_command();
    void complete_current_command(bool success);

    // ROS Publishers and Callbacks

    void pose_callback(const unity_robotics_demo_msgs::msg::PosRot::SharedPtr msg);
    void joint_state_callback_pos(const panda_kinematics::msg::JointCommand::SharedPtr msg);
    void joint_state_callback_vel(const panda_kinematics::msg::JointCommand::SharedPtr msg);

    void publish_joint_command_pos(const Vector7d& joint_angles, float gripper_pos = 0.0);
    void publish_joint_command_vel(const Vector7d& joint_vels, float gripper_pos = 0.0);
    void publish_joint_positions(const Eigen::Matrix<double, 8, 3> jointPositions);

    // Demo Sequence Builder

    void load_demo_sequence();

    // Member Variables

    // ROS communication
  rclcpp::Publisher<panda_kinematics::msg::JointCommand>::SharedPtr joint_cmd_publisher_;
  rclcpp::Publisher<panda_kinematics::msg::AllJointPos>::SharedPtr joint_pos_publisher_;
  rclcpp::Subscription<unity_robotics_demo_msgs::msg::PosRot>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<panda_kinematics::msg::JointCommand>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<panda_kinematics::msg::JointCommand>::SharedPtr joint_vel_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Robot state
  Vector7d current_joint_angles_;
  Vector7d current_joint_velocities_;
  Vector7d target_joint_angles_;
  float gripper_current_pos_;
  float gripper_target_pos_;
  Eigen::Matrix4d T_target_;
  KinematicsCache cache_;
  double last_time_;
  
  // Command queue state
  State state_;
  std::queue<std::unique_ptr<panda_kinematics::RobotCommand>> command_queue_;
  std::unique_ptr<panda_kinematics::RobotCommand> current_command_;
  rclcpp::Time command_start_time_;
  
  size_t count_;
}; // class

} // namespace panda_arm_controller

#endif