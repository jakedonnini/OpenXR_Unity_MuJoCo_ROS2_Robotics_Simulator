#include "panda_kinematics/commands/move_to_pose_command.hpp"
#include "panda_kinematics/panda_arm_controller.hpp"

namespace panda_kinematics {

// In move_to_pose_command.cpp
MoveToPoseCommand::MoveToPoseCommand(
    ArmController* controller,
    const Eigen::Matrix4d& target_pose,
    double pos_tolerance,
    double angle_tolerance,
    double timeout
) : controller_(controller),
    target_pose_ptr_(nullptr),  // Set first
    target_pose_copy_(target_pose),  // Then copy
    use_reference_(false),
    pos_tolerance_(pos_tolerance),
    angle_tolerance_(angle_tolerance),
    timeout_(timeout)
{
    target_pose_ptr_ = &target_pose_copy_;  // Point to our copy
}

MoveToPoseCommand::MoveToPoseCommand(
    ArmController* controller,
    const Eigen::Matrix4d& target_pose,
    ByReference,
    double pos_tolerance,
    double angle_tolerance,
    double timeout
) : controller_(controller),
    target_pose_ptr_(&target_pose),  // Point to external reference
    target_pose_copy_(),  // Default initialize (won't be used)
    use_reference_(true),
    pos_tolerance_(pos_tolerance),
    angle_tolerance_(angle_tolerance),
    timeout_(timeout)
{
}

void MoveToPoseCommand::on_start() {
    // Optional: log target position
    // RCLCPP_INFO(controller_->get_logger(), "Starting move to pose...");
}

bool MoveToPoseCommand::execute() {
    auto& cache = controller_->get_cache();
    return controller_->move_arm_step_vel(
        cache, 
        *target_pose_ptr_,
        10.0,  // kp_pos
        5.0,  // kp_rot
        0.2,  // joint_centering_rate
        pos_tolerance_, 
        angle_tolerance_
    );
    // return controller_->move_arm_step_pos(
    //     cache,
    //     target_pose_,
    //     0.8,  // step_size
    //     0.2,  // joint_centering_rate
    //     pos_tolerance_,
    //     angle_tolerance_
    // );
}

} // namespace panda_kinematics
