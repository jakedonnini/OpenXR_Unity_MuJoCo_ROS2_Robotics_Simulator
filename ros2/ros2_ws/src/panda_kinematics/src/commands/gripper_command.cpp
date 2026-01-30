#include "panda_kinematics/commands/gripper_command.hpp"
#include "panda_kinematics/panda_arm_controller.hpp"

namespace panda_kinematics {

// ============================================================================
// OpenGripperCommand
// ============================================================================

OpenGripperCommand::OpenGripperCommand(ArmController* controller, double timeout)
    : controller_(controller), timeout_(timeout)
{
}

void OpenGripperCommand::on_start() {
    controller_->open_gripper();
}

bool OpenGripperCommand::execute() {
    controller_->open_gripper();
    return controller_->gripper_reached_target();
}

// ============================================================================
// CloseGripperCommand
// ============================================================================

CloseGripperCommand::CloseGripperCommand(ArmController* controller, double timeout)
    : controller_(controller), timeout_(timeout)
{
}

void CloseGripperCommand::on_start() {
    controller_->close_gripper();
}

bool CloseGripperCommand::execute() {
    controller_->close_gripper();
    return controller_->gripper_reached_target();
}

} // namespace panda_kinematics
