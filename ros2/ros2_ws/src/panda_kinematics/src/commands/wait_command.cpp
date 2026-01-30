#include "panda_kinematics/commands/wait_command.hpp"
#include "panda_kinematics/panda_arm_controller.hpp"

namespace panda_kinematics {

WaitCommand::WaitCommand(ArmController* controller, double wait_seconds)
    : controller_(controller), wait_seconds_(wait_seconds)
{
}

void WaitCommand::on_start() {
    start_time_ = controller_->now();
}

bool WaitCommand::execute() {
    auto elapsed = (controller_->now() - start_time_).seconds();
    return elapsed >= wait_seconds_;
}

} // namespace panda_kinematics
