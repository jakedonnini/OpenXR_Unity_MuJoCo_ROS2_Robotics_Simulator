#include "panda_kinematics/commands/move_home_command.hpp"
#include "panda_kinematics/panda_arm_controller.hpp"

namespace panda_kinematics {

MoveHomeCommand::MoveHomeCommand(ArmController* controller, double timeout)
    : controller_(controller), timeout_(timeout), started_(false)
{
}

void MoveHomeCommand::on_start() {
    controller_->move_arm_home_pos();
    started_ = true;
}

bool MoveHomeCommand::execute() {
    if (!started_) {
        controller_->move_arm_home_pos();
        started_ = true;
    }
    
    return controller_->reached_arm_home_position();
}

} // namespace panda_kinematics
