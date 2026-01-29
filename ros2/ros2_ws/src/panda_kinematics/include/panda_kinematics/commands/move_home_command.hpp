#ifndef PANDA_KINEMATICS_MOVE_HOME_COMMAND_HPP
#define PANDA_KINEMATICS_MOVE_HOME_COMMAND_HPP

#include "panda_kinematics/commands/robot_command.hpp"

namespace panda_kinematics {

// Forward declaration
class ArmController;

/**
 * @brief Command to move the arm to its home/default position
 */
class MoveHomeCommand : public RobotCommand {
public:
    explicit MoveHomeCommand(ArmController* controller, double timeout = 15.0);
    
    bool execute() override;
    void on_start() override;
    double get_timeout() const override { return timeout_; }
    std::string get_name() const override { return "MoveHome"; }

private:
    ArmController* controller_;
    double timeout_;
    bool started_;
};

} // namespace panda_kinematics

#endif // PANDA_KINEMATICS_MOVE_HOME_COMMAND_HPP