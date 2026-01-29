#ifndef PANDA_KINEMATICS_GRIPPER_COMMAND_HPP
#define PANDA_KINEMATICS_GRIPPER_COMMAND_HPP

#include "panda_kinematics/commands/robot_command.hpp"

namespace panda_kinematics {

// Forward declaration
class ArmController;

/**
 * @brief Command to open the gripper
 */
class OpenGripperCommand : public RobotCommand {
public:
    explicit OpenGripperCommand(ArmController* controller, double timeout = 5.0);
    
    bool execute() override;
    void on_start() override;
    double get_timeout() const override { return timeout_; }
    std::string get_name() const override { return "OpenGripper"; }

private:
    ArmController* controller_;
    double timeout_;
};

/**
 * @brief Command to close the gripper
 */
class CloseGripperCommand : public RobotCommand {
public:
    explicit CloseGripperCommand(ArmController* controller, double timeout = 5.0);
    
    bool execute() override;
    void on_start() override;
    double get_timeout() const override { return timeout_; }
    std::string get_name() const override { return "CloseGripper"; }

private:
    ArmController* controller_;
    double timeout_;
};

} // namespace panda_kinematics

#endif // PANDA_KINEMATICS_GRIPPER_COMMAND_HPP