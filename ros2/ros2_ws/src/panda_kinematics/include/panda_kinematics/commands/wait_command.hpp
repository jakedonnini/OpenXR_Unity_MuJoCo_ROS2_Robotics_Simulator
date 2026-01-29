#ifndef PANDA_KINEMATICS_WAIT_COMMAND_HPP
#define PANDA_KINEMATICS_WAIT_COMMAND_HPP

#include "panda_kinematics/commands/robot_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace panda_kinematics {

// Forward declaration
class ArmController;

/**
 * @brief Command to wait/delay for a specified duration
 * 
 * Useful for adding pauses between actions (e.g., wait for object to settle).
 */
class WaitCommand : public RobotCommand {
public:
    /**
     * @brief Construct a new Wait Command
     * 
     * @param controller Pointer to arm controller
     * @param wait_seconds Duration to wait in seconds
     */
    WaitCommand(ArmController* controller, double wait_seconds);
    
    bool execute() override;
    void on_start() override;
    double get_timeout() const override { return wait_seconds_ + 1.0; }
    std::string get_name() const override { return "Wait"; }

private:
    ArmController* controller_;
    double wait_seconds_;
    rclcpp::Time start_time_;
};

} // namespace panda_kinematics

#endif // PANDA_KINEMATICS_WAIT_COMMAND_HPP
