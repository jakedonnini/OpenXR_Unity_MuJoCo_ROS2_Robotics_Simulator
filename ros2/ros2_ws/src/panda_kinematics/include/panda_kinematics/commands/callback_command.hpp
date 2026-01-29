#ifndef PANDA_KINEMATICS_CALLBACK_COMMAND_HPP
#define PANDA_KINEMATICS_CALLBACK_COMMAND_HPP

#include "panda_kinematics/commands/robot_command.hpp"
#include <functional>

namespace panda_kinematics {

/**
 * @brief Command that executes a custom callback function
 * 
 * Useful for executing arbitrary code within the command queue,
 * such as sensor checks, notifications, or custom logic.
 */
class CallbackCommand : public RobotCommand {
public:
    using CallbackFn = std::function<bool()>; // Returns true when done
    
    /**
     * @brief Construct a new Callback Command
     * 
     * @param name Human-readable name for logging
     * @param callback Function to execute (should return true when complete)
     * @param timeout Maximum execution time in seconds
     */
    CallbackCommand(
        const std::string& name,
        CallbackFn callback,
        double timeout = 10.0
    );
    
    bool execute() override { return callback_(); }
    double get_timeout() const override { return timeout_; }
    std::string get_name() const override { return name_; }

private:
    std::string name_;
    CallbackFn callback_;
    double timeout_;
};

} // namespace panda_kinematics

#endif // PANDA_KINEMATICS_CALLBACK_COMMAND_HPP