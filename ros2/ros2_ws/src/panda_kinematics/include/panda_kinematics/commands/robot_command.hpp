#ifndef PANDA_KINEMATICS_ROBOT_COMMAND_HPP
#define PANDA_KINEMATICS_ROBOT_COMMAND_HPP

#include <string>
#include <memory>

namespace panda_kinematics {

/**
 * @brief Base class for all robot commands
 * 
 * Commands are executed sequentially in a queue. Each command's execute()
 * method is called repeatedly until it returns true (indicating completion).
 */
class RobotCommand {
public:
    virtual ~RobotCommand() = default;
    
    /**
     * @brief Execute one step of the command
     * @return true when command is complete, false otherwise
     * 
     * This method is called repeatedly (e.g., every 100ms) until it returns true.
     * Should be non-blocking and perform incremental work.
     */
    virtual bool execute() = 0;
    
    /**
     * @brief Called once when command starts execution
     * 
     * Use this to initialize state, start timers, or log start of execution.
     */
    virtual void on_start() {}
    
    /**
     * @brief Called once when command completes (success or failure)
     * @param success true if command completed successfully, false if timed out
     * 
     * Use this for cleanup, logging, or triggering follow-up actions.
     */
    virtual void on_complete([[maybe_unused]] bool success) {}
    
    /**
     * @brief Get maximum execution time for this command
     * @return timeout in seconds, or 0.0 for no timeout
     */
    virtual double get_timeout() const { return 0.0; }
    
    /**
     * @brief Get human-readable name for logging/debugging
     * @return command name
     */
    virtual std::string get_name() const = 0;
};

} // namespace panda_kinematics

#endif // PANDA_KINEMATICS_ROBOT_COMMAND_HPP