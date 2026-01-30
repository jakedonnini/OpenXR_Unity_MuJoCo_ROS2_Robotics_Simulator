#ifndef PANDA_KINEMATICS_MOVE_TO_POSE_COMMAND_HPP
#define PANDA_KINEMATICS_MOVE_TO_POSE_COMMAND_HPP

#include "panda_kinematics/commands/robot_command.hpp"
#include <Eigen/Dense>

namespace panda_kinematics {

// Forward declaration
class ArmController;

/**
 * @brief Command to move the arm to a target pose
 * 
 * Moves the end-effector to a target position and orientation.
 * Uses velocity-based inverse kinematics to incrementally approach the target.
 */
class MoveToPoseCommand : public RobotCommand {
public:
    /**
     * @brief Construct a new Move To Pose Command
     * 
     * @param controller Pointer to arm controller
     * @param target_pose Target 4x4 transformation matrix
     * @param pos_tolerance Position tolerance in meters (default: 0.01m = 1cm)
     * @param angle_tolerance Angle tolerance in radians (default: 0.05 rad ≈ 3°)
     * @param timeout Maximum execution time in seconds (default: 30s)
     */
    MoveToPoseCommand(
        ArmController* controller,
        const Eigen::Matrix4d& target_pose,
        double pos_tolerance = 0.05,
        double angle_tolerance = 0.05,
        double timeout = 30.0
    );
    
    bool execute() override;
    void on_start() override;
    double get_timeout() const override { return timeout_; }
    std::string get_name() const override { return "MoveToPose"; }

private:
    ArmController* controller_;
    Eigen::Matrix4d target_pose_;
    double pos_tolerance_;
    double angle_tolerance_;
    double timeout_;
};

} // namespace panda_kinematics

#endif // PANDA_KINEMATICS_MOVE_TO_POSE_COMMAND_HPP