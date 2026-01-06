#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense> 
#include <vector>

// Compute the Denavit-Hartenberg homogeneous transformation
Eigen::Matrix4d Ai(double a_i, double alpha_i, double d_i, double theta_i);

// Forward kinematics for 7-DOF Panda arm
// INPUT:
//   q - vector of 7 joint angles [q0, q1, ..., q6]
// OUTPUT:
//   jointPositions - 8x3 matrix: positions of each joint + end effector in world frame
//   T0e - 4x4 homogeneous transform of the end effector
// RETURNS:
//   T_list - vector of 4x4 transforms from base to each joint + end effector
std::vector<Eigen::Matrix4d> forwardKinematics(
    const Eigen::Vector<double, 7>& q,
    Eigen::Matrix<double, 8, 3>& jointPositions,
    Eigen::Matrix4d& T0e);

// Compute the geometric Jacobian for the 7-DOF Panda arm
// INPUT:  
//   q - vector of 7 joint angles [q0, q1, ..., q6]
//   T_list - vector of 4x4 transforms from base to each joint + end effector
// OUTPUT:
//   J - 6x7 Jacobian matrix
void computeJacobian(const std::vector<Eigen::Matrix4d>& T_list,
                     Eigen::Matrix<double, 6, 7>& J);

// Compute the Euclidean position difference from current to target end-effector pose
// INPUT:
//   T_current - current end-effector pose (4x4 homogeneous transform) 
//   T_target - target end-effector pose (4x4 homogeneous transform)
// OUTPUT:
//   Returns a 3D vector dp = p_target - p_current
Eigen::Vector3d diff_to_target(const Eigen::Matrix4d& T_current,
                               const Eigen::Matrix4d& T_target);

// Helper function for End Effector Orientation Task
// Computes the axis of rotation from the current orientation to the target orientation
// INPUT:
//   R_des - 3x3 matrix representing the desired orientation from end effector to world
//   R_curr - 3x3 matrix representing the current end effector orientation  
// OUTPUT:
//   omega - 3-element vector containing the axis of rotation from current to desired frame
//           The magnitude of this vector is sin(angle), where angle is the rotation angle
Eigen::Vector3d calcAngDiff(const Eigen::Matrix3d& R_des, const Eigen::Matrix3d& R_curr);

// Inverse Kinematics velocity solver (least-squares with minimal-norm solution)
// INPUTS:
//   q_in      - 7x1 vector of current joint configuration
//   v_in      - 3x1 desired linear velocity in world frame; NaN components are unconstrained
//   omega_in  - 3x1 desired angular velocity in world frame; NaN components are unconstrained
// OUTPUT:
//   Returns dq (7x1) joint velocities minimizing least squares error; if underdetermined,
//   returns the minimal L2-norm solution. If all targets are unconstrained, returns zeros.
Eigen::Vector<double, 7> IK_velocity(const Eigen::Vector<double, 7>& q_in,
                                     const Eigen::Vector3d& v_in,
                                     const Eigen::Vector3d& omega_in);

// Inverse Kinematics velocity solver given a 6x7 Jacobian directly
Eigen::Vector<double, 7> IK_velocity_fromJ(const Eigen::Matrix<double, 6, 7>& J,
                                           const Eigen::Vector3d& v_in,
                                           const Eigen::Vector3d& omega_in);

// Compute distance (m) and angle (rad) between two homogeneous transforms
void distance_and_angle(const Eigen::Matrix4d& G,
                        const Eigen::Matrix4d& H,
                        double& distance,
                        double& angle);

/*Given a candidate solution, determine if it achieves the primary task
and also respects the joint limits.

INPUTS
q - the candidate solution, namely the joint angles
target - 4x4 numpy array representing the desired transformation from
end effector to world

OUTPUTS:
success - a Boolean which is True if and only if the candidate solution
produces an end effector pose which is within the given linear and
angular tolerances of the target pose, and also respects the joint
limits.
*/
bool is_valid_solution(const Eigen::Vector<double, 7>& q,
                       const Eigen::Matrix4d& T_target, 
                       double position_tolerance = 0.05, 
                       double angle_tolerance = 0.1);

// Robust damped pseudoinverse that works for tall or wide Jacobians
Eigen::MatrixXd dampedPseudoinverse(const Eigen::MatrixXd& J, double lambda = 0.01);

/*
 Primary task for IK solver. Computes a joint velocity which will reduce
the error between the target end effector pose and the current end
effector pose (corresponding to configuration q).

INPUTS:
q - the current joint configuration, a "best guess" so far for the final answer
target - a 4x4 numpy array containing the desired end effector pose

OUTPUTS:
dq - a desired joint velocity to perform this task, which will smoothly
decay to zero magnitude as the task is achieved
*/
Eigen::Vector<double, 7> end_effector_task(const Eigen::Vector<double, 7>& q, const Eigen::Matrix4d& T_target);

/*
Secondary task for IK solver. Computes a joint velocity which will
reduce the offset between each joint's angle and the center of its range
of motion. This secondary task acts as a "soft constraint" which
encourages the solver to choose solutions within the allowed range of
motion for the joints.

INPUTS:
q - the joint angles
rate - a tunable parameter dictating how quickly to try to center the
joints. Turning this parameter improves convergence behavior for the
primary task, but also requires more solver iterations.

OUTPUTS:
dq - a desired joint velocity to perform this task, which will smoothly
decay to zero magnitude as the task is achieved
*/
Eigen::Vector<double, 7> joint_centering_task(const Eigen::Vector<double, 7>& q, double rate = 0.1);

/*
computes a single step of an inverse kinematics solver which attempts to
move the end effector to the target pose for gradient descent.
*/
Eigen::Vector<double, 7> inverse_kinematics_step(
    const Eigen::Vector<double, 7>& q_current,
    const Eigen::Matrix4d& T_target,
    double alpha = 0.1,
    double joint_centering_rate = 0.1
);

// -------------------------------------------------------------
// KinematicsCache: caches FK (T_list, joint positions, T0e) and Jacobian
// to avoid redundant recomputation inside iterative IK loops.
// -------------------------------------------------------------
class KinematicsCache {
public:
    KinematicsCache();

    void setConfiguration(const Eigen::Vector<double,7>& q);
    const Eigen::Vector<double,7>& q() const { return q_; }

    // Ensure FK computed
    void ensureFK();
    // Ensure Jacobian computed
    void ensureJacobian();

    const std::vector<Eigen::Matrix4d>& T_list() { ensureFK(); return T_list_; }
    const Eigen::Matrix<double,8,3>& jointPositions() { ensureFK(); return jointPositions_; }
    const Eigen::Matrix4d& T0e() { ensureFK(); return T0e_; }
    const Eigen::Matrix<double,6,7>& J() { ensureJacobian(); return J_; }

    void invalidate() { dirty_fk_ = true; dirty_jac_ = true; }

private:
    Eigen::Vector<double,7> q_;
    Eigen::Matrix<double,8,3> jointPositions_;
    Eigen::Matrix4d T0e_ = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Matrix4d> T_list_;
    Eigen::Matrix<double,6,7> J_;
    bool dirty_fk_ = true;
    bool dirty_jac_ = true;
};

// Optimized IK step using cached FK/Jacobian (single computation per call)
Eigen::Vector<double,7> inverse_kinematics_step_optimized(
    KinematicsCache& cache,
    const Eigen::Matrix4d& T_target,
    double alpha = 0.1,
    double joint_centering_rate = 0.1
);
#endif // KINEMATICS_H
