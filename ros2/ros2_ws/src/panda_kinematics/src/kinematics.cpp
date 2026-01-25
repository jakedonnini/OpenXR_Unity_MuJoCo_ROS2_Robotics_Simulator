#define _USE_MATH_DEFINES

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "kinematics.h"

// Helper function: homogeneous transform using DH parameters
Eigen::Matrix4d Ai(double a_i, double alpha_i, double d_i, double theta_i) {
    Eigen::Matrix4d T;
    T << cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i),
         sin(theta_i), cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i),
         0,            sin(alpha_i),               cos(alpha_i),                d_i,
         0,            0,                           0,                          1;
    return T;
}


// Forward kinematics function
std::vector<Eigen::Matrix4d> forwardKinematics(
    const Vector7d& q,
    Eigen::Matrix<double, 8, 3>& jointPositions,
    Eigen::Matrix4d& T0e
) {
    // Link lengths
    double l1 = 0.141, l2 = 0.192, l3 = 0.195, l4 = 0.121, l5 = 0.0825;
    double l6 = 0.0825, l7 = 0.125, l8 = 0.259, l9 = 0.088, l10 = 0.051;
    double l11 = 0.159, l12 = 0.015;

    // Compute transforms
    Eigen::Matrix4d T01 = Ai(0, M_PI/2, l1+l2, q[0]);
    Eigen::Matrix4d T12 = Ai(0, -M_PI/2, 0, -q[1]);
    Eigen::Matrix4d T23 = Ai(l6, M_PI/2, l3+l4, q[2]);
    Eigen::Matrix4d T34 = Ai(-l5, -M_PI/2, 0, q[3]);
    Eigen::Matrix4d T45 = Ai(0, M_PI/2, l7+l8, q[4]);
    Eigen::Matrix4d T56 = Ai(l9, M_PI/2, 0, q[5]);
    Eigen::Matrix4d T6e = Ai(0, 0, l10+l11, q[6]-M_PI/4);

    Eigen::Matrix4d T02 = T01*T12;
    Eigen::Matrix4d T03 = T02*T23;
    Eigen::Matrix4d T04 = T03*T34;
    Eigen::Matrix4d T05 = T04*T45;
    Eigen::Matrix4d T06 = T05*T56;
    T0e = T06*T6e;

    Eigen::Vector4d base(0,0,0,1);

    // Optional offsets
    Eigen::Matrix4d T0_offset = Ai(0,0,l1,0);
    Eigen::Matrix4d T02_offset = Ai(0,0,l3,0);
    Eigen::Matrix4d T04_offset = Ai(0,0,l7,0);
    Eigen::Matrix4d T05_offset = Ai(0,0,-l12,0);
    Eigen::Matrix4d T06_offset = Ai(0,0,l10,0);

    // Build transform list
    std::vector<Eigen::Matrix4d> T_list = {
        T0_offset,
        T01,
        T02*T02_offset,
        T03,
        T04*T04_offset,
        T05*T05_offset,
        T06*T06_offset,
        T0e
    };

    jointPositions.row(0) = (T0_offset*base).head<3>();
    jointPositions.row(1) = (T01*base).head<3>();
    jointPositions.row(2) = (T02*T02_offset*base).head<3>();
    jointPositions.row(3) = (T03*base).head<3>();
    jointPositions.row(4) = (T04*T04_offset*base).head<3>();
    jointPositions.row(5) = (T05*T05_offset*base).head<3>();
    jointPositions.row(6) = (T06*T06_offset*base).head<3>();
    jointPositions.row(7) = (T0e*base).head<3>();

    return T_list;
}

// create the jacobian function
void computeJacobian(const std::vector<Eigen::Matrix4d>& T_list,
             Eigen::Matrix<double, 6, 7>& J) {
    Eigen::Vector3d On = T_list.back().block<3,1>(0,3); // end-effector position

    for (int i = 0; i < 7; ++i) {
        Eigen::Vector3d Oi = T_list[i].block<3,1>(0,3); // joint origin
        Eigen::Vector3d z_vector = T_list[i].block<3,1>(0,2); // joint axis (revolute about z in DH frame)

        Eigen::Vector3d Ji_pos = z_vector.cross(On - Oi);
        Eigen::Vector3d Ji_ori = z_vector;

        // Joint 2 (index 1) was modeled with a negative angle (-q[1]) in FK (T12 uses -q[1]).
        // That flips the derivative sign relative to using +q[1]. Apply correction so the
        // analytic Jacobian matches the numeric derivative and MuJoCo convention.
        if (i == 1) {
            Ji_pos = -Ji_pos;
            Ji_ori = -Ji_ori;
        }

        J.block<3,1>(0,i) = Ji_pos;
        J.block<3,1>(3,i) = Ji_ori;
    }
}

// difference to target function (Euclidean distance only)
Eigen::Vector3d diff_to_target(const Eigen::Matrix4d& T_current, const Eigen::Matrix4d& T_target) {
    Eigen::Vector3d p_current = T_current.block<3,1>(0,3);
    Eigen::Vector3d p_target = T_target.block<3,1>(0,3);
    Eigen::Vector3d linear_v = p_target - p_current;
    return linear_v;
}

// Helper function for End Effector Orientation Task
Eigen::Vector3d calcAngDiff(const Eigen::Matrix3d& R_des, const Eigen::Matrix3d& R_curr) {
    // Compute the rotation matrix from current to desired orientation
    Eigen::Matrix3d R_diff = R_curr * R_des.transpose();
    
    // Extract the axis of rotation from the skew-symmetric part of R_diff
    // The skew-symmetric part is (R_diff - R_diff.T) / 2
    Eigen::Matrix3d skew_sym = (R_diff - R_diff.transpose()) / 2.0;
    
    // Extract the components of the omega vector (axis of rotation)
    // From the skew-symmetric matrix: [0 -z y; z 0 -x; -y x 0]
    // The axis vector is [x, y, z] = [skew(2,1), skew(0,2), skew(1,0)]
    Eigen::Vector3d omega;
    omega[0] = -skew_sym(2, 1);  // -(-z) = z component  
    omega[1] = -skew_sym(0, 2);  // -(y) = -y component
    omega[2] = -skew_sym(1, 0);  // -(-x) = x component
    
    return omega;
}

// IK velocity solver implementing the Python behavior
Vector7d IK_velocity(const Vector7d& q_in,
                                     const Eigen::Vector3d& v_in,
                                     const Eigen::Vector3d& omega_in) 
                                     {
    // Build Jacobian at q_in using existing forwardKinematics/computeJacobian
    Vector7d q = q_in; // mutable copy for forwardKinematics
    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    T_list = forwardKinematics(q, jointPositions, T0e);

    Eigen::Matrix<double, 6, 7> J;
    computeJacobian(T_list, J);

    std::cout << "J:\n" << J << std::endl;

    // Build target 6x1 velocity [v; omega]
    Eigen::Matrix<double, 6, 1> target;
    target.head<3>() = v_in;
    target.tail<3>() = omega_in;

    // Filter rows with NaN in target
    std::vector<int> validRows;
    validRows.reserve(6);
    for (int i = 0; i < 6; ++i) {
        if (!std::isnan(target[i])) {
            validRows.push_back(i);
        }
    }

    // If no constraints, return zeros
    Vector7d dq = Vector7d::Zero();
    if (validRows.empty()) {
        return dq;
    }

    // Build filtered J and target
    const int r = static_cast<int>(validRows.size());
    Eigen::MatrixXd Jf(r, 7);
    Eigen::VectorXd tf(r);
    for (int i = 0; i < r; ++i) {
        Jf.row(i) = J.row(validRows[i]);
        tf[i] = target[validRows[i]];
    }

    std::cout << "Jf:\n" << Jf << std::endl;

    // Solve least-squares Jf * dq = tf with minimal-norm solution
    // Use SVD for robustness; Eigen's JacobiSVD with thin U/V is sufficient
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jf, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& S = svd.singularValues();
    Eigen::VectorXd Sinv = S;
    double tol = 1e-8;
    for (int i = 0; i < S.size(); ++i) {
        Sinv[i] = (S[i] > tol) ? 1.0 / S[i] : 0.0;
    }
    dq = svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose() * tf;

    return dq;
}

// Compute distance (m) and angle (rad) between two homogeneous transforms
void distance_and_angle(const Eigen::Matrix4d& G,
                        const Eigen::Matrix4d& H,
                        double& distance,
                        double& angle) {
    // Distance between origins
    Eigen::Vector3d pG = G.block<3,1>(0,3);
    Eigen::Vector3d pH = H.block<3,1>(0,3);
    distance = (pG - pH).norm();

    // Angle between orientations using trace formula
    Eigen::Matrix3d RG = G.block<3,3>(0,0);
    Eigen::Matrix3d RH = H.block<3,3>(0,0);
    double cos_angle = ( (RG.transpose() * RH).trace() - 1.0 ) * 0.5;
    // Clamp for numerical safety
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;
    angle = std::acos(cos_angle);
}

bool is_valid_solution(const Vector7d& q, const Eigen::Matrix4d& T_target, double position_tolerance, double angle_tolerance) {
    // Joint limits for Franka Emika Panda
    const double lower_limits[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const double upper_limits[7] = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

    for (int i = 0; i < 7; ++i) {
        if (q[i] < lower_limits[i] || q[i] > upper_limits[i]) {
            return false; // Out of bounds
        }
    }


    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    Vector7d q_copy = q; // forwardKinematics requires non-const reference
    T_list = forwardKinematics(q_copy, jointPositions, T0e);

    double distance, angle;
    distance_and_angle(T0e, T_target, distance, angle);

    if (distance > position_tolerance || angle > angle_tolerance) {
        return false; // Not close enough to target
    }

    return true; // All joints within limits
}

// Robust damped pseudoinverse that works for tall or wide Jacobians
Eigen::MatrixXd dampedPseudoinverse(const Eigen::MatrixXd& J, double lambda) {
    int m = (int)J.rows();
    int n = (int)J.cols();
    if (m <= n) {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);
        Eigen::MatrixXd JJt = J * J.transpose();
        return J.transpose() * (JJt + (lambda * lambda) * I).inverse();
    } else {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        Eigen::MatrixXd JtJ = J.transpose() * J;
        return (JtJ + (lambda * lambda) * I).inverse() * J.transpose();
    }
}

Vector7d end_effector_task(const Vector7d& q, const Eigen::Matrix4d& T_target) {
    // use the J psudoinverse to compute dq

    // Use FK to get current end-effector pose and Jacobian
    Vector7d q_copy = q; // forwardKinematics requires non-const reference
    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    T_list = forwardKinematics(q_copy, jointPositions, T0e);

    // Calculate displacement and axis of rotation
    Eigen::Vector3d displacement = diff_to_target(T0e, T_target);
    Eigen::Vector3d axis = calcAngDiff(T_target.block<3,3>(0,0), T0e.block<3,3>(0,0));
    std::cout << "displacement: " << displacement.transpose() << ", axis: " << axis.transpose() << std::endl;

    // compute Jacobian and psuedoinverse
    Eigen::Matrix<double, 6, 7> J;
    computeJacobian(T_list, J);
    Eigen::Matrix<double, 7, 6> J_pseudo = dampedPseudoinverse(J);


    // Compute joint velocities
    Vector7d dq = J_pseudo * (Eigen::Matrix<double, 6, 1>() << displacement, axis).finished();
    return dq;
}

Vector7d joint_centering_task(const Vector7d& q, double rate) {
    // Joint limits for Franka Emika Panda
    const double lower_limits[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const double upper_limits[7] = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

    // create a secondary task in the nullspace of the primary task
    Vector7d offset;
    for (int i = 0; i < 7; ++i) {
        double center = lower_limits[i] + (upper_limits[i] - lower_limits[i]) / 2.0;
        offset[i] = (q[i] - center) / (upper_limits[i] - lower_limits[i]);
    }

    Vector7d dq = -offset * rate; // move towards center
    return dq;
}

// can be optimized further by calculating relevent matrices only once (e.g. FK, Jacobian)
Vector7d inverse_kinematics_step(
    const Vector7d& q_current,
    const Eigen::Matrix4d& T_target,
    double alpha,
    double joint_centering_rate
) {
    // Primary task: end-effector position and orientation
    Vector7d dq_primary = end_effector_task(q_current, T_target);

    // Secondary task: joint centering
    Vector7d dq_secondary = joint_centering_task(q_current, joint_centering_rate);

    // Combine tasks using nullspace projection
    Vector7d q_copy = q_current; // forwardKinematics requires non-const reference
    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    T_list = forwardKinematics(q_copy, jointPositions, T0e);

    std::cout << "T0e:\n" << T0e << std::endl;

    Eigen::Matrix<double, 6, 7> J;
    computeJacobian(T_list, J);
    Eigen::Matrix<double, 7, 6> J_pseudo = dampedPseudoinverse(J, 0.05);

    // std::cout << "J:\n" << J << std::endl;
    // for (int i = 0; i < T_list.size(); ++i)
        // std::cout << "T_list " << i << ": " << T_list[i] << std::endl;
    // std::cout << "J_pseudo:\n" << J_pseudo << std::endl;

    Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 7, 7> N = I - J_pseudo * J; // Nullspace projector

    Vector7d dq = dq_primary + N * dq_secondary;

    // Scale by step size alpha
    dq *= alpha;

    return dq;
}

// --------------------- KinematicsCache Implementation --------------------
KinematicsCache::KinematicsCache() {
    q_.setZero();
    jointPositions_.setZero();
    T_list_.clear();
    J_.setZero();
    dirty_fk_ = true;
    dirty_jac_ = true;
}

void KinematicsCache::setConfiguration(const Vector7d& q) {
    q_ = q;
    invalidate();
}

void KinematicsCache::ensureFK() {
    if (!dirty_fk_) return;
    Vector7d q_copy = q_;
    T_list_ = forwardKinematics(q_copy, jointPositions_, T0e_);
    dirty_fk_ = false;
    dirty_jac_ = true; // FK update invalidates Jacobian
}

void KinematicsCache::ensureJacobian() {
    ensureFK();
    if (!dirty_jac_) return;
    computeJacobian(T_list_, J_);
    dirty_jac_ = false;
}

Vector7d inverse_kinematics_step_optimized_position(
    KinematicsCache& cache,
    const Eigen::Matrix4d& T_target,
    double alpha,
    double joint_centering_rate
) {
    cache.ensureJacobian();
    const auto& T0e = cache.T0e();
    const auto& J = cache.J();
    const auto& q_current = cache.q();

    // Primary task twist (position + orientation error)
    // replecates end_effector_task but uses cached FK/Jacobian
    Eigen::Vector3d displacement = diff_to_target(T0e, T_target);
    Eigen::Vector3d axis = calcAngDiff(T_target.block<3,3>(0,0), T0e.block<3,3>(0,0));
    Eigen::Matrix<double,6,1> twist;
    twist.head<3>() = displacement;
    twist.tail<3>() = axis;

    Eigen::Matrix<double,7,6> J_pseudo = dampedPseudoinverse(J, 0.05);
    Vector7d dq_primary = J_pseudo * twist;

    Vector7d dq_secondary = joint_centering_task(q_current, joint_centering_rate);
    Eigen::Matrix<double,7,7> N = Eigen::Matrix<double,7,7>::Identity() - J_pseudo * J;
    Vector7d dq = dq_primary + N * dq_secondary;
    dq *= alpha;
    return dq;
}

Vector7d inverse_kinematics_velocity(
    KinematicsCache& cache,
    const Eigen::Matrix4d& T_target,
    double kp_pos,
    double kp_rot,
    double joint_centering_rate
) {
    cache.ensureJacobian();
    const auto& T0e = cache.T0e();
    const auto& J = cache.J();
    const auto& q = cache.q();

    Eigen::Vector3d dp = diff_to_target(T0e, T_target);
    Eigen::Vector3d dtheta = calcAngDiff(
        T_target.block<3,3>(0,0),
        T0e.block<3,3>(0,0)
    );

    // proportional velocity controller
    Eigen::Matrix<double,6,1> xdot;
    xdot.head<3>() = kp_pos * dp;
    xdot.tail<3>() = kp_rot * dtheta;

    Eigen::Matrix<double,7,6> J_pinv = dampedPseudoinverse(J, 0.05);

    Vector7d dq = J_pinv * xdot;

    Vector7d dq_null = joint_centering_task(q, joint_centering_rate);
    Eigen::Matrix<double,7,7> N = Eigen::Matrix<double,7,7>::Identity() - J_pinv * J;

    dq += N * dq_null;

    return dq;   // <-- joint velocities (rad/s)
}


// (Removed duplicate KinematicsCache implementation block)