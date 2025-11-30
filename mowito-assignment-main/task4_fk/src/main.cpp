#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

// ---------------------------------------------------------
// Task 4: Forward Kinematics for 4-Link Perpendicular Robot
// Author: Nitheeshkumar
// Date:   November 2025
//
// Robot:
//  - 4 revolute joints
//  - Link length L = 1.0 m
//  - DH (modified to match Task 2 style):
//
//  Joint |  a_i |  alpha_i |   d_i |  theta_i (variable)
//  ------|------|----------|-------|---------------------
//    1   |  L   |  +pi/2   |   0   |    q1
//    2   |  L   |  +pi/2   |   0   |    q2
//    3   |  L   |  +pi/2   |   0   |    q3
//    4   |  L   |    0     |   0   |    q4
//
// This is consistent with the Python FK Task 2.
// ---------------------------------------------------------

struct DHParam
{
    double a;       // link length
    double alpha;   // twist
    double d;       // link offset
    double theta0;  // fixed offset (if any), here 0 for all
};

// Standard DH homogeneous transform
Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha)
{
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = ct;     T(0, 1) = -st * ca;  T(0, 2) =  st * sa;  T(0, 3) = a * ct;
    T(1, 0) = st;     T(1, 1) =  ct * ca;  T(1, 2) = -ct * sa;  T(1, 3) = a * st;
    T(2, 0) = 0.0;    T(2, 1) =  sa;       T(2, 2) =  ca;       T(2, 3) = d;
    T(3, 0) = 0.0;    T(3, 1) =  0.0;      T(3, 2) =  0.0;      T(3, 3) = 1.0;

    return T;
}

// Forward kinematics: joint angles (rad) -> T_0_4
Eigen::Matrix4d forwardKinematics(const std::vector<double>& joints_rad,
                                  const std::vector<DHParam>& dh)
{
    if (joints_rad.size() != dh.size())
    {
        std::cerr << "Error: joint vector size (" << joints_rad.size()
                  << ") does not match DH parameter size (" << dh.size() << ")\n";
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();

    for (std::size_t i = 0; i < joints_rad.size(); ++i)
    {
        double theta = joints_rad[i] + dh[i].theta0;
        const auto& p = dh[i];

        Eigen::Matrix4d T_i = dhTransform(theta, p.d, p.a, p.alpha);
        T_total = T_total * T_i;
    }

    return T_total;
}

// Utility: degrees -> radians
inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

int main()
{
    const double L = 1.0;

    // Define DH parameters for the 4-link perpendicular robot
    std::vector<DHParam> dh_params = {
        {L,  M_PI / 2.0, 0.0, 0.0},  // Joint 1
        {L,  M_PI / 2.0, 0.0, 0.0},  // Joint 2
        {L,  M_PI / 2.0, 0.0, 0.0},  // Joint 3
        {L,  0.0,        0.0, 0.0}   // Joint 4
    };

    // ------------ Test 1: All joints = 0 deg (home pose) ------------
    std::vector<double> q_home = {0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix4d T_home = forwardKinematics(q_home, dh_params);

    Eigen::Vector3d p_home = T_home.block<3,1>(0, 3);
    Eigen::Matrix3d R_home = T_home.block<3,3>(0, 0);
    Eigen::Quaterniond quat_home(R_home);

    std::cout << "================= TASK 4: FORWARD KINEMATICS (C++) =================\n";
    std::cout << "\n[Test 1] Home configuration (all joints = 0 rad)\n";
    std::cout << "End-effector position:\n";
    std::cout << "  x = " << p_home.x() << " m\n";
    std::cout << "  y = " << p_home.y() << " m\n";
    std::cout << "  z = " << p_home.z() << " m\n";
    std::cout << "Orientation (quaternion [w, x, y, z]):\n";
    std::cout << "  [" << quat_home.w() << ", "
                     << quat_home.x() << ", "
                     << quat_home.y() << ", "
                     << quat_home.z() << "]\n";

    // ------------ Test 2: Some arbitrary joint configuration --------
    std::vector<double> q_test = {
        deg2rad(30.0),
        deg2rad(45.0),
        deg2rad(-20.0),
        deg2rad(60.0)
    };

    Eigen::Matrix4d T_test = forwardKinematics(q_test, dh_params);
    Eigen::Vector3d p_test = T_test.block<3,1>(0, 3);
    Eigen::Matrix3d R_test = T_test.block<3,3>(0, 0);
    Eigen::Quaterniond quat_test(R_test);

    std::cout << "\n[Test 2] Arbitrary configuration (30°, 45°, -20°, 60°)\n";
    std::cout << "Joint angles (rad): ";
    for (double q : q_test) std::cout << q << " ";
    std::cout << "\nEnd-effector position:\n";
    std::cout << "  x = " << p_test.x() << " m\n";
    std::cout << "  y = " << p_test.y() << " m\n";
    std::cout << "  z = " << p_test.z() << " m\n";
    std::cout << "Orientation (quaternion [w, x, y, z]):\n";
    std::cout << "  [" << quat_test.w() << ", "
                     << quat_test.x() << ", "
                     << quat_test.y() << ", "
                     << quat_test.z() << "]\n";

    // ------------ Test 3: Print full T matrix for verification -------
    std::cout << "\n[Test 3] Full transformation matrix for Test 2:\n";
    std::cout << T_test << "\n";

    std::cout << "\nAll C++ FK tests completed.\n";
    return 0;
}

