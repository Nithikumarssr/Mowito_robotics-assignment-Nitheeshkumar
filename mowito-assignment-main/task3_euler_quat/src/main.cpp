#include <iostream>
#include <Eigen/Dense>  // For Matrix/Quaternion
#include <cmath>        // For sin/cos/atan2

// Function: Euler (ZYX) to Quaternion
// Input: roll (x), pitch (y), yaw (z) in DEGREES
// Output: Quaternion (w, x, y, z) normalized
Eigen::Quaterniond eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg) {
    // Convert to radians
    double roll = M_PI * roll_deg / 180.0;
    double pitch = M_PI * pitch_deg / 180.0;
    double yaw = M_PI * yaw_deg / 180.0;

    // Rotation matrices (ZYX convention)
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    // Compose rotation: yaw * pitch * roll
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    q.normalize();  // Ensure unit quaternion
    return q;
}

// Alternative: Manual matrix → quat (for understanding)
Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& R) {
    double trace = R.trace();
    Eigen::Quaterniond q;
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        q.w() = 0.25 / s;
        q.x() = (R(2, 1) - R(1, 2)) * s;
        q.y() = (R(0, 2) - R(2, 0)) * s;
        q.z() = (R(1, 0) - R(0, 1)) * s;
    } else {
        // Handle gimbal lock cases (one of x,y,z max)
        if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            q.w() = (R(2, 1) - R(1, 2)) / s;
            q.x() = 0.25 * s;
            q.y() = (R(0, 1) + R(1, 0)) / s;
            q.z() = (R(0, 2) + R(2, 0)) / s;
        } // Add cases for y/z similarly...
        q.normalize();
    }
    return q;
}

int main() {
    // Sample input: Euler angles in degrees (e.g., from IMU)
    double roll = 10.0;   // X rotation
    double pitch = 20.0;  // Y
    double yaw = 30.0;    // Z

    // Method 1: AngleAxis (simple)
    Eigen::Quaterniond q1 = eulerToQuaternion(roll, pitch, yaw);
    std::cout << "Euler (deg): roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << std::endl;
    std::cout << "Quaternion (w,x,y,z): " << q1.w() << ", " << q1.x() << ", " 
              << q1.y() << ", " << q1.z() << std::endl;

    // Method 2: Build matrix manually
    Eigen::AngleAxisd rollAxis(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAxis(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAxis(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = yawAxis.toRotationMatrix() * pitchAxis.toRotationMatrix() * rollAxis.toRotationMatrix();
    Eigen::Quaterniond q2 = matrixToQuaternion(R);
    std::cout << "From Matrix: " << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << std::endl;

    // Verify equality (should match)
    std::cout << "Match? " << (q1.isApprox(q2) ? "Yes" : "No") << std::endl;

    // Edge Case: Gimbal lock (pitch=90°)
    Eigen::Quaterniond q_lock = eulerToQuaternion(0, 90, 0);
    std::cout << "Gimbal Lock (pitch=90): " << q_lock.coeffs().transpose() << std::endl;

    return 0;
}

