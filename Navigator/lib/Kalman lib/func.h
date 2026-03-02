#ifndef FUNC_H
#define FUNC_H

#include <Eigen/Dense>
#include <Eigen/Geometry>  // For Quaternion

// Define a custom class for your helper functions
class help_func {
public:
    // Small angle approximation for quaternion generation
    static Eigen::Quaternionf small_angle_quat(const Eigen::Vector3f& delta_theta);

    // Skew-symmetric matrix for a 3D vector
    static Eigen::Matrix3f skew(const Eigen::Vector3f& w);

    // NED to Body frame conversion with quaternion (using roll, pitch, yaw)
    static Eigen::Vector3f ned_to_body_with_quaternion(const Eigen::Vector3f& mag_nav, float roll, float pitch, float yaw);

    // Convert a rotation matrix to a quaternion
    static Eigen::Vector4f rotation_matrix_to_quaternion(const Eigen::Matrix3f& R);

    // TRIAD algorithm for initial orientation estimation
    static Eigen::Vector4f triad_algorithm(const Eigen::Vector3f& acc_meas, 
                                           const Eigen::Vector3f& mag_meas, 
                                           const Eigen::Vector3f& g_n = Eigen::Vector3f(0, 0, -1), 
                                           const Eigen::Vector3f& m_n = Eigen::Vector3f(1, 0, 0));

    // Adaptive fusion for state and uncertainty update
    static std::pair<float, float> adaptive_fusion(float x1, float sigma1, float x2, float sigma2);

    // Apply attitude correction
    static void apply_attitude_correction(Eigen::VectorXf& x, Eigen::Quaternionf& q_nom);
};

// Define a custom class for the EKF (Extended Kalman Filter)
class ekf {
public:
    // EKF predict step
    static void predict(Eigen::VectorXf& x, Eigen::MatrixXf& P, Eigen::Quaternionf& q_nom, 
                        const Eigen::Vector3f& acc_body, const Eigen::Vector3f& gyro_body, 
                        float Ts, const Eigen::MatrixXf& Q);

    // EKF barometer update step
    static void update_barometer(Eigen::VectorXf& x, Eigen::MatrixXf& P, float z_baro, 
                                 const Eigen::MatrixXf& R_baro, int pz_idx = 11, int b_idx = 12);

    // EKF magnetometer update step
    static void update_mag(Eigen::VectorXf& x, Eigen::MatrixXf& P, const Eigen::Quaternionf& q_nom,
                           const Eigen::Vector3f& mag_meas, const Eigen::Vector3f& mag_ref, const Eigen::Matrix3f& R_mag);
};

// A general update function (not specific to EKF, could be used for other updates)
void update(const Eigen::VectorXf& y_pred, 
            const Eigen::VectorXf& y_sensor, 
            const Eigen::MatrixXf& H, 
            Eigen::VectorXf& x, 
            Eigen::MatrixXf& P, 
            const Eigen::MatrixXf& R_sensor);

#endif // FUNC_H