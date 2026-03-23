#ifndef FUNC_H
#define FUNC_H

#include "Sensors.h"
#include <ArduinoEigen.h>

extern Eigen::MatrixXf x; // State vector for Kalman filter

// Define a custom class for your helper functions
class help_func {
public:
    // Skew-symmetric matrix for a 3D vector
    static Eigen::Matrix3f skew(const float w[3]);

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
    static void apply_attitude_correction(Eigen::MatrixXf *x, float *q_nom);
};


// EKF barometer update step
void update_barometer(Eigen::MatrixXf *x, Eigen::MatrixXf *P, float *z_baro, Eigen::MatrixXf *R_baro, int pz_idx = 11, int b_idx = 12);

// EKF magnetometer update step
void update_mag(Eigen::MatrixXf *x, Eigen::MatrixXf *P, const float *q_nom, float mag_meas[3], float mag_ref[3], Eigen::MatrixXf *R_mag);

void predict(Eigen::MatrixXf *x, Eigen::MatrixXf *P, float *q_nom, float acc_body[3], float gyro_body[3], float Ts, Eigen::MatrixXf *Q, Eigen::MatrixXf *F, Eigen::MatrixXf *P_pred);


// A general update function (not specific to EKF, could be used for other updates)
void update(const Eigen::VectorXf& y_pred, 
            const Eigen::VectorXf& y_sensor, 
            const Eigen::MatrixXf& H, 
            Eigen::MatrixXf& x, 
            Eigen::MatrixXf& P, 
            const Eigen::MatrixXf& R_sensor);

#endif // FUNC_H