#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "quaternion.h"
#include "func.h"

using namespace Eigen;
help_func helper;


Eigen::Quaternionf help_func::small_angle_quat(const Eigen::Vector3f& delta_theta) {
    float angle = delta_theta.norm();
    if (angle < 1e-12) {
        return Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);  // identity quaternion
    }
    Eigen::Vector3f axis = delta_theta.normalized();
    float half = 0.5f * angle;
    return Eigen::Quaternionf(cos(half), axis.x() * sin(half), axis.y() * sin(half), axis.z() * sin(half));
}

Eigen::Matrix3f help_func::skew(const Eigen::Vector3f& w){
    Eigen::Matrix3f W;
    W <<  0,      -w.z(),  w.y(),
          w.z(),  0,      -w.x(),
         -w.y(),  w.x(),       0;
    return W;
}

Eigen::Quaternionf quat_mul(const Eigen::Quaternionf& q1, const Eigen::Quaternionf& q2) {
    return q1 * q2;  // Use Eigen's built-in quaternion multiplication
}


// Convert mag field vector from NED frame to body frame using roll, pitch, and yaw (using quaternions)
Eigen::Vector3f help_func::ned_to_body_with_quaternion(const Eigen::Vector3f& mag_nav, float roll, float pitch, float yaw) {
    
    MyQuaternion q = euler_to_quaternion(roll, pitch, yaw);

    // Rotation from body to nav
    Eigen::Matrix3f R_bn = quaternion_to_rotation_matrix(q);

    Eigen::Matrix3f R_nb = R_bn.transpose();

    // Convert to body frame
    Eigen::Vector3f mag0 = R_nb * mag_nav;
    return mag0;
}


// For TRIAD algorithm
Eigen::Vector4f help_func::rotation_matrix_to_quaternion(const Eigen::Matrix3f& R) {
    Eigen::Vector4f q;  
    float tr = R.trace();  

    if (tr > 0) {
        // S = sqrt(trace(R) + 1) * 2
        float Sy = std::sqrt(tr + 1.0f) * 2.0f;
        q(0) = 0.25f * Sy;                // w component
        q(1) = (R(2, 1) - R(1, 2)) / Sy;  // x component
        q(2) = (R(0, 2) - R(2, 0)) / Sy;  // y component
        q(3) = (R(1, 0) - R(0, 1)) / Sy;  // z component
    } else {
        // Handle the case where the trace is non-positive
        if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
            float S = std::sqrt(1.0f + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0f;
            q(0) = (R(2, 1) - R(1, 2)) / S; 
            q(1) = 0.25f * S;  
            q(2) = (R(0, 1) + R(1, 0)) / S;  
            q(3) = (R(0, 2) + R(2, 0)) / S; 
        } else if (R(1, 1) > R(2, 2)) {
            float S = std::sqrt(1.0f + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0f;
            q(0) = (R(0, 2) - R(2, 0)) / S; 
            q(1) = (R(0, 1) + R(1, 0)) / S;  
            q(2) = 0.25f * S;  
            q(3) = (R(1, 2) + R(2, 1)) / S; 
        } else {
            float S = std::sqrt(1.0f + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0f;
            q(0) = (R(1, 0) - R(0, 1)) / S; 
            q(1) = (R(0, 2) + R(2, 0)) / S;
            q(2) = (R(1, 2) + R(2, 1)) / S;  
            q(3) = 0.25f * S; 
        }
    }
    return q.normalized();  
}

// TRIAD Algorithm for initial orientation estimation
Eigen::Vector4f help_func::triad_algorithm(
    const Eigen::Vector3f& acc_meas, 
    const Eigen::Vector3f& mag_meas, 
    const Eigen::Vector3f& g_n = Eigen::Vector3f(0, 0, -1), 
    const Eigen::Vector3f& m_n = Eigen::Vector3f(1, 0, 0)
) {
    // Normalize all vectors
    Eigen::Vector3f t1_b = acc_meas.normalized();
    Eigen::Vector3f t2_b = t1_b.cross(mag_meas).normalized();
    Eigen::Vector3f t3_b = t1_b.cross(t2_b);

    Eigen::Vector3f t1_n = g_n.normalized();
    Eigen::Vector3f t2_n = t1_n.cross(m_n).normalized();
    Eigen::Vector3f t3_n = t1_n.cross(t2_n);

    // Construct the rotation matrix R_b and R_n
    Eigen::Matrix3f R_b;
    R_b << t1_b, t2_b, t3_b;

    Eigen::Matrix3f R_n;
    R_n << t1_n, t2_n, t3_n;

    // Compute the rotation matrix from body to navigation frame
    Eigen::Matrix3f R_nb = R_n * R_b.transpose();

    // Convert rotation matrix to quaternion
    return rotation_matrix_to_quaternion(R_nb);
}

//std::pair  --> returning multiple values
std::pair<float, float> help_func::adaptive_fusion(float x1, float sigma1, float x2, float sigma2) {

    float w1 = 1.0f / (sigma1 * sigma1);
    float w2 = 1.0f / (sigma2 * sigma2);

    float s_fused = 1.0f / std::sqrt(w1 + w2);  // Fused uncertainty
    float f = (w1 * x1 + w2 * x2) / (w1 + w2); // Fused state

    return std::make_pair(f, s_fused); 
}

void help_func::apply_attitude_correction(Eigen::VectorXf& x, Eigen::Quaternionf& q_nom)
{
    // small-angle error
    Eigen::Vector3f delta_theta = x.segment<3>(0);

    // small-angle quaternion
    Eigen::Quaternionf dq = small_angle_quat(delta_theta);

    // correction
    q_nom = (dq * q_nom).normalized();

    // Reset error-state attitude
    x.segment<3>(0).setZero();
}

void ekf::predict(Eigen::VectorXf& x, Eigen::MatrixXf& P, Eigen::Quaternionf& q_nom, 
                  const Eigen::Vector3f& acc_body, const Eigen::Vector3f& gyro_body, 
                  float Ts, const Eigen::MatrixXf& Q){

    Eigen::Vector3f delta_theta = x.segment(0, 3);   // Angular position
    Eigen::Vector3f bg = x.segment(3, 3);            // Gyro biases
    Eigen::Vector3f v = x.segment(6, 3);            
    Eigen::Vector3f p = x.segment(9, 3);            
    Eigen::Quaternionf q_nom_new = q_nom;           

    // Correct angular velocity
    Eigen::Vector3f omega = gyro_body - bg;

    // Small angle approximation for quaternion update
    Eigen::Quaternionf dq = helper.small_angle_quat(omega * Ts);  
    q_nom_new = (quat_mul(q_nom, dq)).normalized();  


    Eigen::Matrix3f R = q_nom_new.toRotationMatrix();
    // Transform acceleration to world frame (including gravity)
    Eigen::Vector3f acc_world = R * acc_body + Eigen::Vector3f(0, 0, -9.81);

    // State propagation (position and velocity update)
    Eigen::Vector3f v_new = v + acc_world * Ts;
    Eigen::Vector3f p_new = p + v * Ts + 0.5 * acc_world * Ts * Ts;


    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(13, 13);  
    
    F.block(6, 0, 3, 3) = -R * helper.skew(acc_body) * Ts; 
    F.block(6, 3, 3, 3) = -R * Ts;                   
    F.block(9, 6, 3, 3) = Eigen::Matrix3f::Identity() * Ts;  

    Eigen::VectorXf x_pred(13);
    x_pred << Eigen::Vector3f::Zero(), bg, v_new, p_new, x[12];  

    Eigen::MatrixXf P_pred = F * P * F.transpose() + Q;

 
    x = x_pred;  
    P = P_pred;  
    q_nom = q_nom_new;  

    //std::cout << "x_pred: \n" << x_pred.transpose() << std::endl;
    //std::cout << "P_pred: \n" << P_pred << std::endl;
}



void ekf::update_barometer(Eigen::VectorXf& x, Eigen::MatrixXf& P, float z_baro, const Eigen::MatrixXf& R_baro, int pz_idx = 11, int b_idx = 12) {
        Eigen::MatrixXf IS;

        // Predicted measurement (z_pred = x[pz_idx] + x[b_idx])
        float z_pred = x(pz_idx) + x(b_idx);

        // Innovation (y = z_baro - z_pred)
        Eigen::VectorXf y(1); 
        y(0) = z_baro - z_pred;

        // Measurement Jacobian H (1xN matrix where N = size of x)
        Eigen::MatrixXf H(1, x.size());
        H.setZero();         
        H(0, pz_idx) = 1.0;    
        H(0, b_idx) = 1.0;   

        // Compute S = H * P * H.T + R
        IS = H * P * H.transpose() + R_baro;

        // Compute Kalman Gain K = P * H.T * S.inverse()
        Eigen::MatrixXf K = P * H.transpose() * IS.inverse();

        // State update: X = X + K * (z_baro - H * X)
        x = x + K * y;

        // Covariance update: P = (I - K * H) * P
        P = (Eigen::MatrixXf::Identity(P.rows(), P.cols()) - K * H) * P;
    }


void ekf::update_mag(Eigen::VectorXf& x, Eigen::MatrixXf& P, const Eigen::Quaternionf& q_nom,
                     const Eigen::Vector3f& mag_meas, const Eigen::Vector3f& mag_ref, const Eigen::Matrix3f& R_mag) {
    
    Eigen::Matrix3f Rb2n = q_nom.toRotationMatrix();

    // Step 2: Predicted magnetometer measurement (mag_pred)
    Eigen::Vector3f mag_pred = Rb2n.transpose() * mag_ref;

    // Step 3: Innovation (y = mag_meas - mag_pred)
    Eigen::Vector3f y = mag_meas - mag_pred;

    Eigen::MatrixXf H(3, 13);
    H.setZero(); 
    H.block<3, 3>(0, 0) = -helper.skew(mag_pred); 

    Eigen::Matrix3f S = H * P * H.transpose() + R_mag;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    x = x + K * y;

    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(P.rows(), P.cols());
    P = (I - K*H) * P;
}

void update(const Eigen::VectorXf& y_pred, 
            const Eigen::VectorXf& y_sensor, 
            const Eigen::MatrixXf& H, 
            Eigen::VectorXf& x, 
            Eigen::MatrixXf& P, 
            const Eigen::MatrixXf& R_sensor) {
    
    Eigen::VectorXf y = y_sensor - y_pred;

    // Compute S = H * P * H.T + R_sensor
    Eigen::MatrixXf S = H * P * H.transpose() + R_sensor;
    // Kalman Gain: K = P * H.T * S.inverse()
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();
    // State update
    x = x + K * y;

    // Covariance update: P = (I - K * H) * P
    P = (Eigen::MatrixXf::Identity(P.rows(), P.cols()) - K * H) * P;
}


/*void ekf::R_update(){
    
    
    //R_mean=last_Z.rowwise().sum()/last_Z.cols();

    R_mean = R_mean - Z_rem + Z;

    for(int i=0; i<Z.size();i++)
        R(i,i) = R(i,i) + ((Z(i)-R_mean(i)*R_mean(i)))/((float)last_Z.rows());

}*/

/*void ekf::Q_update(){
    
    //Q_mean=last_X.rowwise().sum()/last_X.cols();

    Q_mean = Q_mean - X_rem + X;

    for(int i=0; i<X.size();i++)
        Q(i,i) = Q(i,i) + ((X(i)-Q_mean(i)*Q_mean(i)))/((float)last_X.rows());
        
}*/

/* 
FALTA:
- introduzir dados/parâmetros;
- confirmar dimensões de H;
- fazer ciclos (com frequências?);
- confirmar função de H;


- perguntar sobre float e int; delta_t multiplicar por 1000?
*/