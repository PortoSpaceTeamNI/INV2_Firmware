#include <ArduinoEigen.h>
#include <iostream>
#include <cmath>
#include "quaternion.h"
#include "func.h"

using namespace Eigen;
help_func helper;


/*Eigen::Quaternionf help_func::small_angle_quat(const Eigen::Vector3f& delta_theta) {
    float angle = delta_theta.norm();
    if (angle < 1e-12) {
        return Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);  // identity quaternion
    }
    Eigen::Vector3f axis = delta_theta.normalized();
    float half = 0.5f * angle;
    return Eigen::Quaternionf(cos(half), axis.x() * sin(half), axis.y() * sin(half), axis.z() * sin(half));
}*/

Eigen::Matrix3f help_func::skew(const float w[3]) {
    Eigen::Matrix3f W;
    W <<  0,      -w[2],  w[1],
          w[2],  0,      -w[0],
         -w[1],  w[0],    0.0f;
    return W;
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
    const Eigen::Vector3f& g_n, 
    const Eigen::Vector3f& m_n
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

void help_func::apply_attitude_correction(Eigen::MatrixXf *x, float *q_nom)
{
    // small-angle error
    float delta_theta[3] = {(*x)(0), (*x)(1), (*x)(2)}; 

    // small-angle quaternion
    MyQuaternion dq = small_angle_quat(delta_theta);

    // correction
    MyQuaternion q_nom_quat {q_nom[0], q_nom[1], q_nom[2], q_nom[3]};
    MyQuaternion q_nom_updated = (dq * q_nom_quat);
    q_nom_updated.normalize(); 
    q_nom[0] = q_nom_updated.qw;
    q_nom[1] = q_nom_updated.qx;
    q_nom[2] = q_nom_updated.qy;
    q_nom[3] = q_nom_updated.qz;

    // Reset error-state attitude
    (*x).block(0, 0, 3, 1).setZero();
}

const float DEG2RAD = M_PI / 180.0f;
float q_nom[4] = {1,0,0,0}; // w, x, y, z
float acc_body[3] = {0, 0, 0}; // ax, ay, az
float gyro_body[3] = {0, 0, 0}; // gx, gy, 

static int kalman_count = 0;
const int LOOPS = 10; // skip first 10 iterations

void predict(Eigen::MatrixXf *x, Eigen::MatrixXf *P, float acc_fused[3],
    float gyro_body[3], uint32_t Ts_us, Eigen::MatrixXf *Q, Eigen::MatrixXf *F, Eigen::MatrixXf *P_pred) {
    
    static float avg_gx[10] = {0}, avg_gy[10] = {0}, avg_gz[10] = {0};
    static float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    static uint8_t idx = 0;
    
    // Add current gyro reading to moving average
    sum_gx += gyro_body[0];
    sum_gx -= avg_gx[idx];
    avg_gx[idx] = gyro_body[0];

    sum_gy += gyro_body[1];
    sum_gy -= avg_gy[idx];
    avg_gy[idx] = gyro_body[1];

    sum_gz += gyro_body[2];
    sum_gz -= avg_gz[idx];
    avg_gz[idx] = gyro_body[2];

    // Use averaged gyro for delta calculation
    float gyro_avg[3] = {
        sum_gx / 10.0f,
        sum_gy / 10.0f,
        sum_gz / 10.0f
    };

    static float avg_ax[10] = {0}, avg_ay[10] = {0}, avg_az[10] = {0};
    static float sum_ax = 0, sum_ay = 0, sum_az = 0;

    sum_ax += acc_fused[0] - avg_ax[idx];
    sum_ay += acc_fused[1] - avg_ay[idx];
    sum_az += acc_fused[2] - avg_az[idx];

    avg_ax[idx] = acc_fused[0];
    avg_ay[idx] = acc_fused[1];
    avg_az[idx] = acc_fused[2];

    float acc_avg[3] = {
        sum_ax / 10.0f,
        sum_ay / 10.0f,
        sum_az / 10.0f
    };
    
    idx = (idx + 1) % 10; // rotate buffer

    const float dt = Ts_us * 1e-6f;

    float bimu[3] = {(*x)(3), (*x)(4), (*x)(5)};          // imu bias
    float bg[3] = {(*x)(6), (*x)(7), (*x)(8)};            // gyro biases
    float v[3] = {(*x)(9), (*x)(10), (*x)(11)};          // velocity
    float p[3] = {(*x)(12), (*x)(13), (*x)(14)};         // position   

    // Correct angular velocity 
    float delta[3];
    for(int i=0;i<3;i++){
        delta[i] = (gyro_avg[i] - bg[i]) * dt; // gyro in rad/s
    }

    //Serial.print(" wx = "); Serial.print(gyro_avg[0]);
    //Serial.print(" wy = "); Serial.print(gyro_avg[1]);
    //Serial.print(" wz = "); Serial.print(gyro_avg[2]);

    //Serial.print(" quat: "); Serial.print(q_nom[0]); Serial.print(", "); Serial.print(q_nom[1]); Serial.print(", "); Serial.print(q_nom[2]); Serial.print(", "); Serial.print(q_nom[3]);

    MyQuaternion dq = small_angle_quat(delta); 
    MyQuaternion q1 = floatArrayToQuaternion(q_nom);
    MyQuaternion q_nom_updated = q1 * dq;  // Apply small-angle correction
    q_nom_updated.normalize();  // Normalize the result

    q_nom[0] = q_nom_updated.qw;
    q_nom[1] = q_nom_updated.qx;
    q_nom[2] = q_nom_updated.qy;
    q_nom[3] = q_nom_updated.qz;

    /*float euler[3];
    quaternion_to_euler(q_nom_updated, euler);
    Serial.print(" roll = "); Serial.print(euler[0]);
    Serial.print(" pitch = "); Serial.print(euler[1]);
    Serial.print(" yaw = "); Serial.print(euler[2]);*/

    Eigen::Matrix3f R = quaternion_to_rotation_matrix(q_nom_updated);  // Rotation from body to nav frame
    
    float acc_body[3] = {acc_avg[0], acc_avg[1], acc_avg[2]};

    Eigen::Vector3f acc_body_vec(acc_body[0], acc_body[1], acc_body[2]);
    Eigen::Vector3f gravity_nav(0, 0, -9.8f); // downward
    Eigen::Vector3f gravity_body = R.transpose() * gravity_nav;

    Eigen::Vector3f acc_body_corrected2 = acc_body_vec - gravity_body;
    Eigen::Vector3f acc_world2 = R * acc_body_corrected2;

    float acc_world[3] = {acc_world2(0), acc_world2(1), acc_world2(2)};
    acc_world[0] = truncf(acc_world[0] * 10.0f) / 10.0f;
    acc_world[1] = truncf(acc_world[1] * 10.0f) / 10.0f;
    acc_world[2] = truncf(acc_world[2] * 10.0f) / 10.0f;

    float acc_body_corrected[3] = {acc_body_corrected2(0), acc_body_corrected2(1), acc_body_corrected2(2)};

    /*const float GRAVITY = 9.81f;
    Eigen::Vector3f gravity_nav(0.0f, 0.0f, GRAVITY); // Rotate gravity to body frame using the quaternion (nav to body rotation)
    Eigen::Vector3f gravity_body = R.transpose() * gravity_nav; // Subtract gravity from body frame acceleration
    float acc_body_corrected[3] = { acc_body[0] - gravity_body[0], acc_body[1] - gravity_body[1], acc_body[2] - gravity_body[2] }; // Transform corrected acceleration to navigation frame
    float acc_world[3] = {R(0,0)*acc_body_corrected[0] + R(0,1)*acc_body_corrected[1] + R(0,2)*acc_body_corrected[2],
                          R(1,0)*acc_body_corrected[0] + R(1,1)*acc_body_corrected[1] + R(1,2)*acc_body_corrected[2],
                          R(2,0)*acc_body_corrected[0] + R(2,1)*acc_body_corrected[1] + R(2,2)*acc_body_corrected[2]};*/

    //Serial.print("  acc_world_x = "); Serial.print(acc_world[0]);
    //Serial.print("  acc_world_y = "); Serial.print(acc_world[1]);
    //Serial.print("  acc_world_z = "); Serial.print(acc_world[2]);

    if (kalman_count < LOOPS) {
        kalman_count++;
        return; // Skip prediction for first few iterations
    } else{
        
        float v_new[3] = {v[0] + acc_world[0] * dt,
                          v[1] + acc_world[1] * dt,
                          v[2] + acc_world[2] * dt};

        float p_new[3] = {(float)(p[0] + v[0] * dt + 0.5f * acc_world[0] * dt * dt),
                          (float)(p[1] + v[1] * dt + 0.5f * acc_world[1] * dt * dt),
                          (float)(p[2] + v[2] * dt + 0.5f * acc_world[2] * dt * dt)};
    

        (*F).block<3,3>(0,6) = -Eigen::Matrix3f::Identity() * dt; // dtheta_dbg
        (*F).block<3,3>(9,0) = -R * helper.skew(acc_body_corrected) * dt; // dv/dtheta
        (*F).block<3,3>(9,3) = -R * dt; // dv/dbimu
        (*F).block<3,3>(12,9) = Eigen::Matrix3f::Identity() * dt; // dp/dv
            
        (*x)(0) = 0.0f;
        (*x)(1) = 0.0f;
        (*x)(2) = 0.0f;
        (*x)(3) = bimu[0];
        (*x)(4) = bimu[1];
        (*x)(5) = bimu[2];
        (*x)(6) = bg[0];
        (*x)(7) = bg[1];
        (*x)(8) = bg[2];
        (*x)(9) = v_new[0];
        (*x)(10) = v_new[1];
        (*x)(11) = v_new[2];
        (*x)(12) = p_new[0];
        (*x)(13) = p_new[1];
        (*x)(14) = p_new[2];

        
        *P_pred = (*F) * (*P) * (*F).transpose() + *Q;
    }

    //std::cout << "x_pred: \n" << x_pred.transpose() << std::endl;
    //std::cout << "P_pred: \n" << P_pred << std::endl;
}


void update_barometer(Eigen::MatrixXf *x, Eigen::MatrixXf *P, float *z_baro, Eigen::MatrixXf *R_baro, int pz_idx, int b_idx) {
        Eigen::MatrixXf IS;

        // Predicted measurement (z_pred = x[pz_idx] + x[b_idx])
        float z_pred = (*x)(pz_idx) + (*x)(b_idx);
        //Serial.print(" z_pred = "); Serial.print(z_pred);
        
        // Innovation (y = z_baro - z_pred)
        float y = *z_baro - z_pred;

        // Measurement Jacobian H (1x13)
        Eigen::MatrixXf H(1, (*x).size());
        H.setZero();         
        H(0, pz_idx) = 1.0;    
        H(0, b_idx) = 1.0;   

        // Compute S = H * P * H.T + R
        IS = H * (*P) * H.transpose() + (*R_baro);

        // Compute Kalman Gain K = P * H.T * S.inverse()
        MatrixXf K = (*P) * H.transpose() * IS.inverse();

        // State update: X = X + K * (z_baro - H * X)
        (*x)(pz_idx) = (*x)(pz_idx) + K(pz_idx) * y;

        // Covariance update: P = (I - K * H) * P
        *P = (Eigen::MatrixXf::Identity((*P).rows(), (*P).cols()) - K * H) * (*P);
        //Serial.print(" update barometer done");
    }


void update_mag(Eigen::MatrixXf *x, Eigen::MatrixXf *P, const float *q_nom, const float *mag_meas, const float *mag_ref, Eigen::MatrixXf *R_mag) {

    MyQuaternion q;
    q.qw = q_nom[0];
    q.qx = q_nom[1];
    q.qy = q_nom[2];
    q.qz = q_nom[3];
    Eigen::Matrix3f Rb2n = quaternion_to_rotation_matrix(q);  // Rotation from body to nav frame

    // Step 2: Predicted magnetometer measurement (mag_pred)
    float mag_pred[3] = {Rb2n.transpose()(0,0)*mag_ref[0] + Rb2n.transpose()(0,1)*mag_ref[1] + Rb2n.transpose()(0,2)*mag_ref[2],
                         Rb2n.transpose()(1,0)*mag_ref[0] + Rb2n.transpose()(1,1)*mag_ref[1] + Rb2n.transpose()(1,2)*mag_ref[2],
                         Rb2n.transpose()(2,0)*mag_ref[0] + Rb2n.transpose()(2,1)*mag_ref[1] + Rb2n.transpose()(2,2)*mag_ref[2]};

    // Step 3: Innovation (y = mag_meas - mag_pred)
    float y[3] = {mag_meas[0] - mag_pred[0], mag_meas[1] - mag_pred[1], mag_meas[2] - mag_pred[2]};

    Eigen::MatrixXf H(3, 13);
    H.setZero(); 
    H.block<3, 3>(0, 0) = -helper.skew(mag_pred); 

    Eigen::Matrix3f S = H * (*P) * H.transpose() + (*R_mag);
    Eigen::MatrixXf K = (*P) * H.transpose() * S.inverse();

    *x = (*x) + K * Eigen::Map<Eigen::Vector3f>(y);

    Eigen::MatrixXf I = Eigen::MatrixXf::Identity((*P).rows(), (*P).cols());
    *P = (I - K*H) * (*P);
}

/*void update(const Eigen::VectorXf& y_pred, 
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
}*/


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