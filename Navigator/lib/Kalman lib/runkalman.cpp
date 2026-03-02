#include <ArduinoEigen.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "quaternion.h"
#include "func.h"
#include "Sensors.h"
#include "Display.h"

ekf mekf;
help_func helper;
BMP581DataResult bmp581;
LSM6DSODataResult lsm;

// Indices
const int pz_idx = 11;
const int b_idx  = 12;
int N_filter = 1000; // 89099

// externs (from your firmware main file)
extern float imu_ax_local;
extern float imu_ay_local;
extern float imu_az_local;
extern float imu_gx_local;
extern float imu_gy_local;
extern float imu_gz_local;
extern float altitude_local;

extern float mag_x_local;
extern float mag_y_local;
extern float mag_z_local;

// State and covariance
Eigen::VectorXf x = Eigen::VectorXf::Zero(13);
Eigen::MatrixXf P = Eigen::MatrixXf::Identity(13, 13) * 0.1;
Eigen::Vector4f q_nom(1,0,0,0); // Quaternion

Eigen::MatrixXf pos_est    = Eigen::MatrixXf::Zero(N_filter, 3);  
Eigen::MatrixXf vel_est    = Eigen::MatrixXf::Zero(N_filter, 3); 
Eigen::MatrixXf q_log      = Eigen::MatrixXf::Zero(N_filter, 4); 
Eigen::MatrixXf pos_true   = Eigen::MatrixXf::Zero(N_filter, 3); 
Eigen::MatrixXf vel_true   = Eigen::MatrixXf::Zero(N_filter, 3); 
Eigen::MatrixXf magg       = Eigen::MatrixXf::Zero(N_filter, 3); 
Eigen::MatrixXf acc_meas_all = Eigen::MatrixXf::Zero(N_filter, 3);
Eigen::MatrixXf acc_vector   = Eigen::MatrixXf::Zero(N_filter, 3);

// Sensor noise matrices
float sigma_baro1 = 0.0175, sigma_baro2 = 0.0283, sigma_mag = 0.03;
Eigen::MatrixXf RR1 = Eigen::MatrixXf::Identity(13,13) * sigma_baro1 * sigma_baro1;   // baro1
Eigen::MatrixXf RR2 = Eigen::MatrixXf::Identity(13,13) * sigma_baro2 * sigma_baro2; // baro2
Eigen::Matrix3f RR_mag = Eigen::Matrix3f::Identity() * sigma_mag; // mag
Eigen::Vector3f R_mean_mag = Eigen::Vector3f::Zero();
Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(13, 13);
Eigen::VectorXf Q_mean = Eigen::VectorXf::Zero(13);
Eigen::Matrix3Xf mag_nav;
Eigen::VectorXf pressure_upsampled_noisy;

// Buffers
std::vector<Eigen::Vector3f> acc_buffer;
std::vector<Eigen::Vector3f> gyro_buffer;


int count_mag = 0;
int count_Q   = 0;

bool parachute_opened1 = false;
bool parachute_opened2 = false;

int count_baro = 0;
float z1, z2, pr1= 0, pr2=0;
float Ts = 0.05;
extern bool baro1_ready, baro2_ready, mag_ready;
float p0_1, p0_2, pr1, pr2, z_fused;

void runKalmanFilter() {

        Eigen::Vector3f acc_meas;
        Eigen::Vector3f gyro1, gyro2;

        Eigen::Vector3f acc_fused = lsm.AccelX, lsm.AccelY, lsm.AccelZ; 
        uint8_t vector[3] = {lsm.AccelX, lsm.AccelY, lsm.AccelZ};
        
        Eigen::Vector3f gyro_fused(lsm.GyroX, lsm.GyroY, lsm.GyroZ);

        Eigen::Quaternionf q(q_nom(0), q_nom(1), q_nom(2), q_nom(3)); // w, x, y, z
        mekf.predict(x, P, q, acc_fused, gyro_fused, Ts, Eigen::MatrixXf::Identity(13,13)*0.1);

        // Barometer update

        if(baro1_ready && baro2_ready){
            z1 = bmp581.Altitude; 
            z2 = bmp581.Altitude;
            z_fused = z1*0.5 + z2*0.5; // PESOS!

            Eigen::MatrixXf P(13, 13); 
            P.setIdentity();  

            Eigen::MatrixXf R_baro(1,1); 
            R_baro = RR1.block(11, 11, 1, 1);

            // Adaptive R (variance update)
            count_baro++;
            RR1[pz_idx, pz_idx] = max(1e-9, RR1[pz_idx, pz_idx] + (z1 - x[pz_idx]) * (z1 - x[pz_idx]) / count_baro);
            RR2[pz_idx, pz_idx] = max(1e-9, RR2[pz_idx, pz_idx] + (z2 - x[pz_idx]) * (z2 - x[pz_idx]) / count_baro);
            R_baro = RR1*0.5 + RR2*0.5;   // depois PESOS!
            mekf.update_barometer(x, P, z_fused, R_baro, pz_idx, b_idx);
        }


        // Magnetometer update
        if(mag_ready){
            // Magnetic field in navigation frame
            Eigen::Vector3f mag_nav_t;
            Eigen::Vector3f mag_ref = mag_nav_t.normalized();  // reference in nav frame

            // Rotate into body frame
            Eigen::Matrix3f R_nb = q.toRotationMatrix();
            Eigen::Vector3f mag_body = R_nb.transpose() * mag_nav_t;

            count_mag++;
            for (int k = 0; k < 3; ++k) {  //  cicle for the three components
                R_mean_mag(k) = (R_mean_mag(k) * (count_mag - 1) + mag_body(k)) / count_mag;

                RR_mag(k, k) = std::max(1e-9f, RR_mag(k, k) + (mag_body(k) - R_mean_mag(k)) * (mag_body(k) - R_mean_mag(k)) / count_mag);
            }

            Eigen::Matrix3f RR_mag_diag = RR_mag.block<3, 3>(0, 0).diagonal().asDiagonal();
            mekf.update_mag(x, P, q, mag_body, mag_ref, RR_mag_diag);

            count_Q++;
            for (int k = 0; k < x.size(); ++k) {
                Q_mean(k) = (Q_mean(k) * (count_Q - 1) + x(k)) / count_Q;
                
                Q(k,k) = std::max(1e-9f, Q(k, k) + (x(k) - Q_mean(k)) * (x(k) - Q_mean(k)) / count_Q);
            }

            helper.apply_attitude_correction(x, q);
        }


        if(!parachute_opened1 && x(8) < 0 && x(8) > -1.5){   // x(8) == vz
            parachute_opened1 = true;
            //std::cout << "Paraquedas 1 aberto em t=" << t_real << "s\n";
        }

        if(!parachute_opened2 && std::abs(x(11)-450)<2){     // x(11) == z
            parachute_opened2 = true;
            //std::cout << "Paraquedas 2 aberto em t=" << t_real << "s\n";
        }
    }