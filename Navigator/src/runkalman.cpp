#include <ArduinoEigen.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "quaternion.h"
#include "func.h"
#include "Sensors.h"
#include "Display.h"

extern help_func helper;

// Indices
const int pz_idx = 14;
const int b_idx  = 15;

/*extern float imu_ax_local;
extern float imu_ay_local;
extern float imu_az_local;
extern float imu_gx_local;
extern float imu_gy_local;
extern float imu_gz_local;
extern float altitude_local;

extern float mag_x_local;
extern float mag_y_local;
extern float mag_z_local;*/

// State and covariance
Eigen::MatrixXf x(16,1);  // 16 rows, 1 column
Eigen::MatrixXf P = Eigen::MatrixXf::Identity(16,16); // Initial covariance
Eigen::MatrixXf P_pred = Eigen::MatrixXf::Identity(16,16);
Eigen::MatrixXf F = Eigen::MatrixXf::Identity(16,16); // discrete-time state transition Jacobian 

// Sensor noise matrices
float sigma_baro1 = 0.001f, sigma_baro2 = 0.001f, sigma_mag = 0.03f;
float sigma_acc1 = 3.0f, sigma_acc2 = 1.0f, sigma_gyro1 = 0.5f, sigma_gyro2 = 0.5f; 

Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(16,16); // Process noise covariance

void initKalmanMatrices() {
    // Attitude error (gyro noise)
    float sigma_theta = 0.5f;
    Q.block(0,0,3,3) = Eigen::Matrix3f::Identity() * sigma_theta * sigma_theta;

    // Acc bias
    Q(3,3) = sigma_acc2 * sigma_acc2;
    Q(4,4) = sigma_acc2 * sigma_acc2;
    Q(5,5) = sigma_acc1 * sigma_acc1;

    // Gyro bias
    Q.block(6,6,3,3) = Eigen::Matrix3f::Identity() * sigma_gyro1 * sigma_gyro1;

    //velocity
    Q(9,9) = sigma_acc2 * sigma_acc2;
    Q(10,10) = sigma_acc2 * sigma_acc2;
    Q(11,11) = sigma_acc1 * sigma_acc1;

    //position
    Q(12,12) = sigma_acc2 * sigma_acc2;
    Q(13,13) = sigma_acc2 * sigma_acc2;
    Q(14,14) = sigma_acc1 * sigma_acc1;

    // Baro bias
    Q(15,15) = sigma_baro1 * sigma_baro1;
}

Eigen::MatrixXf RR1 = Eigen::MatrixXf::Identity(16,16) * sigma_baro1 * sigma_baro1;   // baro1
Eigen::MatrixXf RR2 = Eigen::MatrixXf::Identity(16,16) * sigma_baro2 * sigma_baro2; // baro2
Eigen::Matrix3f RR_mag = Eigen::Matrix3f::Identity() * sigma_mag * sigma_mag; // mag
Eigen::Vector3f R_mean_mag = Eigen::Vector3f::Zero();
Eigen::VectorXf Q_mean = Eigen::VectorXf::Zero(16);
float mag_nav[3];  // Current navigation magnetic field
float gyro0[3] = {0, 0, 0};

//std::vector<Eigen::Vector3f> acc_buffer;
//std::vector<Eigen::Vector3f> gyro_buffer;


int count_mag = 0;
int count_Q   = 0;
float altitude_local = 0;

bool parachute_opened1 = false, parachute_opened2 = false;

int count_baro = 0;
float z1, z2;
//float Ts = 0.01; // 100Hz update rate
extern bool baro1_ready, baro2_ready, mag_ready, imu_ready;
static bool imu_calibrated = false;
float z_fused;
float acc_fused[3], gyro_fused[3];
float baro_offset = 0;
bool baro_offset_set = false;
bool gyro_offset_set = false;
float bias;
float gyro_offset[3] = {0, 0, 0};
static bool initialized = false;
Eigen::MatrixXf R_baro(1,1);


void runKalmanFilter(SensorDataResult *sensorData, Eigen::MatrixXf *x, uint32_t Ts_us) {
    
    /* Calculate filter coefficients
    const float fs_hz = 1.0f / (Ts_us * 1e-6f);
    const float cuttoff_frequency_alt = fs_hz / 4.0f;    
    const float betha_alt = 1.0f - expf(-Ts_us * 1e-6f * 2.0f * M_PI * cuttoff_frequency_alt); 

    const float cuttoff_frequency_imu = fs_hz / 4.0f; 
    const float betha_imu = 1.0f - expf(-Ts_us * 1e-6f * 2.0f * M_PI * cuttoff_frequency_imu);*/

    if (!imu_ready) {
        Serial.print(" IMU not ready! ");
        return;
    } else{
        if (!initialized) {
            *x << 0,0,0,       // δθ
                0,0,0,        // b_imu
                0,0,0,       // b_g
                0,0,0,      // v
                0,0,0,     // p
                0;        // bias_baro
            initialized = true;
        }

        acc_fused[0] = -sensorData->lsmData.AccelZ;
        acc_fused[1] = -sensorData->lsmData.AccelY;
        acc_fused[2] = sensorData->lsmData.AccelX;

        gyro_fused[0] = sensorData->lsmData.GyroZ;
        gyro_fused[1] = sensorData->lsmData.GyroY;
        gyro_fused[2] = sensorData->lsmData.GyroX;

        //Serial.print(" Q: "); Serial.print(Q(14,14), 5);
        predict(x, &P, acc_fused, gyro_fused, Ts_us, &Q, &F, &P_pred);
    }

    // Barometer update
    if(baro1_ready && baro2_ready) {

        z1 = sensorData->bmpData.Altitude + 6.0f; // offset for baro1
        //Serial.print(" Baro1: "); Serial.print(z1);
        z2 = sensorData->lpsData.Altitude;
        //Serial.print(" Baro2: "); Serial.print(z2);
        z_fused = z1*0.8 + z2*0.2; // PESOS!

        if (!baro_offset_set) {
            (*x)(pz_idx) = 0.0;
            bias = z_fused;
            //Serial.print(" offset : "); Serial.print(z_fused);
            //Serial.print(" altura: "); Serial.print(z1, z2);
            baro_offset_set = true;
        }

        if (z_fused - bias < 0) {
            z_fused = bias; // prevent negative altitude
        }

        z_fused -= bias; // TARAR o barómetro
        //Serial.print(" Baro fused: "); Serial.print(z_fused);

        // LOW PASS altitude
        //altitude_local += (z_fused - altitude_local) * betha_alt;

        Eigen::MatrixXf R_baro(1,1); 
        R_baro = RR1.block(14, 14, 1, 1);

        // Adaptive R (variance update)
        /*count_baro++;
        if (count_baro < 10) {
            RR1(pz_idx,pz_idx) = sigma_baro1*sigma_baro1;
            RR2(pz_idx,pz_idx) = sigma_baro2*sigma_baro2;
        } else {
            RR1(pz_idx,pz_idx) += (z1 - (*x)(pz_idx) - bias)*(z1 - (*x)(pz_idx) - bias)/count_baro;
            RR2(pz_idx,pz_idx) += (z2 - (*x)(pz_idx) - bias)*(z2 - (*x)(pz_idx) - bias)/count_baro;
            //Serial.print("RR1: "); Serial.print(RR1(pz_idx,pz_idx));
            //Serial.print(" RR2: "); Serial.print(RR2(pz_idx,pz_idx));
        }*/
        //R_baro(0, 0) = RR1(pz_idx, pz_idx)*0.5 + RR2(pz_idx, pz_idx)*0.5;   // depois PESOS!
        R_baro(0, 0) = sigma_baro1*sigma_baro1*0.5 + sigma_baro2*sigma_baro2*0.5;   // depois PESOS!
        //Serial.print(" R_baro: "); Serial.print(R_baro(0, 0), 5);
        
        update_barometer(x, &P, &z_fused, &R_baro, pz_idx, b_idx);
        //Serial.print(" estado: "); Serial.println((*x)(pz_idx));

    } else {
        //Serial.print(" Baros not ready! ");
        return;
    }

    //count_baro=0; // reset count after update

    // Magnetometer update
    /*if(mag_ready){
        // Magnetic field in navigation frame
        float mag_ref[3] = {mag_nav[0]/sqrt(mag_nav[0]*mag_nav[0] + mag_nav[1]*mag_nav[1] + mag_nav[2]*mag_nav[2]), 
                            mag_nav[1]/sqrt(mag_nav[0]*mag_nav[0] + mag_nav[1]*mag_nav[1] + mag_nav[2]*mag_nav[2]), 
                            mag_nav[2]/sqrt(mag_nav[0]*mag_nav[0] + mag_nav[1]*mag_nav[1] + mag_nav[2]*mag_nav[2])};  // reference in nav frame

        // Rotate into body frame
        // Assuming q is your float array and quaternion_to_rotation_matrix expects MyQuaternion
        MyQuaternion quat = floatArrayToQuaternion(q);
        Eigen::Matrix3f R_nb = quaternion_to_rotation_matrix(quat);

        float mag_body[3] = {R_nb.transpose()(0,0) * mag_nav[0] + R_nb.transpose()(0,1) * mag_nav[1] + R_nb.transpose()(0,2) * mag_nav[2],  // measured in body frame
                                R_nb.transpose()(1,0) * mag_nav[0] + R_nb.transpose()(1,1) * mag_nav[1] + R_nb.transpose()(1,2) * mag_nav[2],
                                R_nb.transpose()(2,0) * mag_nav[0] + R_nb.transpose()(2,1) * mag_nav[1] + R_nb.transpose()(2,2) * mag_nav[2]};

        count_mag++;
        for (int k = 0; k < 3; ++k) {  //  cicle for the three components
            R_mean_mag(k) = (R_mean_mag(k) * (count_mag - 1) + mag_body[k]) / count_mag;

            RR_mag(k, k) = std::max(1e-9f, RR_mag(k, k) + (mag_body[k] - R_mean_mag(k)) * (mag_body[k] - R_mean_mag(k)) / count_mag);
        }

        Eigen::MatrixXf RR_mag_diag = RR_mag.block<3, 3>(0, 0).diagonal().asDiagonal();
        update_mag(x, &P, q, mag_body, mag_ref, &RR_mag_diag);

        count_Q++;
        for (int k = 0; k < (*x).size(); ++k) {
            Q_mean(k) = (Q_mean(k) * (count_Q - 1) + (*x)(k)) / count_Q;
            
            Q(k,k) = std::max(1e-9f, Q(k, k) + ((*x)(k) - Q_mean(k)) * ((*x)(k) - Q_mean(k)) / count_Q);
        }

        helper.apply_attitude_correction(x, q);
    }*/


    if(!parachute_opened1 && (*x)(11) < 0 && (*x)(11) > -1.5){   // x(14) == vz
        parachute_opened1 = true;
        //std::cout << "Paraquedas 1 aberto em t=" << t_real << "s\n";
    }

    if(!parachute_opened2 && std::abs((*x)(11)-450)<2){     // x(17) == z
        parachute_opened2 = true;
        //std::cout << "Paraquedas 2 aberto em t=" << t_real << "s\n";
    }
    

    imu_ready = false;
    baro1_ready = false;
    baro2_ready = false;
    mag_ready = false;

    /*Serial.print(" fusedx = "); Serial.print(acc_fused[0]);
    Serial.print(" fusedy = "); Serial.print(acc_fused[1]);
    Serial.print(" fusedz = "); Serial.print(acc_fused[2]);*/
    //Serial.print(" acc_norm = "); Serial.print(acc_norm);
}