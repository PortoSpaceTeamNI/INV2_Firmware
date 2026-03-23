#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include <FastTrig.h>
#include <ArduinoEigen.h>

using namespace Eigen;

struct MyQuaternion{
        float qw,qx,qy,qz;

        /// @brief Quaternion multiplication
        /// @param B quaternion to be multiplied with
        /// @return resulting quaternion
        inline MyQuaternion operator*(MyQuaternion B){
            MyQuaternion Q;

            Q.qw = qw*B.qw - qx*B.qx - qy*B.qy - qz*B.qz;
            Q.qx = qw*B.qx + qx*B.qw + qy*B.qz - qz*B.qy;
            Q.qy = qw*B.qy - qx*B.qz + qy*B.qw + qz*B.qx;
            Q.qz = qw*B.qz + qx*B.qy - qy*B.qx + qz*B.qw;
            
            return Q;
        }

        /// @brief Normalize the quaternion to unit length
        inline void normalize(){
            float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
                qw /= norm;
                qx /= norm;
                qy /= norm;
                qz /= norm;
        }
};

/// @brief Transforms euler angles into quaternion representation
/// @param roll angle in x axis
/// @param pitch angle in y axis
/// @param yaw angle in z axis
/// @return quaternion value
MyQuaternion euler_to_quaternion(float roll, float pitch, float yaw);

/// @brief Transforms quaternion to euler angles
/// @param Q quaternion being transformed
/// @param euler 3 value vector where angles are returned
/// @return 3 value array with roll pitch yaw respectivelly
int quaternion_to_euler(MyQuaternion Q,  float *euler);


/// @brief Transform orientation quaternion to rotation matrix
/// @param q quaternion in question
/// @return rotation matrix
Eigen::Matrix3f quaternion_to_rotation_matrix(MyQuaternion q);

/// @brief Small angle approximation for quaternion generation
/// @param delta_theta 3 value array with small angle changes in roll, pitch and yaw
MyQuaternion small_angle_quat(float delta_theta[3]);

/// @brief Convert a float array of size 4 to MyQuaternion struct
/// @param q float array with quaternion values in order w, x, y, z
MyQuaternion floatArrayToQuaternion(const float q[4]);

#endif