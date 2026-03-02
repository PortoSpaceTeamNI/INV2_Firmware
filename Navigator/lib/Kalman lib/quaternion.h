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

            Q.qw = -B.qx * qx - B.qy * qy - B.qz * qz + B.qw * qw;
            Q.qx = B.qx * qw + B.qy * qz - B.qz * qy + B.qw * qx;
            Q.qy = -B.qx * qz + B.qy * qw + B.qz * qx + B.qw * qy;
            Q.qz = B.qx * qy - B.qy * qx + B.qz * qw + B.qw * qz;
            
            return Q;
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
Matrix3f quaternion_to_rotation_matrix(MyQuaternion q);

#endif