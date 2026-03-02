#include <quaternion.h>

MyQuaternion euler_to_quaternion(float roll, float pitch, float yaw){

    MyQuaternion Q;

    Q.qx = isin(roll/2) * icos(pitch/2) * icos(yaw/2) - icos(roll/2) * isin(pitch/2) * isin(yaw/2);
    Q.qy = icos(roll/2) * isin(pitch/2) * icos(yaw/2) + isin(roll/2) * icos(pitch/2) * isin(yaw/2);
    Q.qz = icos(roll/2) * icos(pitch/2) * isin(yaw/2) - isin(roll/2) * isin(pitch/2) * icos(yaw/2);
    Q.qw = icos(roll/2) * icos(pitch/2) * icos(yaw/2) + isin(roll/2) * isin(pitch/2) * isin(yaw/2);

    return Q;
}

int quaternion_to_euler(MyQuaternion Q,  float *euler){


        float t0,t1,t2,t3,t4;


        t0 = +2.0 * (Q.qw * Q.qx + Q.qy * Q.qz);
        t1 = +1.0 - 2.0 * (Q.qx * Q.qx + Q.qy * Q.qy);
        euler[0] = atan2Fast(t0, t1) * 180 / M_PI;

        t2 = +2.0 * (Q.qw * Q.qy - Q.qz * Q.qx);
        if(abs(t2) > 1.0)
            t2 = 1.0*t2/abs(t2);
        
        euler[1] = iasin(t2);

        t3 = +2.0 * (Q.qw * Q.qz + Q.qx * Q.qy);
        t4 = +1.0 - 2.0 * (Q.qy * Q.qy + Q.qz * Q.qz);
        euler[2] = atan2Fast(t3, t4) * 180 / M_PI;

        //Serial.printf("%f %f %f\n", euler[0], euler[1], euler[2]);

        return 1;
}


Matrix3f quaternion_to_rotation_matrix(MyQuaternion q){
    
    Matrix3f R;
    R << 1 - 2*(q.qy*q.qy + q.qz*q.qz),     2*(q.qx*q.qy - q.qz*q.qw),     2*(q.qx*q.qz + q.qy*q.qw),
             2*(q.qx*q.qy + q.qz*q.qw), 1 - 2*(q.qx*q.qx + q.qz*q.qz),     2*(q.qy*q.qz - q.qx*q.qw),
             2*(q.qx*q.qz - q.qy*q.qw),     2*(q.qy*q.qz + q.qx*q.qw), 1 - 2*(q.qx*q.qx + q.qy*q.qy);
    return R;
}