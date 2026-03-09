#ifndef LSM6DSO_H
#define LSM6DSO_H

#define LSM6DSO_ADDR 0x6A

struct LSM6DSODataResult
{
    /// @brief Temperature in Celcius
    float Temperature = 0.0f;

    /// @brief Acceleration in m/s^2
    float AccelX = 0.0f;
    float AccelY = 0.0f;
    float AccelZ = 0.0f;

    /// @brief Gyro in rad/s
    float GyroX = 0.0f;
    float GyroY = 0.0f;
    float GyroZ = 0.0f;
};

int InitializeLSM6DSO();
int ConfigureLSM6DSO();
int ReadLSM6DSO(LSM6DSODataResult& result);
bool IsLSM6DSOReady();

#endif