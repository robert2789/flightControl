#if !defined(IMU_H)
#define IMU_H

#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 0
#define X_GYRO_OFFSET 159
#define Y_GYRO_OFFSET 34
#define Z_GYRO_OFFSET 31
#define Z_ACCEL_OFFSET 956

class IMU
{
private:
    MPU6050 mpu;
    bool dmpReady = false;   // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    float euler[3];      // [psi, theta, phi]    Euler angle container
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float gyro_rates[3];

public:
    IMU();
    ~IMU();

public:
    float* updateAngles();
    float* updateRates();

    void setupIMU();
    void setupGyro();
};

void dmpDataReady();

#endif // IMU_H
