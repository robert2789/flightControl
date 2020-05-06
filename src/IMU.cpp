#include "IMU.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"
#include <iostream>



volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

float* IMU::updateAngles()
{
    if (!dmpReady)
        return false;

    while (!mpuInterrupt && fifoCount < packetSize)
    {
        if (mpuInterrupt && fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize)
    {
    }
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        mpu.resetFIFO();
        Serial.println("FIFO overflow");
    }
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        while (fifoCount >= packetSize)
        {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    return ypr;
}


float* IMU::updateRates()
{
    int16_t gyroX, gyroY, gyroZ;

    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x43);                  //Starting register for Gyro Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
    while (Wire.available() < 6)
        ;

    byte lowByte, highByte;

    highByte = Wire.read();
    lowByte = Wire.read();
    gyroX = (highByte << 8 | lowByte);
    highByte = Wire.read();
    lowByte = Wire.read();
    gyroY = (highByte << 8 | lowByte);
    highByte = Wire.read();
    lowByte = Wire.read();
    gyroZ = (highByte << 8 | lowByte);

    gyro_rates[0] = gyroZ / 131;
    gyro_rates[1] = gyroY / 131;
    gyro_rates[2] = gyroX / 131;

    return gyro_rates;
}

void dmpDataReady()
{
    mpuInterrupt = true;
}

void IMU::setupIMU()
{
    Wire.setSDA(18);
    Wire.setSCL(19);
    Wire.begin();
    Wire.begin();
    Wire.setClock(400000); // 400dHz I2C clocd. Comment this line if having compilation difficulties

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    mpu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(X_GYRO_OFFSET);
    mpu.setYGyroOffset(Y_GYRO_OFFSET);
    mpu.setZGyroOffset(Z_GYRO_OFFSET);
    mpu.setZAccelOffset(Z_ACCEL_OFFSET);

    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print("error");
    }
}



void IMU::setupGyro(){

    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.write(0x6B);                  //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(0b00000000);            //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.endTransmission();
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x1B);                  //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
    Wire.write(0x00000000);            //Setting the gyro to full scale +/- 250deg./s
    Wire.endTransmission();
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x1C);                  //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
    Wire.write(0b00000000);            //Setting the accel to +/- 2g
    Wire.endTransmission();
}


IMU::IMU()
{
    Serial.println("IMU created");
}

IMU::~IMU()
{
    Serial.println("IMU destroyed");
}