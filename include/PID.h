#ifndef PID_H
#define PID_H

#include "IMU.h"
#include <Servo.h>
#include "Gains.h"
#include "Motors.h"

class PID
{

public: //constructors


    PID(IMU&, Gains&, Motors&, short)
    
    PID( IMU&, Servo&, Servo&, Servo&, Servo&, short, float, float, float, float,float, float, float, float, float);
    ~PID();

public: //main methods
    void run_loop();


private:


    Gains gains;
    Motors motors;
    IMU imu;
    short mode;

    float p_gain_x = 0;
    float d_gain_x = 0;
    float i_gain_x = 0;

    float p_gain_y = 0;
    float d_gain_y = 0;
    float i_gain_y = 0;

    float p_gain_z = 0;
    float d_gain_z = 0;
    float i_gain_z = 0;

    float x_setpoint = 0;
    float y_setpoint = 0;
    float z_setpoint = 0;

    float x_error = 0;
    float x_error_prev = 0;
    float x_current = 0;
    float y_error = 0;
    float y_error_prev = 0;
    float y_current = 0;
    float z_error = 0;
    float z_error_prev = 0;
    float z_current = 0;

    float d_error_x = 0;
    float p_error_x = 0;
    float i_error_x = 0;
    float d_error_y = 0;
    float p_error_y = 0;
    float i_error_y = 0;
    float d_error_z = 0;
    float p_error_z = 0;
    float i_error_z = 0;

    float x_output = 0;
    float y_output = 0;
    float z_output = 0;

    int max_x_output;
    int max_z_output;
    int max_y_output;


    float* x_output_ = &x_output;
    float* y_output_ = &y_output;
    float* z_output_ = &z_output;

    unsigned long time_prev = 0;
    unsigned long time_elapsed = 0;
    bool first_time = true;
};

#endif //PID_H