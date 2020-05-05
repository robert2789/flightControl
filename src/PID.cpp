#include "PID.h"
#include "IMU.h"
#include "MACROS.h"

#define MAX_ESC_OUT 1900

void PID::run_loop()
{

    static float* angle_;

    if(mode == RATE_MODE){
        imu.updateRates();
        if(!first_time) imu.getRates();    
    }else if(mode == ANGLE_MODE){
        imu.updateAngles();
        if(!first_time) imu.getAngles();   
    }


    x_current = *(angle_);
    y_current = *(angle_ + 1);
    z_current = *(angle_ + 2);


    x_error = x_current - x_setpoint;
    y_error = y_current - y_setpoint;
    z_error = z_current - z_setpoint;

    p_error_x = x_error * gains.p_gain_x;
    x_output = p_error_x;
    if (!first_time)
    {
        i_error_x += (x_error * time_elapsed) * gains.i_gain_x;
        if (i_error_x > MAX_ESC_OUT)
        {
            i_error_x = MAX_ESC_OUT;
        }
        else if (i_error_x > MAX_ESC_OUT * -1)
        {
            i_error_x = MAX_ESC_OUT * -1;
        }
        d_error_x = ((x_error - x_error_prev) / time_elapsed) * gains.d_gain_x;
        x_output += d_error_x;
        x_output += i_error_x;
    }

    p_error_y = y_error * gains.p_gain_y;
    y_output = p_error_y;
    if (!first_time)
    {
        i_error_y += (y_error * time_elapsed) * gains.i_gain_y;
        if (i_error_y > MAX_ESC_OUT)
        {
            i_error_y = MAX_ESC_OUT;
        }
        else if (i_error_y > MAX_ESC_OUT * -1)
        {
            i_error_y = MAX_ESC_OUT * -1;
        }
        d_error_y = ((y_error - y_error_prev) / time_elapsed) * gains.d_gain_y;
        y_output += d_error_y;
        y_output += i_error_y;
    }

    p_error_z = z_error * gains.p_gain_z;
    z_output = p_error_z;
    if (!first_time)
    {
        i_error_z += (z_error * time_elapsed) * gains.i_gain_z;
        if (i_error_z > MAX_ESC_OUT)
        {
            i_error_z = MAX_ESC_OUT;
        }
        else if (i_error_z > MAX_ESC_OUT * -1)
        {
            i_error_z = MAX_ESC_OUT * -1;
        }
        d_error_z = ((z_error - z_error_prev) / time_elapsed) * gains.d_gain_z;
        z_output += d_error_z;
        z_output += i_error_z;
    }

    if(x_output > max_x_output){
        x_output = max_x_output;
    }
    if(y_output > max_y_output){
        y_output = max_y_output;
    }
    if(z_output > max_z_output){
        z_output = max_z_output;
    }

    x_error_prev = x_error;
    y_error_prev = y_error;
    z_error_prev = z_error;

    motors.


}



PID::PID(IMU& imu, short mode, float p_gain_x, float d_gain_x, float i_gain_x,
         float p_gain_y, float d_gain_y, float i_gain_y,
         float p_gain_z, float d_gain_z, float i_gain_z)
{

    this->p_gain_x = p_gain_x;
    this->d_gain_x = d_gain_x;
    this->i_gain_x = i_gain_x;

    this->p_gain_y = p_gain_y;
    this->d_gain_y = d_gain_y;
    this->i_gain_y = i_gain_y;

    this->p_gain_z = p_gain_z;
    this->d_gain_z = d_gain_z;
    this->i_gain_z = i_gain_z;

    this->mode = mode;
    this->imu = imu;



}

PID::PID(IMU& imu, Gains& gains, Motors& motors, short mode){
    this->imu = imu;
    this->gains = gains;
    this->motors = motors;
    this->mode = mode;

    if(mode == ANGLE_MODE){
        max_x_output = MAX_ROLL_DELTA;
        max_y_output = MAX_PITCH_DELTA;
        max_z_output = MAX_YAW_DELTA;
    }else if(mode == RATE_MODE){
        max_x_output = MAX_X_RATE_DELTA;
        max_y_output = MAX_Y_RATE_DELTA;
        max_z_output = MAX_Z_RATE_DELTA;
    }

}