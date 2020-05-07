#include "PID.h"
#include "IMU.h"
#include "MACROS.h"
#include "MotorController.h"

void PID::run_loop()
{

    time_current = millis();
    time_elapsed = time_current - time_prev;

    float *angle_;
    if (mode == RATE_MODE)
        angle_ = imu.updateRates();
    else if (mode == ANGLE_MODE)
        angle_ = imu.updateAngles();

    
    
    x.current = *angle_;
    y.current = *(angle_ + 1);
    z.current = *(angle_ + 2);
    

    x.error = x.current - x.setpoint;
    y.error = y.current - y.setpoint;
    z.error = z.current - z.setpoint;

    PIDCalc(x);
    PIDCalc(y);
    PIDCalc(z);

    if(mode == RATE_MODE){
        if(x.output > MAX_X_RATE_DELTA){
            x.output = MAX_X_RATE_DELTA;
        }
        if(y.output > MAX_Y_RATE_DELTA){
            y.output = MAX_Y_RATE_DELTA;
        }
        if(z.output > MAX_Y_RATE_DELTA){
            y.output = MAX_Y_RATE_DELTA;
        }
    }else if (mode == ANGLE_MODE){
         if(x.output > MAX_ROLL_DELTA){
            x.output = MAX_ROLL_DELTA;
        }
        if(y.output > MAX_PITCH_DELTA){
            y.output = MAX_PITCH_DELTA;
        }
        if(z.output > MAX_YAW_DELTA){
            z.output = MAX_YAW_DELTA;
        }
    }

    x.error_prev = x.error;
    y.error_prev = y.error;
    z.error_prev = z.error;
   
    int fl, fr, bl, br;
    fl = MotorController::throttle - y.output - x.output + z.output;
    fr = MotorController::throttle - y.output + x.output - z.output;
    bl = MotorController::throttle + y.output - x.output - z.output;
    br = MotorController::throttle + y.output + x.output + z.output;

    mapToRange(fl);
    mapToRange(fr);
    mapToRange(bl);
    mapToRange(br);

    motors.setMotors(fl, fr, bl, br);

    time_prev = time_current;
}

void PID::PIDCalc(PIDValues& axis)
{
    axis.p_error = axis.error * axis.p_gain;
    axis.output = axis.p_error;
    if(!axis.first_time){
        axis.i_error += (axis.error * time_elapsed) * axis.i_gain;
        if(axis.i_error > MotorController::max_out){
            axis.i_error = MotorController::max_out;
        }else if(axis.i_error < MotorController::max_out * -1){
            axis.i_error = MotorController::max_out * -1;
        }
    axis.d_error = ((axis.error - axis.error_prev) / time_elapsed) * axis.d_gain;
    axis.output += axis.d_error;
    axis.output += axis.i_error;
    }

}

void PID::mapToRange(int &out)
{
    if (out > MotorController::max_out)
    {
        out = MotorController::max_out;
    }
    else if (out < MotorController::min_out)
    {
        out = MotorController::min_out;
    }
    else if (out < MotorController::max_out * -1)
    {
        out = MotorController::max_out * -1;
    }
    else if (out < 0 && out > MotorController::min_out * -1)
    {
        out = MotorController::min_out * -1;
    }
    else
    {
        return;
    }
}

void PID::updateSetpoints(float* val){
    x.setpoint = *(val + 0);
    y.setpoint = *(val + 1);
    z.setpoint = *(val + 2);
}

PID::PID(IMU& imu, float* gains, MotorController& motorController, short mode)
{

    this->x.p_gain = *(gains + 0);
    this->x.d_gain = *(gains + 1);
    this->x.i_gain = *(gains + 2); 

    this->y.p_gain = *(gains + 3);
    this->y.d_gain = *(gains + 4);
    this->y.i_gain = *(gains + 5);

    this->z.p_gain = *(gains + 6);
    this->z.d_gain = *(gains + 7);
    this->z.i_gain = *(gains + 8);


    this->imu = imu;
    this->motors = motorController;
    this->mode = mode;

}