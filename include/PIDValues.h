#ifndef PID_VALUES_H
#define PID_VALUES_H

struct values
{

    float p_gain;
    float d_gain;
    float i_gain;

    float setpoint;
    float current;

    float error;
    float error_prev;

    float p_error;
    float d_error;
    float i_error;

    float output;

    bool first_time;
};
typedef struct values PIDValues;

#endif