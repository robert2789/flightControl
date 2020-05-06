#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "ESC.h"



class MotorController
{
private:
    ESC front_left;
    ESC front_right;
    ESC back_left;
    ESC back_right;
    
public:
    void setMotors(int, int, int, int);
    MotorController();
    ~MotorController();
    static int throttle;
    static int max_out;
    static int min_out;
};



#endif //MOTOR_CONTROLLER_H