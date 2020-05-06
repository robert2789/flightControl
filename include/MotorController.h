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
};



#endif //MOTOR_CONTROLLER_H