#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>

struct motors
{

Servo front_left;
Servo front_right;
Servo back_left;
Servo back_right;

};
typedef struct motors Motors;


#endif