#ifndef PID_H
#define PID_H

#include "IMU.h"
#include <Servo.h>
#include "MotorController.h"
#include "PIDValues.h"

class PID
{

public: //constructors
    PID(IMU &, Gains &, MotorController &, short)
    ~PID();

public: //main methods
    void run_loop();

private: //helper methods
    void mapToRange(int &);
    void PIDCalc(PIDValues&);

private:
    Gains gains;
    MotorController motors;
    IMU imu;
    short mode;

    PIDValues x = {0, 0, 0,
                   0, 0,
                   0, 0,
                   0, 0, 0,
                   0, false};

    PIDValues y = {0, 0, 0,
                   0, 0,
                   0, 0,
                   0, 0, 0,
                   0, false};

    PIDValues z = {0, 0, 0,
                   0, 0,
                   0, 0,
                   0, 0, 0,
                   0, false};

  

    unsigned long time_current = 0;
    unsigned long time_prev = 0;
    unsigned long time_elapsed = 0;
    bool first_time = true;
};

#endif //PID_H