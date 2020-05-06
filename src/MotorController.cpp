#include "MotorController.h"

void MotorController::setMotors(int fl, int fr, int bl, int br){
    front_left.write_esc(fl);
    front_right.write_esc(fr);
    back_left.write_esc(bl);
    back_right.write_esc(br);
}

