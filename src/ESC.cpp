#include "ESC.h"


float ESC::getCurrent(){
    return current;
}

int ESC::max_pulse = 2000;
int ESC::min_pulse = 1000;


void ESC::write_esc(int val){
    servo.write(val);
    current = val;
}

