#include "ESC.h"

void ESC::setProposed(float proposed){
    this->proposed = proposed;
}

float ESC::getProposed(){
    return proposed;
}   

float ESC::getCurrent(){
    return current;
}

void ESC::updateCurrent(){
    current = proposed;
}

int ESC::max_pulse = 2000;
int ESC::min_pulse = 1000;

int ESC::max_out = 1900;
int ESC::min_out = 1020;

void ESC::write_esc(int val){
    servo.write(val);
}

