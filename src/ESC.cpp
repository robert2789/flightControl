#include "ESC.h"

void ESC::setCurrentPtr(float* ptr){
    this->current_ = ptr;
}

void ESC::updateCurrent(){
    current = *current_;
}