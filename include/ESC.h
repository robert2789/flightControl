#ifndef ESC_H
#define ESCh_H

#include <Servo.h>


class ESC
{
private:
    float current = 0;
    float proposed;
    float* current_;
    Servo servo;

public:

    void write_esc(int);

    void setProposed(float);
    float getProposed();

    float getCurrent();

    void updateCurrent();

   
    static int max_pulse;
    static int min_pulse;

    static int max_out;
    static int min_out;

  
    ESC(/* args */);
    ~ESC();
};



#endif //ESC_H