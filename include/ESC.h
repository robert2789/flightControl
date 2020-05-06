#ifndef ESC_H
#define ESCh_H

#include <Servo.h>

class ESC
{
private:
    float current = 0;

    Servo servo;

public:
    void write_esc(int);
    float getCurrent();

public:
    static int max_pulse;
    static int min_pulse;
public:
    ESC(/* args */);
    ~ESC();
};

#endif //ESC_H