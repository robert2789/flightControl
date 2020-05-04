#ifndef ESC_H
#define ESCh_H


class ESC
{
private:
    float current = 0;
    float* current_;
public:

    void setCurrentPtr(float*);
    void updateCurrent();

  
    ESC(/* args */);
    ~ESC();
};



#endif //ESC_H