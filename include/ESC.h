#ifndef ESC_H
#define ESCh_H


class ESC
{
private:
    float current = 0;
    float proposed;
    float* current_;
public:


    void setProposed(float);
    float getProposed();

    float getCurrent();

    void updateCurrent();

   
  
    ESC(/* args */);
    ~ESC();
};



#endif //ESC_H