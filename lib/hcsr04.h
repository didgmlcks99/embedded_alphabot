#ifndef hcsr04_H
#define hcsr04_H
#include "mbed.h"


 
class HCSR04 {
  public:
    HCSR04(PinName t, PinName e, RawSerial pc, int flag, int limit);
    long echo_duration();
    void distance();
    void Trig();
    long returndistance();
    
    /*void setMode(bool mode);
    int getStatus();
    void clearStatus();*/
    int flag;
 
    private:
        /*bool _repeat;
        float _interval;
        int _done;*/
        DigitalOut trig;
        DigitalIn echo;
        RawSerial _pc;
        Timer timer;
        long duration; 
        int distance_cm;
        int _limit;
};
 
#endif