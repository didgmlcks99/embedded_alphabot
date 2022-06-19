#ifndef MBED_ULTRASONIC_H
#define MBED_ULTRASONIC_H

#include "mbed.h"
/*header file Ultrasonic module HC-SR04*/
class Ultrasonic {
    public:
        Ultrasonic(PinName trigPin, PinName echoPin, float tick = 0.1, bool repeat = false);
        ~Ultrasonic();
        
        void trig();
        int getDistance(void);
        int getPulseDuration(void);
        int getStatus(void);
        
        void clearStatus(void);
        void pauseMeasure(void);
        void setMode(bool mode);
        
        void setTick(float tick);
    private:   
        DigitalOut _trig;
        InterruptIn _echo;
        Timer _timer;
        Ticker _ticker;
        
        float _toVal;
        bool _repeat;
        int _done;
        
        int _distance;
        int _pulseDuration;
        
        void _startT(void);
        void _endT(void);
        void _ticker_cb(void);
};

#endif MBED_ULTRASONIC_H