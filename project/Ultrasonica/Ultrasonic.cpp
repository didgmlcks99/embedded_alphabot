#include "Ultrasonic.h"
/* Constructor& Destructor
 * trigPin - attach ultrasonic wave trig pin (or shooting pin)
 * echoPin - attach ultrasonic wave echo pin (or reading pin)
 * tick - float value for repeat mode, default is 0.1.
 * repeat - if true, ultrasonic wave will be triggered and received at every *tick second.
 */
Ultrasonic::Ultrasonic(PinName trigPin, PinName echoPin, float tick, bool repeat)
:    _trig(trigPin), _echo(echoPin), _toVal(tick), _repeat(repeat)
{
    _done = 0;
    _timer.reset();
    _echo.rise(callback(this, &Ultrasonic::_startT));
    _echo.fall(callback(this, &Ultrasonic::_endT));
    if(_repeat) _ticker.attach(callback(this, &Ultrasonic::_ticker_cb), _toVal);
}

Ultrasonic::~Ultrasonic(){}

/*receive echo of ultrasonic wave*/
void Ultrasonic::_startT(void){
    _timer.start();
}
void Ultrasonic::_endT(void){
    _timer.stop();
    _pulseDuration = _timer.read_us();
    _distance = _pulseDuration/58;
    _timer.reset();
    _done = 1;
}

/*shoots ultrasonic wave*/
void Ultrasonic::trig(){
    _echo.enable_irq();
        wait_us(2);
        _trig = 1;
        wait_us(10);
        _trig = 0;    
}

/** The Callback function for ticker
 * Enabled on repeat mode
 * _ticker_cb is triggered every _toVal second.
 */
void Ultrasonic::_ticker_cb(void){
    this->trig();
}

/** getter methods */
int Ultrasonic::getDistance(void){
    return _distance;
}
int Ultrasonic::getPulseDuration(void){
    return _pulseDuration;    
}
int Ultrasonic::getStatus(){
    return _done;
}

/** setter Methods
 * clearStatus clears _done variable. this method should be called by user to acknowledge the library that user read the value
 * pauseMeasure is useful only when _repeat is true, it turns the _repeat to false
 * setMode is a method to manipulate _repeat to true or false
 * setTick method can change every time of each term in repeat mode
 */
void Ultrasonic::clearStatus(void){
    _done = 0;
    _distance = 0;
    _pulseDuration = 0;
}
void Ultrasonic::pauseMeasure(void){
    _repeat = false;
    _ticker.detach();
}
void Ultrasonic::setMode(bool mode){
    _repeat = mode;
    if(_repeat) _ticker.attach(callback(this, &Ultrasonic::_ticker_cb), _toVal);
    else _ticker.detach();
}
void Ultrasonic::setTick(float tick){
    if(tick>0) _toVal = tick;
    else _toVal = -tick;
}
