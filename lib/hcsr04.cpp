#include "hcsr04.h"
#include "mbed.h"
/*
*HCSR04.cpp
*/
HCSR04::HCSR04(PinName t, PinName e, RawSerial pc,int flag, int limit) : trig(t), echo(e), _pc(pc), flag(flag), _limit(limit) {}
 
void HCSR04::Trig()
{
    timer.reset();  //reset timer
    trig=0;   // trigger low 
    wait_us(2); //  wait 
    trig=1;   //  trigger high
    wait_us(10);
    trig=0;  // trigger low
}

/*
void HCSR04::setMode(bool mode)
{
    _repeat = mode;
}

void HCSR04::clearStatus()
{
    _done = 0;
}

int HCSR04::getStatus()
{
    return _done;
}*/



 long HCSR04::echo_duration() {
        
    Trig();
         while(!echo); // start pulseIN
      timer.start();
     while(echo);
      timer.stop();
     return timer.read_us(); 
 
}
 
//return distance in cm 
void HCSR04::distance(){
   // _done++;
    duration = echo_duration();
    distance_cm = int((duration/2)/29.1) ;
    _pc.printf("distance:     %d\r\n", distance_cm);
    if(distance_cm <= _limit){
        flag = 1;
        _pc.printf("\t\tFlag on distance:     %d\r\n", distance_cm);
        
    }
}

long HCSR04::returndistance()
{
    return distance_cm;
}