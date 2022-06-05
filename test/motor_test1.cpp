/*
    Motor Test
    10, 76 line 주석을 해제하면 TeraTerm에서 motor의 현재 값 (pwm.read()) 확인할 수 있음
*/
#include "mbed.h"

#define SERVER_IP "192.168.137.1"
#define SERVER_PORT 50000

//UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);
BufferedSerial *esp = new BufferedSerial(D8, D2, 115200);
ATCmdParser *at = new ATCmdParser(esp, "\r\n");

char buf[80];

//==============Motor===============
PwmOut PWML(D6);  //PB_10,D6
PwmOut PWMR(D5);

DigitalOut LIN1(A0); // left motor backward
DigitalOut LIN2(A1); // left motor forward
DigitalOut RIN3(A2); // right motor backward
DigitalOut RIN4(A3); // right motor forward

#define MIDDLE 2000
int R_PWM;  
int L_PWM;
int weight; int P_weight;
int P_temp; int Fix_temp;
int print_c;
int den;
static int on_line[1];
typedef enum{LEFT = 0, RIGHT} DIRE;
DIRE direction;
DIRE Over_direction;
DIRE P_direction;
DIRE Obs_direction;
//==================================

void Motor_init(){
    LIN1 = 0;
    LIN2 = 1;
    RIN3 = 1;
    RIN4 = 0;
    PWML.period_ms(100);
    PWMR.period_ms(100);
    
    den = 0;
    R_PWM = 0;
    L_PWM = 0;
    weight= 0;
    print_c = 0;
}

void Motor_Stop(){
    //printf("===============================Motor Stop!\r\n");
    LIN1 = 0;
    LIN2 = 0;
    RIN3 = 0;
    RIN4 = 0;
    PWML.pulsewidth_ms(0);
    PWMR.pulsewidth_ms(0);
}



int main(){
    Motor_init();

    PWMR.pulsewidth_ms(100);
    PWML.pulsewidth_ms(100);
    
    while(true) {
        float lmotor = PWML.read();
        float rmotor = PWMR.read();
        //printf("Left motor: %f, Right motor: %f\r\n", lmotor, rmotor);
        ThisThread::sleep_for(3000ms);
    }
}
    