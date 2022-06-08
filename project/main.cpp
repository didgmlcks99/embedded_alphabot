#include "mbed.h"
#include "TRSensors.h"
#include "PID.h"

UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 230400);

/**********tracker sensor***********/
TRSensors TR(D11, D12, D13, D10);
/***********************************/
/**********PID**********************/
#define RATE 0.1
 
//Kc, Ti, Td, interval
PID controller(1.0, 0.0, 0.0, RATE);
/**********Motor********************/
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
/***********************************/

void Motor_init();
void Motor_Stop();

int main() {
    int result = 0;
    Motor_init();
    
    PWMR.pulsewidth_ms(100);
    PWML.pulsewidth_ms(100);
    PWMR = 0.2; PWML = 0.2;
    
    unsigned int sensor_values[5] = {0};
    
    ThisThread::sleep_for(3000ms);
    while(1) {
        TR.calibrate();
        result = TR.readLine(sensor_values, 1);
        printf("result = %d, sensor values: %u %u %u %u %u\r\n", 
            result, sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4]);
        ThisThread::sleep_for(1000ms);
    }
}


/********************Motor********************/
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