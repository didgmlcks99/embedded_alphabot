#include "mbed.h"
#include "TRSensors.h"
//#include "PID.h"

UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 230400);

/**********tracker sensor***********/
TRSensors TR(D11, D12, D13, D10);
/***********************************/
/**********PID**********************/
#define RATE 0.1
 
//Kc, Ti, Td, interval
//PID controller(1.0, 0.0, 0.0, RATE);
/**********Motor********************/
PwmOut PWMA(D6);  //PB_10,D6
PwmOut PWMB(D5);

DigitalOut AIN1(A1);
DigitalOut AIN2(A0);
DigitalOut BIN1(A2);
DigitalOut BIN2(A3);

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
    
    unsigned int sensor_values[5] = {0};
    
    ThisThread::sleep_for(3000ms);
    
    printf("Start alphabot!\r\n");
    
    while(1) {
        TR.calibrate();
        result = TR.readLine(sensor_values, 1);
        printf("result = %d, sensor values: %u %u %u %u %u\r\n", 
            result, sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4]);
        ThisThread::sleep_for(100ms);
    }
}


/********************Motor********************/
void Motor_Stop(){
    printf("===============================Motor Stop!\r\n");
    AIN1 = 0;
    AIN2 = 0;
    BIN1 = 0;
    BIN2 = 0;
    PWMB.pulsewidth_us(0);
    PWMA.pulsewidth_us(0);
}

void Motor_init(){
    AIN1 = 0;
    AIN2 = 1;
    BIN1 = 0;
    BIN2 = 1;
    PWMB.period_us(500);
    PWMA.period_us(500);
    
    den = 0;
    R_PWM = 0;
    L_PWM = 0;
    weight= 0;
    print_c = 0;

    
}