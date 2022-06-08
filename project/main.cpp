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
// PwmOut PWMA(D6);  //PB_10,D6
// PwmOut PWMB(D5);

// left motor
DigitalOut AIN1(A1);
DigitalOut AIN2(A0);
PwmOut PWMA(D6);

// right motoer
DigitalOut BIN1(A2);
DigitalOut BIN2(A3);
PwmOut PWMB(D5);

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

// ******************희찬이가 만든 pid 변수 선언
//시작

float Kp = 0.9;
float Ki = 0.0008;
float Kd = 0.6;
int P, I, D;

int lastError = 0;

float maxspeeda = 150;
float maxspeedb = 150;
float basespeeda = 50;
float basespeedb = 50;

void PID_control(unsigned int *sensor_values);

// ******************희찬이가 만든 pid 
// 끝


void Motor_init();
void Motor_Stop();

int main() {
    int result = 0;
    Motor_init();
    
    unsigned int sensor_values[5] = {0};
    
    ThisThread::sleep_for(3000ms);
    
    printf("Start alphabot!\r\n");

    PWMA.pulsewidth_us(500);
    PWMB.pulsewidth_us(500);

    PWMA = basespeeda / 200.0; 
    PWMB = basespeedb / 200.0;
    
    while(1) {
        PID_control(sensor_values);
        // TR.calibrate();
        // result = TR.readLine(sensor_values, 1);
        // printf("speed = %f, result = %d  , sensor values: %u %u %u %u %u\r\n", 
        //     (float)(basespeeda / maxspeeda), result, sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4]);
        // ThisThread::sleep_for(100ms);
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
    AIN1 = 1;
    AIN2 = 0;
    BIN1 = 1;
    BIN2 = 0;

    PWMA.period_us(500);
    PWMB.period_us(500);

    I = 0;
    
    // den = 0;
    // R_PWM = 0;
    // L_PWM = 0;
    // weight= 0;
    // print_c = 0;
}

void PID_control(unsigned int *sensor_values){
    TR.calibrate();
    uint16_t position = TR.readLine(sensor_values, 1);
    int error = 2000 - position;

    P = error;
    I = I + error;
    D = error - lastError;
    lastError = error;
    int motorSpeed = P*Kp + I*Ki + D*Kd;

    int motorspeeda = basespeeda + motorSpeed;
    int motorspeedb = basespeedb - motorSpeed;

    if (motorspeeda > maxspeeda) {
        motorspeeda = maxspeeda;
    }
    if (motorspeedb > maxspeedb) {
        motorspeedb = maxspeedb;
    }
    if (motorspeeda < 0) {
        motorspeeda = 0;
    }
    if (motorspeedb < 0) {
        motorspeedb = 0;
    }
    
    PWMA = motorspeeda / 200.0;
    PWMB = motorspeedb / 200.0;
    
    printf("error: %d, speedA: %f, speedB: %f, position: %d\r\n", 
        error, (motorspeeda / maxspeeda), (motorspeedb / maxspeedb), position);
}