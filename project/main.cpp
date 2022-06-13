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

int R_PWM;  
int L_PWM;
int weight; int P_weight;
int P_temp; int Fix_temp;
int print_c;
int den;
static int on_line[1];
//typedef enum{LEFT = 0, RIGHT} DIRE;
//DIRE direction;
//DIRE Over_direction;
//DIRE P_direction;
//DIRE Obs_direction;
/***********************************/
/**********PID********************/
float Kp = 0.08;
float Ki = 0.0006;
float Kd = 10;

int threshold = 300;
int Lweight = 0, Rweight = 0;
enum MotorState {
    Middle, Left, Right
};

MotorState curState = Middle;
MotorState prevState = Middle;
int prev_Pd = 0;

const int maximum = 255;
int last_proportional = 0;
int integral = 0;

float maxspeeda = 150;
float maxspeedb = 150;
float basespeeda = 110;
float basespeedb = 110;

/**********functions********************/
void Motor_init();
void Motor_Stop();
void PID_control(unsigned int *sensor_values);

int main() {
    printf("\r\n\nStart!!\t\n\n");
    
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
    }
}

void PID_control(unsigned int *sensor_values) {
    TR.calibrate();
    uint16_t position = TR.readLine(sensor_values); //ir1, ir2, ir3, ir4, ir5에 값이 들어감
    
    if(position == 0 || position == 4000) {
        if(prevState == Left) {
            Rweight = -1 * basespeeda + 10;
            //Rweight = 10;
        }
        else {
            //Lweight = 10;
            Lweight = -1 * basespeedb + 10;
        }
        PWMA = (basespeeda + Lweight) / 150.0 ;
        PWMB = (basespeedb + Rweight) / 150.0 ;
        printf("\tOut! position: %d\t\n", position);
        return;
    }     
    
    int proportional = (int)position - 2000;
    int derivative = proportional - last_proportional;
    integral += proportional;
    last_proportional = proportional;
    int power_difference = proportional * Kp + integral * Ki + derivative * Kd;
    
    if (power_difference > maximum)
        power_difference = maximum;
    if (power_difference < -maximum)
        power_difference = -maximum;
    
    printf("position: %d proportion: %d last_proportional: %d pd: %d\r\n", position, proportional, last_proportional, power_difference);
    
    // Left
    if (power_difference < 0)
    {
        curState = Left;
        if(prevState == Right) {
            Rweight = 8;
            Lweight = 3;
            // printf("Right -> Left Turn\r\n");
        }
        PWMA = ((basespeeda + (float)power_difference / maximum * basespeeda) + Lweight) / 150.0;
        PWMB = (basespeedb + Rweight) / 150.0 ;
        // printf("Turn Left: position: %d proportion: %d last_proportional: %d pd: %d\r\n", position, proportional, last_proportional, power_difference);
        prevState = Left;
    }
    // Right
    else
    {
        curState = Right;
        if(prevState == Left) {
            Lweight = 8;
            Rweight = 3;
            //printf("Left -> Right Turn\r\n");
        }
        PWMA = (basespeeda + Lweight) / 150.0 ;
        PWMB = ((basespeedb - (float)power_difference / maximum * basespeedb) + Rweight) / 150.0;
        //printf("\tTurn Right: position: %d proportion: %d last_proportional: %d pd: %d\r\n", position, proportional, last_proportional, power_difference);
        prevState = Right;
    }      
    Lweight = 0; Rweight = 0;
    
    ThisThread::sleep_for(10ms);
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
    
    // den = 0;
    // R_PWM = 0;
    // L_PWM = 0;
    // weight= 0;
    // print_c = 0;
}