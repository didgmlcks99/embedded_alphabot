#include "mbed.h"
#include "TRSensors.h"
#include "RemoteIR.h"
#include "ReceiverIR.h"
#include "Ultrasonic.h"

UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 230400);

/**********tracker sensor***********/
TRSensors TR(D11, D12, D13, D10);
/********************************FF***/
/**********PID**********************/
#define RATE 0.1
#define numSensors 5

/**********Motor********************/
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
/***********************************/
/**********PID********************/
float Kp = 0.08;
float Ki = 0.0009;
float Kd = 10;

//int threshold = 300;

int Lweight = 0, Rweight = 0;
enum MotorState {
    Middle, Left, Right
};

int startFlag = 0;
uint16_t prev_sensor_values[5] = {0};
MotorState curState = Middle;
MotorState prevState = Middle;
int prev_Pd = 0;

const int maximum = 255;
int last_proportional = 0;
int integral = 0;

float maxspeeda = 150;
float maxspeedb = 150;
float basespeeda = 130;
float basespeedb = 130;

/**********IR********************/
float rightvar = 0.013;
float speed = 0.3;
uint8_t buf[8];
bool calibration = true;

RemoteIR::Format format = RemoteIR::NEC;
ReceiverIR irrecv(D4);

/**********Ultrasonic********************/
Ultrasonic ultra(D3,D7);
Ticker ticker;
Thread ultra_thread;

/**********functions********************/
void Motor_init();
void Motor_Stop();
void PID_control(unsigned int *sensor_values);
void tick();
void REMOTE_control(unsigned int *sensor_values);

void tick(){
    ultra.trig();
}

void rx_thread() {
    while(true) {
        ultra.trig();
        if(ultra.getStatus() == 1) {
            int distance = ultra.getDistance();
            //printf("ultra getstatus = 1, distance = %d\t\n", distance);
            
            if(distance < 30){
                PWMA = 0.0;
                PWMB = 0.0;
            }
        }
        ultra.clearStatus();
    }
}

int main() {
    printf("\r\n\nAlphabot on!!\t\n\n");
    
    int result = 0;
    Motor_init();
    
    unsigned int sensor_values[5] = {0};
    
    ThisThread::sleep_for(3000ms);
    
    printf("Start alphabot!\r\n");

    
    REMOTE_control(sensor_values);

    PWMA.pulsewidth_us(500);
    PWMB.pulsewidth_us(500);

    PWMA = basespeeda / 200.0; 
    PWMB = basespeedb / 200.0;
    
    printf("start line tracing!\r\n");
    float distance = 0.0;
    
    // start ultrasonic
    ultra_thread.start(&rx_thread);
    
    AIN1 = 1;
    AIN2 = 0;
    BIN1 = 1;
    BIN2 = 0;

    
    while(1) {
        PID_control(sensor_values);
    }
    ultra_thread.join();
}


/********************PID********************/
void PID_control(unsigned int *sensor_values) {
    int changeFlag = 0;
    
    TR.calibrate();
    uint16_t position = TR.readLine(sensor_values);
    
    printf("sensor values: %d %d %d %d %d\r\n", sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4]);
    
    if(position == 0 || position == 4000) {
        if(prevState == Left) {
            Rweight = -1 * basespeeda + 5;
            ThisThread::sleep_for(5ms);
            printf("Turn Left\r\n");
        }
        else {
            Lweight = -1 * basespeedb + 5;
            ThisThread::sleep_for(5ms);
            printf("\tTurn Right\r\n");
        }
        PWMA = (basespeeda + Lweight) / 150.0 ;
        PWMB = (basespeedb + Rweight) / 150.0 ;
        
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
    // Left
    if (power_difference < 0)
    {
        curState = Left;
        if(prevState == Right) {
            Rweight = 15;
            Lweight = 0;
        }
        PWMA = ((basespeeda + (float)power_difference / maximum * basespeeda) + Lweight) / 150.0;
        PWMB = (basespeedb + Rweight) / 150.0 ;
        prevState = Left;
    }
    // Right
    else
    {
        curState = Right;
        if(prevState == Left) {
            Lweight = 15;
            Rweight = 0;
        }
        PWMA = (basespeeda + Lweight) / 150.0 ;
        PWMB = ((basespeedb - (float)power_difference / maximum * basespeedb) + Rweight) / 150.0;
        prevState = Right;
    }      
    
    for(int i=0; i<5; i++) 
        prev_sensor_values[i] = sensor_values[i];
    
    Lweight = 0; Rweight = 0;
    ThisThread::sleep_for(10ms);
}

/********************IR********************/
void REMOTE_control(unsigned int *sensor_values){
    while(1){
        if(irrecv.getState() == ReceiverIR::Received
            && irrecv.getData(&format, buf, sizeof(buf)*8) != 0){
            switch(buf[3]){
                case 0xBA: 
                    break; //ch --
                case 0xB9: 
                    break; //ch
                case 0xBB: 
                    rightvar -= 0.05;
                    if(rightvar < -2.0) rightvar = -2.0;
                    break; //<<
                case 0xBF: 
                    rightvar += 0.05;
                    if(rightvar > 2.0) rightvar = 2.0;
                    break; //>>
                case 0xBC: 
                    calibration = false; 
                    printf("done with remote mode\r\n");
                    break;    //>||
                case 0xF8: 
                    speed -= 0.1;
                    if( speed < -1.0) speed = -1.0;
                    printf("speed down\r\n");
                    break;    // -
                case 0xEA: 
                    speed += 0.1;
                    if( speed >1.0) speed = 1.0;
                    printf("speed up\r\n");
                    break;    // +
                case 0xF6:
                    printf("Calibrating\r\n");

                    for (int i =0; i< 1000; i++)
                        TR.calibrate();

                    printf("done calibrating\r\n");
                    
                    break;    //EQ
                case 0xE6: 
                    break;    //100+
                case 0xF2: 
                    break;    //200+
                case 0xE9: 
                    break;    //0
                case 0xF3: 
                    break;    //1
                case 0xE7:
                
                    AIN1 = 1;
                    AIN2 = 0;
                    BIN1 = 1;
                    BIN2 = 0;
                    
                    PWMA = speed;
                    PWMB = speed + rightvar;
                    printf("front\r\n");
                    // left.speed(speed);
                    // right.speed(speed + rightvar);
                    break;    //2
                case 0xA1: 
                    break;    //3
                case 0xF7:
                    AIN1 = 1;
                    AIN2 = 0;
                    BIN1 = 0;
                    BIN2 = 1;
                    
                    PWMA = speed;
                    PWMB = speed + rightvar;
                    printf("left\r\n"); 
                    break;    //4
                case 0xE3:                 
                    PWMA = 0.0;
                    PWMB = 0.0;
                    printf("stop\r\n");
                    break;    //5
                case 0xA5: 
                
                    AIN1 = 0;
                    AIN2 = 1;
                    BIN1 = 1;
                    BIN2 = 0;
                    
                    PWMA = speed;
                    PWMB = speed + rightvar;
                    printf("right\r\n");
                    break;    //6
                case 0xBD: 
                    break;    //7
                case 0xAD: 
                
                    AIN1 = 0;
                    AIN2 = 1;
                    BIN1 = 0;
                    BIN2 = 1;

                    PWMA = speed;
                    PWMB = (speed + rightvar);
                    printf("back\r\n");
                    break;    //8
                case 0xB5: 
                    break;    //9 
                default:
                    break;
            }
        }

        TR.calibrate();
        uint16_t position = TR.readLine(sensor_values);
        
        if(!calibration) {
            printf("calibration false\r\n");
            break;
        }
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
}