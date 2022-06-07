//8조 최은송 김한결
#include "mbed.h"
#include "IRreflection.h"
#include "ReceiverIR.h"
#include "PCF8574.h"
#include "hcsr04.h"
#include "Adafruit_SSD1306.h"
#include "WS2812.h"
#include "PixelArray.h"

#define WS2812_BUF 77   //number of LEDs in the array
#define NUM_COLORS 6    //number of colors to store in the array
#define NUM_STEPS 8    //number of steps between colors

Timer total;
Timer Etime;
Timer Otime;
Timer AngleTimer;

//WS2812 neopixel================
PixelArray px(WS2812_BUF);
WS2812 ws(D7, WS2812_BUF, 6,17,9,14);   //nucleo-f411re
Thread neothread;

// temperary=========================
RawSerial pc(USBTX, USBRX, 115200);
//ultrasonic sensor=====================
int limit = 5;
HCSR04 sensor(D3, D2,pc,0,limit); 
int turn = 0;
int Ultra_distance = 0;

//PCF8574============================
PCF8574 io(I2C_SDA,I2C_SCL,0x40);
//IR_Reflection : declaration========
TRSensors TR(D11,D12,D13,D10);

static int sensor_for_end[NUMSENSORS];
static int sensor_val[NUMSENSORS];
static int calMin[NUMSENSORS];
static int calMax[NUMSENSORS];
//==================================
//==============Motor===============
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
//==================================
//IR_Remote : declaration===========
ReceiverIR ir_rx(D4);

RemoteIR::Format format;
uint8_t IR_buf[32];
uint32_t IR_buf_sum;
int bitcount;
//==========flag===================

typedef enum{NOK = 0, YOK}flag;
// Nessary change to enum!!!
flag flag_cal_Max;
flag flag_cal_Min;
flag flag_start;
flag flag_over;
flag flag_out;
flag flag_IR;
flag flag_end;
flag flag_angle;
flag flag_obstacle;
flag flag_distance;
flag flag_neo;
//==============================
void Initialization(void);
void Motor_init(void);
void Motor_Stop(void);
void Calibration(void);
void Driving(void);
void Actuator(void);
void Distance_check(void);
void Obstacle_check(void);
void Actuator_Obstacle(int dir);
int End_check(int *sensor_values);
int color_set(uint8_t red,uint8_t green, uint8_t blue);
int interpolate(int startValue, int endValue, int stepNumber, int lastStepNumber);
void NeopixelOn(void);

//===========================OLED
class I2CPreInit : public I2C
{
public:
    I2CPreInit(PinName sda, PinName scl) : I2C(sda, scl)
    {
        frequency(400000);
        start();
    };
};
 
I2C i2c(D14, D15);
Adafruit_SSD1306_I2c oled(i2c, D9, 0x78, 64, 128); 

int main(){
    oled.clearDisplay();
    oled.printf("Welcome to Alphabot\r\n");
    oled.display();

    Initialization();
    
    Driving();
   
    if(flag_end){
        // neothread.start(&NeopixelOn);
         oled.clearDisplay();
         int duration = total.read();
         oled.printf("Congratulation!! \r\n");
         oled.printf("Time is %.3f", total.read());
         oled.display();
         NeopixelOn();
        // wait(10);
    }
    //NeopixelOn();

}
//====================================================
//==================initialization====================
//====================================================
void Initialization(){
    
    //추후 다른 변수 초기화 전부 이곳에!!!
    
    flag_over = NOK;
    flag_IR = NOK;
    flag_distance = NOK;
    flag_cal_Min = NOK;
    flag_cal_Max = NOK;
    
    TR.calibrate_init(calMin,calMax);
    
    Motor_init();
    
    Calibration();
    
}

void Motor_Stop(){
    pc.printf("===============================Motor Stop!\r\n");
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
void Calibration(){
    
    while(flag_cal_Max == NOK){
     
    if ((ir_rx.getState() == ReceiverIR::Received)&&(ir_rx.getData(&format, IR_buf, sizeof(IR_buf) * 8)!=0)) {
        for(int i =0 ; i<4; i ++){
            IR_buf_sum |=(IR_buf[i]<<8*(3-i));
        }
        pc.printf("%X\r\n",IR_buf_sum);
            if(IR_buf_sum == 0x00FF0CF3){
                 io.write(0x7F);
                 TR.calibrate(sensor_val, calMin, calMax);
                 wait(0.5);
                 TR.calibrate(sensor_val, calMin, calMax);
                 io.write(0xFF);
    for(int i = 0; i<5; i++){
        pc.printf("Min_sensor_value[%d] = %d\r\n",i+1,calMin[i]);
        pc.printf("Max_sensor_value[%d] = %d\r\n",i+1,calMax[i]);
    }
    pc.printf("================================\r\n");

                 flag_cal_Max = YOK;
                }
        IR_buf_sum = 0;
        }
    }
    
    while(flag_cal_Min == NOK){
     
    if ((ir_rx.getState() == ReceiverIR::Received)&&(ir_rx.getData(&format, IR_buf, sizeof(IR_buf) * 8)!=0)) {
        for(int i =0 ; i<4; i ++){
            IR_buf_sum |=(IR_buf[i]<<8*(3-i));
        } 
        pc.printf("%X\r\n",IR_buf_sum);
            if(IR_buf_sum == 0x00FF18E7){
                 io.write(0xBF);
                 TR.calibrate(sensor_val, calMin, calMax);
                 wait(0.5);
                 TR.calibrate(sensor_val, calMin, calMax);
                 io.write(0xFF);
                 
for(int i = 0; i<5; i++){
pc.printf("Min_sensor_value[%d] = %d\r\n",i+1,calMin[i]);
pc.printf("Max_sensor_value[%d] = %d\r\n",i+1,calMax[i]);
}
pc.printf("================================\r\n");
                 flag_cal_Min = YOK;
                }
        IR_buf_sum = 0;
        }
    }
    
    
    
//=================start===============================
    while(flag_start == NOK){
     
    if ((ir_rx.getState() == ReceiverIR::Received)&&(ir_rx.getData(&format, IR_buf, sizeof(IR_buf) * 8)!=0)) {
        for(int i =0 ; i<4; i ++){
            IR_buf_sum |=(IR_buf[i]<<8*(3-i));
        } 
        
        pc.printf("%X\r\n",IR_buf_sum);
            if(IR_buf_sum == 0x00FF5EA1){
                pc.printf("===============start!===============\r\n");
                flag_start = YOK;
                }
        IR_buf_sum = 0;
        }
    } 
//================================================================   
}








//Using logic control=================================================================
void Driving(){
    
    
    total.start();
    Etime.start();
    while(!flag_end){
    Etime.reset();

   
 Obstacle_check();
    
    if(flag_distance == NOK){
        Actuator();
    }
    
    
    Distance_check();
 
    
    do{
    }while(Etime.read()<=0.0075);
    
    }
    
}


int End_check(int *sensor_values){
    int avg = 0;
    int sum = 0;
    
    for(int i = 0; i < NUMSENSORS; i++){
        sum += sensor_values[i];
    }
    avg = sum / NUMSENSORS;
    return avg;
}



void Actuator(){
    
    den = 30;
    
    
    int temp = TR.readLine(sensor_val, calMin, calMax,on_line,1);
    
    
    TR.readCalibrated(sensor_for_end, calMin, calMax);
    int avg = End_check(sensor_for_end);
    
    if(avg <= 100){
       pc.printf("avg:%d\r\n",avg);
       Motor_Stop(); 
       flag_end = YOK;  
       total.stop();
    }
    
    temp = temp - MIDDLE;
      
    if(temp>=0) direction = LEFT;
    else        direction = RIGHT;
    
    
    weight = abs(temp)/den;
    
    if((print_c%500) == 0){
        pc.printf("flag_out = %d\r\n", flag_out);
        pc.printf("temp = %d\r\n", temp);
        pc.printf("online = %d\r\n", on_line[0]);
        }
    
    R_PWM = 135;
    L_PWM = 133;
    
    
    if(weight >= (2000/den)*3/4){
        if(direction == LEFT){
        PWMA.pulsewidth_us(L_PWM+3+0.0*weight);
        PWMB.pulsewidth_us(R_PWM-2.1*weight);
        }
        else{
        PWMA.pulsewidth_us(L_PWM+3-2.1*weight);
        PWMB.pulsewidth_us(R_PWM+0.0*weight);
        }
    }else{
    
        if(direction == LEFT){
        PWMA.pulsewidth_us(L_PWM+3+0.0*weight);
        PWMB.pulsewidth_us(R_PWM-1.5*weight);
        }
        else{
        PWMA.pulsewidth_us(L_PWM+3-1.5*weight);
        PWMB.pulsewidth_us(R_PWM+0.0*weight);
        }
    }
    
    
    if(weight >= (2000/den)*2/3 && flag_over == NOK)
    {
        flag_over = YOK;
        P_direction = direction;
        P_weight = weight;
    }
    
    if((flag_over == YOK) && (abs(weight)<= (2000/den)*1/5)&&on_line[0]==1)
    {  
        if(P_weight >=(2000/den)*2/3 && P_weight<=(2000/den)*4/5){
            if(P_direction == LEFT){
            PWMA.pulsewidth_us(L_PWM-P_weight+3);
            PWMB.pulsewidth_us(R_PWM); 
            }
            else{
            PWMA.pulsewidth_us(L_PWM+3);
            PWMB.pulsewidth_us(R_PWM-P_weight);
            }
        }
    if(P_weight>=(2000/den)*4/5){
            if(P_direction == LEFT){
            PWMA.pulsewidth_us(L_PWM+3-1.0*P_weight);
            PWMB.pulsewidth_us(R_PWM+1.0*P_weight); 
            }
            else{
            PWMA.pulsewidth_us(L_PWM+3+1.0*P_weight);
            PWMB.pulsewidth_us(R_PWM-1.0*P_weight);
            }
        }
       // pc.printf("I'm Here\r\n");
        
        wait(0.12);
    flag_over = NOK;
    }
    
    P_temp = temp;
    print_c++;
}


void Distance_check(){
    
    if(flag_distance == NOK){
    sensor.distance();
    Ultra_distance = sensor.returndistance();
   //pc.printf("distance = %d\r\n",flag_distance);
    if((Ultra_distance >= 16 && Ultra_distance <= 19)) flag_distance = YOK;
    }
    
    if(flag_distance == YOK){
     while(1){ 
            PWMB.pulsewidth_us(100);
            PWMA.pulsewidth_us(100);
            AIN1.write(1);
            AIN2.write(0);
            BIN1.write(0);
            BIN2.write(1);
            if((abs(TR.readLine(sensor_val, calMin, calMax,on_line,1) - MIDDLE)<=400 && on_line[0] == 1)){
                Motor_init();
                flag_distance = NOK;
                PWMB.pulsewidth_us(0);
                PWMA.pulsewidth_us(0);
                break;
            }
        } 
    }
}

void Obstacle_check(){
    
    if(io.read() == 0x7F){     //왼쪽읽음 
        flag_IR = YOK;
        flag_obstacle = YOK;
        Obs_direction = LEFT;
    }
    else if(io.read() == 0xbF)  //오른쪽 읽음
    {
        flag_IR = YOK;
        flag_obstacle = YOK;
        Obs_direction = RIGHT;
    }
    
    if(flag_IR){
        Otime.start();
        
        while(flag_obstacle == YOK){
             Actuator_Obstacle(Obs_direction); 
             pc.printf("obstacle!\r\n");  
        }
        Otime.stop();
        Otime.reset();
        flag_IR = NOK;
    }

}

void Actuator_Obstacle(int dir){
 
    R_PWM = 100;
    L_PWM = 100;
 
if(Otime.read() <= 0.5){
 
         if(dir == 0){
             PWMA.pulsewidth_us(L_PWM-20);
             PWMB.pulsewidth_us(R_PWM-100); 
              }
         else if(dir == 1 ){
             PWMA.pulsewidth_us(L_PWM-100);
             PWMB.pulsewidth_us(R_PWM-30);
           }
           
}else if(Otime.read() <= 1.0){

         if(dir == 0){
             PWMA.pulsewidth_us(L_PWM+5);
             PWMB.pulsewidth_us(R_PWM); 
              }
         else if(dir == 1 ){
             PWMA.pulsewidth_us(L_PWM+5);
             PWMB.pulsewidth_us(R_PWM);
           }
             
}else if(Otime.read() <= 2.0){

         if(dir == 0){
             PWMA.pulsewidth_us(L_PWM-100);
             PWMB.pulsewidth_us(R_PWM-30); 
              }
         else if(dir == 1 ){
             PWMA.pulsewidth_us(L_PWM-10);
             PWMB.pulsewidth_us(R_PWM-100);
           }
           
}else{
    
        if((abs(TR.readLine(sensor_val, calMin, calMax,on_line,1) - MIDDLE)<=400 && on_line[0] == 1)) flag_obstacle = NOK;
     
        PWMA.pulsewidth_us(L_PWM);
        PWMB.pulsewidth_us(R_PWM);

    }

}


int color_set(uint8_t red,uint8_t green, uint8_t blue)
{
  return ((red<<16) + (green<<8) + blue);   
}

// 0 <= stepNumber <= lastStepNumber
int interpolate(int startValue, int endValue, int stepNumber, int lastStepNumber)
{
    return (endValue - startValue) * stepNumber / lastStepNumber + startValue;
}

void NeopixelOn(){
    int colorIdx = 0;
    int colorTo = 0;
    int colorFrom = 0;
    
    uint8_t ir = 0;
    uint8_t ig = 0;
    uint8_t ib = 0;
    
    ws.useII(WS2812::PER_PIXEL); // use per-pixel intensity scaling
    
    // set up the colours we want to draw with
    int colorbuf[NUM_COLORS] = {0x2f0000,0x2f2f00,0x002f00,0x002f2f,0x00002f,0x2f002f};
    
    // Now the buffer is written, write it to the led array.
    while (1) 
    {
        //pc.printf("thread\r\n");
        //get starting RGB components for interpolation
        std::size_t c1 = colorbuf[colorFrom];
        std::size_t r1 = (c1 & 0xff0000) >> 16;
        std::size_t g1 = (c1 & 0x00ff00) >> 8;
        std::size_t b1 = (c1 & 0x0000ff);
        
        //get ending RGB components for interpolation
        std::size_t c2 = colorbuf[colorTo];
        std::size_t r2 = (c2 & 0xff0000) >> 16;
        std::size_t g2 = (c2 & 0x00ff00) >> 8;
        std::size_t b2 = (c2 & 0x0000ff);
        
        for (int i = 0; i <= NUM_STEPS; i++)
        {
            ir = interpolate(r1, r2, i, NUM_STEPS);
            ig = interpolate(g1, g2, i, NUM_STEPS);
            ib = interpolate(b1, b2, i, NUM_STEPS);
            
            //write the color value for each pixel
            px.SetAll(color_set(ir,ig,ib));
            
            //write the II value for each pixel
            px.SetAllI(32);
            
            for (int i = WS2812_BUF; i >= 0; i—) 
            {
                ws.write(px.getBuf());
            }
        }
        
        colorFrom = colorIdx;
        colorIdx++;
        
        if (colorIdx >= NUM_COLORS)
        {
            colorIdx = 0;
        }
        
        colorTo = colorIdx;
    }    
}