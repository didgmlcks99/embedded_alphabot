#include "mbed.h"

#include "RemoteIR.h"
#include "ReceiverIR.h"

#include "Ultrasonic.h"

#include "Motor.h"

#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "glcdfont.h"
#define SSD1306_DISPLAYON 0xAF

#include "WS2812.h"
#include "PixelArray.h"

#define WS2812_BUF 150
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 10

PixelArray px(WS2812_BUF);
WS2812 ws(D7, WS2812_BUF, 0, 5, 5, 0);

void wsInit(){
    ws.useII(WS2812::PER_PIXEL);
    int colorbuf[NUM_COLORS] = {0x2f0000,0x2f2f00,0x002f00,0x002f2f
    ,0x00002f,0x2f002f
    };
    px.Set(1, 0xFFFFFF);
    px.SetI(1, 0xFFFFFF);
}

Ultrasonic ultra(D3,D2);
Ticker ticker;

SPI spi(D11, D12, D13);
DigitalOut spi_cs(D10,1);

RawSerial pc(USBTX, USBRX, 115200);

Motor right(D5, A3 ,A2);
Motor left(D6, A0 ,A1);

RemoteIR::Format format = RemoteIR::NEC;
ReceiverIR irrecv(D4);

Timer timer;

I2C i2c(D14, D15);
Adafruit_SSD1306_I2c oled(i2c, D9, 0x78, 64, 128);

uint8_t buf[8];

bool white = false, black = false;

int ir[5];
int MINIR[5] = {1023,1023,1023,1023,1023};
int MAXIR[5] = {0,0,0,0,0};
int lastIR = 0;

bool run = false;

void readLine(){
    int ch = 0;
    int value = 0;
//    pc.printf("\r\nSPI Value:\r\n");
    do{
        value = 0;
        spi_cs = 0;
        wait_us(2);
        value = spi.write(ch<<12);
        spi_cs = 1;
        wait_us(18);
        switch(ch){
            case 6: 
//           pc.printf("ch %2d: %.2fmV\r\n",ch, (float)(value >> 6)*3.22580645); break;
            case 7: case 8: case 9: case 10: case 11:
            case 0: case 12: case 13: case 14: case 15: case 16: break;
            case 1: ir[0] = value >> 6;break;
            case 2: ir[1] = value >> 6;break;
            case 3: ir[2] = value >> 6;break;
            case 4: ir[3] = value >> 6;break;
            case 5: ir[4] = value >> 6;break;
//           default: pc.printf("ch %2d: %d\r\n",ch, value >> 6);
        }
//           if(ch == 6) pc.printf("ch %2d: %.2fmV\r\n",ch, (float)(value >> 6)*3.22580645);
//           else pc.printf("ch %2d: %d\r\n",ch, value >> 6);
        ch++; 
    } while (ch < 6);
}

void calibrate(){
    int vMAXIR[5] = {0,0,0,0,0}, vMINIR[5] = {0,0,0,0,0};
    for(int j =0; j< 10; j++){
        readLine();
        for(int i =0;i<5;i++){
            if(j==0 || vMAXIR[i] < ir[i]) vMAXIR[i] = ir[i];
            if(j==0 || vMINIR[i] > ir[i]) vMINIR[i] = ir[i];
        }
    }
    for (int i =0; i<5; i++){
        if(vMINIR[i] > MAXIR[i]) MAXIR[i] = vMINIR[i];
        if(vMAXIR[i] < MINIR[i]) MINIR[i] = vMAXIR[i];
    }
}

void readCalibrated(){
    readLine();
    for(int i =0; i<5;i++){
        int calMINIR, calMAXIR;
        int denoMINIRator, x = 0;
        denoMINIRator = MAXIR[i] - MINIR[i];
        if(denoMINIRator != 0) x= (ir[i] - MINIR[i]) * 1000 /denoMINIRator;
        if(x<0) x= 0;
        else if ( x> 1000) x= 1000;
        ir[i] = x;
    }
}

int readPosition(bool white){
    bool on_line = false;
    int avg = 0, sum = 0;
    readCalibrated();
    for(int i =0;i<5;i++){
        int value = ir[i];
        if(!white){
            value = 1000 - value;
            ir[i] = value;
        }
        if(value > 300){
            on_line = true;
        }
        if(value > 50) {
            avg += value * (i*1000);
            sum += value;   
        }
    }
    if(!on_line){
        if(lastIR < 4000/2){
            return 0;
        }
        else return 4000;
    }
    lastIR = avg/sum;
    return lastIR;
}

void tick(){
    ultra.trig();
//    pc.printf("hello");
}

int last_proportional = 0;
int integral = 0;
const int maximum = 255;
const float maxspeed = 0.25;
const float radiant = 0.6;
float speed = 0.3;
float rightvar = 0.013;

void printIR(){
    for (int i =0; i< 5; i++){
        pc.printf("%d: %d\r\n", i, ir[i]);     
    }    
}


int main()
{
    spi.format(16,0);
    spi.frequency(2000000);
    i2c.frequency(100000);
    oled.begin(SSD1306_SWITCHCAPVCC);
    oled.clearDisplay();
    timer.start();
    oled.printf("Calibration start\r");
    oled.display();
    bool calibration = true;
    int start = 0, end = 0;
    
    while(true){
       if(irrecv.getState() == ReceiverIR::Received
        && irrecv.getData(&format, buf, sizeof(buf)*8) != 0){
            switch(buf[3]){
                case 0xBA: 
                break; //ch --
                case 0xB9: break; //ch
                case 0xBB: 
                    rightvar -= 0.05;
                    if(rightvar < -2.0) rightvar = -2.0;
                break; //<<
                case 0xBF: 
                    rightvar += 0.05;
                    if(rightvar > 2.0) rightvar = 2.0;
                break; //>>
                case 0xBC: calibration = false; break;    //>||
                case 0xF8: 
                    speed -= 0.1;
                    if( speed < -1.0) speed = -1.0;
                break;    // -
                case 0xEA: 
                    speed += 0.1;
                    if( speed >1.0) speed = 1.0;
                break;    // +
                case 0xF6:
                    //left.speed(0.2);
//                    right.speed(-0.2);
                    oled.clearDisplay();
                    oled.printf("Calibrating\r");
                    oled.display();
                    for (int i =0; i< 100; i++)
                        calibrate();
                    oled.clearDisplay();
                    oled.printf("Calibrated!!\r");
                    oled.display();
                    for(int i =0; i<5;i++){
                        pc.printf("MIN: %d, MAX: %d\r\n", MINIR[i], MAXIR[i]);    
                    }
//                    wait(0.7);
//                    left.speed(0.0);
//                    right.speed(0.0);
                    // pc.printf("\r\ncalibrated\r\n"); 
                break;    //EQ
                case 0xE6: break;    //100+
                case 0xF2: break;    //200+
                case 0xE9: break;    //0
                case 0xF3: break;    //1
                case 0xE7: 
                    left.speed(speed);
                    right.speed(speed + rightvar);
                break;    //2
                case 0xA1: break;    //3
                case 0xF7: 
                    left.speed(speed/2);
                    right.speed(speed + rightvar);
                break;    //4
                case 0xE3:                 
                    left.speed(0);
                    right.speed(0);
                break;    //5
                case 0xA5: 
                    left.speed(speed);
                    right.speed(speed/2 + rightvar);
                break;    //6
                case 0xBD: break;    //7
                case 0xAD: 
                    left.speed(-speed);
                    right.speed(-(speed + rightvar));
                break;    //8
                case 0xB5: break;    //9 
                default:break;
            }
        }
        if(!calibration) break;
    }
    oled.clearDisplay();
    oled.printf("Start!\r");
    oled.display();
    
//    ticker.attach(&tick, 1.0);

    start = timer.read_us();
    while (true) {
        ultra.trig();
        if(ultra.getStatus() == 1) {
            int distance = ultra.getDistance();
            if(distance >18 && distance <25){
                left.speed(-0.4);
                right.speed(0.4);
                wait(0.15);
                left.speed(0.0);
                right.speed(0.0);
                integral = 0;
                last_proportional = 0;
            }
            ultra.clearStatus();
        }
        int position = readPosition(false); //ir1, ir2, ir3, ir4, ir5에 값이 들어감
        int proportional = (int)position - 2000;
        int derivative = proportional - last_proportional;
        integral += proportional;
        last_proportional = proportional;
        int power_difference = proportional/20 + integral/10000 + derivative*10;
        
        if (power_difference > maximum)
            power_difference = maximum;
        if (power_difference < -maximum)
            power_difference = -maximum;
//        pc.printf("position: %d proportion: %d last_proportional: %d pd: %d\r\n", position, proportional, last_proportional, power_difference);
        if (power_difference < 0)
        {
            left.speed((maxspeed + (float)power_difference / maximum * maxspeed));
            right.speed(maxspeed);
//            pc.printf("%f\r\n",maxspeed + (float)power_difference * maxspeed *radiant / maximum);
        }
        else
        {
            left.speed(maxspeed);
            right.speed((maxspeed - (float)power_difference / maximum * maxspeed));
//            pc.printf("%f\r\n",maxspeed - (float)power_difference * maxspeed *radiant / maximum);
        }      
        
        if (ir[1] > 800 && ir[2] > 800 && ir[3] > 800)
        {
            left.speed(0.0);
            right.speed(0.0);
            end = timer.read_us();
            oled.clearDisplay();
            oled.printf("lap: %d ms\r", (end - start)/1000);
            oled.display();
            wsInit();
            ws.write_offsets(px.getBuf(),WS2812_BUF,WS2812_BUF,WS2812_BUF);
//        pc.printf("%d\r\n", end - start);
            wait(15.0);
        }
        wait(0.05);
//        left.speed(0.0);
//        right.speed(0.0);
////        pc.printf("\r\n");
////        printIR();
//        pc.printf("pd: %d\r\n",power_difference);
//        pc.printf("p: %d\r\n",last_proportional/20);
//        pc.printf("p: %d\r\n",proportional/20);
//        pc.printf("i: %d\r\n",integral/10000);
//        pc.printf("d: %d\r\n",derivative/20);
//        pc.getc();
    }
}