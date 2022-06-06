#include "mbed.h"

#define WIFI_NAME "iptime_yang"
#define WIFI_PSW "yang1234!"

//UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 115200);
// 192.168.4.1
// 50000
/********************wifi********************/
char buffer[80];

UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 230400);
BufferedSerial *serial = new BufferedSerial(ARDUINO_UNO_D8, ARDUINO_UNO_D2, 115200);
ATCmdParser *parser = new ATCmdParser(serial, "\r\n");

int id;

/********************Motor********************/
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

void wifi_init(){
    char ip[80];
    char gw[80];
    
    sprintf(buffer, "\r\nWifi TCP Server example\r\n\n");
    pc.write(buffer, strlen(buffer));
    
    // AT+CWMODE = 3    
    parser->send("AT+CWMODE=3");
    if(parser->recv("OK")){
        sprintf(buffer, "Set SoftAP+State mode\r\n\n");
        pc.write(buffer, strlen(buffer));
    }else{
        sprintf(buffer, "AT+CWMODE=3 [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }
    // AT+CWJAP
    parser->send("AT+CWJAP=\"%s\",\"%s\"", WIFI_NAME, WIFI_PSW);
    
    if(parser->recv("WIFI DISCONNECT") && parser->recv("WIFI CONNECTED") && parser->recv("WIFI GOT IP") && parser->recv("OK")){
        
        parser->send("AT+CWSAP=\"HCY_ESP\",\"YANG1234!\",3,3");
        if(parser->recv("OK")){
            sprintf(buffer, "Successfully configure for softAP\r\n");
            pc.write(buffer, strlen(buffer));
        }
    }else{
        sprintf(buffer, "Failed connecting ESP to wifi [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }
    // AT+CIPAP?
    parser->send("AT+CIPAP?");
    if(parser->recv("+CIPAP:ip:%s\r\n", ip) && parser->recv("+CIPAP:gateway:%s\r\n", gw) && parser->recv("OK")){
        sprintf(buffer, "IP address: %s\r\n", ip);
        pc.write(buffer, strlen(buffer));
        sprintf(buffer, "GW IP address: %s\r\n\n", gw);
        pc.write(buffer, strlen(buffer));
    }else{
        sprintf(buffer, "AT+CIPAP? [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }
    // AT+CIPMUX=1
    parser->send("AT+CIPMUX=1");
    if(parser->recv("OK")){
        sprintf(buffer, "success: set CIPMUX\r\n\n");
        pc.write(buffer, strlen(buffer));
    }else{
        sprintf(buffer, "AT+CWMUX=1 [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }
    // AT+CIPSERVER=1,50000
    parser->send("AT+CIPSERVER=1,50000");
    if(parser->recv("OK")){
        sprintf(buffer, "success: set a TCP server\r\n\n");
        pc.write(buffer, strlen(buffer));
    }else{
        sprintf(buffer, "AT+CIPSERVER=1,50000 [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }
}


int main(){
    char motor[256];
    float lmotor = 0.0, rmotor=0.0;
    int c_id;
    int size;
    int status;
    
    wifi_init();
    Motor_init();
    
    PWMR.pulsewidth_ms(100);
    PWML.pulsewidth_ms(100);
    
    while(true) {
        if(parser->recv("%d,CONNECT\r\n", &id)){
            sprintf(buffer, "connection request from a client, id: %d\r\n\n", id);
            pc.write(buffer, strlen(buffer));
            
            while(1){
                memset(motor, 0, 256); // initialize motor buffer
                
                lmotor = PWML;
                rmotor = PWMR;
                
                sprintf(motor, "Left motor: %f, Right motor: %f\r\n", lmotor, rmotor);
                
                parser->send("AT+CIPSEND=%d,%d\r\n", id, strlen(motor));
                if(parser->recv("OK")){
                    parser->send("%s\r\n", motor);
                    if(!parser->recv("SEND OK")){
                        sprintf(buffer, "Error sending message to client\r\n");
                        pc.write(buffer, strlen(buffer));
                    }
                }
                
                else if(parser->send("AT+CIPSTATUS") && parser->recv("STATUS:%d\r\n", &status) && parser->recv("OK")){
                    if(status == 4) {
                        sprintf(buffer, "\nConnection closed\r\n");
                        pc.write(buffer, strlen(buffer));
                        
                        sprintf(buffer, " Wait a connection...\r\n");
                        pc.write(buffer, strlen(buffer));
                        
                        break;
                    }
                }
            }
        }
    }
}
    