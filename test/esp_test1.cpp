#include "mbed.h"

#define WIFI_NAME "iptime_yang"
#define WIFI_PSW "yang1234!"

UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 230400);
//UnbufferedSerial wifi(ARDUINO_UNO_D8, ARDUINO_UNO_D2, 115200);

char buffer[80];

BufferedSerial *serial = new BufferedSerial(ARDUINO_UNO_D8, ARDUINO_UNO_D2, 115200);
ATCmdParser *parser = new ATCmdParser(serial, "\r\n");


void wifi_init(){
    sprintf(buffer, "\r\nWifi TCP Server example\r\n\n");
    pc.write(buffer, strlen(buffer));
    
//    parser->debug_on(true);
    
    
    parser->send("AT+CWMODE=3");
    if(parser->recv("OK")){
        sprintf(buffer, "Set SoftAP+State mode\r\n\n");
        pc.write(buffer, strlen(buffer));
    }else{
        sprintf(buffer, "AT+CWMODE=3 [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }

    
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
    
    char ip[80];
    char gw[80];
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
    
    
    parser->send("AT+CIPMUX=1");
    if(parser->recv("OK")){
        sprintf(buffer, "success: set CIPMUX\r\n\n");
        pc.write(buffer, strlen(buffer));
    }else{
        sprintf(buffer, "AT+CWMUX=1 [ERROR]\r\n\n");
        pc.write(buffer, strlen(buffer));
        while(1){}
    }
    
    
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
    wifi_init();
 
    while(1){

        int id;
        if(parser->recv("%d,CONNECT\r\n", &id)){
            
            sprintf(buffer, "connection request from a client, id: %d\r\n\n", id);
            pc.write(buffer, strlen(buffer));
            
            while(1){
                
                int c_id;
                int size;
                char client_in[256];
                memset(client_in, 0, 256);
                int status;
                
                if(parser->recv("+IPD,%d,%d:", &c_id, &size)){
                    parser->read(client_in, size);
                    
                    sprintf(buffer, "client says: %s", client_in);
                    pc.write(buffer, strlen(buffer));
                    
                    parser->send("AT+CIPSEND=%d,%d\r\n", c_id, size);
                    if(parser->recv("OK")){
                        
                        parser->send("%s\r\n", client_in);
                        if(!parser->recv("SEND OK")){
                            sprintf(buffer, "Error sending message to client\r\n");
                            pc.write(buffer, strlen(buffer));
                        }
                    }
                }else if(parser->send("AT+CIPSTATUS") && parser->recv("STATUS:%d\r\n", &status) && parser->recv("OK")){
                    
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
