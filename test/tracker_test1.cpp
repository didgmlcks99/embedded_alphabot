#include "mbed.h"

SPI spi(ARDUINO_UNO_D11, ARDUINO_UNO_D12, ARDUINO_UNO_D13);
DigitalOut cs(ARDUINO_UNO_D10);

UnbufferedSerial pc(CONSOLE_TX, CONSOLE_RX, 230400);

int numSensors = 5;

char tx_buffer[80];
int sensors[5];

int max_sensor_values[5];
int min_sensor_values[5];

int online[5];


void calibrate_init(int *calibratedMin, int *calibratedMax);
void calibrate(int *sensor_values, int *calibratedMin, int *calibratedMax);
void AnalogRead(int *sensor_values);
void readCalibrated(int *sensor_values, int *calibratedMin, int *calibratedMax);
int readLine(int *sensor_values , int *calibratedMin, int *calibratedMax, int *online, char white_line = 0);
int sqrt(int m, int k);

int sqrt(int m, int k){
   int temp = m;
   
   if(k == 0) m = 1;
   else{
        for(int i=0; i<(k-1); i++){
            m = temp *m;
        }
    }

   return  m;
}

int readLine(int *sensor_values , int *calibratedMin, int *calibratedMax, int *online, char white_line){

    char i, on_line = 0;
    long int avg; // this is for the weighted total, which is long
                    // before division
    int sum; // this is for the denominator which is <= 64000
    static int last_value=0; // assume initially that the line is left.

 

    readCalibrated(sensor_values, calibratedMin, calibratedMax);

    

    online[0] = 0;
    avg = 0;
    sum = 0;

  

    for(i=0;i<numSensors;i++) {
        long int value = sensor_values[i];

        //if(!white_line) value = 1000-value;

        if(white_line){
            value = 1000-value;
            //sensor_values[i] = (1000 - sensor_values[i])/100;
        }

        

        //sensor_values[i] = value;
        // keep track of whether we see the line at all
        if(value > 300) {
            online[0] = 1;
        }

        

        // only average in values that are above a noise threshold
        if(value > 30) {
            avg += value*(1000*i);//(value/100)*(1000*i)     // * sqrt(100,i);
            sum += value; //(value/100);
        }
    }

    /*
    if(!on_line)
    {
        // If it last read to the left of center, return 0.
         if(last_value < (_numSensors-1)*1000/2)
             return 0;
        
        // If it last read to the right of center, return the max.
         else
             return (_numSensors-1)*1000;
    }
    */

    

    last_value = avg/sum;

   // if(sum == 0) last_value = 0;

    return last_value;

}




void readCalibrated(int *sensor_values, int *calibratedMin, int *calibratedMax){
    int i;

    // read the needed values
    AnalogRead(sensor_values);

    for(i=0;i<numSensors;i++){

        int denominator;
        denominator = calibratedMax[i] - calibratedMin[i];

        int x = 0;
  
        if(denominator != 0){

            if(((signed int)sensor_values[i] - (signed int)calibratedMin[i])<0){
                x = 0;
            }else {
                x = ((sensor_values[i] - calibratedMin[i])*1000/denominator);
            }

        }

        if( x > 1000) x = 1000;

        sensor_values[i] = x;
    }
}



void calibrate(int *sensor_values, int *calibratedMin, int *calibratedMax){

    int i=0;
    int j=0;

    for(j=0;j<10;j++){
        AnalogRead(sensor_values);

        for(i=0; i<numSensors; i++){

            // set the max we found THIS time
            if(j == 0 || max_sensor_values[i] < sensor_values[i])
                max_sensor_values[i] = sensor_values[i];

 

            // set the min we found THIS time
            if(j == 0 || min_sensor_values[i] > sensor_values[i])
                min_sensor_values[i] = sensor_values[i];
        }

    }

    for(i=0;i<numSensors;i++){

        if(min_sensor_values[i] > calibratedMax[i])
            calibratedMax[i] = min_sensor_values[i];

        if(max_sensor_values[i] < calibratedMin[i])
            calibratedMin[i] = max_sensor_values[i];

    }
}

void calibrate_init(int *calibratedMin, int *calibratedMax){

    for(int i=0; i<numSensors; i++){
        calibratedMin[i] = 1023;
        calibratedMax[i] = 0;
    }

}

void AnalogRead(int *sensor_values) {
    int i,j,ch;

    int values[] = {0,0,0,0,0};


    for(j = 0;j < numSensors + 1;j++){
        ch = j;

        cs = 0;
        wait_us(2);

        values[j] = spi.write(ch<<12);

        cs = 1;
        wait_us(21);

        values[j] = (values[j]>>6);
    }

    

    for(i = 0; i < numSensors; i++){
        sensor_values[i] = values[i+1];
    }
}

int main() {
    sprintf(tx_buffer, "Test SPI Program\r\n");
    pc.write(tx_buffer, strlen(tx_buffer));

    spi.format(16,0);
    spi.frequency(2000000);
    cs = 1;


    calibrate_init(min_sensor_values, max_sensor_values);
    calibrate(sensors, min_sensor_values, max_sensor_values);
    readLine(sensors, min_sensor_values, max_sensor_values, online, 0);
    
    sprintf(tx_buffer, "%d %d %d %d %d\r\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4]);
    pc.write(tx_buffer, strlen(tx_buffer));

    ThisThread::sleep_for(3000ms);
    
    while(1){
        readLine(sensors, min_sensor_values, max_sensor_values, online, 0);
        
        sprintf(tx_buffer, "%d %d %d %d %d\r\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4]);
        pc.write(tx_buffer, strlen(tx_buffer));
    }

}