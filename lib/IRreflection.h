#ifndef TRSensors_h
#define TRSensors_h

#include "mbed.h"
#define NUMSENSORS 5

class TRSensors
{
  public:
    
    TRSensors(PinName MOSI,PinName MISO, PinName CLK, PinName CS);
    
    void calibrate_init(int *calibratedMin, int *calibratedMax);
  
    void calibrate(int *sensor_values, int *calibratedMin, int *calibratedMax);
    
    void AnalogRead(int *sensor_values);

    void readCalibrated(int *sensor_values, int *calibratedMin, int *calibratedMax);

    int readLine(int *sensor_values , int *calibratedMin, int *calibratedMax, int *online, char white_line = 0);
    
    int sqrt(int m, int k);

  char _numSensors;
  int max_sensor_values[NUMSENSORS];
  int min_sensor_values[NUMSENSORS];
    
 private:
  SPI _spi;
  DigitalOut _spi_cs;
 
};

#endif