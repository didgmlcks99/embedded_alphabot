#include "IRreflection.h"

 

// Base class data member initialization (called by derived class init())

TRSensors::TRSensors(PinName MOSI,PinName MISO, PinName CLK, PinName CS) : _spi(MOSI,MISO,CLK), _spi_cs(CS)

{   

    _spi.format(16,0);

    _spi.frequency(2000000);

    _spi_cs = 1;

    _numSensors = NUMSENSORS;

}

 

 

 

void TRSensors::AnalogRead(int *sensor_values)

{

    int i,j,ch;

    int values[] = {0,0,0,0,0,0};

 

    for(j = 0;j < _numSensors + 1;j++)

    {

     ch =j;

     _spi_cs = 0;

     wait_us(2);

     values[j] = _spi.write(ch<<12);

     _spi_cs = 1;

     wait_us(21);

     values[j] = (values[j]>>6);

    }

    

    for(i = 0;i < _numSensors;i++)

    {

        sensor_values[i] = values[i+1];

    }

 

}

 

// Reads the sensors 10 times and uses the results for

// calibration.  The sensor values are not returned; instead, the

// maximum and minimum values found over time are stored internally

// and used for the readCalibrated() method.

void TRSensors::calibrate_init(int *calibratedMin, int *calibratedMax)

{

        for(int i=0;i<_numSensors;i++)

    {

        calibratedMin[i] = 1023;

        calibratedMax[i] = 0;

    }

}

void TRSensors::calibrate(int *sensor_values, int *calibratedMin, int *calibratedMax)

{

    int i=0;

    int j=0;

    

    for(j=0;j<10;j++)

    {

        AnalogRead(sensor_values);

        for(i=0;i<_numSensors;i++)

        {

            // set the max we found THIS time

            if(j == 0 || max_sensor_values[i] < sensor_values[i])

                max_sensor_values[i] = sensor_values[i];

 

            // set the min we found THIS time

            if(j == 0 || min_sensor_values[i] > sensor_values[i])

                min_sensor_values[i] = sensor_values[i];

        }

    }

    

  // record the min and max calibration values

  for(i=0;i<_numSensors;i++)

  {

    if(min_sensor_values[i] > calibratedMax[i])

      calibratedMax[i] = min_sensor_values[i];

    if(max_sensor_values[i] < calibratedMin[i])

      calibratedMin[i] = max_sensor_values[i];

  }

}

 

 

// Returns values calibrated to a value between 0 and 1000, where

// 0 corresponds to the minimum value read by calibrate() and 1000

// corresponds to the maximum value.  Calibration values are

// stored separately for each sensor, so that differences in the

// sensors are accounted for automatically.

void TRSensors::readCalibrated(int *sensor_values, int *calibratedMin, int *calibratedMax)

{

    int i;

 

    // read the needed values

    AnalogRead(sensor_values);

 

    for(i=0;i<_numSensors;i++)

    {

        int denominator;

 

        denominator = calibratedMax[i] - calibratedMin[i];

 

        int x = 0;

        

        

        if(denominator != 0){

            if(((signed int)sensor_values[i] - (signed int)calibratedMin[i])<0){

                x = 0;}

            else {

                x = ((sensor_values[i] - calibratedMin[i])*1000/denominator);

                }

        }

        

        if( x > 1000) x = 1000;

        

        sensor_values[i] = x;

    

 

}

 

}

 

 

 

// Operates the same as read calibrated, but also returns an

// estimated position of the robot with respect to a line. The

// estimate is made using a weighted average of the sensor indices

// multiplied by 1000, so that a return value of 0 indicates that

// the line is directly below sensor 0, a return value of 1000

// indicates that the line is directly below sensor 1, 2000

// indicates that it's below sensor 2000, etc.  Intermediate

// values indicate that the line is between two sensors.  The

// formula is:

// 

//    0*value0 + 1000*value1 + 2000*value2 + ...

//   --------------------------------------------

//         value0  +  value1  +  value2 + ...

//

// By default, this function assumes a dark line (high values)

// surrounded by white (low values).  If your line is light on

// black, set the optional second argument white_line to true.  In

// this case, each sensor value will be replaced by (1000-value)

// before the averaging.

 

int TRSensors::readLine(int *sensor_values , int *calibratedMin, int *calibratedMax, int *online, char white_line)

{

    char i, on_line = 0;

    long int avg; // this is for the weighted total, which is long

                       // before division

    int sum; // this is for the denominator which is <= 64000

    static int last_value=0; // assume initially that the line is left.

 

    readCalibrated(sensor_values, calibratedMin, calibratedMax);

    

    online[0] = 0;

    avg = 0;

    sum = 0;

  

    for(i=0;i<_numSensors;i++) {

        

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

 

int TRSensors::sqrt(int m, int k){

   int temp = m;

   if(k == 0) m = 1;

   else{

   for(int i=0; i<(k-1); i++){

     m = temp *m;

    }

    }

   return  m;

}