#include "mbed.h"
 
#ifndef MBED_PCF8574_H
#define MBED_PCF8574_H
 
/** Interface to the popular PCF8574 I2C 8 Bit IO expander */
class PCF8574 {
public:
    /** Create an instance of the PCF8574 connected to specfied I2C pins, with the specified address.
     *
     * @param sda The I2C data pin
     * @param scl The I2C clock pin
     * @param address The I2C address for this PCF8574
     */
    PCF8574(PinName sda, PinName scl, int address);
 
    /** Read the IO pin level
     *
     * @return The byte read
     */
    int read();
    
    /** Write to the IO pins
     * 
     * @param data The 8 bits to write to the IO port
     */
    void write(int data);
 
private:
    I2C _i2c;
    int _address;
};
 
#endif