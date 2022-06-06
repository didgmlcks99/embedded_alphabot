#include "PCF8574.h"
#include "mbed.h"
 
PCF8574::PCF8574(PinName sda, PinName scl, int address)
        : _i2c(sda, scl) {
    _address = address;
    _i2c.frequency(100000);
}
 
int PCF8574::read() {
    char foo[1];
    _i2c.read(_address, foo, 1);
    return foo[0];
}
 
void PCF8574::write(int data) {
    char foo[1];
    foo[0] = data;
    _i2c.write(_address, foo, 1);
}
