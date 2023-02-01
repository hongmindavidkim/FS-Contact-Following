#ifndef _FPSensor_H
#define _FPSensor_H

#include <mbed.h>

class FPSensor {
    protected:
    SPI _spi;
    DigitalOut _cs;

    public:
    FPSensor(PinName mosi, PinName miso, PinName clk, PinName cs);
    uint16_t binary(int ch);
};

#endif