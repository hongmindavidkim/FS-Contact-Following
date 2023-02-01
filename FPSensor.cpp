#include "FPSensor.h"
#include "PinNames.h"

FPSensor::FPSensor(PinName mosi, PinName miso, PinName clk, PinName cs): _spi(mosi,miso,clk),_cs(cs) {
    _spi.frequency(2000000);
    _spi.format(16,3);
    _cs = 1;
}

uint16_t FPSensor::binary(int ch) {
    _cs = 0;
    wait_us(10);

    uint16_t ret = _spi.write((0x18|ch)<<2);
    uint16_t adb = _spi.write(0)>>4;
    _cs = 1;
    return adb;
}
