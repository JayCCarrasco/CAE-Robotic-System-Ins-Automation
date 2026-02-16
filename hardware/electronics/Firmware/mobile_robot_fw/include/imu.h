#pragma once
#include <Wire.h>

class Imu {
public:
    Imu(uint8_t address = 0x68);
    
    void init();
    void update();
    float get_gyro_Z();

private:
    float gyro_Z;
    uint8_t address_;
};