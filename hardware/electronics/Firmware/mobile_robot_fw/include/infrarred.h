#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class Infrarred {
public:
    Infrarred(uint8_t xshut_pin, uint8_t address);

    void init();
    uint16_t read_distance();

private:
    uint8_t xshut_pin_;
    uint8_t address_;
    VL53L0X sensor;
};