#pragma once
#include <Arduino.h>

class Ultrasonic {
public:
    Ultrasonic(uint8_t trig_pin, uint8_t echo_pin);

    void init();
    float read_distance();

private:
    uint8_t trig_pin_;
    uint8_t echo_pin_;
};