#pragma once

#include <stdint.h>

class Motor {
public:
    //Constructor
    Motor(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin);

    void init();
    void set_speed(int speed);
    void stop();

private:
    uint8_t pwm_pin_;
    uint8_t in1_pin_;
    uint8_t in2_pin_;   
};