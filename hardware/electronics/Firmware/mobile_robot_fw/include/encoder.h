#pragma once
#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pin, float pulses_per_rev, float dt);

    void init();
    void update();
    float get_speed();

private:
    void handle_interrupt();

    uint8_t pin_;
    uint8_t interrupt_num;

    float pulses_per_rev_;
    float dt_;

    volatile unsigned long pulse_count;
    float speed;
    
    static Encoder* instances[2];

    static void isr0();
    static void isr1();

};