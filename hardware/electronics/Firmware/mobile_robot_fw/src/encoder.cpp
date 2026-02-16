#include "encoder.h"

//Constructor
Encoder::Encoder(uint8_t pin, float pulses_per_rev, float dt) :
pin_(pin), pulses_per_rev_(pulses_per_rev), dt_(dt), pulse_count(0), speed(0){}

void Encoder::init(){
    pinMode(pin_, INPUT_PULLUP);

    interrupt_num = digitalPinToInterrupt(pin_);

    if (interrupt_num == NOT_AN_INTERRUPT) return;

    if (interrupt_num < 2){
        instances[interrupt_num] = this;

        if (interrupt_num == 0){
            attachInterrupt(interrupt_num, isr0, RISING);
        } else if (interrupt_num == 1){
            attachInterrupt(interrupt_num, isr1, RISING);
        }
    }
}

void Encoder::isr0(){
    if (instances[0] != nullptr){
        instances[0] -> handle_interrupt();
    }
}

void Encoder::isr1(){
    if (instances[1] != nullptr){
        instances[1] -> handle_interrupt();
    }
}

void Encoder::handle_interrupt(){
    pulse_count++;
}

void Encoder::update(){
    noInterrupts();
    unsigned long count = pulse_count;
    pulse_count = 0;
    interrupts();

    float rad_per_pulse = (2.0 * PI) / pulses_per_rev_;
    speed = (count * rad_per_pulse) / dt_;
}

float Encoder::get_speed(){
    return speed;
}