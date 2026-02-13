#include "motors.h"
#include <Arduino.h>

Motor :: Motor(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin) :
    pwm_pin_(pwm_pin), in1_pin_(in1_pin), in2_pin_(in2_pin){}

void Motor::init(){
    pinMode(pwm_pin_, OUTPUT);
    pinMode(in1_pin_, OUTPUT);
    pinMode(in2_pin_, OUTPUT);
    stop();
}

void Motor::set_speed(int speed){
    speed = constrain(speed, -255, 255);

    if(speed > 0){
        digitalWrite(in1_pin_, HIGH);
        digitalWrite(in2_pin_, LOW);
        analogWrite(pwm_pin_, speed);
    } else if (speed < 0){
        digitalWrite(in1_pin_, LOW);
        digitalWrite(in2_pin_, HIGH);
        analogWrite(pwm_pin_, -speed);
    } else {
        digitalWrite(in1_pin_, LOW);
        digitalWrite(in2_pin_, LOW);
    }  
}

void Motor::stop(){
    digitalWrite(in1_pin_, LOW);
    digitalWrite(in2_pin_, LOW);
    analogWrite(pwm_pin_, 0);
}