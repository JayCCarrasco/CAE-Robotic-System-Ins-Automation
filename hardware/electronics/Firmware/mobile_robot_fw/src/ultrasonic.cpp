#include "ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig_pin, uint8_t echo_pin) :
trig_pin_(trig_pin), echo_pin_(echo_pin){};

void Ultrasonic::init(){
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
}

float Ultrasonic::read_distance(){
    digitalWrite(trig_pin_, LOW);
    delayMicroseconds(2);

    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);

    digitalWrite(trig_pin_, LOW);

    //Until 20ms to detect ultrasonic echo
    long duration = pulseIn(echo_pin_, HIGH, 20000);

    float distance = duration * 0.000343 / 2.0;

    return distance;
}