#include "infrarred.h"

Infrarred::Infrarred(uint8_t xshut_pin, uint8_t address) :
xshut_pin_(xshut_pin), address_(address){};

void Infrarred::init(){
    pinMode(xshut_pin_, OUTPUT);
    digitalWrite(xshut_pin_, LOW);
    delay(10);

    digitalWrite(xshut_pin_, HIGH);
    delay(10);

    sensor.init();
    sensor.setAddress(address_);
    sensor.startContinuous();
}

uint16_t Infrarred::read_distance(){
    return sensor.readRangeContinuousMillimeters();
}