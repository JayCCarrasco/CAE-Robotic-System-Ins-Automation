#include "imu.h"

Imu::Imu(uint8_t address) :
address_(address), gyro_Z(0){}


void Imu::init(){
    Wire.begin();

    Wire.beginTransmission(address_);
    Wire.write(0x68);
    Wire.write(0);
    Wire.endTransmission(true);
}

void Imu::update(){
    Wire.beginTransmission(address_);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(address_, 2, true);

    int16_t raw = Wire.read() << 8 | Wire.read();
    gyro_Z = raw / 131.0;
}

float Imu::get_gyro_Z(){
    return gyro_Z;
}