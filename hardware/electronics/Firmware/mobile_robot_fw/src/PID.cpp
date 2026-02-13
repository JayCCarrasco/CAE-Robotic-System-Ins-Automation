#include "PID.h"

PID::PID(float kp, float ki, float kd, float dt, float out_min, float out_max) :
kp_(kp), ki_(ki), kd_(kd), dt_(dt), out_min_(out_min), out_max_(out_max), integral(0), prev_error(0){}

float PID::compute(float setpoint, float measurement){
    float error = setpoint - measurement;

    integral += error * dt_;
    float derivative = (error - prev_error) / dt_;

    float output = kp_ * error +
                   ki_ * integral +
                   kd_ * derivative;

    prev_error = error;
    
    if (output > out_max_) output = out_max_;
    if (output < out_min_) output = out_min_;

    return output;
}

void PID::reset(){
    integral = 0;
    prev_error = 0;
}