#pragma once

class PID {
public:
    PID(float kp, float ki, float kd, float dt, float out_min, float out_max);

    float compute(float setpoint, float measurement);
    void reset();

private:
    float kp_, ki_, kd_;
    float dt_;
    float out_min_, out_max_;

    float integral;
    float prev_error;
};