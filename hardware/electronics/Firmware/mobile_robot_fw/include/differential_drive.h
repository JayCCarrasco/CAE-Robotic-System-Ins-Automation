#pragma once

class DifferentialDrive {
public:
    DifferentialDrive(float radius, float distance);

    void compute(float vx, float wz, float& wleft, float& wright);

private:
    float radius_;
    float distance_;
};