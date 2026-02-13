#include "differential_drive.h"

DifferentialDrive::DifferentialDrive(float radius, float distance) :
radius_(radius), distance_(distance){}

void DifferentialDrive::compute(float vx, float wz, float& wleft, float& wright){
    float vleft = vx - (wz * distance_) / 2.0;
    float vright = vx + (wz * distance_) / 2.0;

    wleft = vleft / radius_;
    wright = vright / radius_;
}