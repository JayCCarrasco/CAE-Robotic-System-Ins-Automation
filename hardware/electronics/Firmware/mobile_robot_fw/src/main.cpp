#include <Arduino.h>
#include "pins.h"
#include "motors.h"
#include "encoders.h"
#include "control.h"
#include <Wire.h>

//Motor objects
Motor r_motor(PWM_A, AIN_1, AIN_2);
Motor l_motor(PWM_B, BIN_1, BIN_2);

void setup() {
    Wire.begin();
    Serial.begin(9600);
    Serial.println("Mobile Robot Firmware START");

    r_motor.init();
    l_motor.init();
    Serial.println("Motors initialized");
}

void loop() {      
    //Motor testing
    Serial.println("Robot forward");
    r_motor.set_speed(150);
    l_motor.set_speed(150);
    delay(2000);

    Serial.println("Robot backward");
    r_motor.set_speed(-150);
    l_motor.set_speed(-150);
    delay(2000);

    Serial.println("Stop");
    l_motor.stop();
    r_motor.stop();
    delay(2000);
}
