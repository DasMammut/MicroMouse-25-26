#pragma once 

#include <Arduino.h>
#include "../Hardware/Motors.h"

#define RAMP_STEP_UP 10
#define RAMP_STEP_DOWN 10

class Ramp {
public:
    Ramp(Motors &aMotors);

    void step(const TMotorValues &msv);

private:
    Motors &motors;
    uint16_t curLeft;
    uint16_t curRight;

};