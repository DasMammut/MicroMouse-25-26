#pragma once 

#include <Arduino.h>
#include "../Hardware/Motors.h"

#define RAMP_STEP_UP 5
#define RAMP_STEP_DOWN 5

class Ramp {
public:
    Ramp(Motors &aMotors);

    void set(TMotorValues &aMSV, uint32_t aTicks, int16_t aRPS);

    void step(TMotorValues &aMSV);

private:
    Motors &motors;

    int16_t curLeft;
    int16_t curRight;

};