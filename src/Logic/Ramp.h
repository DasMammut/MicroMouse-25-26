#pragma once 

#include <Arduino.h>
#include "../Hardware/Motors.h"
#include "../Hardware/Impeller.h"

#define RAMP_STEP_UP 5
#define RAMP_STEP_DOWN 5
#define IMPELLER_RAMP_STEP_UP 5
#define IMPELLER_RAMP_STEP_DOWN 5

class Ramp {
public:
    Ramp(Motors &aMotors, Impeller &aImpeller);

    void step(const TMotorValues &aMSV);

private:
    Motors &motors;
    Impeller &impeller;

    TMotorValues curMSV;

};