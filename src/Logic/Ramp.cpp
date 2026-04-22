#include "Ramp.h"

Ramp::Ramp(Motors &aMotors) : motors(aMotors) {

}

void Ramp::step(const TMotorValues &msv) {
    int tarLeft = msv.left;
    int tarRight = msv.right;

    if (curLeft < tarLeft) {
        curLeft = min(curLeft + RAMP_STEP_UP, tarLeft);
    } 
    else if (curLeft > tarLeft) {
        curLeft = max(curLeft - RAMP_STEP_DOWN, tarLeft);
    }

    if (curRight < tarRight) {
        curRight = min(curRight + RAMP_STEP_UP, tarRight);
    } 
    else if (curRight > tarRight) {
        curRight = max(curRight - RAMP_STEP_DOWN, tarRight);
    }

    motors.setRPS(curLeft, curRight);
}