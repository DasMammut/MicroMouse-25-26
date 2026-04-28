#include "Ramp.h"

Ramp::Ramp(Motors &aMotors, Impeller &aImpeller) : motors(aMotors), impeller(aImpeller), curMSV{0, 0, 0} {

}

void Ramp::step(const TMotorValues &aMSV) {
    if (curMSV.left < aMSV.left) {
        curMSV.left = min(curMSV.left + RAMP_STEP_UP, aMSV.left);
    } 
    else if (curMSV.left > aMSV.left) {
        curMSV.left = max(curMSV.left - RAMP_STEP_DOWN, aMSV.left);
    }

    if (curMSV.right < aMSV.right) {
        curMSV.right = min(curMSV.right + RAMP_STEP_UP, aMSV.right);
    } 
    else if (curMSV.right > aMSV.right) {
        curMSV.right = max(curMSV.right - RAMP_STEP_DOWN, aMSV.right);
    }

    if(curMSV.impeller < aMSV.impeller) {
        curMSV.impeller = min(curMSV.impeller + IMPELLER_RAMP_STEP_UP, aMSV.impeller);
    }
    else if(curMSV.impeller > aMSV.impeller) {
        curMSV.impeller = max(curMSV.impeller - IMPELLER_RAMP_STEP_DOWN, aMSV.impeller);
    }

    motors.setRPS(curMSV.left, curMSV.right);
    impeller.set(curMSV.impeller);
}