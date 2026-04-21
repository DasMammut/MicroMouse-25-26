#include "Hugger.h"

Hugger::Hugger(IRSensors &aIrSensors, Motors &aMotors, Gyro &aGyro)
    : irSensors(aIrSensors), motors(aMotors), gyro(aGyro),
      pidLeftHugger(HUGGER_Kp, HUGGER_Ki, HUGGER_Kd),
      pidLeftFrontHugger(HUGGER_Kp, HUGGER_Ki, HUGGER_Kd),
      pidHugger(HUGGER_Kp, HUGGER_Ki, HUGGER_Kd),
      pidFrontHugger(HUGGER_Kp, HUGGER_Ki, HUGGER_Kd),
      pidRightFrontHugger(HUGGER_Kp, HUGGER_Ki, HUGGER_Kd),
      pidRightHugger(HUGGER_Kp, HUGGER_Ki, HUGGER_Kd),
      pidGyroStraight(GYRO_STRAIGHT_Kp, GYRO_STRAIGHT_Ki, GYRO_STRAIGHT_Kd) {

}

void Hugger::Init() {
    pidLeftHugger.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidLeftHugger.setMode(MODE_AUTO);
    pidLeftFrontHugger.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidLeftFrontHugger.setMode(MODE_AUTO);
    pidHugger.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidHugger.setMode(MODE_AUTO);
    pidFrontHugger.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidFrontHugger.setMode(MODE_AUTO);
    pidRightFrontHugger.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidRightFrontHugger.setMode(MODE_AUTO);
    pidRightHugger.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidRightHugger.setMode(MODE_AUTO);
    pidGyroStraight.setOutputLimits(HUGGER_OUT_MIN, HUGGER_OUT_MAX);
    pidGyroStraight.setMode(MODE_AUTO);
}

void Hugger::hug(TMotorValues &aMSV, bool leftWall, bool leftFrontWall, bool frontWall, bool rightFrontWall, bool rightWall) {
    MSV = aMSV;
    if(leftWall && leftFrontWall && rightWall && rightFrontWall) {
        hugger();
    }
    else if(leftFrontWall && rightFrontWall) {
        frontHugger();
    }
    else if(leftWall && leftFrontWall) {
        leftHugger();
    }
    else if(rightWall && rightFrontWall) {
        rightHugger();
    }

    // else if(leftFrontWall) {
    //     leftFrontHugger();
    // }
    // else if(rightFrontWall) {
    //     rightFrontHugger();
    // }

    else {
        gyroStraight();
    }
    aMSV = MSV;
}

void Hugger::reset() {
    pidLeftHugger.reset();
    pidLeftFrontHugger.reset();
    pidHugger.reset();
    pidFrontHugger.reset();
    pidRightFrontHugger.reset();
    pidRightHugger.reset();
    pidGyroStraight.reset();
}



void Hugger::leftHugger() {
    float left = ((float)irSensors.getLeftCenter() + (float)irSensors.getLeft()) / 2.0;
    float input = left - HUGGER_LEFT_OFFSET;
    float correction = pidLeftHugger.run(input, 0.0);

    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}

void Hugger::leftFrontHugger() {
    float leftFront = (float)irSensors.getLeftCenter();
    float input = leftFront - HUGGER_FRONTLEFT_OFFSET;
    float correction = pidLeftFrontHugger.run(input, 0.0);

    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}

void Hugger::hugger() {
    // WICHTIG: erst zu float casten, DANN subtrahieren – sonst uint16_t overflow!
    float left = ((float)irSensors.getLeftCenter() + (float)irSensors.getLeft()) / 2.0;
    float right = ((float)irSensors.getRightCenter() + (float)irSensors.getRight()) / 2.0;
    float input = left - right + HUGGER_OFFSET;
    float correction = pidHugger.run(input, 0.0);

    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}

void Hugger::frontHugger() {
    float left = (float)irSensors.getLeftCenter();
    float right = (float)irSensors.getRightCenter();
    float input = left - right + HUGGER_FRONT_OFFSET;
    float correction = pidFrontHugger.run(input, 0.0);

    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}


// rightFront
void Hugger::rightFrontHugger() {
    float rightFront = (float)irSensors.getRightCenter();
    float input = HUGGER_FRONTRIGHT_OFFSET - rightFront;
    float correction = pidRightFrontHugger.run(input, 0.0);

    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}


// Rechte-Wand-Folger: hält Abstand zur rechten Wand konstant
void Hugger::rightHugger() {
    float right = ((float)irSensors.getRightCenter() + (float)irSensors.getRight()) / 2.0;
    float input = HUGGER_RIGHT_OFFSET - right;
    float correction = pidRightHugger.run(input, 0.0);

    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}

void Hugger::gyroStraight() {
    // Gyro-basierte Geradeauskorrektur: angle sollte ~0 bleiben
    float correction = gyro.angle * GYRO_STRAIGHT_Kp;
    MSV.left  += (int16_t)correction;
    MSV.right -= (int16_t)correction;
}