#ifndef HUGGER_H
#define HUGGER_H

#include <Arduino.h>
#include <AdvancedPID.h>
#include "IRSensors.h"
#include "Motors.h"
#include "Gyro.h"

#define HUGGER_LEFT_OFFSET 18
#define HUGGER_FRONTLEFT_OFFSET 18
#define HUGGER_OFFSET -4
#define HUGGER_FRONT_OFFSET -6
#define HUGGER_FRONTRIGHT_OFFSET 16
#define HUGGER_RIGHT_OFFSET 12
#define HUGGER_Kp 2.5
#define HUGGER_Ki 0.01
#define HUGGER_Kd 0.0
#define HUGGER_OUT_MIN -60
#define HUGGER_OUT_MAX  60

#define GYRO_STRAIGHT_Kp 1.5
#define GYRO_STRAIGHT_Ki 0.0
#define GYRO_STRAIGHT_Kd 0.0

class Hugger {
public:
    Hugger(IRSensors &aIrSensors, Motors &aMotors, Gyro &aGyro);
    
    void Init();

    void hug(TMotorValues &aMSV, bool leftWall, bool leftFrontWall, bool frontWall, bool rightFrontWall, bool rightWall);

    void reset();

private:
    TMotorValues MSV;
    IRSensors &irSensors;
    Motors &motors;
    Gyro &gyro;

    AdvancedPID pidLeftHugger;
    AdvancedPID pidLeftFrontHugger;
    AdvancedPID pidHugger;
    AdvancedPID pidFrontHugger;
    AdvancedPID pidRightFrontHugger;
    AdvancedPID pidRightHugger;
    AdvancedPID pidGyroStraight;
    
    void leftHugger();  // An linker Wand entlangfahren
    void leftFrontHugger(); // An linker Wand entlangfahren, wenn vorne links eine Wand ist
    void hugger();      // Mittelzentrierung: gleicher Abstand links & rechts
    void frontHugger(); // An Vorderwand entlangfahren
    void rightFrontHugger(); // An rechter Wand entlangfahren, wenn vorne rechts eine Wand ist
    void rightHugger(); // An rechter Wand entlangfahren
    void gyroStraight(); // Geradeauskorrektur per Gyro wenn keine Wände sichtbar


};


#endif