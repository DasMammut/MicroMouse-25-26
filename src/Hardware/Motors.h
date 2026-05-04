#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <AdvancedPID.h>
#include "PinOut.h"

typedef struct {
    int16_t left;
    int16_t right;
    uint8_t impeller;
} TMotorValues;

#define ENCODER_COUNTS_PER_REV 3
#define TCA0_PRESCALER 8
#define TCB_MAX 50000

#define Kp 0.5
#define Ki 15.0
#define Kd 0.0
#define OUTPUT_LIMIT 255
#define DERIVATIVE_FILTER 0.7

extern "C" void TCB0_INT_vect(void);
extern "C" void TCB1_INT_vect(void);

class Motors {
    friend void TCB0_INT_vect(void);
    friend void TCB1_INT_vect(void);
public:
    static Motors* instance;
    volatile int32_t ticksL;
    volatile int32_t ticksR;

    Motors();

    void Init();

    void update();

    void setRPS(int16_t aRPSLeft, int16_t aRPSRight);

    void resetPIDs();
    void resetEncoders();
    int32_t getAVGTicks();  // Durchschnitt beider Encoder

    void stop();

private:
    int16_t RPSsetLeft;
    int16_t RPSsetRight;

    AdvancedPID lPID;
    AdvancedPID rPID;

    volatile int32_t encoderLPeriod;
    volatile int32_t encoderRPeriod;

    void getUpdate(int32_t* aRPSLeft, int32_t* aRPSRight);
    void setPower(int16_t aPowerLeft, int16_t aPowerRight);

    void EVENT_SYSTEM_init();
    void TCB0_init();
    void TCB1_init();
};

#endif