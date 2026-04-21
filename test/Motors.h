#pragma once

#include <Arduino.h>
#include "PinsCrazyCar.h"

const uint8_t ENCODER_COUNTS_PER_REV = 3;
const uint8_t TCA0_PRESCALER = 8;
const uint32_t TCB_MAX = 50000;

void motorsInit(void);
void getCurrentSpeed(int32_t* rpsL, int32_t* rpsR);
void setMotorPWM(int pwrL, int pwrR);

void EVENT_SYSTEM_init(void);
void TCB0_init(void);
void TCB1_init(void);
