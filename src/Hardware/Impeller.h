#ifndef IMPELLER_H
#define IMPELLER_H

#include <Arduino.h>
#include "PinOut.h"

#define MAX_POWER 90

#define PRESCALER_TIM_A 8
#define PERIOD_CMP_VAL (F_CPU / PRESCALER_TIM_A / 50)
#define MILLISEC_CNT (F_CPU / PRESCALER_TIM_A / 1000)

extern "C" void TCB3_INT_vect(void);

class Impeller {
    friend void TCB3_INT_vect(void);
public:
    static Impeller* instance;

    Impeller();

    void Init();

    void update();

    void set(uint8_t aDutyCycle);
    void stop();


private:
    uint8_t DutyCycle; // 0 - 255 => map to => 1 - 2 ms
    uint8_t setDutyCycle;

    volatile bool isDutyPart;
    volatile uint16_t Duty_CNT;
    volatile uint16_t Top_CNT;
};


#endif