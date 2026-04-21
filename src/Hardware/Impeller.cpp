#include "Impeller.h"

Impeller* Impeller::instance = nullptr;

Impeller::Impeller() : DutyCycle(0), setDutyCycle(0), isDutyPart(false), Duty_CNT(0), Top_CNT(0) {
    instance = this;
}

void Impeller::Init() {
    instance = this;

    pinMode(PIN_IMP, OUTPUT);
    digitalWrite(PIN_IMP, LOW);

    // Initialwerte setzen bevor Timer startet
    setDutyCycle = 0;
    DutyCycle = 0;
    Duty_CNT = MILLISEC_CNT;          // 1ms (Servo-Minimum)
    Top_CNT = PERIOD_CMP_VAL - Duty_CNT;

    TCB3.CTRLA = 0; 
    
    TCB3.CTRLB = TCB_CNTMODE_INT_gc; 

    TCB3.INTCTRL = TCB_CAPT_bm; 

    isDutyPart = true;
    TCB3.CCMP = Duty_CNT; 

    TCB3.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
}

void Impeller::update() {
    DutyCycle = setDutyCycle;
    Duty_CNT = map(DutyCycle, 0, 255, MILLISEC_CNT, MILLISEC_CNT * 2);
    Top_CNT = PERIOD_CMP_VAL - Duty_CNT;
}

void Impeller::set(uint8_t aDutyCycle) {
    setDutyCycle = aDutyCycle;
}

void Impeller::stop() {
    DutyCycle = 0;
    update();
}



ISR(TCB3_INT_vect) {
    if(Impeller::instance->isDutyPart) {
        digitalWrite(PIN_IMP, LOW);
        TCB3.CCMP = Impeller::instance->Top_CNT;
        Impeller::instance->isDutyPart = false;
    } 
    else {
        digitalWrite(PIN_IMP, HIGH);
        TCB3.CCMP = Impeller::instance->Duty_CNT;
        Impeller::instance->isDutyPart = true;
    }
    TCB3.INTFLAGS = TCB_CAPT_bm;
}