#include "Motors.h"

Motors* Motors::instance = nullptr;

Motors::Motors() : RPSsetLeft(0), RPSsetRight(0),
    lPID(Kp, Ki, Kd), rPID(Kp, Ki, Kd),
    encoderLPeriod(TCB_MAX * ENCODER_COUNTS_PER_REV),
    encoderRPeriod(TCB_MAX * ENCODER_COUNTS_PER_REV) {
    instance = this;
}

void Motors::Init() {
    instance = this;

    pinMode(PIN_MOTL_PWM, OUTPUT);
    pinMode(PIN_MOTL_F, OUTPUT);
    pinMode(PIN_MOTL_B, OUTPUT);

    pinMode(PIN_MOTR_PWM, OUTPUT);
    pinMode(PIN_MOTR_F, OUTPUT);
    pinMode(PIN_MOTR_B, OUTPUT);

    EVENT_SYSTEM_init();
    TCB0_init();
    TCB1_init();

    lPID.setOutputLimits(-OUTPUT_LIMIT, OUTPUT_LIMIT);
    lPID.setDerivativeFilter(DERIVATIVE_FILTER);  
    rPID.setOutputLimits(-OUTPUT_LIMIT, OUTPUT_LIMIT);
    rPID.setDerivativeFilter(DERIVATIVE_FILTER);
}


void Motors::setRPS(int16_t aRPSsetLeft, int16_t aRPSsetRight) {
    RPSsetLeft = aRPSsetLeft;
    RPSsetRight = aRPSsetRight;
}

void Motors::resetEncoders() {
    cli();
    encoderLPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;
    encoderRPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;
    TCB0.CNT = TCB_MAX + 1;  // Erzwingt 0 RPS beim nächsten getUpdate
    TCB1.CNT = TCB_MAX + 1;
    sei();
}

int32_t Motors::getAVGTicks() {
    cli();
    int32_t avg = (ticksL + ticksR) / 2;
    sei();
    return avg;
}

void Motors::update() {
    int32_t curRPSLeft, curRPSRight;
    getUpdate(&curRPSLeft, &curRPSRight);
    float motorPowerL = lPID.run(curRPSLeft, RPSsetLeft, RPSsetLeft/2, NAN);
    float motorPowerR = rPID.run(curRPSRight, RPSsetRight, RPSsetRight/2, NAN);
    if(RPSsetLeft == 0) motorPowerL = 0; // Verhindert Ruckeln bei Stillstand
    if(RPSsetRight == 0) motorPowerR = 0; // Verhindert Ruckeln bei Stillstand
    setPower(motorPowerL, motorPowerR);
}

void Motors::setPower(int16_t aPowerLeft, int16_t aPowerRight) {
    if (aPowerLeft > 0) {
        digitalWrite(PIN_MOTL_F, LOW);
        digitalWrite(PIN_MOTL_B, HIGH);
        analogWrite(PIN_MOTL_PWM, aPowerLeft);
    }
    else {
        digitalWrite(PIN_MOTL_F, HIGH);
        digitalWrite(PIN_MOTL_B, LOW);
        analogWrite(PIN_MOTL_PWM, -aPowerLeft);
    }

    if (aPowerRight > 0) {
        digitalWrite(PIN_MOTR_B, HIGH);
        digitalWrite(PIN_MOTR_F, LOW);
        analogWrite(PIN_MOTR_PWM, aPowerRight);
    }
    else {
        digitalWrite(PIN_MOTR_B, LOW);
        digitalWrite(PIN_MOTR_F, HIGH);
        analogWrite(PIN_MOTR_PWM, -aPowerRight);
    }
}

void Motors::stop() {
    RPSsetLeft = 0;
    RPSsetRight = 0;
    analogWrite(PIN_MOTL_PWM, 0);
    analogWrite(PIN_MOTR_PWM, 0);
    digitalWrite(PIN_MOTL_F, LOW);
    digitalWrite(PIN_MOTL_B, LOW);
    digitalWrite(PIN_MOTR_F, LOW);
    digitalWrite(PIN_MOTR_B, LOW);
}

void Motors::getUpdate(int32_t* aRPSLeft, int32_t* aRPSRight) {
    cli();
    // left
    if (TCB0.CNT > TCB_MAX) {
        encoderLPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;
        *aRPSLeft = 0;
        TCB0.CNT = TCB_MAX;
    }
    else {
        *aRPSLeft = F_CPU / TCA0_PRESCALER / encoderLPeriod;  // 2500000 timer ticks per second @20MHz
    }
    sei();

    cli();
    // right
    if (TCB1.CNT > TCB_MAX) {
        encoderRPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;
        *aRPSRight = 0;
        TCB1.CNT = TCB_MAX;
    }
    else {
        *aRPSRight = F_CPU / TCA0_PRESCALER / encoderRPeriod;  // 2500000 timer ticks per second @20MHz
    }
    sei();
}

void Motors::EVENT_SYSTEM_init() {
    // TCB0 (left)
    EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT0_PIN1_gc; // PA1 is Event generator Port0, Pin1
    EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;     // route EVSYS_Channel0 to TCB0

    // TCB1 (right)
    EVSYS.CHANNEL2 = EVSYS_GENERATOR_PORT1_PIN1_gc; // PD1 is Event generator Port1, Pin1
    EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL2_gc;     // route EVSYS_Channel2 to TCB1
}

void Motors::TCB0_init() {
    TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;  // Clk von TCA0, enable
    TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;  // Frequency measurement mode
    TCB0.EVCTRL = TCB_CAPTEI_bm;      // Capture event input, positive edge
    TCB0.INTCTRL = TCB_CAPT_bm;       // Capture Interrupt Enable
}

void Motors::TCB1_init() {
    TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;  // Clk von TCA0, enable
    TCB1.CTRLB = TCB_CNTMODE_FRQ_gc;  // Frequency measurement mode
    TCB1.EVCTRL = TCB_CAPTEI_bm;      // Capture event input, positive edge
    TCB1.INTCTRL = TCB_CAPT_bm;       // Capture Interrupt Enable
}

ISR(TCB0_INT_vect)
{
    static uint8_t rotationCounter = 0;
    static int32_t periodTime = 0;

    if (digitalReadFast(PIN_ENC_LB)) {
        periodTime += TCB0.CCMP;
        Motors::instance->ticksL++;
    } 
    else {
        periodTime -= TCB0.CCMP;
        Motors::instance->ticksL--;
    }

    rotationCounter++;

    if (rotationCounter >= ENCODER_COUNTS_PER_REV) {
        Motors::instance->encoderLPeriod = periodTime;
        rotationCounter = periodTime = 0;
    }

    TCB0.INTFLAGS = TCB_CAPT_bm;
    
}

ISR(TCB1_INT_vect)
{
    static uint8_t rotationCounter = 0;
    static int32_t periodTime = 0;

    if (!digitalReadFast(PIN_ENC_RB)) {
        periodTime += TCB1.CCMP;
        Motors::instance->ticksR++;
    }
    else {
        periodTime -= TCB1.CCMP;
        Motors::instance->ticksR--;
    }

    rotationCounter++;

    if (rotationCounter >= ENCODER_COUNTS_PER_REV) {
        Motors::instance->encoderRPeriod = periodTime;
        rotationCounter = periodTime = 0;
    }

    TCB1.INTFLAGS = TCB_CAPT_bm;
  //PORTA.OUTTGL = PIN0_bm; /* Toggle PA0 GPIO */
}