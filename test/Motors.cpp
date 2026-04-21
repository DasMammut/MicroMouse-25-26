#include "Motors.h"

int32_t encoderLPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;
int32_t encoderRPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;

// initialize the motors
void motorsInit(void) {
  EVENT_SYSTEM_init();
  TCB0_init();
  TCB1_init();
}

// get the current motor speed in revolutions per second
void getCurrentSpeed(int32_t *rpsL, int32_t *rpsR) {
  cli();
    // left
    if (TCB0.CNT > TCB_MAX) {
      encoderLPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;  // 350000 = 7 couts per rev, 150000 = 3 counts per rev
      *rpsL = 0;
      TCB0.CNT = TCB_MAX;
    }
    else *rpsL = F_CPU / TCA0_PRESCALER / encoderLPeriod;  // 2500000 timer ticks per second @20MHz
  sei();
  
  cli();
    // right
    if (TCB1.CNT > TCB_MAX) {
      encoderRPeriod = TCB_MAX * ENCODER_COUNTS_PER_REV;  // 350000 = 7 couts per rev, 150000 = 3 counts per rev
      *rpsR = 0;
      TCB1.CNT = TCB_MAX;
    }
    else *rpsR = F_CPU / TCA0_PRESCALER / encoderRPeriod;  // 2500000 timer ticks per second @20MHz
  sei();
}

// set the motor power (+/- 255)
void setMotorPWM(int pwrL, int pwrR) {
  if (pwrL > 0) {
    digitalWrite(PIN_MOTL_F, LOW);
    digitalWrite(PIN_MOTL_B, HIGH);
    analogWrite(PIN_MOTL_PWM, pwrL);
  } else {
    digitalWrite(PIN_MOTL_F, HIGH);
    digitalWrite(PIN_MOTL_B, LOW);
    analogWrite(PIN_MOTL_PWM, -pwrL); 
  }
  
  if (pwrR > 0) {
    digitalWrite(PIN_MOTR_B, HIGH);
    digitalWrite(PIN_MOTR_F, LOW);
    analogWrite(PIN_MOTR_PWM, pwrR);
  } else {
    digitalWrite(PIN_MOTR_B, LOW);
    digitalWrite(PIN_MOTR_F, HIGH);
    analogWrite(PIN_MOTR_PWM, -pwrR);
  }
}

void EVENT_SYSTEM_init(void) {
  // TCB0 (left)
  EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT0_PIN1_gc; // PA1 is Event generator Port0, Pin1
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;   // route EVSYS_Channel0 to TCB0

  // TCB1 (right)
  EVSYS.CHANNEL2 = EVSYS_GENERATOR_PORT1_PIN1_gc; // PD1 is Event generator Port1, Pin1
  EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL2_gc;   // route EVSYS_Channel2 to TCB1
}

void TCB0_init(void) {
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;  // Clk von TCA0, enable
  TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;  // Frequency measurement mode
  TCB0.EVCTRL = TCB_CAPTEI_bm;  // Capture event input, positive edge
  TCB0.INTCTRL = TCB_CAPT_bm;   // Capture Interrupt Enable
}

void TCB1_init(void) {
  TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;  // Clk von TCA0, enable
  TCB1.CTRLB = TCB_CNTMODE_FRQ_gc;  // Frequency measurement mode
  TCB1.EVCTRL = TCB_CAPTEI_bm;  // Capture event input, positive edge
  TCB1.INTCTRL = TCB_CAPT_bm;   // Capture Interrupt Enable
}

ISR(TCB0_INT_vect)
{
  static uint8_t rotationCounter = 0;
  static int32_t periodTime = 0;

  if (digitalReadFast(PIN_ENC_LB)) periodTime += TCB0.CCMP;
  else periodTime -= TCB0.CCMP;

  rotationCounter++;

  if(rotationCounter >= ENCODER_COUNTS_PER_REV) {
    encoderLPeriod = periodTime;
    rotationCounter = periodTime = 0;
  }

  //PORTA.OUTTGL = PIN0_bm; /* Toggle PA0 GPIO */
}

ISR(TCB1_INT_vect)
{
  static uint8_t rotationCounter = 0;
  static int32_t periodTime = 0;

  if (!digitalReadFast(PIN_ENC_RB)) periodTime += TCB1.CCMP;
  else periodTime -= TCB1.CCMP;

  rotationCounter++;

  if(rotationCounter >= ENCODER_COUNTS_PER_REV) {
    encoderRPeriod = periodTime;
    rotationCounter = periodTime = 0;
  }

  //PORTA.OUTTGL = PIN0_bm; /* Toggle PA0 GPIO */
}