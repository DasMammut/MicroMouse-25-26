#include "Motors.h"
#include <AdvancedPID.h>

// Timer Verwendung:
// Millis/Micros: TCB2
// Servo: TCB3 (verw. für Lüfter)
// PWM: TCA0, Split Mode, 8kHz, CLK-Prescaler=8 (verw. für Motoren)
// Encoder: links: TCB0 (0IN1, 0IN2), rechts: TCB1 (2IN1, 2IN2)
// Tone: TCB1 (hier nicht verwendet)

uint8_t start;

const int16_t setSpeedL = 100;
const int16_t setSpeedR = 100;
int32_t currentSpeedR, currentSpeedL;
float motorPowerL, motorPowerR;
uint32_t tnow, tdiff;

// --- PID Variablen Global ---
float Kp = 0.5;
float Ki = 15.0;
float Kd = 0.0;

uint32_t tAkku;

// Kp, Ki, Kd, Kb (Back-calculation coefficient)
AdvancedPID lPID(Kp, Ki, Kd, 0.0); 
AdvancedPID rPID(Kp, Ki, Kd, 0.0); 

void setup() {
  pinMode(PIN_SW_ON, INPUT_PULLUP);

  pinMode(PIN_LED_DEB, OUTPUT);

  pinMode(PIN_MOTL_F, OUTPUT);
  pinMode(PIN_MOTL_B, OUTPUT);
  pinMode(PIN_MOTR_F, OUTPUT);
  pinMode(PIN_MOTR_B, OUTPUT);
  pinMode(PIN_MOTL_PWM, OUTPUT);
  pinMode(PIN_MOTR_PWM, OUTPUT);

  analogWriteFrequency(8);
  
  Serial2.swap(1);
  Serial2.begin(115200);
  delay(1000);

  Serial2.println("Hello");
  Serial2.println(F_CPU);

  motorsInit();
  
  lPID.setOutputLimits(-255, 255);
  // Enable derivative filter (0.8 = strong smoothing, 1.0 = OFF)
  lPID.setDerivativeFilter(0.7);  
  
  rPID.setOutputLimits(-255, 255);
  // Enable derivative filter (0.8 = strong smoothing, 1.0 = OFF)
  rPID.setDerivativeFilter(0.7);  
}

// ===== LOOP =====
void loop() {
  // Start und Akkuspannung überwachen
  int akku = analogRead(PIN_AKKU);
 
  if (digitalRead(PIN_SW_ON) == LOW && akku > 600) {
    start = 1;
  }
  if (akku < 600) {
    if(millis() > tAkku+5000){
    start = 0;}
  } 
  else {
    tAkku = millis();
  }
  
  // read current motor speed
  getCurrentSpeed(&currentSpeedL, &currentSpeedR);
  
  // speed PID controller
  if (start == 1) {
    // Run PID with advanced features
    tnow = micros();
    motorPowerL = lPID.run(currentSpeedL, setSpeedL, setSpeedL/2, NAN);  // NAN: no external derivative
    motorPowerR = rPID.run(currentSpeedR, setSpeedR, setSpeedR/2, NAN);  // NAN: no external derivative
    tdiff = micros() - tnow;

    
  } else {
    setMotorPWM(0, 0);
  }

  // set motor power
  setMotorPWM(motorPowerL, motorPowerR);
  Serial2.print(currentSpeedL);
    Serial2.print("\t");
    Serial2.print(currentSpeedR);
    Serial2.print("\t");
    Serial2.print(motorPowerL);
    Serial2.print("\t");
    Serial2.println(motorPowerR);
    Serial2.flush();
  //delay(5);
}
