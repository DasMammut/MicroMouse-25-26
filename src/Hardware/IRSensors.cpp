#include "IRSensors.h"

IRSensors::IRSensors() : 
    left_Distance(0), leftcenter_Distance(0), center_Distance(0), rightcenter_Distance(0), right_Distance(0),
    LightsOn(false),
    MesON_left_Distance(0), MesON_leftcenter_Distance(0), MesON_center_Distance(0), MesON_rightcenter_Distance(0), MesON_right_Distance(0),
    MesOFF_left_Distance(0), MesOFF_leftcenter_Distance(0), MesOFF_center_Distance(0), MesOFF_rightcenter_Distance(0), MesOFF_right_Distance(0) {

}

void IRSensors::Init() {
    pinMode(PIN_SEN_L, INPUT);
    pinMode(PIN_SEN_LM, INPUT);
    pinMode(PIN_SEN_M, INPUT);
    pinMode(PIN_SEN_RM, INPUT);
    pinMode(PIN_SEN_R, INPUT);

    pinMode(PIN_IR_L, OUTPUT);
    pinMode(PIN_IR_LM, OUTPUT);
    pinMode(PIN_IR_M, OUTPUT);
    pinMode(PIN_IR_RM, OUTPUT);
    pinMode(PIN_IR_R, OUTPUT);

    setLEDs(false);
}

void IRSensors::update() {
    uint16_t offL  = analogRead(PIN_SEN_L);
    uint16_t offLM = analogRead(PIN_SEN_LM);
    uint16_t offM  = analogRead(PIN_SEN_M);
    uint16_t offRM = analogRead(PIN_SEN_RM);
    uint16_t offR  = analogRead(PIN_SEN_R);

    // 2. Messung MIT Licht (Signal + Ambient)
    setLEDs(true); 
    delayMicroseconds(400); // Warten, bis die IR-LEDs voll leuchten und Fototransistor reagiert
    
    uint16_t onL  = analogRead(PIN_SEN_L);
    uint16_t onLM = analogRead(PIN_SEN_LM);
    uint16_t onM  = analogRead(PIN_SEN_M);
    uint16_t onRM = analogRead(PIN_SEN_RM);
    uint16_t onR  = analogRead(PIN_SEN_R);

    // 3. LEDs sofort wieder aus (Strom sparen & Wärme reduzieren)
    setLEDs(false);

    // 4. Differenz berechnen
    left_Distance = offL - onL;
    leftcenter_Distance = offLM - onLM;
    center_Distance = offM - onM;
    rightcenter_Distance = offRM - onRM;
    right_Distance = offR - onR;
}

// Hilfsfunktion um Schreibarbeit zu sparen
void IRSensors::setLEDs(bool state) {
    // Falls deine LEDs bei LOW angehen, nutze !state
    digitalWrite(PIN_IR_L, !state);
    digitalWrite(PIN_IR_LM, !state);
    digitalWrite(PIN_IR_M, !state);
    digitalWrite(PIN_IR_RM, !state);
    digitalWrite(PIN_IR_R, !state);
}

void IRSensors::off() {
    LightsOn = false;
    setLEDs(false);
}

uint16_t IRSensors::getLeft() {
    return getDistLookUp(left_Distance);
}

uint16_t IRSensors::getLeftCenter() {
    return getDistLookUp(leftcenter_Distance);
}

uint16_t IRSensors::getCenter() {
    return getDistLookUp(center_Distance);
}

uint16_t IRSensors::getRightCenter() {
    return getDistLookUp(rightcenter_Distance);
}

uint16_t IRSensors::getRight() {
    return getDistLookUp(right_Distance);
}

uint16_t IRSensors::getLeftRaw() {
    return left_Distance;
}

uint16_t IRSensors::getLeftCenterRaw() {
    return leftcenter_Distance;
}

uint16_t IRSensors::getCenterRaw() {
    return center_Distance;
}

uint16_t IRSensors::getRightCenterRaw() {
    return rightcenter_Distance;
}

uint16_t IRSensors::getRightRaw() {
    return right_Distance;
}

uint16_t IRSensors::getDistLookUp(uint16_t aDistance){
    return (LOOK_UP_FACTOR_CONST)/(aDistance + LOOK_UP_OFFSET_CONST);
}