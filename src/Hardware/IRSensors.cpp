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

    digitalWrite(PIN_IR_L, !LightsOn);
    digitalWrite(PIN_IR_LM, !LightsOn);
    digitalWrite(PIN_IR_M, !LightsOn);
    digitalWrite(PIN_IR_RM, !LightsOn);
    digitalWrite(PIN_IR_R, !LightsOn);
}

void IRSensors::update() {
    if(LightsOn){
        MesON_left_Distance = analogRead(PIN_SEN_L);
        MesON_leftcenter_Distance = analogRead(PIN_SEN_LM);
        MesON_center_Distance = analogRead(PIN_SEN_M);
        MesON_rightcenter_Distance = analogRead(PIN_SEN_RM);
        MesON_right_Distance = analogRead(PIN_SEN_R);
        
    }
    else{
        MesOFF_left_Distance = analogRead(PIN_SEN_L);
        MesOFF_leftcenter_Distance = analogRead(PIN_SEN_LM);
        MesOFF_center_Distance = analogRead(PIN_SEN_M);
        MesOFF_rightcenter_Distance = analogRead(PIN_SEN_RM);
        MesOFF_right_Distance = analogRead(PIN_SEN_R);
    }
    left_Distance = MesOFF_left_Distance - MesON_left_Distance;
    leftcenter_Distance = MesOFF_leftcenter_Distance - MesON_leftcenter_Distance;
    center_Distance = MesOFF_center_Distance - MesON_center_Distance;
    rightcenter_Distance = MesOFF_rightcenter_Distance - MesON_rightcenter_Distance;
    right_Distance = MesOFF_right_Distance - MesON_right_Distance;

    LightsOn = !LightsOn;
    digitalWrite(PIN_IR_L, !LightsOn);
    digitalWrite(PIN_IR_LM, !LightsOn);
    digitalWrite(PIN_IR_M, !LightsOn);
    digitalWrite(PIN_IR_RM, !LightsOn);
    digitalWrite(PIN_IR_R, !LightsOn);
}

void IRSensors::off() {
    LightsOn = false;
    digitalWrite(PIN_IR_L, !LightsOn);
    digitalWrite(PIN_IR_LM, !LightsOn);
    digitalWrite(PIN_IR_M, !LightsOn);
    digitalWrite(PIN_IR_RM, !LightsOn);
    digitalWrite(PIN_IR_R, !LightsOn);
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

uint16_t IRSensors::getDistLookUp(uint16_t aDistance){
    if(aDistance < OUT_OF_RANGE_VALUE) {
        aDistance = OUT_OF_RANGE_VALUE;
    }
    return LOOK_UP_CONST / aDistance;
}