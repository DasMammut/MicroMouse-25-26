#ifndef IRSENSORS_H
#define IRSENSORS_H

#include <Arduino.h>
#include "PinOut.h"

#define LOOK_UP_CONST 5000
#define OUT_OF_RANGE_VALUE 50

class IRSensors {
public:
    IRSensors();

    void Init();

    void update();

    void off();

    uint16_t getLeft();
    uint16_t getLeftCenter();
    uint16_t getCenter();
    uint16_t getRightCenter();
    uint16_t getRight();

private:
    uint16_t left_Distance;
    uint16_t leftcenter_Distance;
    uint16_t center_Distance;
    uint16_t rightcenter_Distance;
    uint16_t right_Distance;
    
    bool LightsOn;

    uint16_t MesON_left_Distance;
    uint16_t MesON_leftcenter_Distance;
    uint16_t MesON_center_Distance;
    uint16_t MesON_rightcenter_Distance;
    uint16_t MesON_right_Distance;

    uint16_t MesOFF_left_Distance;
    uint16_t MesOFF_leftcenter_Distance;
    uint16_t MesOFF_center_Distance;
    uint16_t MesOFF_rightcenter_Distance;
    uint16_t MesOFF_right_Distance;

    uint16_t getDistLookUp(uint16_t aDistance);

};

#endif