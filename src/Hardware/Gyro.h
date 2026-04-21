#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>
#include "../PinOut.h"

class Gyro {
public:
    void Init();
    void update();      
    float angle;        // Direktzugriff für Speed
    void reset() { angle = 0; }

private:
    float biasZ;
    uint32_t lastMicros;
};

#endif