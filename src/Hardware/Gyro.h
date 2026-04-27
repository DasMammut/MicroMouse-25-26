#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>
#include "../PinOut.h"

class Gyro {
public:
    Gyro();

    void Init();
    void update();

    float angle;        // Direktzugriff für Speed
    float absAngle;     // absolute Drehung seit letztem Reset (kann >360° werden)
    float shouldAbsAngle; 
    
    void reset() { angle = 0; }
    void resetAbs() {absAngle = 0;}

    void turnTo(float relAngle);

private:
    float biasZ;
    uint32_t lastMicros;
};

#endif