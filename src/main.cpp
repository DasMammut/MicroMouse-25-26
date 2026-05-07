#include <Arduino.h>

#include "PinOut.h"
#include "Hardware/Motors.h"
#include "Hardware/IRSensors.h"
#include "Hardware/Impeller.h"
#include "Hardware/Gyro.h"
#include "Logic/Pilot.h"


#define MIN_AKKU_START_VAL 680
#define MIN_AKKU_VAL 670

#define START_DISTANCE 60

#define DEBUG // _X to disable debug output
#define DEBUG_MILLIS 200

Motors motors;
IRSensors irSensors;
Impeller impeller;
Gyro gyro;
Pilot pilot(irSensors, motors, impeller, gyro);

void startCondition();
void debugOutput();

void setup() {
    Serial2.swap(1);
    Serial2.begin(115200);

    pinMode(PIN_AKKU, INPUT); 
    pinMode(PIN_TEMP_START, INPUT_PULLUP); // Temp Start Button

    analogWriteFrequency(8); // Temp. Test: 8khz Hz für Motoren, damit Impeller geht mit 8er Prescaller

    pilot.Init();

    startCondition();

    motors.resetEncoders(); // TCB-Counter zurücksetzen nach langer Wartezeit in startCondition
}

void loop() {
    pilot.loop();

    if(analogRead(PIN_AKKU) < MIN_AKKU_VAL) {
        // Akku fast leer, sofort anhalten
        motors.stop();
        impeller.stop();
        irSensors.off();
        while(true); // Endlosschleife, damit der Akku nicht weiter entladen wird
    }

    #ifdef DEBUG
    static uint32_t lastDebug = 0;
    if(millis() - lastDebug >= DEBUG_MILLIS) {
        lastDebug = millis();
        debugOutput();
    }
    #endif
}

void startCondition() {
    while(digitalRead(PIN_TEMP_START) != LOW || analogRead(PIN_AKKU) < MIN_AKKU_START_VAL);
    delay(200);
}


void debugOutput(){
    Serial2.print(" | L:    ");Serial2.println(irSensors.getLeft());
    Serial2.print(" | LM:   ");Serial2.println(irSensors.getLeftCenter());
    Serial2.print(" | M:   ");Serial2.println(irSensors.getCenter());
    Serial2.print(" | RM:   ");Serial2.println(irSensors.getRightCenter());
    Serial2.print(" | R:    ");Serial2.println(irSensors.getRight());
    Serial2.print(" | TicksL: ");Serial2.println(motors.ticksL);
    Serial2.print(" | TicksR: ");Serial2.println(motors.ticksR);
    Serial2.print(" | TicksAVG: ");Serial2.println(motors.getAVGTicks());
    Serial2.print(" | Gyro angle: ");Serial2.println(gyro.angle);
    Serial2.print(" | Gyro absAngle: ");Serial2.println(gyro.absAngle);
    Serial2.print(" | Gyro shouldAngle: ");Serial2.println(gyro.shouldAbsAngle);
    Serial2.print(" | Akku: ");Serial2.println(analogRead(PIN_AKKU));
    Serial2.println("--------------------------------------------------");
}

