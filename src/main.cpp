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

    // Temp. Test: Stop-Button und Akku-Überwachung
    if(digitalRead(PIN_TEMP_START) == LOW) {
        motors.stop();
        impeller.stop();
        irSensors.off();
        startCondition();
    }

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
    irSensors.update();
    delay(10);
    irSensors.update(); 

    while(irSensors.getCenter() > START_DISTANCE) {
        irSensors.update();
        delay(20); 
    }

    bool readyToGo = false;
    while(!readyToGo) {
        irSensors.update();
        int akku = analogRead(PIN_AKKU);

        if(irSensors.getCenter() > START_DISTANCE && akku > MIN_AKKU_START_VAL) {
            readyToGo = true;
        }
        
        delay(20);
    }

    #ifdef DEBUG
    debugOutput();
    #endif
}


void debugOutput(){
    const bool leftWall = irSensors.getLeft() < MES_WALL_SIDE_THRESHOLD;
    const bool leftFrontWall = irSensors.getLeftCenter() < MES_WALL_FRONTSIDE_THRESHOLD;
    const bool frontWall = irSensors.getCenter() < MES_WALL_FRONT_THRESHOLD;
    const bool rightFrontWall = irSensors.getRightCenter() < MES_WALL_FRONTSIDE_THRESHOLD;
    const bool rightWall = irSensors.getRight() < MES_WALL_SIDE_THRESHOLD;

    Serial2.print(" | Erkannt (<TH): ");
    if (!leftWall && !leftFrontWall && !frontWall && !rightFrontWall && !rightWall) {
        Serial2.print("keine");
    } else {
        if (leftWall) Serial2.print("L ");
        if (leftFrontWall) Serial2.print("LF ");
        if (frontWall) Serial2.print("F ");
        if (rightFrontWall) Serial2.print("RF ");
        if (rightWall) Serial2.print("R ");
    }
    Serial2.println();
    Serial2.print(" | TH side/frontside/front: ");
    Serial2.print(MES_WALL_SIDE_THRESHOLD);
    Serial2.print("/");
    Serial2.print(MES_WALL_FRONTSIDE_THRESHOLD);
    Serial2.print("/");
    Serial2.println(MES_WALL_FRONT_THRESHOLD);

    Serial2.print(" | L:    ");Serial2.println(irSensors.getLeft());
    Serial2.print(" | LM:   ");Serial2.println(irSensors.getLeftCenter());
    Serial2.print(" | M:   ");Serial2.println(irSensors.getCenter());
    Serial2.print(" | RM:   ");Serial2.println(irSensors.getRightCenter());
    Serial2.print(" | R:    ");Serial2.println(irSensors.getRight());
    Serial2.print(" | TicksL: ");Serial2.println(motors.ticksL);
    Serial2.print(" | TicksR: ");Serial2.println(motors.ticksR);
    Serial2.print(" | TicksAVG: ");Serial2.println(motors.getAVGTicks());
    Serial2.print(" | Akku: ");Serial2.println(analogRead(PIN_AKKU));
    Serial2.println("--------------------------------------------------");
}

