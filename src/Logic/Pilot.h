#ifndef PILOT_H
#define PILOT_H

#include <Arduino.h>
#include <AdvancedPID.h>
#include "../Hardware/IRSensors.h"
#include "../Hardware/Motors.h"
#include "../Hardware/Impeller.h"
#include "../Hardware/Gyro.h"
#include "Map.h"
#include "Hugger.h"
#include "Ramp.h"

#define LOOP_PERIOD_MS 3 // 5

#define MES_WALL_FRONT_THRESHOLD 120
#define MES_WALL_FRONTSIDE_THRESHOLD 150
#define MES_WALL_SIDE_THRESHOLD 140

#define ALIGNMENT_FRONT_THRESHOLD 100 //95
#define ALIGNMENT_FRONT_TURN_THRESHOLD 60
#define ALIGNMENT_MIN_FRONT_THRESHOLD 160

// Geschwindigkeiten
#define PILOT_FORWARD_SPEED 80 //90
#define PILOT_FORWARD_MIN_SPEED 35 
#define PILOT_CURVE_SPEED 45 // 45 
#define PILOT_TURN_SPEED 35

// Gyro Geradeauskorrektur
#define GYRO_ALIGNMENT_THRESHOLD 4

// Tick für Zellen Orientierung
#define TICKS_CELL_CELL 80 
#define TICKS_CURVE_CELL 15 //15
#define TICKS_ALIGNMENT_CELL 10 //10

// Gyro-basierte Drehwinkel (in Grad)
#define GYRO_TURN_90  89.0 
#define GYRO_TURN_180 166.0
#define GYRO_REAL_90 90
#define GYRO_REAL_180 180

typedef enum {
    DECIDE,
    FORWARD,
    LEFT_CURVE,
    RIGHT_CURVE,
    TURN,
    GOAL
} TState;

class Pilot {
public:
    Pilot(IRSensors &aIrSensors, Motors &aMotors, Impeller &aImpeller, Gyro &aGyro);

    void Init();
    void loop();

private:
    IRSensors &irSensors;
    Motors &motors;
    Impeller &impeller;
    Gyro &gyro;
    Map map;
    Hugger hugger;
    Ramp ramp;

    TMotorValues MSV; // Motor Set Values

    TDirection curDirection;
    TDirection newDirection;
    TState curState;
    TState oldState;
    int32_t StateStartTicksL;
    int32_t StateStartTicksR;

    bool leftWall;
    bool leftFrontWall;
    bool frontWall;
    bool rightFrontWall;
    bool rightWall;

    int32_t ticksToGo;
    bool wasLeftWall;
    bool wasRightWall;
    bool aligned;
    

    void StateMachine(); // State Machine for the different states of the mouse
    void startState(TState aNextState);
    void SM_decide(); // Decision which direction to go, based on the map and the wall information
    void SM_forward(); // Going forward one cell
    void SM_leftCurve(); // Turning left while going forward, for better curves 60
    void SM_rightCurve(); // Turning right while going forward, for better curves
    void SM_turn(); // Turning in place, for 180° turns
    void SM_goal(); // Reached the goal, stop everything


    void mesWalls(); // Getting the wall information from the IR Sensors and saving it in the class variables
    int32_t avgStartTicks(); // Hilfsfunktion, um den Durchschnitt der Startticks für die State Machine zu bekommen
};

#endif