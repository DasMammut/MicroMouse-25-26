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

#define LOOP_PERIOD_MS 5

#define MES_WALL_FRONT_THRESHOLD 120
#define MES_WALL_FRONTSIDE_THRESHOLD 150
#define MES_WALL_SIDE_THRESHOLD 120

#define ALIGNMENT_FRONT_THRESHOLD 60
#define ALIGNMENT_MIN_FRONT_THRESHOLD 120

#define START_DIRECTION NORTH

// Geschwindigkeiten
#define PILOT_FORWARD_SPEED 75
#define PILOT_TURN_SPEED 45

// Gyro Geradeauskorrektur
#define GYRO_STRAIGHT_Kp 10.0

// Tick für Zellen Orientierung
#define TICKS_CELL_CELL 80 
#define TICKS_CURVE_CELL  50
#define TICKS_ALIGNMENT_CELL 35

// Gyro-basierte Drehwinkel (in Grad)
#define GYRO_TURN_90  70.0 // 80
#define GYRO_TURN_180 155.0

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

    bool wasLeftWall;
    bool wasRightWall;
    bool aligned;
    

    void StateMachine(); // State Machine for the different states of the mouse
    void startState(TState aNextState);
    void SM_decide(); // Decision which direction to go, based on the map and the wall information
    void SM_forward(uint16_t ticks = TICKS_CELL_CELL); // Going forward one cell
    void SM_leftCurve(); // Turning left while going forward, for better curves 60
    void SM_rightCurve(); // Turning right while going forward, for better curves
    void SM_turn(); // Turning in place, for 180° turns
    void SM_goal(); // Reached the goal, stop everything


    void mesWalls(); // Getting the wall information from the IR Sensors and saving it in the class variables
    TDirection rotateLeft(TDirection aDirection);
    TDirection turnAround(TDirection aDirection);
    TDirection rotateRight(TDirection aDirection);
    int32_t avgStartTicks(); // Hilfsfunktion, um den Durchschnitt der Startticks für die State Machine zu bekommen
    void setAvgStartTicks(int32_t ticks); // Hilfsfunktion, um die Startticks für die State Machine zu setzen
    

    #define GYRO_Kp 1.5
    #define GYRO_Ki 0.0
    #define GYRO_Kd 0.0
    AdvancedPID pidGyroCurve; // PID für die Gyro-basierte Kurvenfahrt
};

#endif