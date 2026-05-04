#include "Pilot.h"
#include "Map.h"

Pilot::Pilot(IRSensors &aIrSensors, Motors &aMotors, Impeller &aImpeller, Gyro &aGyro)
    : irSensors(aIrSensors), motors(aMotors), impeller(aImpeller), gyro(aGyro), ramp(aMotors, aImpeller),
      curDirection(START_DIRECTION), newDirection(START_DIRECTION), curState(DECIDE), oldState(DECIDE), StateStartTicksL(0), StateStartTicksR(0),
      leftWall(false), rightWall(false), frontWall(false), wasLeftWall(false), wasRightWall(false), aligned(false), ticksToGo(0),
      hugger(aIrSensors, aMotors, aGyro),
      pidGyroCurve(GYRO_STRAIGHT_Kp, GYRO_STRAIGHT_Ki, GYRO_STRAIGHT_Kd) {
    MSV.left = 0;
    MSV.right = 0;
    MSV.impeller = 0;
}

void Pilot::Init() {
    StateStartTicksL = motors.ticksL;
    StateStartTicksR = motors.ticksR;

    irSensors.Init();
    motors.Init();
    impeller.Init();
    gyro.Init();

    map.Init();
    hugger.Init();

    pidGyroCurve.setOutputLimits(-PILOT_TURN_SPEED, PILOT_TURN_SPEED);
    pidGyroCurve.setMode(MODE_AUTO);
}

void Pilot::loop() {
    static uint32_t lastMicros = micros();
    static uint32_t tempMicros = micros();

    irSensors.update();
    gyro.update();

    StateMachine();

    ramp.step(MSV);
    motors.update();
    impeller.update();

    while ((tempMicros = micros()) - lastMicros < LOOP_PERIOD_MS * 1000);
    lastMicros = tempMicros;
}

// State Machine: Je nach aktuellem State werden unterschiedliche Funktionen aufgerufen, die die MSV-Werte setzen. Alle Funktionen müssen am Ende den nächsten State setzen, damit die State Machine weiterläuft.
void Pilot::StateMachine() {
    switch(curState) {
        case DECIDE:
            SM_decide();
            break;

        case FORWARD:
            SM_forward();
            break;

        case LEFT_CURVE:
            SM_leftCurve();
            break;

        case RIGHT_CURVE:
            SM_rightCurve();
            break;

        case TURN:
            SM_turn();
            break;

        case GOAL:
            SM_goal();
            break;
    }
}

// Neuen State starten: Alle Werte zurücksetzen, Startzeit merken
void Pilot::startState(TState aNextState) {
    if(curState != DECIDE) oldState = curState;
    curState = aNextState;

    StateStartTicksL = motors.ticksL;
    StateStartTicksR = motors.ticksR;

    wasLeftWall = leftWall;
    wasRightWall = rightWall;
    aligned = false;

    switch (curState) {
        case DECIDE:
            break;
        case FORWARD:
            if(oldState == FORWARD || oldState == DECIDE) {
                ticksToGo = TICKS_CELL_CELL;
                break;
            }
            else if(oldState == LEFT_CURVE || oldState == RIGHT_CURVE || oldState == TURN) {
                ticksToGo = TICKS_CURVE_CELL;
            }
            motors.resetPIDs();
            break;
        case LEFT_CURVE:    
            gyro.shouldAbsAngle += GYRO_REAL_90;
            motors.resetPIDs();
            break;
        case RIGHT_CURVE:
            gyro.shouldAbsAngle -= GYRO_REAL_90;
            motors.resetPIDs();
            break;
        case TURN:
            gyro.shouldAbsAngle -= GYRO_REAL_180;
            motors.resetPIDs();
            break;
        case GOAL:
        default:
            break;
    }

    gyro.reset(); 
    hugger.reset();
    pidGyroCurve.reset();
}

// SM Einzelne States: Alle Funktionen müssen am Ende den nächsten State setzen, damit die State Machine weiterläuft. Alle Funktionen setzen die MSV-Werte entsprechend, damit im loop() die Motoren und der Impeller die richtigen Werte bekommen.
void Pilot::SM_decide() {
    mesWalls();

    // Map aktualisieren mit den neuen Wandinformationen
    map.setWalls(curDirection, leftWall, leftFrontWall, frontWall, rightFrontWall, rightWall);

    // Zielzelle erreicht, alles stoppen
    if(map.isAtTarget()) {
        startState(GOAL);
        return;
    }

    // Entscheidung treffen
    map.updateMaze();

    TDirection relTurnDirection = map.getRelativeDirection();

    switch(relTurnDirection) {
        case NORTH: // Geradeaus
            newDirection = curDirection;
            startState(FORWARD);
            break;

        case EAST: // Rechts
            newDirection = rotateRight(curDirection);
            startState(RIGHT_CURVE);
            break;

        case SOUTH: // U-Turn
            newDirection = turnAround(curDirection);
            startState(TURN);
            break;

        case WEST: // Links
            newDirection = rotateLeft(curDirection);
            startState(LEFT_CURVE);
            break;
    }
}

void Pilot::SM_forward() {
    // Wandzentrierung während der Fahrt
    mesWalls();

    int16_t dynamicSpeed = ::map(irSensors.getCenter(), ALIGNMENT_FRONT_THRESHOLD, ALIGNMENT_MIN_FRONT_THRESHOLD, PILOT_FORWARD_MIN_SPEED, PILOT_FORWARD_SPEED);
    dynamicSpeed = constrain(dynamicSpeed, PILOT_FORWARD_MIN_SPEED, PILOT_FORWARD_SPEED);

    MSV.left = dynamicSpeed;
    MSV.right = dynamicSpeed;
    hugger.hug(MSV, leftWall, leftFrontWall, frontWall, rightFrontWall, rightWall);

    if(!leftWall && wasLeftWall && oldState == FORWARD && irSensors.getCenter() > ALIGNMENT_MIN_FRONT_THRESHOLD && !aligned) {
        ticksToGo = motors.getAVGTicks() - avgStartTicks() + TICKS_ALIGNMENT_CELL;
        aligned = true;
    }
    else if(!rightWall && wasRightWall && oldState == FORWARD && irSensors.getCenter() > ALIGNMENT_MIN_FRONT_THRESHOLD && !aligned) {
        ticksToGo = motors.getAVGTicks() - avgStartTicks() + TICKS_ALIGNMENT_CELL;
        aligned = true;
    }

    wasLeftWall = leftWall;
    wasRightWall = rightWall;

    if(oldState == FORWARD && fabs(MSV.left - MSV.right) < GYRO_ALIGNMENT_THRESHOLD) {
        gyro.absAngle = gyro.shouldAbsAngle;
    }

    if(motors.getAVGTicks() - avgStartTicks() >= ticksToGo && irSensors.getCenter() > ALIGNMENT_MIN_FRONT_THRESHOLD) {
        map.moveCell(curDirection);
        startState(DECIDE);
        return;
    }

    if(irSensors.getCenter() < ALIGNMENT_FRONT_THRESHOLD && irSensors.getCenter() <= ALIGNMENT_MIN_FRONT_THRESHOLD) {
        map.moveCell(curDirection);
        startState(DECIDE);
        return;
    }
}

void Pilot::SM_leftCurve() {
    float angle = fabs(gyro.angle);
    float error = angle - GYRO_TURN_90;
    float correction = pidGyroCurve.run(error, 0.0);
    correction = constrain(correction, PILOT_TURN_MIN_SPEED, PILOT_TURN_SPEED);
    MSV.left  = 0;
    MSV.right = correction;

    if(angle >= GYRO_TURN_90) {
        curDirection = newDirection;
        startState(FORWARD);
    }
}

void Pilot::SM_rightCurve() {
    float angle = fabs(gyro.angle);
    float error = angle - GYRO_TURN_90;
    float correction = pidGyroCurve.run(error, 0.0);
    correction = constrain(correction, PILOT_TURN_MIN_SPEED, PILOT_TURN_SPEED);
    MSV.left  = correction;
    MSV.right = 0;

    if(angle >= GYRO_TURN_90) {
        curDirection = newDirection;
        startState(FORWARD);
    }
}

void Pilot::SM_turn() {
    float angle = fabs(gyro.angle);

    MSV.left  =  PILOT_TURN_SPEED;
    MSV.right = -PILOT_TURN_SPEED;

    if(angle >= GYRO_TURN_180) {
        curDirection = newDirection;
        startState(FORWARD);
    }
}

void Pilot::SM_goal() {
    mesWalls();
    MSV.left = 0;
    MSV.right = 0;
    MSV.impeller = 0;

    if(motors.getAVGTicks() - avgStartTicks() <= TICKS_CELL_CELL) {
        MSV.left = PILOT_FORWARD_SPEED / 2;
        MSV.right = PILOT_FORWARD_SPEED / 2;
        hugger.hug(MSV, leftWall, leftFrontWall, frontWall, rightFrontWall, rightWall);
    }
}





// Wanderkennung
void Pilot::mesWalls() {
    leftWall = irSensors.getLeft() < MES_WALL_SIDE_THRESHOLD;
    leftFrontWall = irSensors.getLeftCenter() < MES_WALL_FRONTSIDE_THRESHOLD;
    frontWall = irSensors.getCenter() < MES_WALL_FRONT_THRESHOLD;
    rightFrontWall = irSensors.getRightCenter() < MES_WALL_FRONTSIDE_THRESHOLD;
    rightWall = irSensors.getRight() < MES_WALL_SIDE_THRESHOLD;
    
}
// Hilfsfunktionen zum Drehen der Richtung um 90° nach links oder rechts oder umdrehen um 180°
TDirection Pilot::rotateLeft(TDirection aDirection) {
    return static_cast<TDirection>((aDirection + 3) % 4);
}

TDirection Pilot::turnAround(TDirection aDirection) {
    return static_cast<TDirection>((aDirection + 2) % 4);
}

TDirection Pilot::rotateRight(TDirection aDirection) {
    return static_cast<TDirection>((aDirection + 1) % 4);
}

int32_t Pilot::avgStartTicks() {
    return (StateStartTicksL + StateStartTicksR) / 2;
}

void Pilot::setAvgStartTicks(int32_t ticks) {
    StateStartTicksL = ticks;
    StateStartTicksR = ticks;
}
