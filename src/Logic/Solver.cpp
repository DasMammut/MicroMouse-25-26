#include "Solver.h"
#include "Map.h"

Solver::Solver(Map &aMap) : map(aMap), pathLenght(0) {
    for(int i = 0; i < MAX_PATH_LENGTH; i++) {
        path[i] = NORTH;
    }
}

void Solver::Init() {
    pathLenght = 0;
    currentIndex = 0;
    for(int i = 0; i < MAX_PATH_LENGTH; i++) {
        path[i] = NORTH;
    }
}

void Solver::softReset() {
    currentIndex = 0;
}

void Solver::startSolving() {
    map.softInit();
    TDirection dir = START_DIRECTION;
    for(int i = 0; i < MAX_PATH_LENGTH; i++) {
        dir = map.getRelativeDirection();
        path[i] = dir;
        pathLenght++;
        map.moveCell(dir);
        if(map.isAtTarget()) {
            break;
        }
    } 

}

TDirection Solver::getRelativePath(TDirection aCurrentDirection) {
    if(currentIndex < pathLenght) {
        TDirection nextDirection = path[currentIndex];
        currentIndex++;
        return nextDirection;
    }
    return NORTH;
}

