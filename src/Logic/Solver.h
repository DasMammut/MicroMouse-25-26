#ifndef SOLVER_H
#define SOLVER_H    

#include <Arduino.h>
#include "Map.h"

#define MAX_PATH_LENGTH MAP_SIZE * MAP_SIZE - TARGET_COUNT

class Solver {
public:
    Solver(Map &aMap);

    void Init();

    void softReset();

    void startSolving();

    TDirection getRelativePath(TDirection aCurrentDirection);

private:
    Map &map;
    TDirection path[MAX_PATH_LENGTH]; 
    uint16_t pathLenght;
    uint16_t currentIndex;

};

#endif