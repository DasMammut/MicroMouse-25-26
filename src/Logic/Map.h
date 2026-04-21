#ifndef MAP_H
#define MAP_H

#include <Arduino.h>

#define MAP_SIZE 8

#define TARGET_COUNT 4
#define TARGET_0 MAP_SIZE/2 - 1, MAP_SIZE/2 - 1
#define TARGET_1 MAP_SIZE/2 - 1, MAP_SIZE/2
#define TARGET_2 MAP_SIZE/2, MAP_SIZE/2 - 1
#define TARGET_3 MAP_SIZE/2, MAP_SIZE/2

#define TARGET_VALUE 0


typedef enum {
    NO_WALL = 0,
    WALL = 1,
} TWall;

typedef struct {
    uint8_t x;
    uint8_t y; // x = 0 y = 0 ist gleich SüdWesten also unten links
} TPosition;

typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
} TDirection;

typedef struct {
    TWall *North; // x+
    TWall *East; // y+
    TWall *South; // x-
    TWall *West; // y-
    uint8_t FloodFillValue;
} TCell;
#define FFV FloodFillValue

class Map{
public:
    Map();

    void Init();

    void setWalls(TDirection aDirection, bool aLeft, bool aLeftFront, bool aCenter, bool aRightFront, bool aRight); // Setting the walls in the map based on the direction the mouse is facing and the sensor values
    void moveCell(TDirection aDirection); // Updating the position of the mouse based on the direction it is moving
    void updateMaze();

    bool isAtTarget();

    TDirection getRelativeDirection();

    uint8_t getMouseX() { return MousePosition.x; }
    uint8_t getMouseY() { return MousePosition.y; }
    void printMap(Stream &serial); // Map als ASCII-Art auf Serial ausgeben

private:
    TPosition MousePosition;
    TDirection MouseFacing;
    TPosition targets[TARGET_COUNT];
    TCell map[MAP_SIZE][MAP_SIZE]; // 0-8, 0-8 Wände
    TWall H_walls[MAP_SIZE + 1][MAP_SIZE];
    TWall V_walls[MAP_SIZE][MAP_SIZE + 1];


    void FloodFill();
};


#endif