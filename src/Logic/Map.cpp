#include "Map.h"

// Tie-Breaking Priorität: Forward > Right > Back > Left (pro Blickrichtung)
static const TDirection directionOrder[4][4] = {
    {NORTH, EAST, SOUTH, WEST},  // Facing NORTH: fwd=N, right=E, back=S, left=W
    {EAST, SOUTH, WEST, NORTH},  // Facing EAST:  fwd=E, right=S, back=W, left=N
    {SOUTH, WEST, NORTH, EAST},  // Facing SOUTH: fwd=S, right=W, back=N, left=E
    {WEST, NORTH, EAST, SOUTH}   // Facing WEST:  fwd=W, right=N, back=E, left=S
};

Map::Map() {

}

void Map::Init() {
    MousePosition = {0, 0};
    MouseFacing = NORTH;

    targets[0] = {TARGET_0};
    // targets[1] = {TARGET_1};
    // targets[2] = {TARGET_2};
    // targets[3] = {TARGET_3};

    for(uint8_t x = 0; x < MAP_SIZE; x++) {
        for(uint8_t y = 0; y < MAP_SIZE; y++) {
            map[x][y].North = &H_walls[x + 1][y];
            map[x][y].East = &V_walls[x][y + 1];
            map[x][y].South = &H_walls[x][y];
            map[x][y].West = &V_walls[x][y];
        
            *map[x][y].North = NO_WALL;
            *map[x][y].East = NO_WALL;
            *map[x][y].South = NO_WALL;
            *map[x][y].West = NO_WALL;

            if(x == 0)                  *map[x][y].South = WALL;
            else if(x == MAP_SIZE-1)    *map[x][y].North = WALL;
            if(y == 0)                  *map[x][y].West = WALL;
            else if(y == MAP_SIZE-1)    *map[x][y].East = WALL;
        }
    }
    for(int i = 0; i < TARGET_COUNT; i++) {
        map[targets[i].x][targets[i].y].FFV = TARGET_VALUE;
    }

    for(uint8_t x = 0; x < MAP_SIZE; x++) {
        for(uint8_t y = 0; y < MAP_SIZE; y++) {
            visitedCells[x][y] = false;
        }
    }
}


void Map::softInit() {
    MousePosition = {0, 0};
    MouseFacing = NORTH;
}

void Map::setWalls(TDirection aDirection, bool aLeft, bool aLeftFront, bool aCenter, bool aRightFront, bool aRight) {
    MouseFacing = aDirection;
    TCell &currentCell = map[MousePosition.x][MousePosition.y];
    visitedCells[MousePosition.x][MousePosition.y] = true;

    if (aLeft) {
        switch (MouseFacing) {
            case NORTH:
                *currentCell.West = WALL;
                break;
            case EAST:
                *currentCell.North = WALL;
                break;
            case SOUTH:
                *currentCell.East = WALL;
                break;
            case WEST:
                *currentCell.South = WALL;
                break;
        }
    }

    // if (aLeftFront && !aCenter) {
    //     // Eine Zelle voraus schauen: dort links eine Wand setzen
    //     uint8_t nx = MousePosition.x;
    //     uint8_t ny = MousePosition.y;
    //     switch (MouseFacing) {
    //         case NORTH: nx++; break;
    //         case EAST:  ny++; break;
    //         case SOUTH: nx--; break;
    //         case WEST:  ny--; break;
    //     }
    //     if(nx < MAP_SIZE && ny < MAP_SIZE) {
    //         TCell &nextCell = map[nx][ny];
    //         switch (MouseFacing) {
    //             case NORTH: *nextCell.West = WALL; break;
    //             case EAST:  *nextCell.North = WALL; break;
    //             case SOUTH: *nextCell.East = WALL; break;
    //             case WEST:  *nextCell.South = WALL; break;
    //         }
    //     }
    // }

    if (aCenter) {
        switch (MouseFacing) {
            case NORTH:
                *currentCell.North = WALL;
                break;
            case EAST:
                *currentCell.East = WALL;
                break;
            case SOUTH:
                *currentCell.South = WALL;
                break;
            case WEST:
                *currentCell.West = WALL;
                break;
        }
    }

    // if (aRightFront && !aCenter) {
    //     // Eine Zelle voraus schauen: dort rechts eine Wand setzen
    //     uint8_t nx = MousePosition.x;
    //     uint8_t ny = MousePosition.y;
    //     switch (MouseFacing) {
    //         case NORTH: nx++; break;
    //         case EAST:  ny++; break;
    //         case SOUTH: nx--; break;
    //         case WEST:  ny--; break;
    //     }
    //     if(nx < MAP_SIZE && ny < MAP_SIZE) {
    //         TCell &nextCell = map[nx][ny];
    //         switch (MouseFacing) {
    //             case NORTH: *nextCell.East = WALL; break;
    //             case EAST:  *nextCell.South = WALL; break;
    //             case SOUTH: *nextCell.West = WALL; break;
    //             case WEST:  *nextCell.North = WALL; break;
    //         }
    //     }
    // }

    if (aRight) {
        switch (MouseFacing) {
            case NORTH:
                *currentCell.East = WALL;
                break;
            case EAST:
                *currentCell.South = WALL;
                break;
            case SOUTH:
                *currentCell.West = WALL;
                break;
            case WEST:
                *currentCell.North = WALL;
                break;
        }

    }
}

void Map::moveCell(TDirection aDirection) {
    MouseFacing = aDirection;
    switch (MouseFacing) {
        case NORTH:
            if(MousePosition.x < MAP_SIZE - 1) MousePosition.x++;
            break;
        case EAST:
            if(MousePosition.y < MAP_SIZE - 1) MousePosition.y++;
            break;
        case SOUTH:
            if(MousePosition.x > 0) MousePosition.x--;
            break;
        case WEST:
            if(MousePosition.y > 0) MousePosition.y--;
            break;
    }
}

void Map::updateMaze() {
    FloodFill();
}

bool Map::isAtTarget() {
    for(int i = 0; i < TARGET_COUNT; i++) {
        if(MousePosition.x == targets[i].x && MousePosition.y == targets[i].y) {
            return true;
        }
    }
    return false;
}

TDirection Map::getRelativeDirection() {
    TDirection bestDirection = NORTH; // Default-Wert, wird überschrieben
    uint8_t lowestFFV = map[MousePosition.x][MousePosition.y].FFV;

    for(uint8_t i = 0; i < 4; i++) {
        switch (directionOrder[MouseFacing][i]) {
            case NORTH:
                if(MousePosition.x < MAP_SIZE - 1 && *map[MousePosition.x][MousePosition.y].North == NO_WALL) {
                    if(map[MousePosition.x + 1][MousePosition.y].FFV < lowestFFV) {
                        bestDirection = NORTH;
                        lowestFFV = map[MousePosition.x + 1][MousePosition.y].FFV;
                    }
                }
                break;
            case EAST:
                if(MousePosition.y < MAP_SIZE - 1 && *map[MousePosition.x][MousePosition.y].East == NO_WALL) {
                    if(map[MousePosition.x][MousePosition.y + 1].FFV < lowestFFV) {
                        bestDirection = EAST;
                        lowestFFV = map[MousePosition.x][MousePosition.y + 1].FFV;
                    }
                }
                break;
            case SOUTH:
                if(MousePosition.x > 0 && *map[MousePosition.x][MousePosition.y].South == NO_WALL) {
                    if(map[MousePosition.x - 1][MousePosition.y].FFV < lowestFFV) {
                        bestDirection = SOUTH;
                        lowestFFV = map[MousePosition.x - 1][MousePosition.y].FFV;
                    }
                }
                break;
            case WEST:
                if(MousePosition.y > 0 && *map[MousePosition.x][MousePosition.y].West == NO_WALL) {
                    if(map[MousePosition.x][MousePosition.y - 1].FFV < lowestFFV) {
                        bestDirection = WEST;
                        lowestFFV = map[MousePosition.x][MousePosition.y - 1].FFV;
                    }
                }
                break;
        }
    }

    // Absolute Richtung → relative Richtung umrechnen
    // NORTH=vorwärts, EAST=rechts, SOUTH=rückwärts, WEST=links (relativ zu MouseFacing)
    TDirection relativeDirection = static_cast<TDirection>((bestDirection - MouseFacing + 4) % 4);
    return relativeDirection;
}

// ------------------------------------------------------------------------------

void Map::FloodFill(){
    for(uint8_t x = 0; x < MAP_SIZE; x++) {
        for(uint8_t y = 0; y < MAP_SIZE; y++) {
            map[x][y].FFV = 255;
            for(uint8_t i = 0; i < TARGET_COUNT; i++) {
                if(x == targets[i].x && y == targets[i].y) {
                    map[x][y].FFV = TARGET_VALUE;
                    break;
                }
            }
        }
    }

    bool changed;
    do {
        changed = false; 
        for(uint8_t x = 0; x < MAP_SIZE; x++) {
            for(uint8_t y = 0; y < MAP_SIZE; y++) {
                
                // Ziel-Werte nicht verändern
                if(map[x][y].FFV == TARGET_VALUE) continue;

                uint8_t minFFV = 255;

                // NORTH (x+1) - Prüfe Wand UND Array-Grenze
                if(*map[x][y].North == NO_WALL && x < (MAP_SIZE - 1)) {
                    if(map[x+1][y].FFV < minFFV) minFFV = map[x+1][y].FFV;
                }
                // EAST (y+1)
                if(*map[x][y].East == NO_WALL && y < (MAP_SIZE - 1)) {
                    if(map[x][y+1].FFV < minFFV) minFFV = map[x][y+1].FFV;
                }
                // SOUTH (x-1)
                if(*map[x][y].South == NO_WALL && x > 0) {
                    if(map[x-1][y].FFV < minFFV) minFFV = map[x-1][y].FFV;
                }
                // WEST (y-1)
                if(*map[x][y].West == NO_WALL && y > 0) {
                    if(map[x][y-1].FFV < minFFV) minFFV = map[x][y-1].FFV;
                }

                // Update-Logik
                if(minFFV != 255) {
                    if(minFFV + 1 < map[x][y].FFV) {
                        map[x][y].FFV = minFFV + 1;
                        changed = true;
                    }
                }
            }
        }
    } while (changed);
}


TDirection rotateLeft(TDirection aDirection) {
    return static_cast<TDirection>((aDirection + 3) % 4);
}

TDirection turnAround(TDirection aDirection) {
    return static_cast<TDirection>((aDirection + 2) % 4);
}

TDirection rotateRight(TDirection aDirection) {
    return static_cast<TDirection>((aDirection + 1) % 4);
}