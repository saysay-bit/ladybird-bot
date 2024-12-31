#include "mbed.h"
#include "robot.h"
using namespace rtos;
using namespace mbed;

#define PWM_PIN_A P0_27
#define PWM_PIN_B P1_2
#define DIR_PIN_A P0_4
#define DIR_PIN_B P0_5

#define ENC_PIN_A P1_11
#define ENC_PIN_B P1_12

#define SENSORLEFT_PIN 6   //P1_14 - SENSOR 2
#define SENSORFRONT_PIN 7  //P0_23 - SENSOR 1
#define SENSORRIGHT_PIN 4  //P1_15 - SENSOR 4

mbed::PwmOut MotorAPWM(P0_27);
mbed::PwmOut MotorBPWM(P1_2);

mbed::DigitalOut MotorADir(P0_4);
mbed::DigitalOut MotorBDir(P0_5);

Motor MotorA(PWM_PIN_A, DIR_PIN_A);
Motor MotorB(PWM_PIN_B, DIR_PIN_B);

UltrasonicSensor sensor_left(SENSORLEFT_PIN);
UltrasonicSensor sensor_front(SENSORFRONT_PIN);
UltrasonicSensor sensor_right(SENSORRIGHT_PIN);

float speedDefault = 8;
float distanceDefault = 10;
float turnDefault = 90;

int MovingCoords = 0;

int XMovement = 0;
int YMovement = 0;

int lastCoordsX = 0;
int lastCoordsY = 0;

int x_movement = 0;
int y_movement = 0;

char TargetFacing = 'U';
char Facing = 'U';

// Define grid dimensions
#define GRID_ROWS 12  //one grid box would be 17x17 cm
#define GRID_COLS 9
#define INT_MAX 100

// Define the occupancy grid
int MazeMap[GRID_ROWS][GRID_COLS] = {};

struct Cell {
  int CellX, CellY;
};

int CoordsX = 0;
int CoordsY = 0;

bool MovingDirection = true;

bool ObstacleLeft = false;
bool ObstacleRight = false;
bool ObstacleFront = false;

struct Cell start = { CoordsX, CoordsY };
struct Cell goal = { 4, 4 };

struct Cell path[GRID_ROWS * GRID_COLS];
int pathLength = 0;


//=======================================================================================================================================================================================
//State variables for localization: add each update and never set to 0
float angle = 90;  //0 is along the x-axis (unit circle)
float positionX = 0;
float positionY = 0;

Thread angleChecking;

//Sensor Readings structure and function===================================================================================================================================================
struct SensorReadings {  //group related data items under a single name
  float distance_left;
  float distance_front;
  float distance_right;
};


SensorReadings readAllSensors() {  //returns an instance of SensorReadings structure
  SensorReadings readings;
  readings.distance_left = sensor_left.DistanceMeasurement();
  readings.distance_front = sensor_front.DistanceMeasurement();
  readings.distance_right = sensor_right.DistanceMeasurement();

  return readings;
}



//Robot current state: updated by state functions ===============================================================================================================================
enum robotState { stopState = 0,
                  forwardState,
                  leftState,
                  rightState,
                  backState,
                  defaultState
} behaviour;


//===============================================================================================================================================================================
//Behavior variables: updated in each loop but only reset once behavior finishes (e.g. move x cm)
float distanceMoved = 0.0;
float angleMoved = 0.0;

float distanceRTotal = 0.0;
float distanceLTotal = 0.0;

//angle fixing to 90 ================================================================================================================================================


bool ActionComplete = false;

//moving forward/backward/turning ================================================================================================================================================
void move(float speedIn, float targetDistance, bool moveBack) {
  angleChecking.start(callback(anglesAndCoordsUpdate));

  updateMazeMapObstacles();


  //positionFixing.start(callback(fix_position));

  ActionComplete = false;

  float speedOut = (speedIn + 3) / 40;

  MotorAPWM.write(speedOut);
  MotorBPWM.write(speedOut);
  MotorADir = moveBack ? 0 : 1;
  MotorBDir = moveBack ? 1 : 0;


  float tolerance = 1;  //note distances to actual distances are 1cm off, but tolerance is low to avoid error

  //ISSUE WITH MOVEMENT ADDING 2 TO COORDS SEEMS TO BE HERE, FIXED BY CHECKING COORDINATE NOT DISTANCE

  static float initialDistance = ((std::fabs(distanceRTotal) + std::fabs(distanceLTotal)) * 0.5);
  float currentDistance = ((std::fabs(distanceRTotal) + std::fabs(distanceLTotal)) * 0.5);

  float adjusted_targetDistance = (moveBack ? -targetDistance : targetDistance);
  if (MovingCoords == 1) {
    if (Facing == 'L' || Facing == 'R') {
      if (CoordsX == XMovement) {
        stop();
        ActionComplete = true;
        Serial.println("Distance Met!");
        initialDistance = (std::fabs(distanceRTotal) + std::fabs(distanceLTotal)) / 2;
      }
    } else {
      if (Facing == 'F' || Facing == 'B') {
        if (CoordsY == YMovement) {
          stop();
          ActionComplete = true;
          Serial.println("Distance Met!");
          initialDistance = (std::fabs(distanceRTotal) + std::fabs(distanceLTotal)) / 2;
        }
      } else {
        Serial.println("Unknown Movement Direction!");
      }
    }
  } else {
    if ((std::fabs(initialDistance - currentDistance + adjusted_targetDistance)) <= tolerance) {
      stop();
      ActionComplete = true;
      Serial.println("Distance Met!");
      initialDistance = (std::fabs(distanceRTotal) + std::fabs(distanceLTotal)) / 2;
    }
  }
}


void move_forward(float speedIn, float targetDistance) {
  while (ActionComplete == false) {
    move(speedIn, targetDistance, false);
  }
}

void move_backward(float speedIn, float targetDistance) {
  while (ActionComplete == false) {

    move(speedIn, targetDistance, true);
  }
}

void testing_function() {
  static int testState = 0;

  switch (testState) {
    case 0:
      moveToCoords(2, 2);
      break;
  }
}

void stop() {
  MotorAPWM.write(0);
  MotorBPWM.write(0);
}

static float initialAngle = angle;
static int fullRotations = 0;
float prevDeltaX = 0;
float prevDeltaY = 0;
float cumulativeX = 0;
float cumulativeY = 0;

void turn(float speedIn, float targetAngle, bool turnRight) {
  //Serial.println("Turning");
  angleChecking.start(callback(anglesAndCoordsUpdate));
  ActionComplete = false;

  static float initialAngle = angle;

  float speedOut = (speedIn + 3) / 40;
  MotorAPWM.write(speedOut);
  MotorBPWM.write(speedOut);
  MotorADir = turnRight ? 0 : 1;
  MotorBDir = turnRight ? 0 : 1;

  float tolerance = 1.0;

  if (turnRight) {  //math for turning right
    if (initialAngle < -270) {
      if (std::fabs(angle - 360.0f - initialAngle + targetAngle) <= tolerance) {
        //behaviour = forwardState; //change to stop for testing
        stop();
        angleMoved = 0;
        initialAngle = angle;
        ActionComplete = true;
        wait_us(1000);
      }
    } else {
      if (std::fabs(angle - 0 - initialAngle + targetAngle) <= tolerance) {  //fullRotations to 0
        //behaviour = forwardState; //change to stop for testing
        stop();
        angleMoved = 0;
        initialAngle = angle;
        ActionComplete = true;
        wait_us(1000);
      }
    }
  } else {  //math for turning left
    if (initialAngle > 270) {
      if (std::fabs(angle + 360.0f - initialAngle - targetAngle) <= tolerance) {
        //behaviour = forwardState; //change to stop for testing
        stop();
        angleMoved = 0;
        initialAngle = angle;
        ActionComplete = true;
        wait_us(1000);
      }
    } else {
      if (std::fabs(angle + 0 - initialAngle - targetAngle) <= tolerance) {  //fullRotations to 0
        //behaviour = forwardState; //change to stop for testing
        stop();
        angleMoved = 0;
        initialAngle = angle;
        ActionComplete = true;
        wait_us(1000);
      }
    }
  }


  wait_us(10000);  // Adjust the delay as needed
}

void turn_right(float speedIn, float targetAngle) {
  Serial.println("Turning Right");
  while (ActionComplete == false) {
    turn(speedIn, targetAngle, true);
  }
}


void turn_left(float speedIn, float targetAngle) {
  Serial.println("Turning Left");
  while (ActionComplete == false) {
    turn(speedIn, targetAngle, false);
  }
}


//defining interrupts and timers to count encoders===========================================================================================================================
mbed::InterruptIn EncoderR(P1_11);
mbed::InterruptIn EncoderL(P1_12);

long int WheelRevR;
long int WheelRevL;

long int PulseCountR;
long int PulseCountL;

mbed::Timer timeCheck;



//encoder count read functions =======================================================================================================================================================
void countEncoderR() {
  PulseCountR++;  //encoder count increases on every rising edge (0 to 1)

  if (PulseCountR % 6 == 0) {      //checks if one rotation has completed (6 counts per rotation)
    if (PulseCountR % 100 == 0) {  //increases Shaft Revolutions every 100 counts
      WheelRevR++;
    }
  }
}

void countEncoderL() {
  PulseCountL++;  //encoder count increases on every rising edge (0 to 1)

  if (PulseCountL % 6 == 0) {      //checks if one rotation has completed (6 counts per rotation)
    if (PulseCountL % 100 == 0) {  //increases Shaft Revolutions every 100 counts
      WheelRevL++;
    }
  }
}


//Map stuff =========================================================================================================================================================================
void printMazeMap() {
  for (int i = 0; i < GRID_ROWS; i++) {
    for (int j = 0; j < GRID_COLS; j++) {
      Serial.print(MazeMap[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void initialiseMazeMap() {
  for (int i = 0; i < GRID_ROWS; ++i) {
    for (int j = 0; j < GRID_COLS; ++j) {
      MazeMap[i][j] = 1;
    }
  }
}

int ObstacleX;
int ObstacleY;


void ObstacleMapping() {
  CoordsX = CoordsX;
  CoordsY = CoordsY;
  findFacing();

  Serial.println(Facing);
  SensorReadings currentReadings = readAllSensors();

  if (ObstacleLeft == true) {
    if (Facing == 'F') {
      Serial.println("Facing F");
      ObstacleX = CoordsX - 1;
      ObstacleY = CoordsY;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'B') {
      Serial.println("Facing B");
      ObstacleX = CoordsX + 1;
      ObstacleY = CoordsY;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'R') {
      Serial.println("Facing R");
      ObstacleX = CoordsX;
      ObstacleY = CoordsY + 1;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'L') {
      Serial.println("Facing L");
      ObstacleX = CoordsX;
      ObstacleY = CoordsY - 1;
      MazeMap[ObstacleX][ObstacleY] = 2;
    }
    Serial.print("CoordsX: ");
    Serial.println(CoordsX);
    Serial.print("CoordsY: ");
    Serial.println(CoordsY);
    Serial.println("Obstacle to the left`!");
    Serial.print("Obstacle X: ");
    Serial.println(ObstacleX);
    Serial.print("Obstacle Y: ");
    Serial.println(ObstacleY);

    ObstacleLeft = false;
  }
  if (ObstacleRight == true) {
    if (Facing == 'F') {
      Serial.println("Facing F");
      ObstacleX = CoordsX + 1;
      ObstacleY = CoordsY;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'B') {
      Serial.println("Facing B");
      ObstacleX = CoordsX - 1;
      ObstacleY = CoordsY;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'R') {
      Serial.println("Facing R");
      ObstacleX = CoordsX;
      ObstacleY = CoordsY - 1;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'L') {
      Serial.println("Facing L");
      ObstacleX = CoordsX;
      ObstacleY = CoordsY + 1;
      MazeMap[ObstacleX][ObstacleY] = 2;
    }

    Serial.print("CoordsX: ");
    Serial.println(CoordsX);
    Serial.print("CoordsY: ");
    Serial.println(CoordsY);
    Serial.println("Obstacle to the right!");
    Serial.print("Obstacle X: ");
    Serial.println(ObstacleX);
    Serial.print("Obstacle Y: ");
    Serial.println(ObstacleY);
    ObstacleRight = false;
  }
  if (ObstacleFront == true) {
    if (Facing == 'F') {
      Serial.println("Facing F");
      ObstacleX = CoordsX;
      ObstacleY = CoordsY + 1;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'B') {
      Serial.println("Facing B");
      ObstacleX = CoordsX;
      ObstacleY = CoordsY - 1;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'R') {

      Serial.println("Facing R");
      ObstacleX = CoordsX + 1;
      ObstacleY = CoordsY;
      MazeMap[ObstacleX][ObstacleY] = 2;

    } else if (Facing == 'L') {
      Serial.println("Facing L");
      ObstacleX = CoordsX - 1;
      ObstacleY = CoordsY;
      MazeMap[ObstacleX][ObstacleY] = 2;
    }
    ObstacleFront = false;
    Serial.print("CoordsX: ");
    Serial.println(CoordsX);
    Serial.print("CoordsY: ");
    Serial.println(CoordsY);
    Serial.println("Obstacle ahead!");
    Serial.print("Obstacle X: ");
    Serial.println(ObstacleX);
    Serial.print("Obstacle Y: ");
    Serial.println(ObstacleY);
  }
}

void updateMazeMapObstacles() {
  angleChecking.start(callback(anglesAndCoordsUpdate));
  SensorReadings currentReadings = readAllSensors();
  if (currentReadings.distance_left < 8 && currentReadings.distance_left > 0) {

    ObstacleLeft = true;
    Serial.println("OBSTACLE LEFT");
  }

  if (currentReadings.distance_right < 8 && currentReadings.distance_right > 0) {

    ObstacleRight = true;
    Serial.println("OBSTACLE RIGHT");
  }

  if (currentReadings.distance_front < 8 && currentReadings.distance_front > 0) {

    ObstacleFront = true;
    Serial.println("OBSTACLE FRONT");
  }

  //MazeMap[ObstacleX][ObstacleY] = 2;  //assigns the grid with these coordinates with a 1
  dijkstra(MazeMap, start, goal);
  Serial.println("Occupancy Grid:");
  printMazeMap();
}

//Dijkstra stuff ===================================================================================================================================================================================

int isValid(int row, int col) {
  return (row >= 0) && (row < GRID_ROWS) && (col >= 0) && (col < GRID_COLS);
}

int min(int a, int b) {
  return (a < b) ? a : b;
}

int dijkstra(int maze[GRID_ROWS][GRID_COLS], struct Cell begin, struct Cell goal) {
  begin = { CoordsX, CoordsY };

  angleChecking.start(callback(anglesAndCoordsUpdate));
  pathLength = 0;  // Reset pathLength

  // defining stuff
  int distances[GRID_ROWS][GRID_COLS];
  int visited[GRID_ROWS][GRID_COLS];
  struct Cell predecessors[GRID_ROWS][GRID_COLS];

  for (int i = 0; i < GRID_ROWS; i++) {
    for (int j = 0; j < GRID_COLS; j++) {
      distances[i][j] = INT_MAX;
      visited[i][j] = 0;
      predecessors[i][j] = { -1, -1 };
    }
  }

  distances[start.CellX][start.CellY] = 0;

  for (int count = 0; count < GRID_ROWS * GRID_COLS; count++) {
    int minDistance = INT_MAX;
    int minRow;
    int minCol;

    for (int i = 0; i < GRID_ROWS; i++) {
      for (int j = 0; j < GRID_COLS; j++) {
        if (!visited[i][j] && distances[i][j] < minDistance) {
          minDistance = distances[i][j];
          minRow = i;
          minCol = j;
        }
      }
    }

    visited[minRow][minCol] = 1;

    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        if ((i == 0 || j == 0) && !(i == 0 && j == 0)) {
          int newRow = minRow + i;
          int newCol = minCol + j;

          if (isValid(newRow, newCol) && maze[newRow][newCol] != 2 && !visited[newRow][newCol] && distances[minRow][minCol] != INT_MAX) {
            int newDistance = distances[minRow][minCol] + maze[newRow][newCol];

            if (newDistance < distances[newRow][newCol]) {
              distances[newRow][newCol] = newDistance;
              predecessors[newRow][newCol] = (struct Cell){
                minRow, minCol
              };
            }
          }
        }
      }
    }

    if (minRow == goal.CellX && minCol == goal.CellY) {
      /*
        Serial.println("Maze with marked path: ");
        for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {

          if (maze[i][j] == 1) {  //if explored
            if (i == start.CellX && j == start.CellY) {
              Serial.print("S ");
            } else if (i == goal.CellX && j == goal.CellY) {
              Serial.print("G ");
              /*
                } else if (predecessors[i][j].CellX != -1 || predecessors[i][j].CellY != -1) {  //explored
                Serial.print("* ");

            } else {
              bool isPathCell = false;
              // Check if the current cell is in the path
              for (int k = 0; k < pathLength; k++) {
                if (i == path[k].CellX && j == path[k].CellY) {
                  isPathCell = true;
                  break;
                }
              }
              if (isPathCell) {
                Serial.print("@ ");
              } else {
                Serial.print(". ");
              }
            }

          } else {
            Serial.print("# ");
          }
        }
        Serial.println();
        }
        Serial.println();
      */
      //    if (minRow == goal.CellX && minCol == goal.CellY) {
      Serial.print("Shortest distance from start to goal: ");
      Serial.println(distances[goal.CellX][goal.CellY]);

      Serial.println("Shortest path:");

      struct Cell currentCell = goal;

      while (!(currentCell.CellX == start.CellX && currentCell.CellY == start.CellY)) {
        path[pathLength++] = currentCell;
        currentCell = predecessors[currentCell.CellX][currentCell.CellY];
      }

      // Print the path in reverse order
      for (int i = pathLength; i >= 0; i--) {
        Serial.print("(");
        Serial.print(path[i].CellX);
        Serial.print(", ");
        Serial.print(path[i].CellY);
        Serial.print(") -> ");
      }

      Serial.print("Path Length: ");
      Serial.println(pathLength);

      // Reverse path order
      for (int i = 0; i < pathLength / 2; i++) {
        struct Cell temp = path[i];
        path[i] = path[pathLength - 1 - i];
        path[pathLength - 1 - i] = temp;
      }

      return pathLength;
    }
  }
  // If the goal is unreachable
  Serial.println("Goal is unreachable!");
  return 0;
}


void waitForActionCompletion() {
  // Wait for ActionComplete to become true
  while (!ActionComplete) {
    wait_us(1000);  // You can optionally add a delay here to avoid busy-waiting
  }
  // Reset ActionComplete for the next action
  ActionComplete = false;
}

void RobotMoveUpdateDijkstraMove() {
  angleChecking.start(callback(anglesAndCoordsUpdate));

  int pathLength;


  // Calculate path using Dijkstra's algorithm
  pathLength = dijkstra(MazeMap, start, goal);

  // Print the path for debugging
  Serial.print("Path:");
  for (int i = 0; i < pathLength; i++) {
    Serial.print("(");
    Serial.print(path[i].CellX);
    Serial.print(", ");
    Serial.print(path[i].CellY);
    Serial.print(") ");
  }
  Serial.println("");

  // Update the maze map with obstacles
  updateMazeMapObstacles();

  CoordinateSplit();

  // Wait for some time before recalculating the path
  //wait_us(100000);

  // Update the start position with the current robot position
  start = { CoordsX, CoordsY };
}



void anglesAndCoordsUpdate() {
  while (1) {
    timeCheck.start();

    //timer check and reset =======================================================================================================================================================
    float elapsedTime = timeCheck.read();
    timeCheck.reset();

    float distanceR = PulseCountR * 0.060324;  //OLD NUMBER : 0.05027
    float distanceL = PulseCountL * 0.060324;

    if (MotorADir == 1) {
      distanceRTotal += distanceR;
    } else {
      distanceRTotal -= distanceR;
    }

    if (MotorBDir == 0) {
      distanceLTotal += distanceL;
    } else {
      distanceLTotal -= distanceL;
    }

    float deltaDistance = (distanceR + distanceL) / 2;

    if (MotorADir == 1 && MotorBDir == 0) {  //forwards
      deltaDistance = deltaDistance;
    } else if (MotorADir == 0 && MotorBDir == 0 || MotorADir == 1 && MotorBDir == 1) {  //0,0 is right, 1,1 is left
      deltaDistance = 0;
    } else if (MotorADir == 0 && MotorBDir == 1) {  //backwards
      deltaDistance = -deltaDistance;
    }

    distanceMoved += deltaDistance;

    //angles and coordinates =======================================================================================================================================================
    float deltaangle = ((std::fabs(distanceR)) + std::fabs(distanceL)) * 0.5 *6.708;  //radius is 7.27. OLD NUMBER : 6.45

    if (MotorADir == 0 && MotorBDir == 0) {  //turning right
      deltaangle = -deltaangle;
    }
    if (MotorADir == 1 && MotorBDir == 0 || MotorADir == 0 && MotorBDir == 1) {  //forward
      deltaangle = 0;
    } else {  //turning left
      deltaangle = deltaangle;
    }

    angle += deltaangle;
    angleMoved += deltaangle;
    float absAngle = std::fabs(angle);

    if (absAngle >= 360.0f) {
      angle = 0.0f;
      fullRotations++;
    }



    float deltaX = (deltaDistance)*cos(angle * 0.0174533);  //changed deltaDistance from absolute value - test if that did anything
    float deltaY = (deltaDistance)*sin(angle * 0.0174533);  //radians conversion
    positionX += deltaX;
    positionY += deltaY;


    CoordsX = (int)(positionX / 17); //put in +1 if starting from diff place
    CoordsY = (int)(positionY / 17);

    //reset values to 0 for next reading ============================================================================================================================================
    PulseCountR = 0;
    PulseCountL = 0;
    WheelRevL = 0;
    WheelRevR = 0;
    deltaDistance = 0;  //maybe switch this up?

    /*
      Serial.print("Coordinates X: ");  //prints the coordinates
      Serial.println(CoordsX);

      Serial.print("X Movement: ");
      Serial.println(XMovement);

      Serial.print("Coordinates Y: ");  //prints the coordinates
      Serial.println(CoordsY);

      Serial.print("Y Movement: ");
      Serial.println(YMovement);
    */
    ThisThread::sleep_for(10);
  }
}


void moveToCoords(int DestX, int DestY) {  //Moves the robot to the specified coordinate, by moving in the x axis then the y axis
  MovingCoords = 1;
  angleChecking.start(callback(anglesAndCoordsUpdate));
  Serial.print("DestX:");
  Serial.println(DestX);
  Serial.print("CoordsX:");
  Serial.println(CoordsX);
  Serial.print("DestY:");
  Serial.println(DestY);
  Serial.print("CoordsY:");
  Serial.println(CoordsY);
  XMovement = DestX;
  YMovement = DestY;
  x_movement = DestX;
  y_movement = DestY;

  ObstacleMapping();

  MovingCoords = 1;

  Serial.print("x movement: ");
  Serial.println(x_movement);

  Serial.print("y movement: ");
  Serial.println(y_movement);

  ActionComplete = false;
  //while (ActionComplete == false) {
  //turn_right(8, 90);
  //}
  ActionComplete = false;

  correctOrientation();

  ActionComplete = false;


  if (x_movement != 0) {
    MovingDirection = true;
    move_forward(8, std::fabs(x_movement) * 5);
  }

  if (y_movement != 0) {
    MovingDirection = false;
    move_forward(8, std::fabs(y_movement) * 5);
  }

  if (CoordsX == DestX && CoordsY == DestY) {
    Serial.println("Destination Met!");
    behaviour = stopState;
  }

  if (CoordsX == goal.CellX && CoordsY == goal.CellY) {
    goal.CellX = 0;
    goal.CellY = 0;
  }

  Serial.println("FINISHED COORD MOVE");
  //MovingCoords = 0;
  ActionComplete = false;
  ObstacleMapping();
}

void findFacing() {

  if (((85 < angle) && (95 > angle)) || ((-265 > angle) && (-275 < angle))) {  //forwards
    Facing = 'F';
  }
  if (((-5 < angle) && (5 > angle)) || ((-355 > angle) && (-365 < angle))) {  //right
    Facing = 'R';
  }
  if (((175 < angle) && (195 > angle)) || ((-175 > angle) && (-185 < angle))) {  //left
    Facing = 'L';
  }
  if (((265 < angle) && (275 > angle)) || ((-85 > angle) && (-95 < angle))) {  //backwards
    Facing = 'B';
  }
}

void findTargetFacing() {      //PROBLEM
  if (y_movement > CoordsY) {  //forwards
    TargetFacing = 'F';
  }
  if (y_movement < CoordsY) {  //backwards
    TargetFacing = 'B';
  }
  if (x_movement > CoordsX) {  //right
    TargetFacing = 'R';
  }
  if (x_movement < CoordsX) {  //left
    TargetFacing = 'L';
  }
}

void correctOrientation() {
  //FUNCTION TO TURN TO THE CORRECT AXIS FOR THE X OR Y MOVEMENT

  Serial.println("Finding Facing");
  findFacing();

  Serial.println("Finding Target Facing");
  findTargetFacing();

  ActionComplete = false;

  if (Facing == 'F') {
    Serial.println("Facing = F");
    switch (TargetFacing) {
      case 'F':
        Serial.println("Case = F");
        break;
      case 'B':
        Serial.println("Case = B");
        turn_right(8, 180);
        break;
      case 'L':
        Serial.println("Case = L");
        turn_left(8, 90);
        break;
      case 'R':
        Serial.println("Case = R");
        turn_right(8, 90);
        break;
    }
  }

  if (Facing == 'B') {
    Serial.println("Facing = B");
    switch (TargetFacing) {
      case 'F':
        Serial.println("Case = F");
        turn_right(8, 180);
        break;
      case 'B':
        Serial.println("Case = B");
        break;
      case 'L':
        Serial.println("Case = L");
        turn_right(8, 90);
        break;
      case 'R':
        Serial.println("Case = R");
        turn_left(8, 90);
        break;
    }
  }

  if (Facing == 'L') {
    Serial.println("Facing = L");

    switch (TargetFacing) {
      case 'F':
        Serial.println("Case = F");
        turn_right(8, 90);
        break;
      case 'B':
        Serial.println("Case = B");
        turn_left(8, 90);
        break;
      case 'L':
        Serial.println("Case = L");
        break;
      case 'R':
        Serial.println("Case = R");
        turn_right(8, 180);
        break;
    }
  }

  if (Facing == 'R') {
    Serial.println("Facing = R");

    if (TargetFacing == 'F') {
      Serial.println("Case = F");
      turn_left(8, 90);
    }
    if (TargetFacing == 'B') {
      Serial.println("Case = B");
      turn_right(8, 90);
    }
    if (TargetFacing == 'L') {
      Serial.println("Case = L");
      turn_right(8, 180);
    }
    if (TargetFacing == 'R') {
      Serial.println("Case = R");
    }
  }
}

void CoordinateSplit() {

  moveToCoords(path[0].CellX, path[0].CellY);
  ObstacleMapping();
}

//===================================================================================================================================================================================
void setup() {

  MotorAPWM.period_us(20);  //10us = 100kHz, 20us = 50kHz
  //MotorAPWM.write(0.2f); //duty cycle. 0% doesn't move motor, 100% is full speed

  MotorBPWM.period_us(20);
  //MotorBPWM.write(0.2f);


  EncoderR.rise(&countEncoderR);  // Attach the interrupt to the encoder's rising edge
  EncoderL.rise(&countEncoderL);

  float distanceMoved = 0;
  float angleMoved = 0;

  initialiseMazeMap();

  rtos::Thread angleChecking;
  //rtos::Thread positionFixing;
  angleChecking.start(anglesAndCoordsUpdate);

  //dijkstra(MazeMap, start, goal);

 // turn_right(8, 90);

  Serial.begin(9600);

  updateMazeMapObstacles();
}



//===================================================================================================================================================================================
void loop() {

  angleChecking.start(callback(anglesAndCoordsUpdate));
  //positionFixing.start(callback(fix_position));

  RobotMoveUpdateDijkstraMove();
  //moveToCoords(2, 2);
  //anglesAndCoordsUpdate();

  //setting period of motors =======================================================================================================================================================
  MotorAPWM.period_us(20);  //10us = 100kHz, 20us = 50kHz
  //MotorAPWM.write(0.2f); //duty cycle. 0% doesn't move motor, 100% is full speed

  MotorBPWM.period_us(20);
  //MotorBPWM.write(0.2f);

  //reading direction inputs =======================================================================================================================================================
  if (Serial.available() > 0) {
    char movement = Serial.read();
    switch (movement) {
      case ' ':
        behaviour = stopState;
        break;

      case 'w':
        behaviour = forwardState;
        break;

      case 'a':
        behaviour = leftState;
        break;

      case 's':
        behaviour = backState;
        break;

      case 'd':
        behaviour = rightState;
        break;

      case 'q':
        behaviour = defaultState;
        break;
    }
  }


  //changes movement based on the state (default things) ===========================================================================================================================
  switch (behaviour) {  //does them all at the same time it seems
    case forwardState:  //continuously moving forward
      //Serial.println("Executing forwardState");
      move_forward(speedDefault, distanceDefault);
      /*
        MotorAPWM.write(0.275);
        MotorBPWM.write(0.275);

        MotorADir = 1;
        MotorBDir = 0;
      */
      //updateMazeMapObstacles();
      //fix_position(speedDefault);  // Call fix_position continuously
      break;

    case leftState:  //continuously turning left

      //Serial.println("Executing leftState");
      turn_left(speedDefault, turnDefault);
      break;

    case rightState:
      //Serial.println("Executing rightState");
      turn_right(speedDefault, turnDefault);
      break;

    case backState:  //continuously moving backwards
      //Serial.println("Executing backState");

      MotorAPWM.write(0.275);
      MotorBPWM.write(0.275);

      MotorADir = 0;
      MotorBDir = 1;

      //move_backward(speedDefault, distanceDefault);
      break;

    case stopState:
      //Serial.println("Executing stopState");
      MotorAPWM.write(0);
      MotorBPWM.write(0);
      break;

    case defaultState:
      Serial.println("Executing DefaultState");
      testing_function();
      //CoordsChangeUpdateCall();
      //updateMazeMapObstacles();
      //RobotMoveUpdateDijkstraMove();
      //wait_us(1000000);
      break;

    default:
      Serial.println("Unknown behaviour");
  }

  wait_us(1000);
}
