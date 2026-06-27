#include <Arduino.h>

//Micromouse Maze Solver - Flood Fill Algorithm

#define MAZE_WIDTH 4
#define MAZE_HEIGHT 4
#define GOAL_X 3
#define GOAL_Y 0

enum RunState { WAITING_TO_START, EXPLORATION_RUN, SPEED_RUN };
RunState currentState = WAITING_TO_START;
bool pathSaved = false;

int robotX = 0;
int robotY = 0;
enum Direction { NORTH, EAST, SOUTH, WEST };
Direction robotDir = NORTH;

#define BUTTON_PIN  34
#define TRIG_FRONT  14
#define ECHO_FRONT  27
#define TRIG_LEFT   33
#define ECHO_LEFT   32
#define TRIG_RIGHT  25
#define ECHO_RIGHT  26
#define ENA  23
#define IN1  5
#define IN2  22
#define ENB  18
#define IN3  21
#define IN4  19
#define ENC_LEFT   13
#define ENC_RIGHT  4

float Kp = 5.00, Ki = 0, Kd = 0;
float error = 0, lastError = 0, integral = 0;

int baseSpeed = 130;
int explorationSpeed = 130;
int speedRunSpeed = 150;

int currentTurnSpeed = 100;
int explorationTurnSpeed = 100;
int speedRunTurnSpeed = 100;

const long countsPerCell = 900;
const long countsFor90DegTurn = 300;

bool justTurned = false;
const long postTurnOffsetCounts = 70; 
const long centerOffsetCounts = 100;

const float WALL_THRESHOLD_FRONT = 14.0;
const float WALL_THRESHOLD_SIDE = 14.0;
const float TARGET_SIDE_DISTANCE = 4;

unsigned long leftWallDetectTime = 0;
unsigned long rightWallDetectTime = 0;
bool leftWallValid = false;
bool rightWallValid = false;
const int WALL_DEBOUNCE_MS = 150;

volatile long leftCount = 0;
volatile long rightCount = 0;
int maze[MAZE_WIDTH][MAZE_HEIGHT];
uint8_t walls[MAZE_WIDTH][MAZE_HEIGHT] = {0};
#define NORTH_WALL 0b0001
#define EAST_WALL  0b0010
#define SOUTH_WALL 0b0100
#define WEST_WALL  0b1000

void IRAM_ATTR leftEncoderISR() { leftCount++; }
void IRAM_ATTR rightEncoderISR() { rightCount++; }

void setup();
void loop();
float getDistance(int trigPin, int echoPin);
void setMotor(int leftSpeed, int rightSpeed);
void stopMotor();
void resetEncoders();
void resetPID();
void turnRight90();
void turnLeft90();
void turnAround180();
void moveForwardOneCell();
void initializeMaze();
void printMaze();
void updateWalls();
void floodFill();

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(ENC_LEFT, INPUT_PULLUP); pinMode(ENC_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), rightEncoderISR, RISING);

  Serial.println("\nMicromouse Initialized!");
  
  initializeMaze();
  
  Serial.println("Waiting for Button Press to start...");
}

void loop() {
  
  if (currentState == WAITING_TO_START) {
    if (digitalRead(BUTTON_PIN) == HIGH) { 
      delay(50);
      while(digitalRead(BUTTON_PIN) == HIGH);
      
      Serial.println("Button Pressed! Starting in 1 second...");
      delay(1000);
      
      if (!pathSaved) {
        currentState = EXPLORATION_RUN;
        baseSpeed = explorationSpeed;
        currentTurnSpeed = explorationTurnSpeed;
        
        Serial.println("\n--- STARTING EXPLORATION RUN ---");
        Serial.println("Scanning Starting Cell...");
        updateWalls();
        floodFill(); 
        printMaze();
        delay(500); 
      } else {
        currentState = SPEED_RUN;
        baseSpeed = speedRunSpeed;
        currentTurnSpeed = speedRunTurnSpeed;
        
        robotX = 0;
        robotY = 0;
        robotDir = NORTH;
        justTurned = false;
        
        Serial.println("\n--- STARTING SPEED RUN ---");
        printMaze(); 
        delay(500);
      }
    }
    return;
  }

  if (robotX == GOAL_X && robotY == GOAL_Y) {
    if (currentState == EXPLORATION_RUN) {
      Serial.println("\n**");
      Serial.println("EXPLORATION GOAL REACHED!");
      Serial.println("Path Saved. Place robot back at (0,0).");
      Serial.println("Press the button to begin Speed Run.");
      Serial.println("**");
      stopMotor();
      
      pathSaved = true;
      currentState = WAITING_TO_START;
      return;

    } else if (currentState == SPEED_RUN) {
      Serial.println("\n**");
      Serial.println("SPEED RUN COMPLETE!");
      Serial.println("Press button to run again.");
      Serial.println("*");
      stopMotor();
      
      currentState = WAITING_TO_START;
      return;
    }
  }

  Serial.print("\nNEW CYCLE"); Serial.print(robotX); Serial.print(","); Serial.print(robotY); Serial.println("*");

  int northVal = 1000, eastVal = 1000, southVal = 1000, westVal = 1000;
  if (robotY < MAZE_HEIGHT - 1 && !(walls[robotX][robotY] & NORTH_WALL)) northVal = maze[robotX][robotY + 1];
  if (robotX < MAZE_WIDTH - 1  && !(walls[robotX][robotY] & EAST_WALL))  eastVal  = maze[robotX + 1][robotY];
  if (robotY > 0               && !(walls[robotX][robotY] & SOUTH_WALL)) southVal = maze[robotX][robotY - 1];
  if (robotX > 0               && !(walls[robotX][robotY] & WEST_WALL))  westVal  = maze[robotX - 1][robotY];

  int minVal = min(min(northVal, eastVal), min(southVal, westVal));

  Direction desiredDir;
  if (northVal == minVal) desiredDir = NORTH;
  else if (eastVal == minVal) desiredDir = EAST;
  else if (southVal == minVal) desiredDir = SOUTH;
  else desiredDir = WEST;

  int turns_needed = desiredDir - robotDir;
  if (turns_needed == 1 || turns_needed == -3) turnRight90();
  else if (turns_needed == -1 || turns_needed == 3) turnLeft90();
  else if (turns_needed == 2 || turns_needed == -2) turnAround180();
  
  moveForwardOneCell();

  if (currentState == EXPLORATION_RUN) { 
    Serial.print("Arrived at ("); Serial.print(robotX); Serial.print(","); Serial.print(robotY); Serial.println("). Scanning for walls...");
    updateWalls();
    Serial.println("Recalculating all paths with new info...");
    floodFill();
    printMaze(); 
    delay(500); 
  } else {
    Serial.print("Arrived at ("); Serial.print(robotX); Serial.print(","); Serial.print(robotY); Serial.println(").");
    delay(100); 
  }
}

float getDistance(int trigPin, int echoPin) {
  long readings[5];
  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 20000);
    float distance = duration * 0.0343 / 2.0;
    readings[i] = (distance == 0 || distance > 40) ? 40 : distance;
    delay(4);
  }
  for (int i = 0; i < 4; i++) { for (int j = i + 1; j < 5; j++) { if (readings[j] < readings[i]) { long temp = readings[i]; readings[i] = readings[j]; readings[j] = temp; } } }
  return readings[2];
}

void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255); rightSpeed = constrain(rightSpeed, -255, 255);
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW); digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH); analogWrite(ENA, abs(leftSpeed));
  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW); digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH); analogWrite(ENB, abs(rightSpeed));
}
void stopMotor() { setMotor(0, 0); }
void resetEncoders() { leftCount = 0; rightCount = 0; }
void resetPID() { integral = 0; lastError = 0; }

void turnRight90() {
  Serial.println("Action: Turning Right");
  
  resetEncoders(); resetPID();
  while ((abs(leftCount) + abs(rightCount)) / 2 < centerOffsetCounts) { 
    float distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    if(distFront < 3.0 && distFront > 0.0) {
      break;
    }
    setMotor(currentTurnSpeed, currentTurnSpeed); 
  }
  setMotor(-255, -255); delay(10);
  stopMotor(); delay(50);
  
  resetEncoders(); resetPID();
  while ((abs(leftCount) + abs(rightCount)) / 2 < countsFor90DegTurn) { setMotor(currentTurnSpeed, -currentTurnSpeed); }
  setMotor(-255, 255); delay(15);
  stopMotor();
  robotDir = (Direction)((robotDir + 1) % 4);
  justTurned = true;
}

void turnLeft90() {
  Serial.println("Action: Turning Left");
  
  resetEncoders(); resetPID();
  
  while ((abs(leftCount) + abs(rightCount)) / 2 < centerOffsetCounts ){ 
    float distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    if(distFront < 3.0 && distFront > 0.0) {
      break;
    }
    setMotor(currentTurnSpeed, currentTurnSpeed); 
  }
  setMotor(-255, -255); delay(10);
  stopMotor(); delay(50);

  resetEncoders(); resetPID();
  while ((abs(leftCount) + abs(rightCount)) / 2 < countsFor90DegTurn) { setMotor(-currentTurnSpeed, currentTurnSpeed); }
  setMotor(255, -255); delay(15);
  stopMotor();
  robotDir = (Direction)((robotDir + 3) % 4);
  justTurned = true;
}

void turnAround180() {
    Serial.println("Action: Turning Around 180 & Aligning");
    
    resetEncoders(); resetPID();
    long target = countsFor90DegTurn * 2;
    while ((abs(leftCount) + abs(rightCount)) / 2 < target) { 
        setMotor(currentTurnSpeed, -currentTurnSpeed); 
    }
    stopMotor();
    delay(100);

    int alignAttempts = 0;
    while (alignAttempts < 20) {
        float dLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
        float dRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

        if (dLeft > WALL_THRESHOLD_SIDE && dRight > WALL_THRESHOLD_SIDE) {
            break; 
        }

        float diff = dLeft - dRight;
        
        if (abs(diff) <= 1.5) {
            break; 
        }

        if (dLeft < dRight) {
            setMotor(85, -85); 
        } else {
            setMotor(-85, 85); 
        }
        
        delay(35); 
        stopMotor();
        delay(40); 
        
        alignAttempts++;
    }
    
    robotDir = (Direction)((robotDir + 2) % 4);
    justTurned = true; 
}

void moveForwardOneCell() {
  Serial.println("Action: Moving Forward One Cell");
  
  leftWallDetectTime = 0;
  rightWallDetectTime = 0;
  leftWallValid = false;
  rightWallValid = false;
  
  long targetCounts = justTurned ? (countsPerCell - postTurnOffsetCounts) : countsPerCell;
  justTurned = false;
  
  resetEncoders(); resetPID();
  
  while ((abs(leftCount) + abs(rightCount)) / 2 < targetCounts) {
      float distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
      if (distFront < 3.0 && distFront > 0.0) { 
          Serial.println("EMERGENCY STOP: Front wall < 3cm!");
          stopMotor();
          break;
      }

      float distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
      float distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
      
      if (distLeft < WALL_THRESHOLD_SIDE) {
          if (leftWallDetectTime == 0) leftWallDetectTime = millis();
          if (millis() - leftWallDetectTime > WALL_DEBOUNCE_MS) {
              leftWallValid = true;
          }
      } else {
          leftWallDetectTime = 0;
          leftWallValid = false;
      }

      if (distRight < WALL_THRESHOLD_SIDE) {
          if (rightWallDetectTime == 0) rightWallDetectTime = millis();
          if (millis() - rightWallDetectTime > WALL_DEBOUNCE_MS) {
              rightWallValid = true;
          }
      } else {
          rightWallDetectTime = 0;
          rightWallValid = false;
      }
      
      if (leftWallValid && rightWallValid) { error = distLeft - distRight; }
      else if (leftWallValid) { error = 2*(distLeft - TARGET_SIDE_DISTANCE); }
      else if (rightWallValid) { error = 2*(TARGET_SIDE_DISTANCE - distRight); }
      else { error = 0; }
      
      float correction = Kp * error;
      setMotor(constrain(baseSpeed - correction, 0, 255), constrain(baseSpeed + correction, 0, 255));
   
      delay(10);
  }
  stopMotor();
  
  if (robotDir == NORTH) robotY++;
  if (robotDir == EAST)  robotX++;
  if (robotDir == SOUTH) robotY--;
  if (robotDir == WEST)  robotX--;
}

void initializeMaze() {
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      maze[x][y] = abs(x - GOAL_X) + abs(y - GOAL_Y);
    }
  }
}

void printMaze() {
    Serial.println("\n--- ROBOT'S CURRENT MAP ---");
    for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            Serial.print("+");
            Serial.print((walls[x][y] & NORTH_WALL) ? "---" : "   ");
        }
        Serial.println("+");
        for (int x = 0; x < MAZE_WIDTH; x++) {
            Serial.print((walls[x][y] & WEST_WALL) ? "| " : "  ");
            if (x == robotX && y == robotY) { Serial.print("R"); } 
            else { Serial.print(maze[x][y]); }
            if(maze[x][y] < 10 && !(x == robotX && y == robotY)) Serial.print(" ");
            Serial.print(" ");
        }
        Serial.println("|");
    }
    for (int x = 0; x < MAZE_WIDTH; x++) { Serial.print("+---"); }
    Serial.println("+");
}

void updateWalls() {
    float distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    float distLeft  = getDistance(TRIG_LEFT, ECHO_LEFT);
    float distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("Sensor Readings: F="); Serial.print(distFront);
    Serial.print(" L="); Serial.print(distLeft);
    Serial.print(" R="); Serial.println(distRight);

    if (distFront < WALL_THRESHOLD_FRONT) {
        if (robotDir == NORTH) walls[robotX][robotY] |= NORTH_WALL;
        if (robotDir == EAST)  walls[robotX][robotY] |= EAST_WALL;
        if (robotDir == SOUTH) walls[robotX][robotY] |= SOUTH_WALL;
        if (robotDir == WEST)  walls[robotX][robotY] |= WEST_WALL;
    }
    if (distLeft < WALL_THRESHOLD_SIDE) {
        if (robotDir == NORTH) walls[robotX][robotY] |= WEST_WALL;
        if (robotDir == EAST)  walls[robotX][robotY] |= NORTH_WALL;
        if (robotDir == SOUTH) walls[robotX][robotY] |= EAST_WALL;
        if (robotDir == WEST)  walls[robotX][robotY] |= SOUTH_WALL;
    }
    if (distRight < WALL_THRESHOLD_SIDE) {
        if (robotDir == NORTH) walls[robotX][robotY] |= EAST_WALL;
        if (robotDir == EAST)  walls[robotX][robotY] |= SOUTH_WALL;
        if (robotDir == SOUTH) walls[robotX][robotY] |= WEST_WALL;
        if (robotDir == WEST)  walls[robotX][robotY] |= NORTH_WALL;
    }

    if ((walls[robotX][robotY] & NORTH_WALL) && (robotY < MAZE_HEIGHT - 1)) walls[robotX][robotY+1] |= SOUTH_WALL;
    if ((walls[robotX][robotY] & EAST_WALL)  && (robotX < MAZE_WIDTH - 1))  walls[robotX+1][robotY] |= WEST_WALL;
    if ((walls[robotX][robotY] & SOUTH_WALL) && (robotY > 0))               walls[robotX][robotY-1] |= NORTH_WALL;
    if ((walls[robotX][robotY] & WEST_WALL)  && (robotX > 0))               walls[robotX-1][robotY] |= EAST_WALL;
}

void floodFill() {
    int queueX[MAZE_WIDTH * MAZE_HEIGHT];
    int queueY[MAZE_WIDTH * MAZE_HEIGHT];
    int head = 0, tail = 0;

    for(int i = 0; i < MAZE_WIDTH; i++) for(int j = 0; j<MAZE_HEIGHT; j++) maze[i][j] = 255;

    queueX[tail] = GOAL_X;
    queueY[tail] = GOAL_Y;
    maze[GOAL_X][GOAL_Y] = 0;
    tail++;

    while (head != tail) {
        int curX = queueX[head]; int curY = queueY[head];
        head++;
        if (curY < MAZE_HEIGHT - 1 && !(walls[curX][curY] & NORTH_WALL) && maze[curX][curY+1] == 255) {
            maze[curX][curY+1] = maze[curX][curY] + 1;
            queueX[tail] = curX; queueY[tail] = curY + 1; tail++;
        }
        if (curX < MAZE_WIDTH - 1 && !(walls[curX][curY] & EAST_WALL) && maze[curX+1][curY] == 255) {
            maze[curX+1][curY] = maze[curX][curY] + 1;
            queueX[tail] = curX + 1; queueY[tail] = curY; tail++;
        }
        if (curY > 0 && !(walls[curX][curY] & SOUTH_WALL) && maze[curX][curY-1] == 255) {
            maze[curX][curY-1] = maze[curX][curY] + 1;
            queueX[tail] = curX; queueY[tail] = curY - 1; tail++;
        }
        if (curX > 0 && !(walls[curX][curY] & WEST_WALL) && maze[curX-1][curY] == 255) {
            maze[curX-1][curY] = maze[curX][curY] + 1;
            queueX[tail] = curX - 1; queueY[tail] = curY; tail++;
        }
    }
}