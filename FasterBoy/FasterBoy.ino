 // --- Libaries ---
#include <Wire.h>
#include <Zumo32U4.h>

// --- Hardware setup ---
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED lcd;

//  --- Linesensor varibale ---
#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];


// Base speeds
int baseSpeed = 250;
int maxSpeed = 300;
int turnSpeed = 250;
int originalBaseSpeed = baseSpeed;


// PID constants
float Kp = 0.7;
int lastError = 0;


// Gyro variables
int32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset = 0;
uint32_t gyroLastUpdate = 0;


// Sensor thresholds
int blackThresholdLeftRight = 1150;
int blackThresholdMiddle = 800;


// Maze state
bool mazeFound = false;
int threshold = 1450;
void setup() {
  lineSensors.initThreeSensors();
  Serial.begin(9600);
  turnSensorSetup();
  display.clear();
  display.println("A->start");
  buttonA.waitForButton();
  delay(200);
}

// --------- MAIN LOOP ---------
void loop() {
  if (mazeFound) {
    solveMaze();
  } else {
    findMaze();
  }
}


// --------- Solve maze with Proportional and inner corner detection ---------
void solveMaze() {
  readLineSensors();


  // --- Determine black detection ---
  bool centerBlack = lineSensorValues[1] > blackThresholdMiddle;
  bool leftBlack = lineSensorValues[0] > blackThresholdLeftRight;
  bool rightBlack = lineSensorValues[2] > blackThresholdLeftRight;


  // --- Proportinal wall-following (always active) ---
  int position = lineSensorValues[0] * -1 + lineSensorValues[1] * 0 + lineSensorValues[2] * 1;
  int error = position;
  int correction = Kp * error;
  int leftTarget = constrain(baseSpeed - correction, 0, maxSpeed);
  int rightTarget = constrain(baseSpeed + correction, 0, maxSpeed);
  motors.setSpeeds(leftTarget, rightTarget);

  // --- Inner corner detection ---
  if (centerBlack && (leftBlack || rightBlack)) {
    // Perform smooth 90° right turn using gyro
    rightTurn90();
    // Restore speed
    baseSpeed = originalBaseSpeed;
  }
}


// --------- Read line sensors ---------
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < NUM_SENSORS; i++) {
    lineSensorValues[i] = constrain(lineSensorValues[i], 0, 2000);
  }
  // --- Debugging for threshold ---
  Serial.print("L:");
  Serial.print(lineSensorValues[0]);
  Serial.print(" C:");
  Serial.print(lineSensorValues[1]);
  Serial.print(" R:");
  Serial.println(lineSensorValues[2]);
}



// --------- Gyro setup ---------
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  delay(500);

  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
  turnSensorReset();
}

void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  uint32_t m = micros();
  uint32_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

// --------- Smooth 90° right turn using gyro ---------
void rightTurn90() {
  turnSensorReset();
  int32_t targetAngle = -90;  // negative for right turn
  int currentAngle = 0;

  while (currentAngle > targetAngle) {
    motors.setSpeeds(baseSpeed, -baseSpeed);
    currentAngle = getTurnAngleInDegrees();
  }
}

// --------- Find maze until line/wall is found ---------
void findMaze() {
  int speed = 100;
  lcd.clear();
  lcd.println("Finding Maze");
  readLineSensors();
  if (lineSensorValues[0] > threshold && lineSensorValues[2] > threshold) {  // finder kanten med begge sensorer
    motors.setSpeeds(-speed, -speed);                                        // bakker og drejer til højre ca. 90 grader
    delay(50);
    rightTurn90();
    mazeFound = true;
    lcd.clear();
    lcd.println("Found");
    return;
  }
  if (lineSensorValues[2] > threshold) {          // finder kanten med højre sensor
    motors.setSpeeds(speed * 1.5, -speed * 1.5);  // drejer til højre for at køre loopen igen for at finde linjen med venstre sensor <-- FIXED
    delay(25);
    return;
  }
  if (lineSensorValues[0] > threshold) {          // finder kanten med venstre sensor
    motors.setSpeeds(-speed * 1.5, speed * 1.5);  //drejer til venstre for at køre loopen igen for at finde linjen med højre sensor <-- FIXED
    delay(25);
    return;
  }
  forward();
}
void forward() {
  motors.setSpeeds(100, 100);
}