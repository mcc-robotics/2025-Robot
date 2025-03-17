#include <MeMegaPi.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

const uint8_t qtrSensorSelect = A6;
const uint8_t qtrSensorPins[] = { A7, A8, A9, A10, A11, A12 };
const uint8_t qtrSensorCount = sizeof(qtrSensorPins) / sizeof(qtrSensorPins[0]);

MeMegaPiDCMotor Left_Motor(PORT1A);
MeMegaPiDCMotor Right_Motor(PORT1B);
const uint8_t SpeedMultiplier = 175; // should be multiplied by something from [0, 1]

MPU6050 mpu;
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

const double KP = 20.0;
const double KD = 0.00;
double qtrPosition = 0;
double lineFollowPIDOutput = 0;
double lineFollowSetpoint = 0;
PID LineFollowPID(&qtrPosition, &lineFollowPIDOutput, &lineFollowSetpoint, KP, 0, KD, DIRECT);

const double turn_KP = 1;
const double turn_KD = 0.00;
double turnAngle = 0;
double turnPIDOutput = 0;
PID TurningPID((double*)&ypr[0], &turnPIDOutput, &turnAngle, turn_KP, 0, turn_KD, DIRECT);
const double tollerance = 0.1;

#define LINE_FOLLOWING_TESTING
// #define TURN_TO_ANGLE_TESTING
// #define MPU_TESTING

void setup() {
  Serial.begin(115200);

  // ------------- QTR Init -------------
  // Read from only ODD
  pinMode(qtrSensorSelect, OUTPUT);
  digitalWrite(qtrSensorSelect, HIGH);

  for(uint8_t i = 0; i < qtrSensorCount; i++) {
    pinMode(qtrSensorPins[i], INPUT);
  }

  LineFollowPID.SetOutputLimits(-(double)(SpeedMultiplier/2), (double)(SpeedMultiplier/2));
  LineFollowPID.SetSampleTime(1);
  LineFollowPID.SetMode(AUTOMATIC);

  TurningPID.SetOutputLimits(-(double)(SpeedMultiplier/2), (double)(SpeedMultiplier/2));
  TurningPID.SetSampleTime(1);
  TurningPID.SetMode(AUTOMATIC);
  // ------------------------------------

  // ------------- MPU Init -------------
  // Mostly taken from Example Code

#ifdef MPU_TESTING
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed!!!!!");
    while(true); // Failed so don't continue
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  // ------------- DMP Init -------------
  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  bool DMPReady = false;  // Set true if DMP init was successful
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));

    switch (devStatus) {
      case 1:
        Serial.println(F("DMP configuration updates failed"));
        break;
      case 2:
        Serial.println(F("Initial memory load failed"));
        break;
      default:
        Serial.println(F("Unknown Error Code"));
    }
  }
#endif
  // ------------------------------------
}

int8_t ReadLinePosiition() {
  int8_t position = 0;
  int8_t midpoint = qtrSensorCount / 2;
  for(int8_t i = 0; i < midpoint; i++) {
    position += digitalRead(qtrSensorPins[i]) * (i - midpoint);
    position += digitalRead(qtrSensorPins[(qtrSensorCount - 1) - i]) * (midpoint - i);
  }

  return position;
}

void Drive(uint8_t leftSpeed, uint8_t rightSpeed) {
  Right_Motor.run(rightSpeed);
  Left_Motor.run(-leftSpeed); // Flipped since wheel is facing other way
}

void LineFollowing() {
  qtrPosition = ReadLinePosiition();

  // if(LineFollowPID.Compute()) {
    // Serial.print("QTRoutput: ");
    // Serial.println(lineFollowPIDOutput);
  // }

  uint8_t leftSpeed = SpeedMultiplier - (qtrPosition * KP);
  uint8_t rightSpeed = SpeedMultiplier + (qtrPosition * KP);

  Drive(leftSpeed, rightSpeed);
}

// Returns true when I'm done moving to the desired angle
bool TurnToAngle(double angle) {
  turnAngle = angle;

  if(LineFollowPID.Compute()) {
    // Serial.print("Turnoutput: ");
    // Serial.println(turnPIDOutput);
  }
  
  //Turn in place
  uint8_t leftSpeed = turnPIDOutput;
  uint8_t rightSpeed = turnPIDOutput;

  Drive(leftSpeed, rightSpeed);
  return abs(turnPIDOutput) < tollerance;
}

void loop() {
#ifdef LINE_FOLLOWING_TESTING
  LineFollowing();
#endif

#ifdef TURN_TO_ANGLE_TESTING
  // Keep calling until we reached the desired angle
  while(!TurnToAngle(90));
#endif

  // MPU TESTING
#ifdef MPU_TESTING
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI); // I might need to make this go from 0-360 not -180 to 180
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
  }
#endif
}
