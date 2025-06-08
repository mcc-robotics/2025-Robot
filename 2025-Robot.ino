#include <MeMegaPi.h>  // Needed to use and includes the Servo library
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
//  List all of the functions (subroutines) here.  These are termed as function prototype.
void StraightToBlack(); //function prototype

void LFollow();  //function protype  this is the basic line follow loop.  You can call this from LFollowToBlack and LFollowDistance
void LFollowToBlack();  // function prototype
void LFollowTime(int msecs);  // function prototype to line follow for a specified time in msecs
void LFollowToAngle(int angle);  // function prototype to line follow till a relative angle

void Drive(int16_t leftSpeed, int16_t rightSpeed);
void DriveToBlack();

void Stop();

void ShootPuck();    // Function prototype for a specified robot of "Black" or "Gold"

void RightUntilBlack();  //Function prototype to rotate right until you see a black line
void LeftUntilBlack();  //function prototype to rotate left until you see a black line.

float ConvertToRadians(float degrees); //function prototype to convert Degrees to Radians
float ConvertToDegrees(float radians); //function prototype to convert Radians to Degrees
float RetrieveRelativeAngle(float angle);
bool TurnToAngle(double angle); //function prototype to turn to a certain degree

float GetCurrentTurnAngle(); //function prototype return current turn angle in Radians
//

// Black Robot Variables
int BCSVert = 0;  //Black chop servo vertical angle
int BCSDn = 96;  //Black chop servo down position
int BCSMid = BCSDn-5;  //Black chop servo mid position
int BCSUp = BCSDn-80;  //Black chop servo up position
int BASVert = 90;  //Black align servo up position
int BASDn = 7;  //Black align servo down position
int BASMid = BASDn+10;  //Black align servo mid position
int BASUp = BASDn+45;  //Black align servo up position
int BRulerSOut= 135;  //Black ruler servo out position
int BRulerSDn = 90;  //Black ruler servo down position

// Gold Robot Variables
int GCSVert = 100;  //Gold chop servo vertical angle
int GCSDn = 10;  //Gold chop servo down position
int GCSMid = GCSDn+5;  //Gold chop servo mid position
int GCSUp = GCSDn+80;  //Gold chop servo up position
int GASVert = 0;  //Gold align servo vertical position
int GASDn = 87;  //Gold align servo down position
int GASMid = GASDn-10;  //Gold align servo mid position
int GASUp = GASDn-75;  //Gold align servo up position
int GRulerSOut = 45;  //Gold ruler servo out position
int GRulerSDn= 90;  //Gold ruler servo down position

// Variables that get initialized as setup
int ChopSVert;
int ChopSDn;
int ChopSMid;
int ChopSUp;
int AlignSVert;
int AlignSDn;
int AlignSMid;
int AlignSUp;
int RulerSOut;
int RulerSDn;

int ColorSelectP1 = 55;  // Define pin for ColorSelectP1 (A1)
int ColorSelectP2 = 56;  // Define pin for ColorSelectP2 (A2)

int StartupSensor = 4;  // Define pin for teh start up sensor circuit board signal.

String Type = "Unknown"; // Default string value

// Line following Sensor
int qtrSensorSelect = A6;  // Define pin for qtrSensorSelect (A6 or D60)
const uint8_t qtrSensorPins[] = { A7, A8, A9, A10, A11, A12 };
const uint8_t qtrSensorCount = sizeof(qtrSensorPins) / sizeof(qtrSensorPins[0]);

// Motor Setup
MeMegaPiDCMotor Left_Motor(PORT1A);
MeMegaPiDCMotor Right_Motor(PORT1B);
uint8_t SpeedMultiplier = 175; // should be multiplied by something from [0, 1]

// Servos
Servo ChopServo;   //counterclockwise movement viewed from the top of the servo comes from increasing the angle of degrees 
Servo RulerServo;   //counterclockwise movement viewed from the top of the servo comes from increasing the angle of degrees 
Servo AlignServo;  //counterclockwise movement viewed from the top of the servo comes from increasing the angle of degrees 

// Gyro
MPU6050 mpu;
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float StartTurnAngleOffset = 0;

// Line Following constants
const double TOLLERANCE = 0.1;
const double TURN_TOLLERANCE = 1;
const double TURN_STRENGTH = 15.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(StartupSensor, INPUT);
  pinMode(ColorSelectP1, INPUT);
  pinMode(ColorSelectP2, INPUT);
  pinMode(qtrSensorSelect,OUTPUT);
  digitalWrite(qtrSensorSelect,HIGH);  //Enable the emitters on the sensors board
  Serial.begin(115200);

  //  Find out if it is a gold bot or a black bot.
  int stateP1 = digitalRead(ColorSelectP1);
  int stateP2 = digitalRead(ColorSelectP2);
  if (stateP1 == HIGH && stateP2 == LOW) {
    SetBlackVariables();
  } else if (stateP1 == LOW && stateP2 == HIGH) {
    SetGoldVariables();
  } else {
    Type = "Unknown";
    Serial.println("Type is unknown ");
    while (true);
  }

  // Set to move to the start position
  RulerServo.write(RulerSDn);   //Iniitialize ruler servo down
  ChopServo.write(ChopSUp);     //Initialize chop servo down
  AlignServo.write(AlignSUp);   //Initialize align servo down

  // Initialize the MPU(Gyro)
  // ------------- MPU Init -------------
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

  StartTurnAngleOffset = GetCurrentTurnAngle();
  // ------------------------------------
  
  // ------------- QTR Init -------------
  // Read from only ODD
  pinMode(qtrSensorSelect, OUTPUT);
  digitalWrite(qtrSensorSelect, HIGH);

  for(uint8_t i = 0; i < qtrSensorCount; i++) {
    pinMode(qtrSensorPins[i], INPUT);
  }

  // We only want to start when the Sensor goes low
  int StateStartSensor = LOW;
  while (StateStartSensor == LOW) {
    StateStartSensor = digitalRead(StartupSensor);  //Read startup sensor state
  }

  DriveToBlack();
  SpeedMultiplier = 75;
  TurnToAngle(80);

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  delay(100);
  RulerServo.write(RulerSOut);   //Iniitialize ruler servo down
  AlignServo.write(AlignSDn);   //Initialize align servo down

  SpeedMultiplier = 100;
  LFollowToAngle(80);
  SpeedMultiplier = 75;
  LFollowTime(500);
  Stop();
  ShootPuck();
  // ------------------------------------
}

void loop() {
  // LFollow();

  //  This is the path for the Gold robot                                       //This is the running code section for the Gold Robot.
  // int StateStartSensor = LOW;
  // if (Type=="Gold") {
  //   StraightToBlack();
  // }  //Closing of if statement


  // // This is the path for the Black robot                                      //This is the running code section for the Black Robot.
  // if (Type=="Black"){
  //   StraightToBlack();
  // }  //closing of if statement

  // while(true);
}  //Closing bracket for the main loop

//
//  Functions Section                        Functions Section                   Functions Section:
//
void StraightToBlack(){
  while (digitalRead(qtrSensorPins[qtrSensorCount/2]) == LOW) {
    Drive(SpeedMultiplier, SpeedMultiplier);
  }

  Stop();
}

void SetBlackVariables() {
  Type = "Black";
  Serial.println("Type: " + Type);
  //  initailize my variables
  ChopSVert = BCSVert;
  ChopSDn = BCSDn;
  ChopSMid=BCSMid;
  ChopSUp=BCSUp;
  AlignSVert=BASVert;
  AlignSDn=BASDn;
  AlignSMid=BASMid;
  AlignSUp=BASUp;
  RulerSOut=BRulerSOut;
  RulerSDn=BRulerSDn;
  RulerServo.attach(68);  //Attach the ruler servo
  ChopServo.attach(69);
  AlignServo.attach(67);
}

void SetGoldVariables() {
  Type = "Gold";
  Serial.println("Type: " + Type);
  //  initialize my variables
  ChopSVert=GCSVert;
  ChopSDn=GCSDn;
  ChopSMid=GCSMid;
  ChopSUp=GCSUp;
  AlignSVert=GASVert;
  AlignSDn=GASDn;
  AlignSMid=GASMid;
  AlignSUp=GASUp;
  RulerSOut=GRulerSOut;
  RulerSDn=GRulerSDn;
  RulerServo.attach(68);  //Attach the ruler servo
  ChopServo.attach(67);
  AlignServo.attach(69);
}

void Drive(int16_t leftSpeed, int16_t rightSpeed) {
  Right_Motor.run(rightSpeed);
  Left_Motor.run(-leftSpeed); // Flipped since wheel is facing other way
}

float RetrieveRelativeAngle(float angle) {
  float finalAngle = angle + ConvertToDegrees(GetCurrentTurnAngle());

  finalAngle = fmod(angle, 360.0);  // Ensure angle is within 0-359
  if (finalAngle < 0) {
      finalAngle += 360;  // Handle negative angles
  }
  return finalAngle;
}

// Returns true when I'm done moving to the desired angle
bool TurnToAngle(double targetAngle) {
  float currentAngle;

  do {
    currentAngle = ConvertToDegrees(GetCurrentTurnAngle());
    
    if(currentAngle < targetAngle) {
      //Turn Left
      Drive(SpeedMultiplier, -SpeedMultiplier);
    } else {
      //Turn Right
      Drive(-SpeedMultiplier, SpeedMultiplier);
    }
  }
  while(abs(targetAngle - currentAngle) > TURN_TOLLERANCE);

  Stop();
  return true;
}

// Should problably only be used in LFollow()
int8_t ReadLinePosiition() {
  int8_t position = 0;
  int8_t midpoint = qtrSensorCount / 2;
  for(int8_t i = 0; i < midpoint; i++) {
    position += digitalRead(qtrSensorPins[i]) * (i - midpoint);
    position += digitalRead(qtrSensorPins[(qtrSensorCount - 1) - i]) * (midpoint - i);
  }

  return position;
}

void LFollow() {
  int8_t qtrPosition = ReadLinePosiition();

  if(qtrPosition == 0) {
    if(Type == "Gold") {
      qtrPosition = 1;
    } else {
      qtrPosition = -1;
    }
  }

  uint8_t leftSpeed = SpeedMultiplier - (qtrPosition * TURN_STRENGTH);
  uint8_t rightSpeed = SpeedMultiplier + (qtrPosition * TURN_STRENGTH);

  Drive(leftSpeed, rightSpeed);
}

//
//
void LFollowToBlack(){

}
//
//
void LFollowTime(int msecs) {
  unsigned long StartTime = millis();

  while(millis() - StartTime < msecs) {
    Serial.println(millis() - StartTime);
    LFollow();
  }
}


void LFollowToAngle(int angle) {
  float absoluteAngle = RetrieveRelativeAngle(angle);

  while(abs(absoluteAngle - ConvertToDegrees(GetCurrentTurnAngle())) > TURN_TOLLERANCE) {
    LFollow();
  }
}

void DriveToBlack() {
  while(!digitalRead(qtrSensorPins[qtrSensorCount / 2])) {
    Drive(SpeedMultiplier, SpeedMultiplier);
  }

  Stop();
}

void Stop() {
  Drive(0, 0);
}

//
//
void ShootPuck() {
  Serial.println("ShootPuck Type: " + Type);
  AlignServo.write(AlignSUp);  //align servo Up and out of the way
  delay(500);
  ChopServo.write(ChopSDn);  //chop servo down and out of the way
  delay(500);
  RulerServo.write(RulerSOut);  //ruler servo out and ready to swing
  delay(500);
  ChopServo.write(ChopSUp);  //chop servo mid to stop the ruler
  delay(500);
  RulerServo.write(RulerSDn);  //ruler servo down to put the ruler under tension and ready to swing
  delay(500);
  ChopServo.write(ChopSDn);  //chop servo down to shoot the puck
  delay(500);
  RulerServo.write(RulerSOut);  //ruler servo to out to clear the deck
  delay(500);
  AlignServo.write(AlignSDn);  //align servo down to be ready for movement
  delay(500);
  ChopServo.write(ChopSMid);  //chop servo mid to be ready for movement
  delay(500);
}
//
//
void RightUntilBlack(){

}
//
//
void LeftUntilBlack(){

}
//
//  
//  End of Function section
//

float GetCurrentTurnAngle() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet Add commentMore actions
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] -= StartTurnAngleOffset;
  }

  float convertedAngle;
  if(ypr[0] > 0) {
    convertedAngle = ypr[0];
  } else {
    convertedAngle = (2*PI) + ypr[0];
  }

  return convertedAngle;
}


float ConvertToRadians(float degrees) {
  return degrees*PI/180;
}

float ConvertToDegrees(float radians) {
  return radians*180/PI;
}

bool checkAngle() { //function that returns TRUE when robot has travelled the curve
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  float angle = ypr[0];
  if (angle < ConvertToRadians(90)){
    return FALSE;
  }
  else{
    return TRUE;
  }
}

void goldPath(){
  bool stop = FALSE;
  unsigned long shootdelay = 5000; //delay after the curve to initiate shooting mechanism. This number must be found after testing
  StraightToBlack(); //finds black line
  
  while (!TurnToAngle(90)){
    //turns to the right
  };

  while (!checkAngle){ //line follows until it detects a curved path
    LFollow();
  } 

  unsigned long startTime = millis();

  while (!stop){
    LFollow();

    if (millis() - startTime == shootdelay){
      ShootPuck();
      stop = TRUE;
    }
  }

  while (!TurnToAngle(45)){
    //turns 45 degrees
  };

  StraightToBlack(); //finds black line
}
