#include <MeMegaPi.h>  // Needed to use and includes the Servo library  //This is the MakeBlockDrive Library
#include <MPU6050_6Axis_MotionApps20.h>    //This is from the MPU6050 library by Electronic Cats
#include <PID_v1.h>          //Needs PID library by Brett Beauregard
//  List all of the functions (subroutines) here.  These are termed as function prototype.
void SetBlackVariables();  //function prototype
void SetGoldVariables();  //function prototype
void StraightToBlack(); //function prototype
void blackPath();     //function prototype
void goldPath();      //function prototype

void LFollow();  //function protype  this is the basic line follow loop.  You can call this from LFollowToBlack and LFollowDistance
void LFollowToBlack();  // function prototype
void LFollowTime(int msecs);  // function prototype to line follow for a specified time in msecs
void LFollowToAngle(int angle);  // function prototype to line follow till a relative angle

void Drive(int16_t leftSpeed, int16_t rightSpeed);
void DriveToBlack();

void Stop();

void ShootPuck();    // Function prototype for a specified robot of "Black" or "Gold"

void RightUntilBlack();  //Function prototype to turn right by holding the right wheel at zero until you see a black line.
void RotRightUntilBlack();    // Function prototype to rotate right by turning the right wheel in teh opposite speed of the left wheel.
void LeftUntilBlack();  //function prototype to turn left by holding the left wheel at zero until you see a black line.
void RotLeftUntilBlack();   //function prototype to rotate left until you see a black line by turning the left wheel in the opposite speed of the right wheel.

float ConvertToRadians(float degrees); //function prototype to convert Degrees to Radians
float ConvertToDegrees(float radians); //function prototype to convert Radians to Degrees
void TurnToAngle(double relativeAngle); //function prototype to turn to a certain degree

float GetCurrentTurnAngle(); //function prototype return current turn angle in Radians
//

// Black Robot Variables
float BMCal= .60;  // This is the right motor speed multiplier to adjust and go straight
int BCSVert = 0;  //Black chop servo vertical angle
int BCSDn = 96;  //Black chop servo down position
int BCSHor = BCSDn-5;  // Black chop servo hoizontal position
int BCSBlock = BCSDn-15;  //Black chop servo position for holding the ruler under tension
int BCSMid = BCSDn-45;  //Black chop servo mid position
int BCSUp = BCSDn-80;  //Black chop servo up position
int BASVert = 90;  //Black align servo up position
int BASDn = 7;  //Black align servo down position
int BASHor = BASDn+1;  //Black align servo horizontal position
int BASMid = BASDn+45;  //Black align servo mid position
int BASUp = BASDn+75;  //Black align servo up position
int BRulerSOut= 135;  //Black ruler servo out position
int BRulerSDn = 90;  //Black ruler servo down position

// Gold Robot Variables
float GMCal=1.2;   // This is the right motor speed multiplier to adjust and go straight
int GCSVert = 100;  //Gold chop servo vertical angle
int GCSDn = 10;  //Gold chop servo down position
int GCSHor = GCSDn+5;  // Gold chop horizontal position
int GCSBlock = GCSDn+15;  //Gold chop position for holding the ruller under tension
int GCSMid = GCSDn+45;  //Gold chop servo mid position
int GCSUp = GCSDn+80;  //Gold chop servo up position
int GASVert = 0;  //Gold align servo vertical position
int GASDn = 87;  //Gold align servo down position
int GASHor = GASDn-1;  //Gold align servo horizontal position
int GASMid = GASDn-45;  //Gold align servo mid position
int GASUp = GASDn-75;  //Gold align servo up position
int GRulerSOut = 45;  //Gold ruler servo out position
int GRulerSDn= 90;  //Gold ruler servo down position

// Variables that get initialized as setup
int ChopSVert;
int ChopSDn;
int ChopSHor;
int ChopSBlock;
int ChopSMid;
int ChopSUp;
int AlignSVert;
int AlignSDn;
int AlignSHor;
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
uint8_t SpeedMultiplier = 100; // should be multiplied by something from [0, 1]

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

// Line Following constants
const double TOLLERANCE = 0.1;
const double TURN_TOLLERANCE = 1;
double TURN_STRENGTH = 15.0;

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
  ChopServo.write(ChopSDn);
  delay(500);
  RulerServo.write(RulerSDn);   //Iniitialize ruler servo down
  delay(500);
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

  delay(1000);

  // ------------- DMP Init -------------
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  // ------------------------------------
  
  // ------------- QTR Init -------------
  // Read from only ODD
  pinMode(qtrSensorSelect, OUTPUT);
  digitalWrite(qtrSensorSelect, HIGH);

  for(uint8_t i = 0; i < qtrSensorCount; i++) {
    pinMode(qtrSensorPins[i], INPUT);
  }

  Run();
}

void Run() {
  //  This is the path for the Gold robot                                       //This is the running code section for the Gold Robot.
  // We only want to start when the Sensor goes low
  int StateStartSensor = LOW;
  while (StateStartSensor == LOW) {
    StateStartSensor = digitalRead(StartupSensor);  //Read startup sensor state
  }

  // while(true) {
  //   for(int8_t i = 0; i < qtrSensorCount; i++) {
  //     Serial.print(digitalRead(qtrSensorPins[i]));
  //   }
  //   Serial.println();

  //   delay(100);
  // }

  if (Type=="Gold") {      //This is the running code for the Gold Robot  +++++++++++++++++++ 
    goldPath();
  }  //Closing of If Type is Gold" if statement
  else if (Type=="Black"){
    delay(5000);  
    blackPath();
  }  //closing of if Type is "Black" if statement
}  //Closing bracket for the main void loop

void loop() {
  // DO NOT USE!!!!!
}

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
  ChopSHor=BCSHor;
  ChopSBlock=BCSBlock;
  ChopSMid=BCSMid;
  ChopSUp=BCSUp;
  AlignSVert=BASVert;
  AlignSDn=BASDn;
  AlignSHor=BASHor;
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
  TURN_STRENGTH = 18.0;
  ChopSVert=GCSVert;
  ChopSDn=GCSDn;
  ChopSHor=GCSHor;
  ChopSBlock=GCSBlock;
  ChopSMid=GCSMid;
  ChopSUp=GCSUp;
  AlignSVert=GASVert;
  AlignSDn=GASDn;
  AlignSHor=GASHor;
  AlignSMid=GASMid;
  AlignSUp=GASUp;
  RulerSOut=GRulerSOut;
  RulerSDn=GRulerSDn;
  RulerServo.attach(68);  //Attach the ruler servo
  ChopServo.attach(67);
  AlignServo.attach(69);
}

void Drive(int16_t leftSpeed, int16_t rightSpeed) {
  if (Type="Gold"){
    Right_Motor.run(int(rightSpeed*GMCal));
    Left_Motor.run(-leftSpeed); // Flipped since wheel is facing other way
  }
  if (Type="Black"){
    Right_Motor.run(int(rightSpeed*BMCal));
    Left_Motor.run(-leftSpeed); // Flipped since wheel is facing other way
  }
}

// Returns true when I'm done moving to the desired angle
void TurnToAngle(double relativeAngle) {
  // Get the current heading
  float startAngle = GetCurrentTurnAngle();
  float targetAngle = startAngle + relativeAngle;

  // Normalize target to [-180, 180]
  if (targetAngle > 180) targetAngle -= 360;
  if (targetAngle < -180) targetAngle += 360;

  while (true) {
    float currentAngle = GetCurrentTurnAngle();
    float error = targetAngle - currentAngle;

    // Normalize error to [-180, 180]
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) <= TURN_TOLLERANCE) {
      break;
    }

    if (error > 0) {
      // Turn Left
      Drive(SpeedMultiplier, -SpeedMultiplier);
    } else {
      // Turn Right
      Drive(-SpeedMultiplier, SpeedMultiplier);
    }
  }

  Stop();
}

// Should problably only be used in LFollow()
int8_t ReadLinePosition() {
  int8_t position = 0;
  int8_t midpoint = qtrSensorCount / 2;
  for(int8_t i = 0; i < midpoint; i++) {
    position += digitalRead(qtrSensorPins[i]) * (i - midpoint);
    position += digitalRead(qtrSensorPins[(qtrSensorCount - 1) - i]) * (midpoint - i);
  }

  return position;
}

void LFollow() {
  int8_t qtrPosition = ReadLinePosition();

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
    Serial.println("LFollowTime");
    LFollow();
  }
}


void LFollowToAngle(int relativeAngle) {
  float startAngle = GetCurrentTurnAngle();
  float targetAngle = startAngle + relativeAngle;

  // Normalize to [-180, 180]
  if (targetAngle > 180) targetAngle -= 360;
  if (targetAngle < -180) targetAngle += 360;

  while (true) {
    float currentAngle = GetCurrentTurnAngle();
    float error = targetAngle - currentAngle;

    // Normalize error to [-180, 180]
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) <= TURN_TOLLERANCE) break;

    LFollow();
    Serial.println("LFollowToAngle");
  }

  Stop();
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
  AlignServo.write(AlignSMid);  //align servo Up and out of the way
  ChopServo.write(ChopSDn);  //chop servo down and out of the way
  delay(200);
  RulerServo.write(RulerSOut);  //ruler servo out and ready to swing
  delay(100);
  ChopServo.write(ChopSBlock);  //chop servo mid to stop the ruler
  delay(300);
  RulerServo.write(RulerSDn);  //ruler servo down to put the ruler under tension and ready to swing
  delay(400);
  ChopServo.write(ChopSDn);  //chop servo down to shoot the puck
  delay(500);
  RulerServo.write(RulerSOut);  //ruler servo to out to clear the deck
  AlignServo.write(AlignSHor);  //align servo down to be ready for movement
  delay(500);
  ChopServo.write(ChopSHor);  //chop servo mid to be ready for movement
  delay(500);
}
//
//
void RightUntilBlack(){
  while (digitalRead(qtrSensorPins[5]) == LOW) {   //position 5 is left of center
    Drive(100, 0);
  }
  Stop();
}
//
void RotRightUntilBlack(){
  while (digitalRead(qtrSensorPins[5]) == LOW) {   //position 5 is left of center
    Drive(100, -100);
  }
  Stop();
}
//
void LeftUntilBlack(){
  while (digitalRead(qtrSensorPins[2]) == LOW) {   //position 2 is right of center
    Drive(0, 100);
  }
  Stop();
}
//
void RotLeftUntilBlack(){
  while (digitalRead(qtrSensorPins[2]) == LOW) {   //position 2 is left of center
    Drive(-100, 100);
  }
  Stop();
}
//
float GetCurrentTurnAngle() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet Add commentMore actions
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  return ConvertToDegrees(ypr[0]);
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

void goldPath(){                                               //Start of goldPath    +++++++++++++++++++++++++
  DriveToBlack();
  Drive(SpeedMultiplier, SpeedMultiplier);
  delay(100);
  SpeedMultiplier = 75;
  TurnToAngle(80);

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  delay(100);
  RulerServo.write(RulerSOut);   //Iniitialize ruler servo down
  AlignServo.write(AlignSDn);   //Initialize align servo down

  SpeedMultiplier = 100;
  LFollowToAngle(80);
  SpeedMultiplier = 75;
  LFollowTime(525);
  Stop();
  ShootPuck();

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  SpeedMultiplier = 75;
  LFollowToAngle(80);

  delay(100);
  SpeedMultiplier = 100;
  TurnToAngle(30);
  delay(250);
  Drive(SpeedMultiplier, SpeedMultiplier);
  delay(1000);
  Stop();
  delay(5000);
  mpu.resetSensors();
  TurnToAngle(-60);
  DriveToBlack();
  TurnToAngle(30);

  LFollowTime(1000);


  LFollowToAngle(80);
  SpeedMultiplier = 75;
  LFollowTime(525);
  Stop();
  ShootPuck();
}   //end of void goldPath
//
//
void blackPath(){     //                                          Start of blackPath  ++++++++++++++++++++++++++++++++++++++
  DriveToBlack();
  Drive(SpeedMultiplier, SpeedMultiplier);
  delay(200);
  SpeedMultiplier = 75;
  TurnToAngle(-80);

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  delay(100);
  RulerServo.write(RulerSOut);   //Iniitialize ruler servo down
  AlignServo.write(AlignSDn);   //Initialize align servo down

  delay(500);

  SpeedMultiplier = 100;
  LFollowToAngle(-80);
  SpeedMultiplier = 75;
  LFollowTime(800);
  Stop();
  ShootPuck();

  SpeedMultiplier = 100;
  LFollowTime(5000);
  LFollowToAngle(-80);
  SpeedMultiplier = 75;
  LFollowTime(700);
  Stop();
  ShootPuck();

}  // end of void blackPath
//
//  
//  End of Function section
//
