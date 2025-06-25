#include <EEPROM.h>
#include <QTRSensors.h> 
#include <MeMegaPi.h>  // Needed to use and includes the Servo library  //This is the MakeBlockDrive Library
//  List all of the functions (subroutines) here.  These are termed as function prototype.
void SetBlackVariables();  //function prototype
void SetGoldVariables();  //function prototype
void blackPath();     //function prototype
void goldPath();      //function prototype

void SetQTRCalibration();
void LoadQTRCalibration();

float LFollow();  //function protype  this is the basic line follow loop.  You can call this from LFollowToBlack and LFollowDistance
void LFollowToBlack();  // function prototype
void LFollowTime(int msecs);  // function prototype to line follow for a specified time in msecs
void LFollowUntilAfterTurn();

void Drive(int16_t leftSpeed, int16_t rightSpeed);
void DriveToBlack();

void Stop();

void ShootPuck();    // Function prototype for a specified robot of "Black" or "Gold"

void RightUntilBlack();  //Function prototype to turn right by holding the right wheel at zero until you see a black line.
void RotRightUntilBlack();    // Function prototype to rotate right by turning the right wheel in teh opposite speed of the left wheel.
void LeftUntilBlack();  //function prototype to turn left by holding the left wheel at zero until you see a black line.
void RotLeftUntilBlack();   //function prototype to rotate left until you see a black line by turning the left wheel in the opposite speed of the right wheel.
//

// Black Robot Variables
float BMCal= .70;  // This is the right motor speed multiplier to adjust and go straight
int BCSVert = 0;  //Black chop servo vertical angle
int BCSDn = 98;  //Black chop servo down position
int BCSHor = BCSDn-5;  // Black chop servo hoizontal position
int BCSBlock = BCSDn-15;  //Black chop servo position for holding the ruler under tension
int BCSMid = BCSDn-45;  //Black chop servo mid position
int BCSUp = BCSDn-75;  //Black chop servo up position
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
int GCSUp = GCSDn+78;  //Gold chop servo up position
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

QTRSensors qtr;

// Motor Setup
MeMegaPiDCMotor Left_Motor(PORT1A);
MeMegaPiDCMotor Right_Motor(PORT1B);
uint8_t SpeedMultiplier = 125;

double KP = 0.05;
double KD = 0.14;

// Servos
Servo ChopServo;   //counterclockwise movement viewed from the top of the servo comes from increasing the angle of degrees 
Servo RulerServo;   //counterclockwise movement viewed from the top of the servo comes from increasing the angle of degrees 
Servo AlignServo;  //counterclockwise movement viewed from the top of the servo comes from increasing the angle of degrees 

// Line Following constants
const double TOLLERANCE = 0.1;
const double TURN_TOLLERANCE = 1;
double TURN_STRENGTH = 20.0;

int16_t BiasLimit = 12500;  // This is the limit for detecting after the robot line folowed past a turn.
const int BiasVar = 0.2 * BiasLimit;

void setup() {
  // put your setup code here, to run once:
  pinMode(StartupSensor, INPUT);
  pinMode(ColorSelectP1, INPUT);
  pinMode(ColorSelectP2, INPUT);
  pinMode(qtrSensorSelect,OUTPUT);
  digitalWrite(qtrSensorSelect,HIGH);  //Enable the emitters on the sensors board

  Serial.begin(115200);
  while (!Serial);

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

  // ------------- QTR Init -------------
  qtr.setTypeAnalog(); 
  qtr.setSensorPins(qtrSensorPins, qtrSensorCount);
  qtr.setEmitterPin(qtrSensorSelect);
  qtr.setSamplesPerSensor(2);

  LoadQTRCalibration();
  // SetQTRCalibration();

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
  //   static uint16_t qtrValues[qtrSensorCount];
  //   qtr.readCalibrated(qtrValues);
  //   bool detected = qtrValues[qtrSensorCount / 2] <= 500;

  //   Serial.print("Reading: ");
  //   Serial.println(detected ? "true" : "false");

  //   delay(100);
  // }
  
  if (Type=="Gold") {      //This is the running code for the Gold Robot  +++++++++++++++++++ 
    goldPath();
  }  //Closing of If Type is Gold" if statement
  else if (Type=="Black"){
    delay(2000);  
    blackPath();
  }  //closing of if Type is "Black" if statement
}  //Closing bracket for the main void loop

void loop() {
  // DO NOT USE!!!!!
}

//
//  Functions Section                        Functions Section                   Functions Section:
//
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
  KD = 1.3;
  BiasLimit = 10000;
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
  Right_Motor.run(rightSpeed);
  Left_Motor.run(-leftSpeed); // Flipped since wheel is facing other way
}

float LFollow() {
  static uint16_t qtrValues[qtrSensorCount];
  double qtrPosition = qtr.readLineBlack(qtrValues);

  static float previousError = 0.0f;
  float error = (5000.0f / 2.0f) - qtrPosition;

  float derivative = error - previousError;

  float pidOutput = KP * error + KD * derivative;
  previousError = error;

  int16_t leftSpeed = constrain(SpeedMultiplier + pidOutput, 0, 255);
  int16_t rightSpeed = constrain(SpeedMultiplier - pidOutput, 0, 255);

  Drive(leftSpeed, rightSpeed);

  return error;
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
    LFollow();
  }

  Stop();
}

void LFollowUntilAfterTurn() {
  static const int bufferSize = 10;
  static float errorIntegral[bufferSize] = {0};

  uint32_t bufferIndex = 0;
  int16_t totalBias = 0;
  while(abs(totalBias) < BiasLimit+BiasVar) {
    errorIntegral[bufferIndex] = LFollow();
    bufferIndex++;

    if(bufferIndex >= bufferSize) {
      bufferIndex = 0;
    }

    totalBias = 0;
    for(uint32_t i = 0; i < bufferSize; i++) {
      totalBias += errorIntegral[i];
    }
  }

  while(abs(totalBias) < (BiasLimit + BiasVar)) {
    errorIntegral[bufferIndex] = LFollow();
    bufferIndex++;

    if(bufferIndex >= bufferSize) {
      bufferIndex = 0;
    }

    totalBias = 0;
    for(uint32_t i = 0; i < bufferSize; i++) {
      totalBias += errorIntegral[i];
    }
  }

  Stop();
}


void DriveToBlack() {
  static uint16_t qtrValues[qtrSensorCount];
  qtr.readCalibrated(qtrValues);

  while(qtrValues[qtrSensorCount / 2] <= 100) {
    qtr.readCalibrated(qtrValues);
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
  AlignServo.write(AlignSUp);  //align servo Up and out of the way
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
}
//
//
void RightUntilBlack(){
  static uint16_t qtrValues[qtrSensorCount];
  qtr.readCalibrated(qtrValues);

  Drive(75, -75);
  while(qtrValues[3] <= 100) {
    qtr.readCalibrated(qtrValues);
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
  static uint16_t qtrValues[qtrSensorCount];
  qtr.readCalibrated(qtrValues);

  Drive(-75, 75);
  while(qtrValues[3] <= 100) {
    qtr.readCalibrated(qtrValues);
  }

  Stop();
}
//
void RotLeftUntilBlack(){
  while (digitalRead(qtrSensorPins[2]) == LOW
        || digitalRead(qtrSensorPins[3]) == LOW) {   //position 2 is left of center
    Drive(-100, -100);
  }
  Stop();
}
//

void SetQTRCalibration() {
  Serial.println("Calibrating...");
  for (int i = 0; i < 200; i++) {
    delay(5);
    qtr.calibrate(); // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  int addr = 0;
  // Save calibrationOn minimum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.put(addr, qtr.calibrationOn.minimum[i]);
    addr += sizeof(uint16_t);
  }

  // Save calibrationOn maximum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.put(addr, qtr.calibrationOn.maximum[i]);
    addr += sizeof(uint16_t);
  }

  // Save calibrationOff minimum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.put(addr, qtr.calibrationOff.minimum[i]);
    addr += sizeof(uint16_t);
  }

  // Save calibrationOff maximum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.put(addr, qtr.calibrationOff.maximum[i]);
    addr += sizeof(uint16_t);
  }

  Serial.println("Calibration saved to EEPROM.");
}

void LoadQTRCalibration() {
  int addr = 0;

  qtr.calibrationOn.minimum = (uint16_t *)realloc(qtr.calibrationOn.minimum, sizeof(uint16_t) * qtrSensorCount);
  qtr.calibrationOn.maximum = (uint16_t *)realloc(qtr.calibrationOn.maximum, sizeof(uint16_t) * qtrSensorCount);
  qtr.calibrationOff.minimum = (uint16_t *)realloc(qtr.calibrationOff.minimum, sizeof(uint16_t) * qtrSensorCount);
  qtr.calibrationOff.maximum = (uint16_t *)realloc(qtr.calibrationOff.maximum, sizeof(uint16_t) * qtrSensorCount);

  // Load calibrationOn minimum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.get(addr, qtr.calibrationOn.minimum[i]);
    addr += sizeof(uint16_t);
  }

  // Load calibrationOn maximum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.get(addr, qtr.calibrationOn.maximum[i]);
    addr += sizeof(uint16_t);
  }

  // Load calibrationOff minimum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.get(addr, qtr.calibrationOff.minimum[i]);
    addr += sizeof(uint16_t);
  }

  // Load calibrationOff maximum
  for (int i = 0; i < qtrSensorCount; i++) {
    EEPROM.get(addr, qtr.calibrationOff.maximum[i]);
    addr += sizeof(uint16_t);
  }

  // Mark both calibrations initialized
  qtr.calibrationOn.initialized = true;
  qtr.calibrationOff.initialized = true;

  Serial.println("Calibration loaded from EEPROM.");
}

void goldPath(){                                               //Start of goldPath    +++++++++++++++++++++++++
  DriveToBlack();
  Drive(SpeedMultiplier, SpeedMultiplier);
  delay(50);
  RightUntilBlack();

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  delay(100);
  RulerServo.write(RulerSOut);   //Iniitialize ruler servo down
  AlignServo.write(AlignSDn);   //Initialize align servo down

  LFollowTime(1000);
  LFollowUntilAfterTurn();
  LFollowTime(500);
  Stop();
  ShootPuck();

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  LFollowTime(500);
  LFollowUntilAfterTurn();

  LFollowTime(2000);
  delay(3000);

  LFollowUntilAfterTurn();
  LFollowTime(500);
  Stop();
  ShootPuck();
}   //end of void goldPath
//
//
void blackPath(){     //                                          Start of blackPath  ++++++++++++++++++++++++++++++++++++++
  DriveToBlack();
  Drive(SpeedMultiplier, SpeedMultiplier);
  delay(100);
  LeftUntilBlack();

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  delay(100);
  RulerServo.write(RulerSOut);   //Iniitialize ruler servo down
  AlignServo.write(AlignSDn);   //Initialize align servo down


  LFollowTime(1000);
  LFollowUntilAfterTurn();
  LFollowTime(700);
  Stop();
  ShootPuck();

  ChopServo.write(ChopSDn);     //Initialize chop servo down
  LFollowUntilAfterTurn();

  LFollowTime(500);
  delay(200);
  Drive(0, 75);
  delay(1000);
  Drive(SpeedMultiplier, SpeedMultiplier);
  delay(500);
  Stop();
  Drive(75, 0);
  delay(1500);
  DriveToBlack();

  LFollowTime(1000);
  LFollowUntilAfterTurn();
  LFollowTime(700);
  Stop();
  ShootPuck();

}  // end of void blackPath
//
//  
//  End of Function section
//
