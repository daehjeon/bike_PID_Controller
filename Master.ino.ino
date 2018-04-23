///////////////////////////////////////////////////////////
//  PID_BIKE CONTROLLER
//  
//  Notes: Inputs roll angle from accelerometer and
//         bike speed from HE sensor then uses PID to control
//         steering angle to stabilize the bike
//
///////////////////////////////////////////////////////////
#include <PID_v1.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

//MOTOR CONTROLLER PORTS
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

int motor_direction = 0;

//PID Variables
double Reference;  // will be the desired value
double Error;       // value of Reference - Output
double Input;       // Actual Roll angle of Bike (off balance or not)
double Output ;     // is the output from our PID Controller  so our new YAw angle 

//PID parameters
double Kp = 94.429, Ki = 0.025, Kd = 193.6;     //From Matlab GA solver
int tolerance = 3;                // how accurate do we want to be in terms of our yaw angle

//STUFF FOR ACCELEROMETER
unsigned int rad;
float v = 5;       // velocity the bike is travelling in
int xAxis = A0;
int yAxis = A1;
int zAxis = A2;
int offsetX = 325; //roll angle
int offsetY = 355;
int offsetZ = 440;
int X_Value = 0;
int Offset = -7;
int Y_Value = 0;
int Z_Value = 0;

//Serial Output Stuff
String txt_output = "nothing";

//create PID instance
PID myPID(&Input, &Output, &Reference, Kp, Ki, Kd, DIRECT);

void setup(){
  //For Motor
  AFMS.begin();
  
  myPID.SetOutputLimits(-10000,10000);
  Serial.begin(9600);

  // target roll angle
  Reference = 0;    
  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID.SetSampleTime(200);        //determines time step for PID 

  //Start motor
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
}

void loop(){
   uint8_t i;

  // Sets Input to Roll angle
  measure_angle();      

  // Gets Output from PID
  myPID.Compute();
  
  //gets the error from PID system
  Error = Reference - X_Value;

  //compute value output for PID and tells system to turn Right or Left depending on Error
  Change_Steering_Wheel_Value();
  delay(100);
}

void Change_Steering_Wheel_Value() {
  
  if (Error > tolerance) {
    motor_direction = 1;
    myMotor->run(FORWARD);
    delay(100);

  } else if (Error < -1*tolerance) {
    motor_direction = 2;
    myMotor->run(BACKWARD);
    delay(100);
  
  }else {
    motor_direction = 0;
    myMotor->run(RELEASE);
    delay(100);
  }
}

void measure_angle() {
  // Roll angle is set to value from sensor
  X_Value = (analogRead(xAxis) - offsetX) / 2 + Offset;
  Y_Value = (analogRead(yAxis) - offsetY) / 2;
  Z_Value = (analogRead(zAxis) - offsetZ) / 2;
  Input = X_Value;
}

void Move_Motor() {
  if (motor_direction = 1){
    //Move Motor Counter Clockwise
    myMotor->run(BACKWARD);
  }
  
  else if (motor_direction = 2){
    //Move Motor Clockwise
    myMotor->run(FORWARD);
  
  } else {
    //Don't move motor
    myMotor->run(RELEASE);
  }
}
