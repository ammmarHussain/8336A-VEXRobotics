// import required libraries
#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftMotorA           motor         11              
// leftMotorB           motor         12              
// rightMotorA          motor         16              
// rightMotorB          motor         17              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "PIDController.h"

// include namespace vex
using namespace vex;

int DisToTheta (int dis){
  int theta;
  int gearRatio = 60/36;
  double wheelDiameter = 4.07;
  double wheelCircumference = wheelDiameter * M_PI;
  theta = (dis*360) / (wheelCircumference *gearRatio);
  return theta;
}


// main function
int main() {

  // initialize robot configuration
  vexcodeInit();
  
  
  // brings motors from robot-config.cpp
  // labeled A is front, labeled B is back
  extern motor leftMotorA;
  extern motor leftMotorB;
  extern motor rightMotorA;
  extern motor rightMotorB;

  motor_group leftDriveSmart = motor_group(leftMotorA, leftMotorB);
  motor_group rightDriveSmart = motor_group(rightMotorA, rightMotorB);
  drivetrain Drivetrain = drivetrain(leftDriveSmart, rightDriveSmart);

  // create and configure PID controllers
  PIDController leftMotorsController(0.1, 0.0, 0.0);
  PIDController rightMotorsController(0.1, 0.0, 0.0);

  // sets desired distance
  double desiredDistance = DisToTheta(20);

  // reset motors
  leftMotorA.setPosition(0, degrees);
  leftMotorB.setPosition(0, degrees);
  rightMotorA.setPosition(0, degrees);
  rightMotorB.setPosition(0, degrees);

  // run pid loop
  while (true) {
    
    Drivetrain.setStopping(hold);

    // get all motor positions
    double leftMotorAPos = leftMotorA.position(degrees);
    double leftMotorBPos = leftMotorB.position(degrees);
    double rightMotorAPos = rightMotorA.position(degrees);
    double rightMotorBPos = rightMotorB.position(degrees);

    // average motor positions
    double leftMotorAverage = (leftMotorAPos + leftMotorBPos) / 2;
    double rightMotorAverage = (rightMotorAPos + rightMotorBPos) / 2;

    // calculate PIDs
    double PIDOutputLeftMotors = leftMotorsController.calculatePIDOutput(desiredDistance, leftMotorAverage);
    double PIDOutputRightMotors = rightMotorsController.calculatePIDOutput(desiredDistance, rightMotorAverage);
    
    // send spin command

    leftDriveSmart.spin(directionType::fwd, PIDOutputLeftMotors, voltageUnits::volt);
    rightDriveSmart.spin(directionType::fwd, PIDOutputRightMotors, voltageUnits::volt);
  }  
}