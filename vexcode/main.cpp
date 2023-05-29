/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFrMotor          motor         11              
// leftBaMotor          motor         12              
// rightFrMotor         motor         16              
// rightBaMotor         motor         17              
// ---- END VEXCODE CONFIGURED DEVICES ----



#include "vex.h"
#include "PIDController.h"
#include <cmath>

using namespace vex;

competition Competition;


// defines global instances of motors and other devices 
extern motor leftFrMotor;
extern motor leftBaMotor;
extern motor rightFrMotor;
extern motor rightBaMotor;
motor_group leftDriveSmart = motor_group(leftFrMotor, leftBaMotor);
motor_group rightDriveSmart = motor_group(rightBaMotor, rightFrMotor);
drivetrain Drivetrain = drivetrain(leftDriveSmart, rightDriveSmart);

// creates global variables of the speed for left and right motor groups
double leftMotorsSpeed;
double rightMotorsSpeed;

double targetDistance = 10.0;

bool resetDriveSensors = false;
bool enablePID = true;


/*---------------------------------------------------------------------------*/
/*                 Proportional - Integral - Derivative Function             */
/*---------------------------------------------------------------------------*/

int drivetrainPID() {
  // create and configure PID controllers
  PIDController leftMotorsController(0.05, 0.0, 0.0);
  PIDController rightMotorsController(0.05, 0.0, 0.0);



  // reset motors
  while(enablePID) {
    if (resetDriveSensors) {
      resetDriveSensors = false;
      
        leftFrMotor.setPosition(0,degrees);
        rightBaMotor.setPosition(0,degrees);
        leftBaMotor.setPosition(0,degrees);
        rightFrMotor.setPosition(0,degrees);
    }
  }

  // run pid loop
  while (true) {
    

    // get all motor positions
    double leftFrMotorPos = leftFrMotor.position(degrees);
    double leftBaMotorPos = leftBaMotor.position(degrees);
    double rightFrMotorPos = rightFrMotor.position(degrees);
    double rightBaMotorPos = rightBaMotor.position(degrees);

    // average motor positions
    double leftMotorAverage = (leftBaMotorPos + leftFrMotorPos) / 2;
    double rightMotorAverage = (rightFrMotorPos + rightBaMotorPos) / 2;

    // calculate PIDs
    double PIDOutputLeftMotors = leftMotorsController.calculatePIDOutput(targetDistance, leftMotorAverage);
    double PIDOutputRightMotors = rightMotorsController.calculatePIDOutput(-targetDistance, rightMotorAverage);

    // calculate speeds
    leftMotorsSpeed = 5 + PIDOutputLeftMotors;
    rightMotorsSpeed = 5 + PIDOutputRightMotors;

    leftDriveSmart.spin(directionType::fwd, leftMotorsSpeed, voltageUnits::volt);
    rightDriveSmart.spin(directionType::fwd, rightMotorsSpeed, voltageUnits::volt);
  }
}


/*---------------------------------------------------------------------------*/
/*                               Joystick Curve                              */
/*---------------------------------------------------------------------------*/

double turningCurve = 0;
bool turningRed = true;
double forwardCurve = 0;
bool forwardRed = true;

//graph of red and blue lines from 5225A here
//https://www.desmos.com/calculator/sdcgzah5ya
int curveJoystick(bool red, int input, double t){
  int val = 0;
  if(red){
    val = (std::exp(0.6/10)+std::exp((std::abs(input)-100)/10)*(1-std::exp(-0.6/10))) * input;
  }
  else {
    //blue
    val = std::exp(((std::abs(input)-100)*0.6)/1000) * input;
  }
  return val;
}



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  vexcodeInit();

  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setStopping(brake);
  Drivetrain.setTurnVelocity(100, percent);
}





/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  drivetrainPID();

}







/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    double turnVal = curveJoystick(false, Controller1.Axis1.position(percent), turningCurve); //Get curvature according to settings [-100,100]
    double forwardVal = curveJoystick(false, Controller1.Axis3.position(percent), forwardCurve); //Get curvature according to settings [-100,100]

    double turnVolts = turnVal * 0.12; //Converts to voltage
    double forwardVolts = forwardVal * 0.12; //Converts to voltage

    leftDriveSmart.spin(forward, forwardVolts + turnVolts, voltageUnits::volt); //Apply Via Voltage
    rightDriveSmart.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    

    wait(20, msec);
  }
}






int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
