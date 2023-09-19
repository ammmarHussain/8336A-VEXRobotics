// import required libraries
#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFrontMotor       motor         3              
// leftBackMotor        motor         12              
// rightFrontMotor      motor         16              
// rightBackMotor       motor         17              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "PIDController.h"
#include "vex.h"
#include <cmath>
#include <iostream>


// include namespace vex
using namespace vex;
competition Competition;



// brings motors from robot-config.cpp

extern motor leftBackMotor;
extern motor leftFrontMotor;
extern motor rightBackMotor;
extern motor rightFrontMotor;

extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern drivetrain Drivetrain;


int DisToTheta (int dis){
  double theta;
  double gearRatio = 1.6666666666666666666666666666666666666666666667;
  double wheelDiameter = 4.07;
  double wheelCircumference = wheelDiameter * M_PI;
  theta = (dis*360) / (wheelCircumference *gearRatio);
  return theta;
}

/*---------------------------------------------------------------------------*/
/*                               Joystick Curve                              */
/*---------------------------------------------------------------------------*/

// This adjusts the sensitivity of the controller's left and right joysticks.
// It maintains the maximum speed of the motors, only affecting the middle-range.
// This allows for more precision when driving at lower speeds.


double turningCurve = 30; // Separate curve values allow one to tune the sensitivity of turning and lateral movement individually.
double forwardCurve = 30;

bool turningRed = false; // Allows the choosing between two curves - A red curve and a blue curve - described below.
bool forwardRed = false;

// https://www.desmos.com/calculator/sdcgzah5ya - visualizes the equations used below.
int curveJoystick(bool red, int input, double t){ 
  int val = 0;
  if(red){ // Red curve - Less sensitive in the range 20-70, but more sensitive when controller input is > 80.
    val = (std::exp(t/10)+std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10))) * input; 
  }
  else { // Allows
    // Blue Curve - More sensitive in the mid ranges 20-80, but a more gradual incline as controller input reaches max.
    val = std::exp(((std::abs(input)-100)*20)/1000) * input;
  }
  return val;
}

void resetMotorValues() {
  leftBackMotor.setPosition(0, degrees);
  leftFrontMotor.setPosition(0, degrees);
  rightBackMotor.setPosition(0, degrees);
  rightFrontMotor.setPosition(0, degrees);

}
bool enablePID = false;
// sets desired distance
double desiredDistance = 0;

PIDController motorController(0.05, 0, 0.02);
int drivePID() {

  // run pid loop
  while (enablePID) {
 
    // get all motor positions
    double leftFrontMotorPos = leftFrontMotor.position(degrees);
    double leftBackMotorPos = leftBackMotor.position(degrees);
    double rightFrontMotorPos = rightFrontMotor.position(degrees);
    double rightBackMotorPos = rightBackMotor.position(degrees);

    // average motor positions
    double motorAverage = (leftFrontMotorPos + leftBackMotorPos + rightFrontMotorPos + rightBackMotorPos) / 4;

    // calculate PIDs
    double PIDOutputMotors = motorController.calculatePIDOutput(desiredDistance, motorAverage);
    std::cout << "avg: " << motorAverage << " ";
    std::cout << "err: " << motorController.error << std::endl;
    // send spin command
    LeftDriveSmart.spin(directionType::fwd, PIDOutputMotors, voltageUnits::volt);
    RightDriveSmart.spin(directionType::fwd, PIDOutputMotors, voltageUnits::volt);
    

    vex::task::sleep(7); 
  } 
  return 1; 
}


/*---------------------------------------------------------------------------*/
/*                     Creating threads for Multithreading                   */
/*---------------------------------------------------------------------------*/

// Threads are functions that allow the program and hardware to "split" its resources running.
// Essentially, while typical loops do not allow for other loops to take place until it is done,
// threads will allow you to run multiple functions in a loop simoultaenously.
// This is known as multithreading. 

int joystickThreadCallback() {
  // Performs a callback to the curveJoystick function, taking Axis 1 and Axis 3 of the controller as values among the other variables.
  double turnVal = curveJoystick(turningRed, Controller1.Axis1.position(percent), turningCurve); // Get curvature according to settings [-100,100]
  double forwardVal = curveJoystick(forwardRed, Controller1.Axis3.position(percent), forwardCurve); // Get curvature according to settings [-100,100]

  // Converts the values obtained from the above callbacks to voltages by multiplying by 0.12. (Why? The max voltage on motors is 12 volts.)
  double turnVolts = turnVal * 0.12; 
  double forwardVolts = forwardVal * 0.12; 
  
  // Applies the voltages to the motors.
  LeftDriveSmart.spin(forward, forwardVolts + turnVolts, voltageUnits::volt); 
  RightDriveSmart.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

  // Forces the thread for 10 milliseconds to sleep to prevent it from using all of the CPU's resources.
  this_thread::sleep_for(10);

  // A threads's callback must return an int, even though the code will never
  // get here. You must return an int here. Threads can exit, but this one does not.
  return 0;
}



void pre_auton(void) {
  vexcodeInit();

  Drivetrain.setStopping(brake);
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);

  Brain.Screen.drawImageFromFile("roboticslogo.png", 0, 0);

}

void autonomous(void) {
  enablePID = true;
  vex::task autonomousPD (drivePID);
  resetMotorValues();
  desiredDistance = DisToTheta(24);
  waitUntil(motorController.error==0);
  desiredDistance = DisToTheta(-24);
  // targetDistance = distanceToTheta(18);
  // waitUntil(error == 0);
  // resetMotorValues();
  // targetDistance = distanceToTheta(-18);
  vex::task::sleep(500);
}

void usercontrol(void) {
  // Disables the PID function for user control so it does not interfere with controlling the drivetrain.
  enablePID = false;
  // Sets up the multithreading in a while loop that runs forever.
  while(1){
    thread joystickCurve = thread(joystickThreadCallback);
    vex::task::sleep(20);
  wait(20, msec);
  }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Runs the pre-autonomous function.
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
