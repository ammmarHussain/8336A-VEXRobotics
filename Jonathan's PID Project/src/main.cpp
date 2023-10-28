// import required libraries
#include "vex.h"
#include "PIDController.h"
#include <cmath>
#include <iostream>

// configure program for competition
using namespace vex;
competition Competition;

// add all motors to program
extern motor leftBackMotor;
extern motor leftFrontMotor;
extern motor rightBackMotor;
extern motor rightFrontMotor;

extern motor catapultMotor;
extern motor cataSecondMotor;

// motor groups for left and right
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

// define drivetrain
extern drivetrain Drivetrain;

// misc. definitions
extern distance disSense;
extern digital_out pneuCylinLeft;
extern digital_out pneuCylinRight;

// inertial sensor
extern inertial DrivetrainInertial;

// converts distance to rotations
int distanceToTheta (int distance){
  double gearRatio = 1.67;
  double wheelDiameter = 4.25;
  double wheelCircumference = wheelDiameter * M_PI;
  double theta = (distance*360) / (wheelCircumference *gearRatio);
  return theta;
}

// lateral and turning sensitivity
double turningCurve = 30;
double forwardCurve = 30;

// chooses between two turns
bool turningRed = false; 
bool forwardRed = false;

// https://www.desmos.com/calculator/sdcgzah5ya
int curveJoystick(bool red, int input, double t){ 
  int val = 0;

  // less sensitive at 20-70, more sensitive when controller input is > 80
  if(red){ 
    val = (std::exp(t/10)+std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10))) * input; 
  } else {

    // more sensitive at 20-80, more gradual incline as controller input reaches max
    val = std::exp(((std::abs(input)-100)*20)/1000) * input;
  }
  return val;
}

// resets all motors to 0 degrees
void resetMotorValues() {
  leftBackMotor.setPosition(0, degrees);
  leftFrontMotor.setPosition(0, degrees);
  rightBackMotor.setPosition(0, degrees);
  rightFrontMotor.setPosition(0, degrees); 
  DrivetrainInertial.setHeading(0, degrees);
}

// configuration for PID
bool enablePID = false;
bool enableTurnPID = false;
double desiredDistance = 0;
double desiredTurn = 0;

// driving PID
PIDController straightPID(0.2, 0, 0);
int drivePID() {
  while (enablePID) {
 
    // average motor positions
    double motorAverage = (leftFrontMotor.position(degrees) + leftBackMotor.position(degrees) + 
    rightFrontMotor.position(degrees) + rightBackMotor.position(degrees)) / 4;
                        
    // calculate PIDs
    double PIDOutputMotors = straightPID.calculatePIDOutput(desiredDistance, motorAverage);

    // send spin command
    LeftDriveSmart.spin(directionType::fwd, PIDOutputMotors, voltageUnits::volt);
    RightDriveSmart.spin(directionType::fwd, PIDOutputMotors, voltageUnits::volt);
    
    // overload prevention
    vex::task::sleep(7); 
  }
  return 1;
}

// turning PID
PIDController turnPID(0.08, 0, 0);
int turningPID() {
  while (enableTurnPID) {
    
    double turnVal = DrivetrainInertial.heading(degrees);

    double PIDOutputMotors = turnPID.calculatePIDOutput(desiredTurn, turnVal);

    LeftDriveSmart.spin(directionType::fwd, PIDOutputMotors, voltageUnits::volt);
    RightDriveSmart.spin(directionType::rev, PIDOutputMotors, voltageUnits::volt);

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

// A thread can be made by writing an int/void function as normal, but including "this_thread::sleep_for(10);" at the end of the function.
// This thread can be initalized later in the program.


// This thread allows us to set a curve for the joystick during user control. 
// Without this thread, the movement was very jittery and delayed.
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

bool pneumaticsActive = true;


void pneumaticsControlCallback() {
  while (true) {
    if (Controller1.ButtonL2.pressing() && pneumaticsActive) {
      pneumaticsActive = false;
      pneuCylinLeft.set(true);
      pneuCylinRight.set(true);
      this_thread::sleep_for(1000);
    }
    else if (Controller1.ButtonL2.pressing()) {
      pneumaticsActive = true;
      pneuCylinLeft.set(false);
      pneuCylinRight.set(false);      
      this_thread::sleep_for(1000);
    }
    this_thread::sleep_for(20);
  }
}



bool cataMotorSpin = false;
void toggleCatapult() {

  while (true) {
    // catapultMotor.setVelocity(100, pct);
    // cataSecondMotor.setVelocity(100, pct);
    if (cataMotorSpin) {
      catapultMotor.spin(forward);
      cataSecondMotor.spin(forward);
    } 
    else {
      catapultMotor.stop();
      cataSecondMotor.stop();
    }

    if (Controller1.ButtonR2.pressing()) {
      cataMotorSpin = !cataMotorSpin;
      if (cataMotorSpin) {
      catapultMotor.spin(forward);
      cataSecondMotor.spin(forward);
      } 
      else {
      catapultMotor.stop();
      cataSecondMotor.stop();
      }
      this_thread::sleep_for(500); 
    }
    this_thread::sleep_for(20);
  }
}

/*void holdCatapult() { 
  while(true) { 
    if(Controller1.ButtonR1.pressing()) {
      catapultMotor.setVelocity(100, pct);
      cataSecondMotor.setVelocity(100, pct);
      catapultMotor.spin(forward);
      cataSecondMotor.spin(forward);
    }
  }
  this_thread::sleep_for(5);
}
*/

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  vexcodeInit();
  Drivetrain.setStopping(brake);
  //catapultMotor.setStopping(hold);
  //cataSecondMotor.setStopping(hold);
  DrivetrainInertial.calibrate();

  Drivetrain.setDriveVelocity(100, percent);
  catapultMotor.setVelocity(100, pct);
  cataSecondMotor.setVelocity(100, pct);

  pneuCylinLeft.set(false);
  pneuCylinRight.set(false);


  Brain.Screen.drawImageFromFile("Robotics Logo - Resized for VEX V5.png", 0, 0);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Code                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  enablePID = true;
  enableTurnPID = false;
  vex::task autonomousPD (drivePID);
  resetMotorValues();
  desiredDistance = distanceToTheta(48);
  /*waitUntil(straightPID.error < 10);
  resetMotorValues();
  desiredDistance = (-1)*distanceToTheta(20);
  resetMotorValues();
  */


  // targetDistance = distanceToTheta(18);
  // waitUntil(error == 0);
  // resetMotorValues();
  // targetDistance = distanceToTheta(-18);
  vex::task::sleep(500);
}


/*---------------------------------------------------------------------------*/
/*                             User Control Code                             */  
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  // Disables the PID function for user control so it does not interfere with controlling the drivetrain.
  enablePID = false;

  // Sets up the multithreading in a while loop that runs forever.
  thread toggleCylinders = thread(pneumaticsControlCallback);
  thread toggleCatapultThread = thread(toggleCatapult);
 // thread holdCatapultThread = thread(holdCatapult);
  while(1){
    thread joystickCurve = thread(joystickThreadCallback);
    vex::task::sleep(20);
  wait(20, msec);
  }
}

// program main call
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Runs the pre-autonomous function.
  pre_auton();

  // maintains the program running
  while (true) {
    wait(100, msec);
  }
}
