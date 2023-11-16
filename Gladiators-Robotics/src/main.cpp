// import all configurations and libraries
#include "vex.h"
#include "config.h"
#include "calculations.h"
#include "threads.h"
#include "pid.h"

// configure program for competition
competition Competition;

// resets all motors and sensors
void resetMotorValues() {
  leftBackMotor.setPosition(0, degrees);
  leftFrontMotor.setPosition(0, degrees);
  rightBackMotor.setPosition(0, degrees);
  rightFrontMotor.setPosition(0, degrees); 
  DrivetrainInertial.setHeading(0, degrees);
}

// pre-autonomous code; other words for robot before-start settings!
void pre_auton(void) {

  // configure & setup initial startup elements
  Drivetrain.setStopping(brake);
  Drivetrain.setDriveVelocity(100, percent);
  intake.setBrake(hold);
  pneuCylinLeft.set(false);
  pneuCylinRight.set(false);

  // draw logo on screen
  Brain.Screen.drawImageFromFile("Robotics Logo - Resized for VEX V5.png", 0, 0);
}

// autonomous code here
void autonomousControl(void) {
  resetMotorValues();
  moveLateral(distanceToTheta(48), 0.1, 0.0, 0.01);
  rotate(90, 0.04, 0.00, 0.02);
}

// user control code here
void userControl(void) {

  // sets up all program threads
  pneumaticsControlCallback();
  toggleCatapult();
  intakeToggle();
  joystickThreadCallback();

  // ensure program stays in user control
  while(true) {
    wait(20, msec);
  }
}

// program main call
int main() {

  // set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomousControl);
  Competition.drivercontrol(userControl);

  // Runs the pre-autonomous function.
  pre_auton();

  // maintains the program running
  while (true) {
    wait(100, msec);
  }
}
