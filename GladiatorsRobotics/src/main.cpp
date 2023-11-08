// import all configurations and libraries
#include "config.h"

// configure program for competition
using namespace vex;
competition Competition;

// converts distance to rotations
int distanceToTheta (int distance){
  double gearRatio = 1.67;
  double wheelDiameter = 4.25;
  double wheelCircumference = wheelDiameter * M_PI;
  double theta = (distance*360) / (wheelCircumference *gearRatio);
  return theta;
}

// resets all motors and sensors
void resetMotorValues() {
  leftBackMotor.setPosition(0, degrees);
  leftFrontMotor.setPosition(0, degrees);
  rightBackMotor.setPosition(0, degrees);
  rightFrontMotor.setPosition(0, degrees); 
  DrivetrainInertial.setHeading(0, degrees);
}

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

// determines the joystick positions
void joystickThreadCallback() {
  
  // lateral and turning sensitivity
  double turningCurve = 30;
  double forwardCurve = 30;

  // chooses between two turns
  bool turningRed = false; 
  bool forwardRed = false;
  while (true) {

    // calculates joystick curve based on function
    double turnVal = curveJoystick(turningRed, Controller1.Axis1.position(percent), turningCurve);
    double forwardVal = curveJoystick(forwardRed, Controller1.Axis3.position(percent), forwardCurve);

    // max voltage on motors is twelve volts; this converts joystick input to volts
    double turnVolts = turnVal * 0.12; 
    double forwardVolts = forwardVal * 0.12; 
    
    // Applies the voltages to the motors.
    LeftDriveSmart.spin(forward, forwardVolts + turnVolts, voltageUnits::volt); 
    RightDriveSmart.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    // Forces the thread for 10 milliseconds to sleep to prevent it from using all of the CPU's resources.
    this_thread::sleep_for(20);
  }
}

// pneumatics control code
void pneumaticsControlCallback() {

  // initial state of pneumatics
  bool pneumaticsActive = true;
  while (true) {

    // toggles pneumaticsActive variable and opens pneumatics
    if (Controller1.ButtonL2.pressing() && pneumaticsActive) {
      pneumaticsActive = false;
      pneuCylinLeft.set(true);
      pneuCylinRight.set(true);
      this_thread::sleep_for(1000);
    }

    // toggles pneumaticsActive variable and closes pneumatics
    else if (Controller1.ButtonL2.pressing()) {
      pneumaticsActive = true;
      pneuCylinLeft.set(false);
      pneuCylinRight.set(false);      
      this_thread::sleep_for(1000);
    }

    // prevent robot from throttling
    this_thread::sleep_for(20);
  }
}

// WIP FUNCTION - SEMI FUNCTIONAL!
void toggleCatapult() {

  // initial toggle state
  bool cataMotorSpin = false;
  while (true) {
    if (cataMotorSpin) {
      catapult.spin(forward);
    } 
    else {
      
      // run catapult until distance < 80mm
      while (disSense.objectDistance(mm) >=  80) {
        catapult.spin(forward);
      }
      catapult.stop();
    }

    // if R2 is pressed toggle spinning status of catapult
    if (Controller1.ButtonR2.pressing()) {
      cataMotorSpin = !cataMotorSpin;
      if (cataMotorSpin) {
        catapult.spin(forward);
      } 
      else {
        //while (disSense.objectDistance(mm) >=  80) {
        //catapult.spin(forward);
      }
      catapult.stop();
    }
    this_thread::sleep_for(200); 
  }
  this_thread::sleep_for(20);
}

// WIP FUNCTION - NON FUNCTIONAL!
void holdCatapult() { 
  /*
  while(true) { 
    if(Controller1.ButtonR1.pressing()) {
      catapultMotor.setVelocity(100, pct);
      cataSecondMotor.setVelocity(100, pct);
      catapultMotor.spin(forward);
      cataSecondMotor.spin(forward);
    }
  }
  this_thread::sleep_for(5);
  */
}

// pre-autonomous code; other words for robot before-start settings!
void pre_auton(void) {

  // configure & setup initial startup elements
  Drivetrain.setStopping(brake);
  catapult.setStopping(brake);
  DrivetrainInertial.calibrate();
  Drivetrain.setDriveVelocity(100, percent);
  catapult.setVelocity(100, percent);
  pneuCylinLeft.set(false);
  pneuCylinRight.set(false);

  // draw logo on screen
  Brain.Screen.drawImageFromFile("Robotics Logo - Resized for VEX V5.png", 0, 0);
}

// autonomous code here
void autonomous(void) {
  resetMotorValues();
  PIDController PID(&leftFrontMotor, &leftBackMotor, &rightFrontMotor, &rightBackMotor, &LeftDriveSmart, &RightDriveSmart, &DrivetrainInertial);
  PID.moveLateral(distanceToTheta(48), 0.2, 0.0, 0.0);
  PID.rotate(90, 0.14, 0.00, 0.02);
}

// user control code here
void usercontrol(void) {

  // Sets up the multithreading in a while loop that runs forever.
  thread toggleCylinders = thread(pneumaticsControlCallback);
  thread toggleCatapultThread = thread(toggleCatapult);
  thread joystickCurve = thread(joystickThreadCallback);

  // ensure program stays in user control
  while(true) {
    wait(20, msec);
  }
}

// program main call
int main() {

  // set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Runs the pre-autonomous function.
  pre_auton();

  // maintains the program running
  while (true) {
    wait(100, msec);
  }
}
