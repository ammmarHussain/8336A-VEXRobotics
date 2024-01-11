// import all configurations and libraries
#include "vex.h" 
#include <iostream>
#include "odometry.h"
#include <stdio.h>

// configure program for competition
using namespace vex;

odometry Test1;
motor_group LeftDriveSmart (leftBackMotor, leftFrontMotor);
motor_group RightDriveSmart (rightBackMotor, rightFrontMotor);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 320, 279.4, 279.4, mm, 1.67);
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

  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();

  DrivetrainInertial.resetRotation();
  DrivetrainInertial.resetHeading();

}

// https://www.desmos.com/calculator/sdcgzah5ya
int curveJoystick(bool red, int input, double t){ 
  int val = 0;

  // less sensitive at 20-70, more sensitive when controller input is > 80
  if(red){ 
    val = (exp(t/10)+exp((abs(input)-100)/10)*(1-exp(-t/10))) * input; 
  } else { 

    // more sensitive at 20-80, more gradual incline as controller input reaches max
    val = exp(((abs(input)-100)*20)/1000) * input;
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
    if (Controller1.ButtonR1.pressing() && pneumaticsActive) {
      pneumaticsActive = false;
      pneuCylinLeft.set(true);
      pneuCylinRight.set(true);
      this_thread::sleep_for(200);
    }

    // toggles pneumaticsActive variable and closes pneumatics
    else if (Controller1.ButtonR1.pressing()) {
      pneumaticsActive = true;
      pneuCylinLeft.set(false);
      pneuCylinRight.set(false);      
      this_thread::sleep_for(200);
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
      catapult.spin(fwd, 12, voltageUnits::volt);
    } 
    else {
      catapult.stop();
    }

    // if R2 is pressed toggle spinning status of catapult
    if (Controller1.ButtonR2.pressing()) {
      cataMotorSpin = !cataMotorSpin;
      if (cataMotorSpin) {
        catapult.spin(fwd, 12, voltageUnits::volt);
      } 
      else {
        catapult.stop();
      }
    }
    this_thread::sleep_for(150);
  }
  this_thread::sleep_for(20);
}

void intakeControl( ){
 // initial state of intake
  bool intakeSpin = false;
  directionType intakeDirection = directionType::fwd;
  while (true) {

    // if A is pressed toggle spinning status
    if (Controller1.ButtonL1.pressing()) {
      intakeSpin = !intakeSpin;
      if (intakeSpin) {
        intake.spin(intakeDirection);
      }
      else {
        intake.stop();
      }
      
      // cooldown after toggle
      this_thread::sleep_for(200); 
    }

    // reverse direction of intake
    else if (Controller1.ButtonL2.pressing()) {
      intakeSpin = !intakeSpin;
      if (intakeSpin) {
        intake.spin(reverse);
      }
      else {
        intake.stop();
      }

  
      // cooldown after toggle
      this_thread::sleep_for(300);
    }

    // prevent robot from dying
    this_thread::sleep_for(20);
  }
}


int ForwardPID (double targetValue, double P, double I, double D, bool fwrd){
  double totalError = 0;
  double lastError = Test1.getRbtYPos();
  while(1){
    double currValue = Test1.getRbtYPos();
    double currError = targetValue - currValue;
    totalError += currError;
    double diffError = currError - lastError;

    double currVolt = currError*P + totalError*I + diffError*D;
    if(fwrd){
      LeftDriveSmart.spin(forward, currVolt, voltageUnits::volt);
      RightDriveSmart.spin(forward, currVolt, voltageUnits::volt); 
    }
    else{
      LeftDriveSmart.spin(reverse, currVolt, voltageUnits::volt);
      RightDriveSmart.spin(reverse, currVolt, voltageUnits::volt);
    }

    lastError = currError;
    printf( " Forward Current Error %f", currError );
    printf(", Motor Voltage: %f\n",  currVolt);


    vex::task::sleep(20);

    if(currError < 1){
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      break;
    }
  }
  return 1;
}


int ReversePID (double targetValue, double P, double I, double D, bool fwrd){
  double totalError = 0;
  double lastError = Test1.getRevYPos();
  while(1){
    double currValue = Test1.getRevYPos();
    double currError = targetValue - currValue;
    totalError += currError;
    double diffError = currError - lastError;

    double currVolt = currError*P + totalError*I + diffError*D;
    if(fwrd){
      LeftDriveSmart.spin(reverse, currVolt, voltageUnits::volt);
      RightDriveSmart.spin(reverse, currVolt, voltageUnits::volt);
    }
    else{
      LeftDriveSmart.spin(forward, currVolt, voltageUnits::volt);
      RightDriveSmart.spin(forward, currVolt, voltageUnits::volt);
    }

    lastError = currError;
     printf( " Reverse Current Error: %f", currError);
    printf(", Motor Voltage: %f\n",  currVolt);

    vex::task::sleep(20);

    if(currError < 1){
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      printf("DONE!");
      break;
    }
  }
  return 1;
}


int rightRotatingPID (double turnTargetValue, double tP, double tI, double tD){
  double turnTotalError = 0;
  double turnLastError = Test1.toDegrees( Test1.getPosition().h );
  //printf( " Current Value %f\n", lastError );
  while(1){
    double turnCurrValue = Test1.toDegrees( Test1.getPosition().h );
    double turnCurrError = turnTargetValue - turnCurrValue;
    turnTotalError += turnCurrError;
    double turnDiffError = turnCurrError - turnLastError;
    double turnCurrVolt = turnCurrError*tP + turnTotalError*tI + turnDiffError*tD;
    LeftDriveSmart.spin(forward, turnCurrVolt, voltageUnits::volt);
    RightDriveSmart.spin(reverse, turnCurrVolt, voltageUnits::volt);

    turnLastError = turnCurrError;

    vex::task::sleep(20);
    //Brain.Screen.printAt(1, 20, "PID volt", turnCurrVolt);
    //printf( " PID volt %f\n", currVolt );
     printf( " Right Turn Current Error: %f", turnCurrError);
    printf(", Motor Voltage: %f\n",  turnCurrVolt);


    if(turnCurrError < 1){
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      break;
    }
  }
  return 1;
}

int leftRotatingPID (double turnTargetValue, double tP, double tI, double tD){
  double turnTotalError = 0;
  double turnLastError = Test1.toDegrees( Test1.getPosition().h );
  //printf( " Current Value %f\n", lastError );
  while(1){
    double turnCurrValue = Test1.toDegrees( Test1.getPosition().h );
    double turnCurrError = turnTargetValue + turnCurrValue;
    turnTotalError -= turnCurrError;
    double turnDiffError = turnCurrError - turnLastError;

    double turnCurrVolt = turnCurrError*tP + turnTotalError*tI + turnDiffError*tD;

    LeftDriveSmart.spin(reverse, turnCurrVolt, voltageUnits::volt);
    RightDriveSmart.spin(forward, turnCurrVolt, voltageUnits::volt);

    turnLastError = turnCurrError;
    vex::task::sleep(20);

    //Brain.Screen.printAt(1, 20, "PID volt", turnCurrVolt);
    //printf( " PID volt %f\n", currVolt );
    printf( " Left Turn Current Error: %f", turnCurrError);
    printf(", Motor Voltage: %f\n",  turnCurrVolt);


    if(turnCurrError < 1){
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      break;
    }
  }
  return 1;
}

int RotatingPID (double turnTargetValue, double tP, double tI, double tD){
  double turnTotalError = 0;
  double turnLastError = Test1.toDegrees(Test1.getPosition().h);
  //printf( " Current Value %f\n", lastError );
  while(1){
    double turnCurrValue = Test1.toDegrees(Test1.getPosition().h);
    double turnCurrError = turnTargetValue - turnCurrValue;
    turnTotalError += turnCurrError;
    double turnDiffError = turnCurrError - turnLastError;

    double turnCurrVolt = turnCurrError*tP + turnTotalError*tI + turnDiffError*tD;

    LeftDriveSmart.spin(forward, turnCurrVolt, voltageUnits::volt);
    RightDriveSmart.spin(reverse, turnCurrVolt, voltageUnits::volt);

    turnLastError = turnCurrError;

    vex::task::sleep(20);
    Brain.Screen.printAt(1, 20, "PID volt", turnCurrVolt);
    //printf( " PID volt %f\n", currVolt );
    printf( " Turn Current Error: %f", turnCurrError);
    printf(", Motor Voltage: %f\n",  turnCurrVolt);


    if((turnCurrError) < 1){
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      printf("DONE!");
      break;
    }
  }
  return 1;
}

// pre-autonomous code; other words for robot before-start settings!
void pre_auton(void) {

  // configure & setup initial startup elements
  Drivetrain.setStopping(brake);
 
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setTurnVelocity(100, percent);
  intake.setVelocity(100, percent);
  intake.setBrake(hold);
  //catapult.setVelocity(100, percent);
  pneuCylinLeft.set(false);
  pneuCylinRight.set(false);

  // draw logo on screen
  Brain.Screen.drawImageFromFile("Robotics Logo - Resized for VEX V5.png", 0, 0);
}



void reversePIDThread() {
  ReversePID(10, 1, 0.000, 0.3, true);
  this_thread::sleep_for(10);
}

void reversePIDThread2() {
  ReversePID(8, 0.3, 0.000, 0.0, true);
  this_thread::sleep_for(10);
}

// autonomous code here
void autonomous(void) {
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  Test1 = odometry(DrivetrainInertial);
  vex::thread([]() { 
    while(1){
      Test1.updateRobotPosition(); 
      }
    });

  // wait(3, seconds);
  RotatingPID(0, 0.4, 0.0, 0.3);
  //   ReversePID(1, 0.2, 0.0, 0.1, true);
  ForwardPID(15, 2, 0, 2, true);
  resetMotorValues();

  leftRotatingPID(30, 0.3, 0.0, 0.0);
  resetMotorValues();
 // LeftDriveSmart.resetPosition();
 // RightDriveSmart.resetPosition();

  ForwardPID(26, 0.8, 0, 0.6, true);
  resetMotorValues();

  RotatingPID(78, 0.2, 0, 0.0);
  resetMotorValues();
 // LeftDriveSmart.resetPosition();
  // RightDriveSmart.resetPosition();

  ForwardPID(80, 1, 1.5, 2, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  wait(1, seconds);

  RotatingPID(0, 0.9, 0.0, 0.9);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();

  pneuCylinLeft.set(true); 
  pneuCylinRight.set(true);
  
  ForwardPID(20, 1, 0, 1, true);

// first attempt - 95 - too little
// second attempt - 120 - too much
// third attempt - 115 - too much? - PID values wrong -> 0.7, 0.0, 0.3 - error -> -1.83
// fourth attempt - 110 -> 0.4, 0.0, 0.3, error -> -2.9
// fifth attempt - 110 -> 0.4, 0.0, 0.1, error ->
  //leftRotatingPID(110, 0.4, 0.0, 0.1);
   //intake.spin(reverse);



/* 

  printf("score triballs \n");
  ForwardPID(12.5, 1, 0.0, 0.6, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();

  leftRotatingPID(135, 1, 0.0 , 0.4);

  */
  /*
  //deactivate right wing
  pneuCylinRight.set(false);  

  // correct to align straight with the side of goal post
  printf("align with field perimeter\n");
  RotatingPID(5, 0.29, 0.0, 0.0);  
  resetMotorValues();

  // ram into bar to score triballs
  printf("Triball scoring\n");
  ForwardPID(14, 1, 0.0, 0.0, true);
  resetMotorValues();

  // back up from goal side bar 
  printf("reverse from goal side\n");
  ReversePID(7, 0.8, 0.0, 0.0, true);
  resetMotorValues();

  // turn torwards center of field zone
  printf("turn to center of field\n");
  leftRotatingPID(60, 0.22, 0.0, 0.0);
  resetMotorValues();

  printf("drive to center\n");
  ForwardPID(19, 0.7, 0.0, 0.0, true);
  resetMotorValues();

  // turn around, face intake to triball
  printf("face intake to field goal\n");
  leftRotatingPID(49, 0.22, 0.0, 0.0);

  // outtake triball to goal
  intake.spin(forward);
  wait(0.15, seconds);
  resetMotorValues();

  /// turn around, face intake to triball
  printf("turn intake to triball\n");
  leftRotatingPID(75, 0.23, 0.0,0.0);

  // pick up triball
  intake.spin(reverse);
  resetMotorValues();
  printf("reverse into triball\n");
  ReversePID(10, 0.7, 0.0, 0.0, true);
  resetMotorValues();
  wait(0.35, seconds);
  // turn around towards goal, again
  printf("turn to goal again\n");
  RotatingPID(95,0.3, 0.0,0.0);
  resetMotorValues();
  // reverse closer to goal - not fully 
  printf("bump backward to goal\n"); 
  ReversePID(6, 1 ,0.0,0.0, true);
  resetMotorValues();
  intake.spin(forward);
  wait(0.25, seconds);


  // rotate back to triball
  intake.spin(reverse);
  leftRotatingPID(64, 0.23, 0.0, 0.0);
  resetMotorValues();
  ReversePID(13, 0.6, 0.0, 0.0, true);

*/



  /*ForwardPID(24, 0.5, 0.0, 0.0, true);
resetMotorValues();
  RotatingPID(120+180, 0.06, 0.0, 0.0);
  resetMotorValues();
  intake.spin(forward);
  RotatingPID(225, 0.065, 0.0, 0.0); */
 /*
 //  intake.stop();
  ForwardPID(30, 0.4, 0.000, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  // RotatingPID(270, 0.2, 0.0, 0.0);
  Drivetrain.turnFor(-15, degrees, true);
  // Drivetrain.turnFor(-5, degrees, true);

  ForwardPID(8, 0.8, 0.0, 0.0, true);
  pneuCylinRight.set(true);
  Drivetrain.turnFor(-25.5, degrees, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  wait(0, seconds);
  Drivetrain.turnFor(5, degrees, true);

  ForwardPID(10, 1, 0.0, 0.0, true);
     pneuCylinRight.set(false);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();

  //ForwardPID(10, 0.6, 0.0, 0.0, true);

  //Drivetrain.turnFor(-6, degrees, true);

  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ReversePID(6, 0.5, 0.0, 0.0, true);
  wait(0.3, seconds);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ForwardPID(16, 1, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ReversePID(6, 0.5, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ForwardPID(12, 2, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ReversePID(6, 0.5, 0.0, 0.0, true);
  DrivetrainInertial.setHeading(0, degrees);
  intake.spin(reverse);
  RotatingPID(53, 0.3, 0.0, 0.0);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ForwardPID(4, 0.7, 0.0, 0.0, true);
  intake.spin(forward);
  wait(0.4, seconds);
  RotatingPID(245, 0.3, 0.0, 0.0);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ReversePID(6, 1, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  wait(0.1, seconds);
  ForwardPID(14, 2, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  DrivetrainInertial.resetHeading();
  intake.stop();
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  ReversePID(6, 1, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  wait(0.1, seconds);
  ForwardPID(14, 2, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
   ReversePID(10, 0.6, 0.0, 0.0, true);

*/


  /*
  ReversePID(8, 0.7, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  RotatingPID(450, 0.3, 0.0, 0.0);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  DrivetrainInertial.resetHeading();
  ForwardPID(16, 1, 0.0, 0.0, true);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  RotatingPID(525, 0.7, 0.0, 0.0);
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  DrivetrainInertial.resetHeading();
  ForwardPID(24, 1, 0.0, 0.0, true);
  */

}


bool intakeSpin = false;
// user control code here
void usercontrol(void) {



  // Sets up the multithreading in a while loop that runs forever.
  thread toggleCylinders = thread(pneumaticsControlCallback);
  thread toggleCatapultThread = thread(toggleCatapult);
  thread joystickCurve = thread(joystickThreadCallback);
  thread toggleIntake = thread(intakeControl);
    

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
