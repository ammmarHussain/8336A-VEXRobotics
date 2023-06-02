/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ammar Hussain                                             */
/*    Created:      Mon May 22 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftBackMotor        motor          3               
// leftFrontMotor       motor          4               
// rightBackMotor       motor          7               
// rightFrontMotor      motor          8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <iostream>
 

using namespace vex;
competition Competition;


/*---------------------------------------------------------------------------*/
/*                               Externing Motors                            */
/*---------------------------------------------------------------------------*/

// announces motors and devices from robot-config.cpp as variables for functions to reference 
// this 'globalizes' them and allows these variables to be used anywhere in this program
// whereas beforehand these motors could only be referenced in robot-config.cpp


extern motor leftBackMotor;
extern motor leftFrontMotor;
extern motor rightBackMotor;
extern motor rightFrontMotor;
extern motor flexWheel;
extern motor secondFlexWheel;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern drivetrain Drivetrain;
extern brain Brain;




/*---------------------------------------------------------------------------*/
/*                               Joystick Curve                              */
/*---------------------------------------------------------------------------*/

// This adjusts the sensitivity of the controller's left and right joysticks.
// It maintains the maximum speed of the motors, only affecting the middle-range.
// This allows for more precision when driving at lower speeds.


double turningCurve = 20; // Separate curve values allow one to tune the sensitivity of turning and lateral movement individually.
double forwardCurve = 20;

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




/*---------------------------------------------------------------------------*/
/*                  Converting Motor Encoder Ticks to Inches                 */
/*---------------------------------------------------------------------------*/

// converting degrees to distance
int distanceToTheta (int dis){
  float theta;
  double gearRatio = 60/36; // input gear / output gear
  double wheelDiameter = 4.07; // Omni Wheels have a slightly larger diamater than traditional 4 inch wheels.
  double wheelCircumference = wheelDiameter * M_PI;
  theta = (dis*360) / (wheelCircumference *gearRatio);
  return theta;
}



/*---------------------------------------------------------------------------*/
/*                 Proportional - Integral - Derivative Function             */
/*---------------------------------------------------------------------------*/

// A Proportional Integral Derivative Function - PID for short - uses values about the positions of the motors in a drivetrain.
// A PID that is fine tuned for a specific build of robot is highly useful as it allows for precise movements that autocorrect if need be.
// PIDs are broken up into 3 math terms that are explained further below.


//////////////////////////
// Prerequisites /////////
//////////////////////////


// Constants for the PID controller
const double kP = 0.01;  // Proportional gain
// const double kI = 0.0;  // Integral gain - Not recommended for drivetrain, so it is left out.
const double kD = 0.1;  // Derivative gain

const double turnkP = 0.0;
const double turnkD = 0.0;

int targetDistance;  // Desired distance to travel
int targetTurnValue = 0;

// Variables for the PID controller
double error, integral, derivative, lastError;
double turnError, turnIntegral, turnDerivative, turnLastError;

// Controlling PID Function
bool resetMotorValues = false;
bool enablePIDFunction = true;


// Function to calculate the PID output.
int calculatePIDOutput() {

  // Reset motor values to 0 once program is initialized and PID loop is enabled.
  while(enablePIDFunction) {
    if (resetMotorValues) { 
      resetMotorValues = false;
        
        leftBackMotor.setPosition(0, degrees);
        leftFrontMotor.setPosition(0, degrees);
        rightBackMotor.setPosition(0, degrees);
        rightFrontMotor.setPosition(0, degrees);

    }
    // Grabs motor position information
    double leftBackMotorPosition = leftBackMotor.position(degrees);
    double leftFrontMotorPosition = leftFrontMotor.position(degrees);
    double rightBackMotorPosition = rightBackMotor.position(degrees);
    double rightFrontMotorPosition = rightFrontMotor.position(degrees);

    // averages motor position values
    int averagePosition = (leftBackMotorPosition + leftFrontMotorPosition + rightBackMotorPosition + rightFrontMotorPosition)/ 4; 


  // Calculate the Proportional.
  error = targetDistance - averagePosition;

  // Calculate the Derivative
  derivative = error - lastError;

  // Calculate the integral
  // integral = integral + error;



  //////////////////////////
  // Turning PD ////////////
  //////////////////////////

  // averages motor turning position informtation
  int turnDifference = (leftBackMotorPosition + leftFrontMotorPosition - rightBackMotorPosition - rightFrontMotorPosition) / 4;

  // Calculate the turning Potential
  turnError = targetTurnValue - turnDifference;
   
  // Calculate the turning Derivative
  turnDerivative = turnError - turnLastError;


  ////////////////////////////////
  // Setting Motor Power ////////
  ///////////////////////////////

  // Calculates motor power for turns
  double turnMotorPower = (turnkP * turnError) + (turnkD * turnDerivative);
  // Calculate motor power for lateral movement
  double lateralMotorPower = (kP * error) + (kD * derivative);
  
  LeftDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
  RightDriveSmart.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

  // Refreshes lastError variable to update data
  lastError = error;
  turnLastError = turnError - turnLastError;
  
  // tells the VEX brain to sleep for 20ms so as to not overload system resources
  vex::task::sleep(20);


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


int printRpmThreadCallback () {
  // Gets the RPM of the left rear motor and stores it in a variable. 
  int leftBVelocity = leftBackMotor.velocity(rpm);
  // Prints the velocity using the variable to the terminal.
  std::cout << leftBVelocity << std::endl;
  wait(1000, msec);
  this_thread::sleep_for(10);
  return 0;
}

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

  thread printMotorRPM = thread(printRpmThreadCallback);

  // Forces the thread for 10 milliseconds to sleep to prevent it from using all of the CPU's resources.
  this_thread::sleep_for(10);

  // A threads's callback must return an int, even though the code will never
  // get here. You must return an int here. Threads can exit, but this one does not.
  return 0;
}

int flexThreadCallback () {
 // 
  if (Controller1.ButtonR2.pressing()) {
    flexWheel.spin(forward, 13, voltageUnits::volt); 
    secondFlexWheel.spin(forward, 13, voltageUnits::volt);
  

  }
  else if(Controller1.ButtonR1.pressing()){
    flexWheel.spin(reverse, 13, voltageUnits::volt);
    secondFlexWheel.spin(reverse, 13, voltageUnits::volt);
  }
  else {
    flexWheel.stop();
    secondFlexWheel.stop();
  }

  this_thread::sleep_for(10);
  return 0;
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  vexcodeInit();

  Drivetrain.setStopping(brake);
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);



}

void autonomous(void) {
  vex::task autonomousPD (calculatePIDOutput);

  enablePIDFunction = true;
  resetMotorValues = true;

  targetDistance = distanceToTheta(10);


  
  vex::task::sleep(500);

  
}

void usercontrol(void) {

  // Disables the PID function for user control so it does not interfere with controlling the drivetrain.
 enablePIDFunction = false;

  // Sets up the multithreading in a while loop that runs forever.
  while(1){
    thread joystickCurve = thread(joystickThreadCallback);
    thread flexWheelMotorControl = thread(flexThreadCallback);
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
