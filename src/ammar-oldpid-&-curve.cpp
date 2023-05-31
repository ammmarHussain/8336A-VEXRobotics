/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lilyn                                            */
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

double turningCurve = 10;
bool turningRed = false;

double forwardCurve = 10;
bool forwardRed = false;


//graph of red and blue lines from 5225A here
//https://www.desmos.com/calculator/sdcgzah5ya
int curveJoystick(bool red, int input, double t){
  int val = 0;
  if(red){
    val = (std::exp(t/10)+std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10))) * input;
  }
  else {
    //blue
    val = std::exp(((std::abs(input)-100)*20)/1000) * input;
  }
  return val;
}

// announces motors from robot-config.cpp as variables for functions to reference
extern motor leftBackMotor;
extern motor leftFrontMotor;
extern motor rightBackMotor;
extern motor rightFrontMotor;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern drivetrain Drivetrain;

// converting degrees to distance
int DisToTheta (int dis){
  int theta;
  int gearRatio = 60/36;
  double wheelDiameter = 4.07;
  double wheelCircumference = wheelDiameter * M_PI;
  theta = (dis*360) / (wheelCircumference *gearRatio);
  return theta;
}



// Constants for the PID controller
const double kP = 0.0;  // Proportional gain
// const double kI = 0.0;  // Integral gain - Not used for Drivetrain
const double kD = 0.0;  // Derivative gain

const double turnkP = 0.0;
const double turnkD = 0.0;

const double targetDistance = DisToTheta(5);  // Desired distance to travel
const double targetTurnValue = 0;

// Variables for the PID controller
double error, integral, derivative, lastError;
double turnError, turnIntegral, turnDerivative, turnLastError;

// Controlling PID Function
bool resetMotorValues = false;
bool enablePIDFunction = true;


// Function to calculate the PID output

double calculatePIDOutput() {

  // Reset motor values to 0 once program is initialized and PID loop is enabled
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
    double averagePosition = (leftBackMotorPosition + leftFrontMotorPosition + rightBackMotorPosition + rightFrontMotorPosition)/ 4; 


  // Calculate the Potential
  error = targetDistance - averagePosition;

  // Calculate the Derivative
  derivative = error - lastError;

  // Calculate the integral
  // integral = integral + error;



  //////////////////////////
  // Turning PD ////////////
  //////////////////////////

  // averages motor turning position informtation
  float turnDifference = (leftBackMotorPosition + leftFrontMotorPosition - rightBackMotorPosition - rightFrontMotorPosition) / 4;

  // Calculate the turning Potential
  turnError = targetTurnValue - turnDifference;
   
  // Calculate the turning Derivative
  turnDerivative = turnError - turnLastError;

  
  // Calculates motor power for turns
  double turnMotorPower = (turnkP * turnError) + (turnkD * turnDerivative);

  // Calculate motor power for lateral movement
  double lateralMotorPower = (kP * error) + (kD * derivative);



  ////////////////////////////////
  // Setting Motor Power ////////
  ///////////////////////////////
  
  LeftDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
  RightDriveSmart.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

  // Refreshes lastError variable to update data
  lastError = error;
  
  // tells the VEX brain to sleep for 20ms so as to not overload system resources
  vex::task::sleep(20);


  }
return 1;
}




int main() {
  // Initialize the VEX V5 components
  vexcodeInit();

  Drivetrain.setStopping(hold);
  LeftDriveSmart.setStopping(hold);
  RightDriveSmart.setStopping(hold);

  enablePIDFunction = false;
  resetMotorValues = true;

  // vex::task calculatePIDOutput;

  
  while(1){
    double turnVal = curveJoystick(turningRed, Controller1.Axis1.position(percent), turningCurve); // Get curvature according to settings [-100,100]
    double forwardVal = curveJoystick(forwardRed, Controller1.Axis3.position(percent), forwardCurve); // Get curvature according to settings [-100,100]

    double turnVolts = turnVal * 0.12; //Converts to voltage
    double forwardVolts = forwardVal * 0.12; //Converts to voltage

    LeftDriveSmart.spin(forward, forwardVolts + turnVolts, voltageUnits::volt); //Apply Via Voltage
    RightDriveSmart.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    int leftBVelocity = leftBackMotor.velocity(rpm);
    std::cout << leftBVelocity << std::endl;

    vex::task::sleep(20);
  }


}
