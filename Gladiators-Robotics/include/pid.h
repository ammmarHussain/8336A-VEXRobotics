// prevent nesting
#pragma once

// include everything needed
#include "config.h"

// pid movement to go forward
void moveLateral(double targetValue, double proportionalGain, double integralGain, double derivativeGain) {

  // establish accumulative variables
  double integral = 0.0;
  double lastError = 0.0;

  // repeat until target is reached
  while (true) {

    // get average motor positions
    double motorAveragePosition = (leftFrontMotor.position(degrees) + rightFrontMotor.position(degrees) + 
    leftBackMotor.position(degrees) + rightBackMotor.position(degrees)) / 4;

    // get error margin
    double error = targetValue - motorAveragePosition;
    integral += error;
    double derivative = error - lastError;
    lastError = error;

    // output value
    double outputValue = (proportionalGain * error + integralGain * integral + derivativeGain * derivative);

    // movement
    LeftDriveSmart.spin(directionType::fwd, outputValue, voltageUnits::volt);
    RightDriveSmart.spin(directionType::fwd, outputValue, voltageUnits::volt);

    // stop movement and break out of pid if error is less than 1
    if (error < 1) {
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      break;
    }

    // prevent brain from dying
    task::sleep(20);
  }
}

// pid movement to rotate
void rotate(double targetValue, double proportionalGain, double integralGain, double derivativeGain) {

  // establish accumulative variables
  double integral = 0.0;
  double lastError = 0.0;
  
  // repeat until target is reached
  while (true) {

    // get average motor positions
    double currentRotation = DrivetrainInertial.heading(degrees);

    // get error margin
    double error = targetValue - currentRotation;
    integral += error;
    double derivative = error - lastError;
    lastError = error;

    // output value
    double outputValue = (proportionalGain * error + integralGain * integral + derivativeGain * derivative);

    // movement
    LeftDriveSmart.spin(directionType::rev, outputValue, voltageUnits::volt);
    RightDriveSmart.spin(directionType::fwd, outputValue, voltageUnits::volt);

    // stop movement and break out of pid if error is less than 1
    if (error < 1) {
      LeftDriveSmart.stop(brake);
      RightDriveSmart.stop(brake);
      break;
    }

    // prevent brain from dying
    task::sleep(20);
  }
}
