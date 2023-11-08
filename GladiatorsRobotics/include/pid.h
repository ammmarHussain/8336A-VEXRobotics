#include "vex.h"

// create PID controller class
class PIDController {

  // class private variables
  private:
    vex::motor* motorA;
    vex::motor* motorB;
    vex::motor* motorC;
    vex::motor* motorD;
    vex::motor_group* motor_groupA;
    vex::motor_group* motor_groupB;
    vex::inertial* drivetrain_intertial;
  
  // class public variables
  public:
    PIDController(vex::motor* motA, vex::motor* motB, vex::motor* motC, vex::motor* motD, vex::motor_group* motGroupA, vex::motor_group* motGroupB, vex::inertial* driInertial) {
      motorA = motA;
      motorB = motB;
      motorC = motC;
      motorD = motD;
      motor_groupA = motGroupA;
      motor_groupB = motGroupB;
      drivetrain_intertial = driInertial;
    }

    // pid movement to go forward
    void moveLateral(double targetValue, double proportionalGain, double integralGain, double derivativeGain) {

      // establish accumulative variables
      double integral;
      double lastError;
      double derivative;

      // repeat until target is reached
      while (true) {

        // get average motor positions
        double motorAveragePosition = (motorA->position(vex::degrees) + motorB->position(vex::degrees) + 
        motorC->position(vex::degrees) + motorD->position(vex::degrees)) / 4;

        // get error margin
        double error = targetValue - motorAveragePosition;
        integral += error;
        derivative = error - lastError;
        lastError = error;

        // output value
        double outputValue = (proportionalGain * error + integralGain * integral + derivativeGain * derivative);

        // movement
        motor_groupA->spin(vex::directionType::fwd, outputValue, vex::voltageUnits::volt);
        motor_groupB->spin(vex::directionType::fwd, outputValue, vex::voltageUnits::volt);

        // stop movement and break out of pid if error is less than 1
        if (error < 1) {
          motorA->stop(vex::brake);
          motorB->stop(vex::brake);
          break;
        }

        // prevent brain from dying
        vex::task::sleep(20);
      }
    }

    void rotate(double targetValue, double proportionalGain, double integralGain, double derivativeGain) {

      // establish accumulative variables
      double integral;
      double lastError;
      double derivative;
      
      // repeat until target is reached
      while (true) {

        // get average motor positions
        double currentRotation = drivetrain_intertial->heading(vex::degrees);

        // get error margin
        double error = targetValue - currentRotation;
        integral += error;
        derivative = error - lastError;
        lastError = error;

        // output value
        double outputValue = (proportionalGain * error + integralGain * integral + derivativeGain * derivative);

        // movement
        motor_groupA->spin(vex::directionType::fwd, outputValue, vex::voltageUnits::volt);
        motor_groupB->spin(vex::directionType::rev, outputValue, vex::voltageUnits::volt);

        // stop movement and break out of pid if error is less than 1
        if (error < 1) {
          motorA->stop(vex::brake);
          motorB->stop(vex::brake);
          break;
        }

        // prevent brain from dying
        vex::task::sleep(20);
      }
    }
};
