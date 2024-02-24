// checks to see if the macro is included before compilation
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "vex.h"
#include <iostream>

// these are macros that the program includes at compile time - these consume zero memory and are constant
// defines PI as the definition of atan(1) *4
#define PI atan(1)*4
#define WHEEL_DIAMETER 4.125
#define GEAR_RATIO 1.67

// used for odometry pods - but our drivetrain does not utilize them, so they are commented out
// #define PPR500 9/50
// #define PPR5120 9/512


using namespace vex;
  

// || left ||
// == ^
// == |
// back <- 'B' -> 'L'
// == |
// == v
// || right ||


struct Vec {
  double x = 0.0;
  double y = 0.0;
  double h = 0.0;
  Vec(double _x, double _y, double _h) : x(_x), y(_y), h(_h) {}
};

// the constructors have the same name as the class for its use
class odometry {
  public:
    odometry() {}
    odometry(double _length, double _back) : L(_length), B(_back) {}
    odometry(double _length, double _back, inertial& useImu) : L(_length), B(_back), imu_1(&useImu) {}
    odometry(inertial& useImu) : imu_1(&useImu) { singleTracerWheel = true; }

    void updateRobotPosition();
    const Vec& getPosition();
    double getGoalDirection(Vec origin, unsigned int quadrant);
    double getDistanceFromGoal(Vec origin, unsigned int quadrant);
    void displayPosition();
    void setStartingPosition(const Vec starting_pos) { pos = Vec(starting_pos.x, starting_pos.y, starting_pos.h); }

    const double getRbtYPos();
    const double getRevYPos();

    double toDegrees(double radians);
    double toRadians(double degrees);
    double wrapAngle(double degrees);
    bool displayPositionOnScreen = true;
    bool isCalibrating();
  private:
    inertial* imu_1 = nullptr;
    // inertial sensor values - do not change!
    double errorAdj = 1.025;
    double L = 7.6;
    double B = 0.8;
    // use only if using odometry pods - such as external encoder sensors
    bool singleTracerWheel = false;
    Vec pos = Vec(0.0, 0.0, 0.0);
};


void odometry::updateRobotPosition() {
  double oldForwardRotation = 0.0;
  double oldLeftRotation = 0.0;
  double oldRightRotation = 0.0;
  double oldHorizontalRotation = 0.0;
  double oldHeading = 0.0;
  double currentForwardRotation = 0.0;
  double currentLeftRotation = 0.0;
  double currentRightRotation = 0.0;
  double currentHorizontalRotation = 0.0;
  double currentHeading = 0.0;


  while (true) {
    // currentForwardRotation = forwardRotation.position(rev)*PPR500;
    currentForwardRotation = -1.0*((leftFrontMotor.position(rev) + rightFrontMotor.position(rev))/2.0);
    //currentLeftRotation = leftRotation.position(rev)*PPR500;
    //currentRightRotation = rightRotation.position(rev)*PPR500;
    //currentHorizontalRotation = horizontalRotation.position(rev)*PPR500;
    currentHeading = (imu_1 != nullptr) ? imu_1->rotation(rotationUnits::deg) * (errorAdj) * ((PI) / 180) : 0;


    // finds the derivative of the robot's rotation 
    double dn = currentForwardRotation - oldForwardRotation;

    // since these are set to 0 and odometry isn't used, these values are not updated and do not do anything
    double dn1 = currentLeftRotation - oldLeftRotation;
    double dn2 = currentRightRotation - oldRightRotation;
    double dn3 = currentHorizontalRotation - oldHorizontalRotation;


    // ROBOT AXIS
    double dtheta = (imu_1 != nullptr) ? currentHeading - oldHeading : ((((PI)*(WHEEL_DIAMETER)) * (dn1-dn2)) / L);
    double dx = (singleTracerWheel) ? 0 : ((PI*WHEEL_DIAMETER) * (dn3 - (dn1-dn2) * B / L));
    double dy = (singleTracerWheel) ? ((PI*WHEEL_DIAMETER) * dn) : ((PI*WHEEL_DIAMETER) * (dn1+dn2) / 2.0);
    // double dy = ((PI*WHEEL_DIAMETER) * dn);


    // FIELD AXIS
    double theta = pos.h + (dtheta / 2.0);
    pos.x += (dx*cos(theta) + dy*sin(theta));
    pos.y += (dy*cos(theta) - dx*sin(theta));
    pos.h += dtheta;


    if (displayPositionOnScreen) displayPosition();

    oldForwardRotation = currentForwardRotation;
    oldLeftRotation = currentLeftRotation;
    oldRightRotation = currentRightRotation;
    oldHorizontalRotation = currentHorizontalRotation;
    oldHeading = currentHeading;
    
    wait(10, msec); 
    }
}


double odometry::toDegrees(double radians) {
  return radians * (180 / (PI));
}


double odometry::toRadians(double degrees) {
  return degrees * ((PI) / 180);
}


double odometry::wrapAngle(double degrees) {
  while (degrees > 180) {
  degrees -= 360; 
  } 

  while (degrees < -180) {
    degrees += 360;
  }
  return degrees;
}
//For the forward axis
 /*
const double odometry::getRbtYPos() {
  return ((PI*WHEEL_DIAMETER) * 1.0 * ((leftFrontMotor.position(rev) + rightFrontMotor.position(rev))/2.0));
}
const double odometry::getRevYPos() {
  return ((PI*WHEEL_DIAMETER) * -1.0 * ((leftFrontMotor.position(rev) + rightFrontMotor.position(rev))/2.0));
}
*/

const double odometry::getRbtYPos() {
  return ((PI*WHEEL_DIAMETER*GEAR_RATIO) * 1.0 *((leftFrontMotor.position(rev) + rightFrontMotor.position(rev))/2.0));
}

const double odometry::getRevYPos() {
  return ((PI*WHEEL_DIAMETER*GEAR_RATIO) * -1.0 * ((leftFrontMotor.position(rev) + rightFrontMotor.position(rev))/2.0));
}


double odometry::getGoalDirection(Vec origin, unsigned int quadrant) {
  Vec goalPosition = (quadrant == 2 || quadrant == 4) ? Vec(130, 130, 0) : Vec(18, 120, 0);
  double x = goalPosition.x - origin.x - pos.x;
  double y = goalPosition.y - origin.y - pos.y;
  return wrapAngle(toDegrees(atan2(x, y) - pos.h));
}


double odometry::getDistanceFromGoal(Vec origin, unsigned int quadrant) {
  Vec goalPosition = (quadrant == 2 || quadrant == 4) ? Vec(130, 130, 0) : Vec(24, 120, 0);
  double x = goalPosition.x - origin.x - pos.x;
  double y = goalPosition.y - origin.y - pos.y;
  return hypot(x, y);
}


bool odometry::isCalibrating() { 
  return (imu_1 != nullptr && imu_1->isCalibrating()); 
}


void odometry::displayPosition() {
  Brain.Screen.setPenColor(vex::color(170, 170, 170));
  // Brain.Screen.setCursor(8, 27);
  // Brain.Screen.print(getGoalDirection(Vec(96 - 9, 0, 0), 4) - pos.h);
  Brain.Screen.setCursor(9, 27);
  Brain.Screen.print(pos.x);
  Brain.Screen.setCursor(10, 27);
  Brain.Screen.print(pos.y);
  if (imu_1 == nullptr || (imu_1 != nullptr && !imu_1->isCalibrating())) {
  Brain.Screen.setCursor(11, 27);
  Brain.Screen.print(toDegrees(pos.h));
  }
}
//For turning
const Vec& odometry::getPosition() {
  return pos;
}


#endif ODOMETRY_H




/*
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
      double integral = 0.0;
      double lastError = 0.0;

      // repeat until target is reached
      while (true) {

        // get average motor positions
        double motorAveragePosition = (motorA->position(vex::degrees) + motorB->position(vex::degrees) + 
        motorC->position(vex::degrees) + motorD->position(vex::degrees)) / 4;

        // get error margin
        double error = targetValue - motorAveragePosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        // output value
        double outputValue = (proportionalGain * error + integralGain * integral + derivativeGain * derivative);

        // movement
        motor_groupA->spin(vex::directionType::fwd, outputValue, vex::voltageUnits::volt);
        motor_groupB->spin(vex::directionType::fwd, outputValue, vex::voltageUnits::volt);

        // stop movement and break out of pid if error is less than 1
        if (error < 1) {
          motor_groupA->stop(vex::brake);
          motor_groupB->stop(vex::brake);
          break;
        }

         std::cout << error << std::endl;

        // prevent brain from dying
        vex::task::sleep(20);
      }
    }

    void rotate(double targetValue, double proportionalGain, double integralGain, double derivativeGain) {

      // establish accumulative variables
      double integral = 0.0;
      double lastError = 0.0;
      
      // repeat until target is reached
      while (true) {

        // get average motor positions
        double currentRotation = drivetrain_intertial->heading(vex::degrees);

        // get error margin
        double error = targetValue - currentRotation;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        // output value
        double outputValue = (proportionalGain * error + integralGain * integral + derivativeGain * derivative);

        // movement
        motor_groupA->spin(vex::directionType::rev, outputValue, vex::voltageUnits::volt);
        motor_groupB->spin(vex::directionType::fwd, outputValue, vex::voltageUnits::volt);

        // stop movement and break out of pid if error is less than 1
        if (error < 1) {
          motor_groupA->stop(vex::brake);
          motor_groupB->stop(vex::brake);
          break;
        }

       
        
        // prevent brain from dying
        vex::task::sleep(20);
      }
    }
};

*/



