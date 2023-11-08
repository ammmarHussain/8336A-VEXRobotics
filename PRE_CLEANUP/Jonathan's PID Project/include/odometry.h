#ifndef ODOMETRY_H
#include "vex.h"
#include <iostream>

// pi is equal to the inverse tan of 1 * 4
// arctan is a curve, pi is a constant
#define PI atan(1) *4

// inertial is its own distancing method, it does not use the real world wheel diameter
#define WHEEL_DIAMETER 2.9

#define PPR500 9/50
#define PPR5120 9/512

using namespace vex;

extern motor rightFrontMotor;
extern motor leftFrontMotor;


struct Vec {
  // change these values to change the origin of the inertial sensor
  
  double x = 0.0; // side to side position
  double y = 0.0; // front and back position
  double h = 0.0; // angle 

  // not majorly important - it is security in the case of error 
  Vec(double _x, double _y, double _h) : x(_x), y(_y), h(y) {}
};


// Constructors - 

class odometry {
  public:
    odometry() {}
    odometry(double _length, double _back) : L(_length), B(_back) {}
    odometry(double _length, double _back, inertial& useImu) : L(_length), B(_back), imu_1(&useImu) {}
    odometry(inertial& useImu) : imu_1(&useImu) {singleTracerWheel = true; }
    void updateRobotPosition();
    const Vec& getPosition();
    // 
    // custom calculations for certain goals can be made per game
    double closeToElevationPole (Vec origin, unsigned int quadrant); 
    // vec is renamed to the word "origin" - allows refering the original spot as origin and not "Vec"

    void displayPosition();
    void startingPosition(const Vec starting_pos) { 
      pos = Vec(starting_pos.x, starting_pos.y, starting_pos. h);}
      // Y is always front forward, never side to side
      const double getRbtYPos();
      
      // option of radians and degrees
      double toDegrees(double radians);
      double toRadians(double degrees);

      // having true or false statements for display

      bool displayPosOnScrn = true;
    
      // statement for calibration of the inertial sensor
      bool isCalibrating();

      private: 
      //ptr means it is specifically for the inertial sensor
      // any time there is a constant, it should be adjusted on a robot-by-robot basis
      inertial* imu_1 = nullptr;
      double errorAdj = 1.025;
      double L = 7.6; 
      double B = 0.8;
      bool singleTracerWheel = false;
      // do not recommend changing this, it is the origin
      // if not for pre-autonomous, you can adjust deisred position to desired origin on map
      Vec pos = Vec(0.0, 0.0, 0.0);
    };

  // method for updating the robot with the inertial sensor
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
  
  while(1) {
    // shows a relative position / angle in accordance to the motor's encoders to the original starting point
    currentForwardRotation = -1.0 *( (leftFrontMotor.position(rev) + rightFrontMotor.position(rev) )/2.0); 
    // imu_1 is the name of our object for the inertial sensor
    currentHeading = (imu_1 != nullptr) ? imu_1 -> rotation(rotationUnits::deg)*(errorAdj) *((PI)/ 180) : 0;
    
    // derivative function
    double dnFR = currentForwardRotation - oldForwardRotation;
    double dnL = currentLeftRotation - oldLeftRotation;
    double dnR = currentRightRotation - oldRightRotation;
    // do not need
    double dnH = currentHorizontalRotation - oldHorizontalRotation;
    
    // robot axis 
    double dtheta = (imu_1 != nullptr) ? currentHeading-oldHeading : ((((PI) * (WHEEL_DIAMETER)) * (dnL - dnR))/L);
    // question mark  is a true or false, similar to if else statement


    double dx = ((PI)*(WHEEL_DIAMETER)) * (dnH - (dnL - dnR) * B/L);
    double dy = ((PI) *WHEEL_DIAMETER) * ((dnL+dnR) / 2.0);
    // double dy = ((PIWHEEL_DIAMETER) dn);

    // FIELD AXIS
    double theta = pos.h + (dtheta / 2.0);
    pos.x += (dx* cos(theta) + dy* sin(theta));
    pos.y += (dy* cos(theta) - dx* sin(theta));
    pos.h += dtheta;

    if (displayPosOnScrn) displayPosition();

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

//For the foward axis
const double odometry::getRbtYPos() {
  return ((PI* WHEEL_DIAMETER)) * 1.0 * ((leftFrontMotor.position(rev) + rightFrontMotor.position(rev))/2.0);
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