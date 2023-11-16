// prevent nesting
#pragma once

// include everything needed
#include "vex.h"
#include <cmath>

// converts distance to rotations
int distanceToTheta (int distance){
  double gearRatio = 1.67;
  double wheelDiameter = 4.25;
  double wheelCircumference = wheelDiameter * M_PI;
  double theta = (distance*360) / (wheelCircumference *gearRatio);
  return theta;
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
