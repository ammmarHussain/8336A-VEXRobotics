// prevent nesting
#pragma once

// include everything needed
#include "config.h"
#include "calculations.h"

// determines the joystick positions
thread joystickThreadCallback() {
  
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
    
    // applies voltages
    LeftDriveSmart.spin(forward, forwardVolts + turnVolts, voltageUnits::volt); 
    RightDriveSmart.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    // prevent robot from dying
    this_thread::sleep_for(20);
  }
}

// toggles the catapult movement
thread toggleCatapult() {

  // initial toggle state
  bool cataMotorSpin = false;
  while (true) {
  
    // if R2 is pressed toggle spinning status of catapult
    if (Controller1.ButtonR2.pressing()) {
      cataMotorSpin = !cataMotorSpin;
      if (cataMotorSpin) {
        catapult.spin(fwd, 12, voltageUnits::volt);
      } 
      else {
        catapult.stop();
      }

      // cooldown after toggle
      this_thread::sleep_for(200); 
    }

    // prevent robot from dying
    this_thread::sleep_for(20);
  }
}

// pneumatics control code
thread pneumaticsControlCallback() {

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

// intake control code
thread intakeToggle() {

  // initial state of intake
  bool intakeSpin = false;
  while (true) {

    // if A is pressed toggle spinning status
    if (Controller1.ButtonA.pressing()) {
      intakeSpin = !intakeSpin;
      if (intakeSpin) {
        intake.spin(fwd, 12, voltageUnits::volt);
      }
      else {
        catapult.stop();
      }
      
      // cooldown after toggle
      this_thread::sleep_for(200); 
    }

    // prevent robot from dying
    this_thread::sleep_for(20);
  }
}
