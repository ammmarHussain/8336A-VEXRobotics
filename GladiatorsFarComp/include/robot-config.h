/* include everything needed
#include "vex.h"
#include <cmath>
#include <iostream>

// namespace setup
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// global instance of the brain
brain Brain;

// create controller
controller Controller1 = controller(primary);

// create motors
motor leftFrontMotor = motor(PORT3, ratio18_1, true);
motor leftBackMotor = motor(PORT12, ratio18_1, true);
motor rightBackMotor = motor(PORT16, ratio18_1, false);
motor rightFrontMotor = motor(PORT17, ratio18_1, false);
motor catapultMotor = motor(PORT9, ratio36_1, false);
motor cataSecondMotor = motor(PORT13, ratio36_1, true);
motor intake = motor (PORT18, ratio36_1, true);

// create motor groups
motor_group LeftDriveSmart = motor_group(leftBackMotor,leftFrontMotor);
motor_group RightDriveSmart = motor_group(rightBackMotor, rightFrontMotor);
motor_group catapult = motor_group(catapultMotor, cataSecondMotor);

// misc definitions

inertial DrivetrainInertial = inertial(PORT19);
digital_out pneuCylinLeft = digital_out(Brain.ThreeWirePort.A);
digital_out pneuCylinRight = digital_out(Brain.ThreeWirePort.B);

// define drivetrain
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 320, 279.4, 279.4, mm, 1.67);
*/



using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial DrivetrainInertial;
extern controller Controller1;
extern motor leftFrontMotor;
extern motor leftMidMotor;
extern motor rightFrontMotor;
extern motor leftBackMotor;
extern motor rightBackMotor;
extern motor intake;
extern motor_group catapult;
extern digital_out pneuCylinLeft;
extern digital_out pneuCylinRight;
extern digital_out rHang;
extern digital_out lHang;




/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );