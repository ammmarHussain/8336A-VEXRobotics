#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

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


motor_group catapult = motor_group(catapultMotor, cataSecondMotor);

// misc definitions

inertial DrivetrainInertial = inertial(PORT19);
digital_out pneuCylinLeft = digital_out(Brain.ThreeWirePort.A);
digital_out pneuCylinRight = digital_out(Brain.ThreeWirePort.B);

// define drivetrain


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}