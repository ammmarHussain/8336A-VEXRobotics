#include "main.h"
#include "PIDController.h"

lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;
lv_obj_t * mySlider;

lv_style_t myButtonStyleREL; // idle state
lv_style_t myButtonStylePR; // pressed state
lv_style_t myBarStyle;


static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
		sprintf(buffer, "Button was clicked %ims from start of program", pros::millis());
		lv_label_set_text(myLabel, buffer);
    }

    return LV_RES_OK;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	

	lv_style_copy(&myButtonStyleREL, &lv_style_plain);
    myButtonStyleREL.body.main_color = LV_COLOR_MAKE(150, 0, 0);
    myButtonStyleREL.body.grad_color = LV_COLOR_MAKE(0, 0, 150);
    myButtonStyleREL.body.radius = 0;
    myButtonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&myButtonStylePR, &lv_style_plain);
    myButtonStylePR.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    myButtonStylePR.body.grad_color = LV_COLOR_MAKE(0, 0, 255);
    myButtonStylePR.body.radius = 0;
    myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    myButton = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(myButton, 0); //set button is to 0
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); //set the relesed style
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); //set the pressed style
    lv_obj_set_size(myButton, 200, 50); //set the button size
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); //set the position to top mid

    myButtonLabel = lv_label_create(myButton, NULL); //create label and puts it inside of the button
    lv_label_set_text(myButtonLabel, "Click the Button"); //sets label text

    myLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_label_set_text(myLabel, "Button has not been clicked yet"); //sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_IN_LEFT_MID, 10, -40); //set the position to center
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */




// driving control function
void opcontrol() {

	// left side motor group
	pros::Motor leftMotorBack(3);
	pros::Motor leftMotorFront(8);
	pros::Motor_Group leftDriveSmart({leftMotorBack,leftMotorFront});
	
	// right side motor group
	pros::Motor rightMotorBack(4);
	pros::Motor rightMotorFront(7);
	pros::Motor_Group rightDriveSmart({rightMotorFront, rightMotorBack});

	// controller initialization 
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// parse input from controller - arcade driving method
	float acceleration;
	float timeElapsed = 0;
	while (true) {

		// grab input from controller
		float rightInputX = master.get_analog(ANALOG_RIGHT_X);
		float leftInputY = master.get_analog(ANALOG_LEFT_Y);

		// detect if leftInputY is forward
		if (leftInputY > 100) {
			timeElapsed = timeElapsed + 0.55;
		} else if (rightInputX < -100) {
			timeElapsed = -(timeElapsed + 0.55);
		} else {
			timeElapsed = 0;
		}

		// detect if rightInputY is backwards
		

		// adjust acceleration
		acceleration = -pow((timeElapsed-5),2)+25;

		// control wheels
		float left = acceleration + rightInputX * 40;
		float right = acceleration - rightInputX * 40;
		leftDriveSmart.move_voltage(left);
		rightDriveSmart.move_voltage(-right);

		// set voltage to 0 when -5 < joystick < 5
		if ((master.get_analog(ANALOG_LEFT_Y)) > -5 && (master.get_analog(ANALOG_LEFT_Y)) < 5) {
			leftDriveSmart.move_velocity(0);
			rightDriveSmart.move_velocity(0);
		}


		// prevent system from overworking
		pros::delay(2);
	}
}