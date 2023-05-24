#include "main.h"
#include "PIDController.h"

// creation of button and labels within LVGL library
lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;
// lv_obj_t * sliderLabel;
// lv_obj_t * mySlider;

// separate styles of button when idle and pressed
lv_style_t myButtonStyleREL; // idle state
lv_style_t myButtonStylePR; // pressed state
// lv_style_t myBarStyle;

// left side motor group
pros::Motor leftMotorBack(3);
pros::Motor leftMotorFront(8);
pros::Motor_Group leftDriveSmart({leftMotorBack,leftMotorFront});
	
// right side motor group
pros::Motor rightMotorBack(4);
pros::Motor rightMotorFront(7);
pros::Motor_Group rightDriveSmart({rightMotorFront, rightMotorBack});

// sets
static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
		sprintf(buffer, "GET TROLLLLLLED");
		lv_label_set_text(myLabel, buffer);
		while(1) {
			leftDriveSmart.move_voltage(12000);
			rightDriveSmart.move_voltage(12000);
		}
    }

    return LV_RES_OK;
}



////////////////////////////
// NOT WORKING SLIDER BAR //
////////////////////////////

/* extern "C" {
	static lv_obj_t * label;

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    /*Refresh the text
    lv_label_set_text_fmt(label, "%"LV_PRId32, lv_slider_get_value(slider));
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider
}

/*
 * Create a slider and write its value on a label.
 
void lv_example_get_started_4(void)
{
    /*Create a slider in the center of the display
    lv_obj_t * slider = lv_slider_create(lv_scr_act());
    lv_obj_set_width(slider, 200);                          /*Set the width
    lv_obj_center(slider);                                  /*Align to the center of the parent (screen)
    lv_obj_add_event(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);     /*Assign an event function

    /*Create a label above the slider
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "0");
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider
}
*/
///////////////////////////////////
// END OF NOT WORKING SLIDER BAR //
///////////////////////////////////




// Runs initialization code. This occurs as soon as the program is started.
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

    myButton = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is default screen object
    lv_obj_set_free_num(myButton, 0); //set button is to 0
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action); // sets function to be called on button click
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); // sets the released style
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); // sets the pressed style
    lv_obj_set_size(myButton, 200, 50); // sets the button size
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); // sets the position to top left

    myButtonLabel = lv_label_create(myButton, NULL); // create label and puts it inside of the button
    lv_label_set_text(myButtonLabel, "FREE MONEY!!!!"); // sets label text

    myLabel = lv_label_create(lv_scr_act(), NULL); // create label and puts it on the screen
    lv_label_set_text(myLabel, "CLICK BUTTON FOR FREE MOOLA"); // sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_IN_LEFT_MID, 10, -40); // sets the position to center
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




// Runs the user autonomous code when the Competition Field Switch is enabled to autonomous.
void autonomous() {}



// Driver Control function
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