#include "main.h"
#include "logger.hpp"
#include "robot.hpp"
#include "motion.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

namespace Wall_e{



void lift_weight() {
	robot::arm_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	robot::arm_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	robot::controller.set_text(0, 0, "Arm logging");
	robot::controller.clear_line(1);
	robot::controller.clear_line(2);

	while (true) {
		if (robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			robot::arm_left.move_velocity(50);
			robot::arm_right.move_velocity(-50);
		} else if (robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			robot::arm_left.move_velocity(-50);
			robot::arm_right.move_velocity(50);
		} else {
			robot::arm_left.move_velocity(0);
			robot::arm_right.move_velocity(0);
		}

		FILE* file = std::fopen(logger::file_name, "a");
		if (file != nullptr) {
			std::fprintf(file, "%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
			             pros::millis() / 1000.0, robot::arm_left.get_position(), robot::arm_left.get_actual_velocity(),
			             robot::arm_left.get_torque(), robot::arm_left.get_power(), robot::arm_left.get_current_draw() / 1000.0,
			             robot::arm_right.get_position(), robot::arm_right.get_actual_velocity(), robot::arm_right.get_torque(),
			             robot::arm_right.get_power(), robot::arm_right.get_current_draw() / 1000.0);
			std::fclose(file);
		}
		pros::delay(100);
	}
}



} // namespace Wall_e


// void opcontrol() {
//     // loop forever
//     while (true) {
//         // get left y and right x positions
//         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);



//         // delay to save resources
//         pros::delay(25);
//     }
// }



void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
		int leftY = robot::controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = robot::controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        motion::chassis.curvature(leftY, rightX);


		//move the robot using arcade drive
		motion::chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	TRACE("Entering initialize\n");
	// initialize the controller
	robot::controller.clear();

	logger::load_recent_date();
	logger::edit_date_screen();
	if (!logger::log_file_created) logger::save_date_to_sd();
	TRACE("Start callibration\n");
	robot::calibrate_sensors();
	pros::lcd::initialize(); // initialize brain screen
    //calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", motion::chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", motion::chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", motion::chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
	});
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}
/* disabled() is implemented above. */

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}
/* competition_initialize() is implemented above. */

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
void autonomous() {
    // set position to x:0, y:0, heading:0
    motion::chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    motion::chassis.turnToHeading(90, 100000);
	motion::chassis.moveToPoint(0, 48, 10000);

}
/* autonomous() is implemented above. */

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