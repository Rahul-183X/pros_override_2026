#include "main.h"
//#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */














namespace {
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu inertial(17);
pros::Motor arm_left(5, pros::MotorGears::red);
pros::Motor arm_right(15, pros::MotorGears::red);

int date_values[5] = {26, 8, 16, 5, 30};
int cursor_column = 0;
int cursor_row = 0;
int displayed_cursor_column = -1;
bool log_file_created = false;
char file_name[32] = "default.csv";

const char* date_text() {
	static char text[32];
	std::snprintf(text, sizeof(text), "%02d/%02d/%02d-%02d:%02d", date_values[0],
	              date_values[1], date_values[2], date_values[3], date_values[4]);
	return text;
}

void draw_cursor(bool visible) {
	// printf("[TRACE] Entering draw_cursor visible=%d\n", visible);  

	int old_cursor_position = 0;
	const int field_positions[5] = {0, 3, 6, 9, 11};
	char cursor_text[15] = {};
	const int field = cursor_column / 2;
	char field_value[3];
	std::snprintf(field_value, sizeof(field_value), "%02d", date_values[field]);
	// Clear the cursor text and set the current field to be highlighted
	for (int index = 0; index < 14; ++index) cursor_text[index] = ' ';
	cursor_text[field_positions[field]] = '[';
	cursor_text[field_positions[field] + 1] = field_value[0];
	cursor_text[field_positions[field] + 2] = field_value[1];
	cursor_text[field_positions[field] + 3] = ']';
	controller.set_text(1, 0, " ");

	if (displayed_cursor_column < 0) {
		controller.clear_line(1);
	} else {
//		controller.set_text(1, 0, "              ");		
		controller.set_text(0, 0, "_");
	}
// 	// Update the cursor position based on the controller input
// 	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
// 		old_cursor_position = old_cursor_position  + 1;
// 	}
// 	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
// 		old_cursor_position = old_cursor_position  - 1;
// 	}
// 	if (displayed_cursor_column < 1) {
// 		controller.clear_line(2);
// 	} else {
// //		/*controller.set_text(1, 0, "             ");	Clear the previous cursor line */
// 		controller.set_text(2, old_cursor_position, "_"); /* Sets the curor position to blink */
// 		pros::delay(50); /* Waits the cusror blink timer */
// 		/*controller.clear_line(0);*/
// 	}

	if (visible) {
		/*TRACE("It came to the visible part of the draw_cursor function\n");*/
		controller.set_text(1, 0, cursor_text);
	} else {
		//controller.set_text(1, 0, "             ");
		controller.set_text(0, cursor_column, "_");
	}

	displayed_cursor_column = cursor_column;

	// printf("Cursor: %s,displayed_cursor_column: %d visible: %d \n", cursor_text, displayed_cursor_column, visible);
}

void show_date() {
	// TRACE("Entering show_date\n");
	int32_t error_code = 0;
	error_code = controller.set_text(0, 0, date_text());
	if (error_code != 1) {
		printf("Error setting column %d,date error: %d\n",displayed_cursor_column, error_code);
	}
	pros::delay(50);
	// error_code = controller.set_text(0, cursor_column, "_");
	// 	if (error_code != 1) {
	// 	printf("Error vaazhli column %d,date error: %d\n",displayed_cursor_column, error_code);
	// }
	// TRACE("Date text: %s\n");
	//draw_cursor(true);
	// pros::delay(10);

	// error_code = controller.set_text(0, 0, date_text());
	// if (error_code != 1) {
	// 	printf("Error setting column %d,date error: %d\n",displayed_cursor_column, error_code);
	// }
	// TRACE("cursor drawn\n");
	// error_code = controller.set_text(2, 0, "UP/DN  L/R  A");
	// 	if (error_code != 1) {
	// 	printf("Error arrow column %d,date error: %d\n",displayed_cursor_column, error_code);
	// }
	TRACE("Exiting show_date\n");
}

void load_recent_date() {
	TRACE("Entering load_recent_date\n");

	if (!pros::usd::is_installed()) {
		controller.set_text(2, 0, "Insert SD card");
		return;
	}

	FILE* file = std::fopen("recent_file.txt", "r");
	if (file == nullptr) return;
	char content[32] = {};
	if (std::fgets(content, sizeof(content), file) != nullptr) {
		int loaded[5];
		if (std::sscanf(content, "%d-%d-%d-%d-%d", &loaded[0], &loaded[1], &loaded[2],
		                &loaded[3], &loaded[4]) == 5) {
			for (int index = 0; index < 5; ++index) date_values[index] = loaded[index];
			date_values[4] = (date_values[4] + 1) % 60;
		}
	}
	std::fclose(file);
}

void save_date_to_sd() {
	TRACE("Entering save_date_to_sd\n");

	if (!pros::usd::is_installed()) {
		controller.set_text(2, 0, "Error: no SD card");
		return;
	}

	std::snprintf(file_name, sizeof(file_name), "%02d-%02d-%02d-%02d-%02d.csv",
	              date_values[0], date_values[1], date_values[2], date_values[3],
	              date_values[4]);
	FILE* log_file = std::fopen(file_name, "w");
	if (log_file == nullptr) {
		controller.set_text(2, 0, "File create failed");
		return;
	}
	std::fprintf(log_file, "timestamp,arm_position_deg,arm_velocity_rpm,arm_torque_nm,arm_power_w,arm_current_a,arm_right_position_deg,arm_right_velocity_rpm,arm_right_torque_nm,arm_right_power_w,arm_right_current_a\n");
	std::fclose(log_file);

	FILE* recent_file = std::fopen("recent_file.txt", "w");
	if (recent_file != nullptr) {
		std::fprintf(recent_file, "%02d-%02d-%02d-%02d-%02d", date_values[0], date_values[1],
		             date_values[2], date_values[3], date_values[4]);
		std::fclose(recent_file);
	}
	log_file_created = true;
	controller.set_text(2, 0, "Saved date log");
}

void edit_date_screen() {
	TRACE("Entering edit_date_screen\n");

	const std::uint32_t start = pros::millis();
	bool previous_up = false;
	bool previous_down = false;
	bool previous_left = false;
	bool previous_right = false;
	bool previous_a = false;
	bool cursor_visible = true;
	std::uint32_t last_blink = pros::millis();
	show_date();

	while (pros::millis() - start < 300000 && !log_file_created) {
		//TRACE("Entering edit_date_screen loop\n");

		const std::uint32_t now = pros::millis();
		const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
		const bool left = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
		const bool right = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
		const bool button_a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		const int date_index = cursor_column / 2;

		if (now - last_blink >= 200) {
			cursor_visible = !cursor_visible;
			draw_cursor(cursor_visible);
			last_blink = now;
		}

		if (up && !previous_up) {
			const int limits[5] = {99, 12, 31, 23, 59};
			date_values[date_index] = date_values[date_index] % limits[date_index] + 1;
			cursor_visible = true;
			show_date();
			last_blink = now;
		}
		if (down && !previous_down) {
			const int limits[5] = {99, 12, 31, 23, 59};
			date_values[date_index] = (date_values[date_index] + limits[date_index] - 2) % limits[date_index] + 1;
			cursor_visible = true;
			show_date();
			last_blink = now;
		}
		// WHen changing from python to C++, the max coulumn position was changed from 1 - 12 to 1 - 9
		// Because the date has 12 fields 
		if (left && !previous_left) cursor_column = cursor_column == 0 ? 12 : cursor_column - 1;
		if (right && !previous_right) cursor_column = cursor_column == 12 ? 0 : cursor_column + 1;

		if ((left && !previous_left) || (right && !previous_right)) {
			cursor_visible = true;
			show_date();
			last_blink = now;
		}
		if (button_a && !previous_a) save_date_to_sd();

		previous_up = up;
		previous_down = down;
		previous_left = left;
		previous_right = right;
		previous_a = button_a;
		pros::delay(50);
	}
	controller.clear_line(0);
}

void lift_weight() {
	arm_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	controller.set_text(0, 0, "Arm logging");
	controller.clear_line(1);
	controller.clear_line(2);

	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			arm_left.move_velocity(50);
			arm_right.move_velocity(-50);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			arm_left.move_velocity(-50);
			arm_right.move_velocity(50);
		} else {
			arm_left.move_velocity(0);
			arm_right.move_velocity(0);
		}

		FILE* file = std::fopen(file_name, "a");
		if (file != nullptr) {
			std::fprintf(file, "%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
			             pros::millis() / 1000.0, arm_left.get_position(), arm_left.get_actual_velocity(),
			             arm_left.get_torque(), arm_left.get_power(), arm_left.get_current_draw() / 1000.0,
			             arm_right.get_position(), arm_right.get_actual_velocity(), arm_right.get_torque(),
			             arm_right.get_power(), arm_right.get_current_draw() / 1000.0);
			std::fclose(file);
		}
		pros::delay(100);
	}
}

void calibrate_sensors() {
	TRACE("Entering calibrate_sensors\n");

	inertial.reset(false);
	while (inertial.is_calibrating()) pros::delay(100);
}




void joystick() {
pros::MotorGroup left_motors({-1, 10}, pros::MotorGearset::green); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({-11,20 }, pros::MotorGearset::green); // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              	&right_motors, // right motor group
                              	11, // 10 inch track width
                             	lemlib::Omniwheel::NEW_275, // using new 4" omnis
                             	360, // drivetrain rpm is 360
                             	2 // horizontal drift is 2 (for now)
	);
	// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);
	// create an imu on port 10
	//pros::Imu imu(9);
lemlib::OdomSensors sensors(&horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            //&imu // inertial sensor
);
	




}  // namespace

void initialize() {
	TRACE("Entering initialize\n");
	load_recent_date();
	edit_date_screen();
	if (!log_file_created) save_date_to_sd();
	TRACE("Start callibration\n");
	calibrate_sensors();
	
}

void disabled() {}
void competition_initialize() {


}
void autonomous() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
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
void opcontrol() {
	lift_weight();
}







}