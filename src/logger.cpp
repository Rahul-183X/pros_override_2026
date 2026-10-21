#include "logger.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "motion.hpp"
#include <cstdio>
#include <string>

namespace logger{

namespace {
std::string format_2dp(double value) {
	char buffer[32];
	std::snprintf(buffer, sizeof(buffer), "%.2f", value);
	return buffer;
}
} // namespace

int date_values[5] = {26, 8, 16, 5, 30};
int cursor_column = 0;
int cursor_row = 0;
int displayed_cursor_column = -1;
bool log_file_created = false;
char file_name[32] = "default.csv";


/**
 * Formats the current date and time values for controller display.
 *
 * @return A pointer to a static buffer containing the date in
 *         MM/DD/YY-HH:MM format.
 */
const char* date_text() {
	static char text[32];
	std::snprintf(text, sizeof(text), "%02d/%02d/%02d-%02d:%02d", date_values[0],
	              date_values[1], date_values[2], date_values[3], date_values[4]);
	return text;
}


/**
 * Draws or hides the cursor highlight for the currently selected date field.
 *
 * @param visible Whether the selected field should be displayed.
 */
void draw_cursor(bool visible) {
	printf("[TRACE] @ %d Entering draw_cursor visible=%d\n", pros::millis(), visible);  

	const int field_positions[5] = {0, 3, 6, 9, 12};
	char cursor_text[16] = {};
	const int field = cursor_column;
	char field_value[3];
	
	//initialize the cursor text with the current date
	std::snprintf(cursor_text, sizeof(cursor_text) - 1, "%s", date_text());
	//without explicitly null-terminating crashes can occur, snprintf ensures null-termination
	cursor_text[sizeof(cursor_text) - 1] = END_OF_STRING;

	std::snprintf(field_value, sizeof(field_value), "%02d", date_values[field]);


	//updating the cursor text with the current field value based on visibility
	cursor_text[field_positions[field]] = visible ? field_value[0] : '_';
	cursor_text[field_positions[field] + 1] = visible ? field_value[1] : '_';

	TRACE("Drawing cursor state %d at column %d\n", visible, cursor_column);
	if (robot::controller.is_connected() == false) {
		printf("Controller is not connected\n");
		return;
	}

	// Send one complete line per blink state; controller text updates are rate-limited.
	const char* display_text = cursor_text;
	const int ret_val = robot::controller.set_text(0, 0, display_text);
	if (ret_val != 1) {
		printf("Error drawing cursor at column %d, ret_val: %d, errno: %d\n",
		       cursor_column, ret_val, errno);
	}

	displayed_cursor_column = cursor_column;
}


/**
 * Displays the current date on the controller.
 */
void show_date() {
	// TRACE("Entering show_date\n");
	if (!robot::controller.is_connected()) {
		printf("Controller is not connected; cannot display date\n");
		return;
	}
	int32_t error_code = 0;
	error_code = robot::controller.set_text(0, 0, date_text());
	if (error_code != 1) {
		printf("Error setting date, column %d, ret_val: %d, errno: %d\n",
		       displayed_cursor_column, error_code, errno);
	}

	
	draw_cursor(true);
	pros::delay(10);

	TRACE("Exiting show_date\n");
}

/**
 * Loads the most recent date from the SD card.
 */
void load_recent_date() {
	TRACE("Entering load_recent_date\n");

	if (!pros::usd::is_installed()) {
		robot::controller.set_text(2, 0, "Insert SD card");
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

/**
 * Saves the current date to the SD card.
 */
void save_date_to_sd() {
	TRACE("Entering save_date_to_sd\n");

	if (!pros::usd::is_installed()) {
		robot::controller.set_text(2, 0, "Error: no SD card");
		return;
	}

	std::snprintf(file_name, sizeof(file_name), "%02d-%02d-%02d-%02d-%02d.csv",
	              date_values[0], date_values[1], date_values[2], date_values[3],
	              date_values[4]);
	FILE* log_file = std::fopen(file_name, "w");
	if (log_file == nullptr) {
		robot::controller.set_text(2, 0, "File create failed");
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
	robot::controller.set_text(2, 0, "I am Thankful for...");
	//after updating the controller 50ms delay is required to ensure the text is displayed correctly
	pros::delay(50);

	robot::controller.set_text(0, 0, "Saved date log");
}

/**
 * Allows the user to edit the date on the controller screen.
 */
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

	while (pros::millis() - start < 300000 && !log_file_created &&
	       !robot::stop_requested.load()) {
		//TRACE("Entering edit_date_screen loop\n");

		const std::uint32_t now = pros::millis();
		const bool up = robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		const bool down = robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
		const bool left = robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
		const bool right = robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
		const bool button_a = robot::controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		const int date_index = cursor_column;

		//once in 200ms, toggle the cursor visibility
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
		if (left && !previous_left) cursor_column = cursor_column == 0 ? 4 : cursor_column - 1;
		if (right && !previous_right) cursor_column = cursor_column == 4 ? 0 : cursor_column + 1;

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
		pros::delay(100);
	}
	robot::controller.clear_line(0);
	robot::controller.clear_line(1);
	robot::controller.clear_line(2);
}

/** 
Definition of the Data_logger_to_sd class constructor, destructor, and save_inertial_sensors_to_sd method.
 */

	Data_logger_to_sd::Data_logger_to_sd(std::string file_name) {
			// Constructor logic here
			// Initialize the date and time for the logger
			this->file_name = file_name;
			// append the file name with the current date and then add ".csv" if it doesn't already have it
			//get the data information from the global namespace
			int date_snapshot[5];
			for (int index = 0; index < 5; ++index) {
				date_snapshot[index] = logger::date_values[index];
			}
			char date_stamp[32];
			std::snprintf(date_stamp, sizeof(date_stamp), "%02d-%02d-%02d-%02d-%02d",
			              date_snapshot[0], date_snapshot[1], date_snapshot[2],
			              date_snapshot[3], date_snapshot[4]);
			
			this->file_name += "_" + std::string(date_stamp);

			//append the file name with ".csv" if it doesn't already have it
			if (this->file_name.find(".csv") == std::string::npos) {
				this->file_name += ".csv";
				printf(">>>>>!! File name: %s\n", this->file_name.c_str());
			}
			

			FILE* log_file = std::fopen(this->file_name.c_str(), "w");
			
			if (log_file == nullptr) {
				robot::controller.set_text(0, 0, "File create failed");
				return;
			}
			else{
				robot::controller.set_text(0, 0, "File created successfully");
				printf("File created successfully: %s\n", this->file_name.c_str());
			}
			//std::fprintf(log_file, "timestamp,drivetrain_imu.x,drivetrain_imu.y,drivetrain_imu.z, gps_imu.x,gps_imu.y,gps_imu.z\n");
			std::fclose(log_file);

		}

	Data_logger_to_sd::~Data_logger_to_sd() {
			// Destructor logic here
			// Close the log file if it was opened
			FILE* log_file = std::fopen(this->file_name.c_str(), "a");
			if (log_file != nullptr) {
				std::fclose(log_file);
			}
	}
		
	void Data_logger_to_sd::save_inertial_sensors_to_sd(pros::gps_accel_s_t gps_accel, pros::imu_accel_s_t imu_accel){
			// Logic to save inertial sensor data to the log file
			FILE* log_file = std::fopen(this->file_name.c_str(), "a");
			if (log_file != nullptr) {
				//check if the file is empty, if it is empty, write the header
				fseek(log_file, 0, SEEK_END);
				if (ftell(log_file) == 0) {
					std::fprintf(log_file, "timestamp,drivetrain_imu.x,drivetrain_imu.y,drivetrain_imu.z, gps_imu.x,gps_imu.y,gps_imu.z\n");
				}
				// Example of saving data, replace with actual sensor data retrieval
				std::fprintf(log_file, "%u,%f,%f,%f,%f,%f,%f\n", static_cast<unsigned>(pros::millis()), imu_accel.x, imu_accel.y, imu_accel.z, gps_accel.x, gps_accel.y, gps_accel.z);
				std::fclose(log_file);
			}
			printf("GPS Acceleration: %f, %f, %f\n", gps_accel.x, gps_accel.y, gps_accel.z);
			printf("IMU Acceleration: %f, %f, %f\n", imu_accel.x, imu_accel.y, imu_accel.z);
		}
	
		#if 0
		void Data_logger_to_sd::save_claw_sensors_to_sd(pros::motor claw_gripper, pros::motor claw){
			// Logic to save inertial sensor data to the log file
			FILE* log_file = std::fopen(this->file_name.c_str(), "a");

			if (log_file != nullptr) {
				//check if the file is empty, if it is empty, write the header
				fseek(log_file, 0, SEEK_END);
				if (ftell(log_file) == 0) {
					std::fprintf(log_file, "timestamp,claw_gripper_position,claw_position,\n");
				}

				// Example of saving data, replace with actual sensor data retrieval
				std::fprintf(log_file, "%u,%f,%f,%f,%f,%f,%f\n", static_cast<unsigned>(pros::millis()), claw_gripper.get_position(), claw.get_position(), imu_accel.x, imu_accel.y, imu_accel.z, gps_accel.x);
				std::fclose(log_file);
			}
		}
		#endif

		void Data_logger_to_sd::save_arm_to_sd(){
			// Logic to save inertial sensor data to the log file
			FILE* log_file = std::fopen(this->file_name.c_str(), "a");

			if (log_file != nullptr) {
				//check if the file is empty, if it is empty, write the header
				fseek(log_file, 0, SEEK_END);
				if (ftell(log_file) == 0) {
					std::fprintf(log_file, "timestamp,arm_left_position,arm_left_velocity,arm_left_torque,arm_left_power,arm_left_current,arm_left_temperature,arm_right_position,arm_right_velocity,arm_right_torque,arm_right_power,arm_right_current,arm_right_temperature\n");
				}

				// Example of saving data, replace with actual sensor data retrieval
				std::fprintf(log_file, "%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", static_cast<unsigned>(pros::millis()), robot::arm_left.get_position(), robot::arm_left.get_actual_velocity(), robot::arm_left.get_torque(), robot::arm_left.get_power(), robot::arm_left.get_current_draw() / 1000.0, robot::arm_left.get_temperature(),\
					 robot::arm_right.get_position(), robot::arm_right.get_actual_velocity(), robot::arm_right.get_torque(), robot::arm_right.get_power(), robot::arm_right.get_current_draw() / 1000.0, robot::arm_right.get_temperature());

				std::fclose(log_file);
			}
		}



	//
	void log_to_controller() {
		//pros::delay(30000);
		robot::controller.clear();
		double initial = robot::gps.get_heading();
		while (!robot::stop_requested.load()) {

			// log data to the controller screen
			//robot::controller.set_text(0, 0, "X: " + std::to_string(motion::chassis.getPose().x)+ " Y: " + std::to_string(motion::chassis.getPose().y));
			pros::delay(100);
			//robot::controller.set_text(1, 0, "Y: " + std::to_string(motion::chassis.getPose().y));
			robot::controller.set_text(2, 0, "Theta: " + format_2dp(motion::chassis.getPose().theta));
			pros::delay(100);
			robot::controller.set_text(0, 0, " i:" + format_2dp(initial) + " c: " + format_2dp(robot::gps.get_heading()));
			//robot::controller.set_text(1, 0, "X: " + format_2dp(robot::gps.get_position().x) + " Y: " + format_2dp(robot::gps.get_position().y) + " Theta: " + format_2dp(robot::gps.get_heading()));
			//robot::controller.set_text(2, 0, " Theta: " + format_2dp(robot::gps.get_heading())+ " " + " ");
		}
	}








	void log_inertial_sensors() {
		// Implement the logic for inertial sensors here for both the inertial sensor of GPS sensor
		// and the inertial sensor of the robot itself

		// This code just prints the X, Y, and Z coordinates of the inertial sensors on the brain screen
		robot::controller.clear();
		int status = 0;
		bool to_sd_card = true; //*********** forcing here to save to SD card */

		//initialize the data logger class
		logger::Data_logger_to_sd log_drivetrain_to_sd("drive_train");

		while (!robot::stop_requested.load()) {
			// This section is for the GPS inertial sensor
			pros::gps_accel_s_t gps_accel = robot::gps.get_accel();
			pros::imu_accel_s_t imu_accel = robot::drivetrain_imu.get_accel();

			if (to_sd_card) {
				log_drivetrain_to_sd.save_inertial_sensors_to_sd(gps_accel, imu_accel);
			}
			else{
				// It gets the X, Y, and Z acceleration values from the GPS inertial sensor, and prints it on controller
				//gps_accel.x, gps_accel.y, gps_accel.z;
			
				// robot::controller.set_text(0, 0, "GPS Inertial X: " + format_2dp(robot::gps.get_accel().x));
				// robot::controller.set_text(1, 0, "GPS Inertial Y: " + format_2dp(robot::gps.get_accel().y));
				// robot::controller.set_text(2, 0, "GPS Inertial Z: " + format_2dp(robot::gps.get_accel().z));

				status = robot::controller.set_text(0, 0, "GPS X: " + format_2dp(gps_accel.x)+ " Y: " + format_2dp(gps_accel.y)+ " Z: " + format_2dp(gps_accel.z));
				if (status != 0) {
					// Handle the error if setting the text fails
					TRACE("Failed to set GPS inertial sensor text on controller\n");
				}

				// This section is for the robot's inertial sensor
				// It gets the X, Y, and Z acceleration values from the robot's inertial sensor, and prints it on controller
				
				pros::delay(200);
			
				status = robot::controller.set_text(1, 0, "IMU X: " + format_2dp(imu_accel.x)+ " Y: " + format_2dp(imu_accel.y)+ " Z: " + format_2dp(imu_accel.z));
				if (status != 0) {
					// Handle the error if setting the text fails
					TRACE("Failed to set IMU inertial sensor text on controller\n");
				}
			// End of combined GPS and IMU inertial sensor display
			}
			pros::delay(200);
			
		} // end of while(true) loop

	} // log_inertial_sensors

} //namespace logger
