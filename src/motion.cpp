#include "motion.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "robot.hpp"

namespace motion {

	// drivetrain settings
	lemlib::Drivetrain drivetrain(&robot::left_motors, // left motor group
									&robot::right_motors, // right motor group
									11, // 10 inch track width
									lemlib::Omniwheel::NEW_325, // using new 4" omnis
									200, // drivetrain rpm is 360
									2 // horizontal drift is 2 (for now)
		);

	// horizontal tracking wheel encoder
	//pros::Rotation horizontal_encoder(2);
	// vertical tracking wheel encoder
	// THE NEGATIVE SIGN because the roation sensor is mounted in the opposite direction
	pros::Rotation vertical_encoder(-2);
	//pros::adi::Encoder vertical_encoder('C', 'D', true);
	// horizontal tracking wheel
	//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
	// vertical tracking wheel
	lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, -5.75);
		// create an imu on port 10
		//pros::Imu imu(9);
	lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
								nullptr, // vertical tracking wheel 2
								nullptr, // horizontal tracking wheel 1
								nullptr, // horizontal tracking wheel 2
								&robot::drivetrain_imu // inertial sensor
	);
	// lateral PID controller
	lemlib::ControllerSettings lateral_controller(1, // proportional gain (kP)
												0, // integral gain (kI)
												0, // derivative gain (kD)
												0, // anti windup
												0, // small error range, in inches
												0, // small error range timeout, in milliseconds
												0, // large error range, in inches
												0, // large error range timeout, in milliseconds
												0 // maximum acceleration (slew)
	);

	// angular PID controller
	lemlib::ControllerSettings angular_controller(1, // proportional gain (kP)
												0, // integral gain (kI)
												2, // derivative gain (kD)
												0, // anti windup
												0, // small error range, in degrees
												0, // small error range timeout, in milliseconds
												0, // large error range, in degrees
												0, // large error range timeout, in milliseconds
												0 // maximum acceleration (slew)
	);
	// input curve for throttle input during driver control
	lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
										10, // minimum output where drivetrain will move out of 127
										1.019 // expo curve gain
	);

	// input curve for steer input during driver control
	lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
									10, // minimum output where drivetrain will move out of 127
									1.019 // expo curve gain
	);

	// create the chassis
	lemlib::Chassis chassis(drivetrain,
							lateral_controller,
							angular_controller,
							sensors,
							&throttle_curve,
							&steer_curve
	);

	/**
 * Re-localization helper for LemLib v0.5.6
 */
void sync_gps_to_lemlib() {
    // VEX GPS outputs in meters. LemLib tracks in inches (1 meter = 39.3701 inches)
    double gps_x_inches = robot::gps.get_position_x() * 39.3701;
    double gps_y_inches = robot::gps.get_position_y() * 39.3701;
    
    // Get absolute heading in degrees from the GPS
    double gps_heading = robot::gps.get_heading();

    // Instantiate LemLib's specific Pose object
    // Structure: lemlib::Pose(float x, float y, float theta)
    lemlib::Pose current_gps_pose(gps_x_inches, gps_y_inches, gps_heading);

    // Feed it directly into the chassis
    // The second parameter defaults to false (meaning theta is in DEGREES)
    motion::chassis.setPose(current_gps_pose, false); 
}

}// namespace motion
// HIHI is a company which makes a variety of products, including the HIHI 3D printer. The company was founded in 2015 and is based in Shenzhen, China.