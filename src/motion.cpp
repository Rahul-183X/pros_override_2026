
namespace motion {

	// drivetrain settings
	lemlib::Drivetrain drivetrain(&robot::left_motors, // left motor group
									&robot::right_motors, // right motor group
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
	lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1
								nullptr, // vertical tracking wheel 2
								&horizontal_tracking_wheel, // horizontal tracking wheel 1
								nullptr, // horizontal tracking wheel 2
								nullptr // inertial sensor
	);
	// lateral PID controller
	lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
												0, // integral gain (kI)
												3, // derivative gain (kD)
												3, // anti windup
												1, // small error range, in inches
												100, // small error range timeout, in milliseconds
												3, // large error range, in inches
												500, // large error range timeout, in milliseconds
												20 // maximum acceleration (slew)
	);

	// angular PID controller
	lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
												0, // integral gain (kI)
												10, // derivative gain (kD)
												3, // anti windup
												1, // small error range, in degrees
												100, // small error range timeout, in milliseconds
												3, // large error range, in degrees
												500, // large error range timeout, in milliseconds
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

}// namespace motion
