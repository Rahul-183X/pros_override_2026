/** Robot configuration file
 Everything the robot uses is configured here, including sensors and actuators.

*/

#include "robot.hpp"
#include "pros/gps.hpp"
#include "pros/motor_group.hpp"


namespace robot {

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::Imu inertial(9);
pros::Rotation rotation_sensor(2); // example rotation sensor on port 2
pros::GPS gps(3); // example GPS on port 3

//GPS sensor
double xInitial = 0.0;
double yInitial = 0.0;
double headingInitial = 0.0;
double xOffset = 0.0;
double yOffset = 0.0;

//Actuators

//chassis
//pros::MotorGroup left_motors({1, 10}, pros::MotorGearset::green); // left motors use 600 RPM cartridges
//pros::MotorGroup right_motors({-11,-20 }, pros::MotorGearset::green); // right motors use 200 RPM cartridges

pros::MotorGroup left_motors({20, 10}, pros::MotorGearset::green); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({11,1 }, pros::MotorGearset::green); // right motors use 200 RPM cartridges


// Arm motors
pros::Motor arm_left(-8, pros::MotorGears::red);
pros::Motor arm_right(18, pros::MotorGears::red);
pros::MotorGroup arm({-8, 18}, pros::MotorGears::red);

// Wrist motors
pros::Motor wrist_left(5, pros::MotorGears::red);

// Claw motors
pros::Motor claw(4, pros::MotorGears::red); 


/**
 * Calibrates the robot's sensors.
 * Currently, it only calibrates the inertial sensor.
 */

void calibrate_sensors() {
	TRACE("Entering calibrate_sensors\n");

	inertial.reset(false);
	while (inertial.is_calibrating()) pros::delay(100);

    rotation_sensor.reset();
    //while (rotation_sensor.is_calibrating()) pros::delay(100);  

    gps.initialize_full( xInitial,  yInitial,  headingInitial,  xOffset,  yOffset);
    //while (gps.is_calibrating()) pros::delay(100);  

}
} // namespace robot