/** Robot configuration file
 Everything the robot uses is configured here, including sensors and actuators.

*/

#include "robot.hpp"


namespace robot {

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::Imu inertial(9);

//Actuators

//chassis
pros::MotorGroup left_motors({1, 10}, pros::MotorGearset::green); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({-11,-20 }, pros::MotorGearset::green); // right motors use 200 RPM cartridges


// Arm motors
pros::Motor arm_left(8, pros::MotorGears::red);
pros::Motor arm_right(-18, pros::MotorGears::red);

// Wrist motors
pros::Motor wrist_left(5, pros::MotorGears::red);

// Claw motors
pros::Motor claw(4, pros::MotorGears::red); 

} // namespace robot