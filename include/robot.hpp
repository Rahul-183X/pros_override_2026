#pragma once

#include "main.h"
#include <atomic>

namespace robot {

// Set only when the controller's X button requests an application halt.
extern std::atomic<bool> stop_requested;

extern pros::Controller controller;
extern pros::Imu drivetrain_imu;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Motor arm_left;
extern pros::Motor arm_right;
extern pros::Motor wrist;
extern pros::Motor claw;
extern pros::MotorGroup arm;


extern pros::GPS gps;

void calibrate_sensors();
} // namespace robot