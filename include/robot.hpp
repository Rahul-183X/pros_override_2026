#pragma once

#include "main.h"

namespace robot {

extern pros::Controller controller;
extern pros::Imu inertial;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Motor arm_left;
extern pros::Motor arm_right;
extern pros::Motor wrist_left;
extern pros::Motor claw;

} // namespace robot