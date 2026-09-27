#pragma once

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
//#include "lemlib/controller/controller.hpp"
#include "lemlib/api.hpp"
//#include "lemlib/tracking/tracking.hpp"

namespace motion {
    extern lemlib::Chassis chassis;
    extern lemlib::ControllerSettings lateral_controller;
    extern lemlib::ControllerSettings angular_controller;   
    extern lemlib::ExpoDriveCurve throttle_curve;
    extern lemlib::ExpoDriveCurve steer_curve;  


} // namespace motion