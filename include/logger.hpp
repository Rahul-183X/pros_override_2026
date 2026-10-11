/**
 * \file logger.hpp
 *
 * Contains definitions for the logger module.
 *
 */

#pragma once

#include "main.h"
#include <string>

namespace logger{
    #define END_OF_STRING '\0'
    
    extern bool log_file_created;
    extern char file_name[];
    void load_recent_date();
    void save_date_to_sd();
    void edit_date_screen();
    
    void calibrate_sensors();

    // Logs the robot's position to the controller screen in a separate task.
    void log_to_controller();
    // Logs the inertial sensor data to the controller screen in a separate task
    void log_inertial_sensors();

    class Data_logger_to_sd {
    private:
        std::string file_name = "log.csv";
    public:
        Data_logger_to_sd(std::string file_name);
        ~Data_logger_to_sd();
        void save_inertial_sensors_to_sd(pros::gps_accel_s_t gps_accel, pros::imu_accel_s_t imu_accel);
    };
}