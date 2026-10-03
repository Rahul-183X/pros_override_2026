/**
 * \file logger.hpp
 *
 * Contains definitions for the logger module.
 *
 */

#pragma once

#include "main.h"

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
}