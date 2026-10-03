#include "robot.hpp"
#include "motion.hpp"
#include "pros/misc.h"

#define VISION_PORT 6 
#define BLU_CONFIG 1
#define RED_CONFIG 2
#define YEL_CONFIG 3

pros::AIVision AI_vision_sensor(VISION_PORT);

void initialize() {
    AI_vision_sensor.reset();


    AI_vision_sensor.enable_detection_types(pros::AivisionModeType::colors);
}



void configure_pin_colors() {
    pros::AIVision::Color blue_pin = {
         .id = BLU_CONFIG,  
        .red = 0, .green = 110, .blue = 177, 
        .hue_range = 20, .saturation_range = 0.4
    };

    pros::AIVision::Color red_pin = {
        .id = RED_CONFIG,  
        .red = 201, .green = 0, .blue = 20, 
        .hue_range = 20, .saturation_range = 0.4
    };
    
     pros::AIVision::Color yellow_pin = {
        .id = YEL_CONFIG,  
        .red = 208, .green = 178, .blue = 0, 
        .hue_range = 20, .saturation_range = 0.4
    };
}

void opcontrol() {
    while (true) {
        pros::AIVision::Object detected_object = AI_vision_sensor.get_object(0);
        if (pros::AIVision::is_type(detected_object, pros::AivisionDetectType::color)) {
            uint8_t color_id = detected_object.id;
            if (color_id == BLU_CONFIG) {
                printf("Detected blue pin\n");
            } else if (color_id == RED_CONFIG) {
                printf("Detected red pin\n");
            } else if (color_id == YEL_CONFIG) {
                printf("Detected yellow pin\n");
            } else {
                printf("Detected unknown color\n");
            }
        } else {
            printf("No color detected\n");
        }
        pros::delay(100);
    }
}