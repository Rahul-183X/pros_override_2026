
#include "main.h"
#include "robot.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
namespace tester {

    // TODO: take robot namespace into account for motor IDs

pros::MotorGroup drive_motors({1, -11, 10, -20});

// Ramp configuration
constexpr std::int32_t START_VOLTAGE_MV = 500;
constexpr std::int32_t MAX_TEST_VOLTAGE_MV = 6000;
constexpr std::int32_t VOLTAGE_STEP_MV = 250;

constexpr std::uint32_t SAMPLE_INTERVAL_MS = 40;
constexpr int SAMPLES_PER_VOLTAGE = 5;  // 200 ms at each voltage

// Motion must persist for several samples to count as breakaway.
constexpr double MOVING_THRESHOLD_RPM = 5.0;
constexpr int REQUIRED_MOVING_SAMPLES = 3;

// Stop the test if a motor reaches this temperature.
constexpr double MAX_TEST_TEMPERATURE_C = 55.0;

struct DriveMeasurements {
    double minimum_abs_rpm;
    double total_current_a;
    double total_torque_nm;
    double maximum_temperature_c;
    bool valid;
};

DriveMeasurements read_drive_measurements() {
    const std::vector<double> velocities =
        drive_motors.get_actual_velocity_all();

    const std::vector<std::int32_t> currents_ma =
        drive_motors.get_current_draw_all();

    const std::vector<double> torques_nm =
        drive_motors.get_torque_all();

    const std::vector<double> temperatures_c =
        drive_motors.get_temperature_all();

    DriveMeasurements result{
        .minimum_abs_rpm = 0.0,
        .total_current_a = 0.0,
        .total_torque_nm = 0.0,
        .maximum_temperature_c = 0.0,
        .valid = false
    };

    const std::size_t motor_count = velocities.size();

    if (
        motor_count != 4 ||
        currents_ma.size() != motor_count ||
        torques_nm.size() != motor_count ||
        temperatures_c.size() != motor_count
    ) {
        return result;
    }

    result.minimum_abs_rpm = std::abs(velocities[0]);

    for (std::size_t i = 0; i < motor_count; ++i) {
        result.minimum_abs_rpm = std::min(
            result.minimum_abs_rpm,
            std::abs(velocities[i])
        );

        // PROS reports motor current in milliamps.
        result.total_current_a +=
            std::abs(currents_ma[i]) / 1000.0;

        result.total_torque_nm +=
            std::abs(torques_nm[i]);

        result.maximum_temperature_c = std::max(
            result.maximum_temperature_c,
            temperatures_c[i]
        );
    }

    result.valid = true;
    return result;
}

void stop_drive_test() {
    // Zero voltage stops applying drive voltage.
    drive_motors.move_voltage(0);
}

void run_breakaway_test() {
    int consecutive_moving_samples = 0;

    double last_stationary_torque_nm = 0.0;
    double last_stationary_current_a = 0.0;
    std::int32_t last_stationary_voltage_mv = 0;

    std::printf(
        "time_ms,command_mv,min_motor_rpm,total_current_a,"
        "total_torque_nm,max_temp_c,moving\n"
    );

    for (
        std::int32_t command_mv = START_VOLTAGE_MV;
        command_mv <= MAX_TEST_VOLTAGE_MV;
        command_mv += VOLTAGE_STEP_MV
    ) {
        // Apply the same slowly increasing voltage to all four motors.
        drive_motors.move_voltage(command_mv);

        for (
            int sample = 0;
            sample < SAMPLES_PER_VOLTAGE;
            ++sample
        ) {
            pros::delay(SAMPLE_INTERVAL_MS);

            const DriveMeasurements m =
                read_drive_measurements();

            if (!m.valid) {
                stop_drive_test();
                std::printf("ABORT: invalid motor telemetry\n");
                return;
            }

            // minimum_abs_rpm means every motor must exceed the threshold.
            const bool all_motors_moving =
                m.minimum_abs_rpm >= MOVING_THRESHOLD_RPM;

            std::printf(
                "%lu,%ld,%.2f,%.3f,%.3f,%.1f,%d\n",
                static_cast<unsigned long>(pros::millis()),
                static_cast<long>(command_mv),
                m.minimum_abs_rpm,
                m.total_current_a,
                m.total_torque_nm,
                m.maximum_temperature_c,
                all_motors_moving ? 1 : 0
            );

            if (
                m.maximum_temperature_c >=
                MAX_TEST_TEMPERATURE_C
            ) {
                stop_drive_test();

                std::printf(
                    "ABORT: temperature reached %.1f C\n",
                    m.maximum_temperature_c
                );
                return;
            }

            if (all_motors_moving) {
                ++consecutive_moving_samples;
            } else {
                consecutive_moving_samples = 0;

                // Save the last confirmed stationary measurement.
                last_stationary_voltage_mv = command_mv;
                last_stationary_current_a = m.total_current_a;
                last_stationary_torque_nm = m.total_torque_nm;
            }

            if (
                consecutive_moving_samples >=
                REQUIRED_MOVING_SAMPLES
            ) {
                stop_drive_test();

                std::printf("\nBREAKAWAY DETECTED\n");

                std::printf(
                    "Last stationary command: %ld mV\n",
                    static_cast<long>(
                        last_stationary_voltage_mv
                    )
                );

                std::printf(
                    "Last stationary current: %.3f A total\n",
                    last_stationary_current_a
                );

                std::printf(
                    "Last stationary torque: %.3f N*m total\n",
                    last_stationary_torque_nm
                );

                std::printf(
                    "First sustained-moving command: %ld mV\n",
                    static_cast<long>(command_mv)
                );

                std::printf(
                    "First moving torque: %.3f N*m total\n",
                    m.total_torque_nm
                );

                return;
            }
        }
    }

    stop_drive_test();

    std::printf(
        "No breakaway detected at or below %ld mV\n",
        static_cast<long>(MAX_TEST_VOLTAGE_MV)
    );
}

void test_opcontrol() {
    pros::Controller controller(
        pros::E_CONTROLLER_MASTER
    );

    stop_drive_test();

    while (true) {
        if (
            controller.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_A
            )
        ) {
            run_breakaway_test();
        }

        pros::delay(20);
    }
}

} // namespace tester