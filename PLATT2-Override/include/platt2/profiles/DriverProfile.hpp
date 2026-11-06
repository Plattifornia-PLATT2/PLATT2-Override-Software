#ifndef EDRIVERPROFILE_HPP
#define EDRIVERPROFILE_HPP

#include "platt2/EDriverConfig.hpp"
#include "pros/misc.h"

/**
 * @brief Namespace for all PLATT2 library code.
 * @author PLATT2 Development Team
 */
namespace platt2{

/**
 * @brief Namespace for controller profile code.
 * @author Dominic Young
 */
namespace profiles{

    struct DriverProfile{
        // Intake Mappings
        pros::controller_digital_e_t frontIntake_IN;
        pros::controller_digital_e_t frontIntake_OUT;

        // Field Oriented Drive
        pros::controller_digital_e_t fieldOriented_toggle;
        pros::controller_digital_e_t heading_reset;

        // Elevator Mappings
        pros::controller_digital_e_t elevator_up;
        pros::controller_digital_e_t elevator_down;
        pros::controller_digital_e_t elevator_full_down;

        // Claw Mappings
        pros::controller_digital_e_t claw_toggle;
        pros::controller_digital_e_t claw_flip;

        robot::DriverProfile driverEnum;
    };
}
}
#endif