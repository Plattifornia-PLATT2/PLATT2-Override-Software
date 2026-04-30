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

        // ---Digital Mappings---

        // Intake Mappings
        pros::controller_digital_e_t frontIntake_IN;
        pros::controller_digital_e_t frontIntake_OUT;
        pros::controller_digital_e_t auto_unload_button;

        // Pnuematic Actuations
        pros::controller_digital_e_t edMech_toggle;
        pros::controller_digital_e_t highScore_toggle;
        pros::controller_digital_e_t stopper_toggle;
        pros::controller_digital_e_t rakeMech_toggle;
        pros::controller_digital_e_t descore_toggle;
        pros::controller_digital_e_t rearIntake_toggle;

        // Color Sort
        pros::controller_digital_e_t colorSort_toggle;

        // Field Oriented Drive
        pros::controller_digital_e_t fieldOriented_toggle;
        pros::controller_digital_e_t heading_reset;

        // Score Macros
        pros::controller_digital_e_t scoreHighMacro;
        pros::controller_digital_e_t scoreMiddleHighMacro;

        robot::DriverProfile driverEnum;
    };
}
}
#endif