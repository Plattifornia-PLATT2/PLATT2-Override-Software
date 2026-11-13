#ifndef JONPROFILE_HPP
#define JONPROFILE_HPP

#include "platt2/EDriverConfig.hpp"
#include "platt2/profiles/DriverProfile.hpp"
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

    /**
     * @brief A struct for Jon's controller mappings
     * 
     */
    struct JonProfile : public DriverProfile{
    JonProfile() {
        // Intake Mappings
        frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;

        // Field Oriented Drive
        fieldOriented_toggle = pros::E_CONTROLLER_DIGITAL_RIGHT;
        heading_reset = pros::E_CONTROLLER_DIGITAL_UP;

        // Elevator Mappings
        elevator_up = pros::E_CONTROLLER_DIGITAL_R1;
        elevator_down = pros::E_CONTROLLER_DIGITAL_R2;
        elevator_full_down = pros::E_CONTROLLER_DIGITAL_B;

        // Claw Mappings
        claw_toggle = pros::E_CONTROLLER_DIGITAL_LEFT;
        claw_flip = pros::E_CONTROLLER_DIGITAL_DOWN;

        driverEnum = robot::JON;
    }
    };
}
}

#endif