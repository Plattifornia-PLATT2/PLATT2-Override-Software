#ifndef QUINNPROFILE_HPP
#define QUINNPROFILE_HPP

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
     * @brief A struct for Quinn's controller mappings
     * 
     */
    struct QuinnProfile : public DriverProfile{
        QuinnProfile() {
        frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;
        auto_unload_button = pros::E_CONTROLLER_DIGITAL_X;
        edMech_toggle = pros::E_CONTROLLER_DIGITAL_R1;
        highScore_toggle = pros::E_CONTROLLER_DIGITAL_R2;
        stopper_toggle = pros::E_CONTROLLER_DIGITAL_UP;
        colorSort_toggle = pros::E_CONTROLLER_DIGITAL_RIGHT;
        rakeMech_toggle = pros::E_CONTROLLER_DIGITAL_LEFT;
        fieldOriented_toggle = pros::E_CONTROLLER_DIGITAL_X;
        heading_reset = pros::E_CONTROLLER_DIGITAL_X;
        descore_toggle = pros::E_CONTROLLER_DIGITAL_A;
        scoreHighMacro = pros::E_CONTROLLER_DIGITAL_R1;
        scoreMiddleHighMacro = pros::E_CONTROLLER_DIGITAL_R2;
        rearIntake_toggle = pros::E_CONTROLLER_DIGITAL_DOWN;
        driverEnum = robot::JON;
        }
    };

}
}

#endif