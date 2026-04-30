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
        frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;
        //auto_unload_button = pros::E_CONTROLLER_DIGITAL_X;
        edMech_toggle = pros::E_CONTROLLER_DIGITAL_R2;
        highScore_toggle = pros::E_CONTROLLER_DIGITAL_R1;
        stopper_toggle = pros::E_CONTROLLER_DIGITAL_B;
        colorSort_toggle = pros::E_CONTROLLER_DIGITAL_UP;
        rakeMech_toggle = pros::E_CONTROLLER_DIGITAL_X;
        //fieldOriented_toggle = pros::E_CONTROLLER_DIGITAL_X;
        //heading_reset = pros::E_CONTROLLER_DIGITAL_X;
        descore_toggle = pros::E_CONTROLLER_DIGITAL_DOWN;
        scoreHighMacro = pros::E_CONTROLLER_DIGITAL_R1;
        scoreMiddleHighMacro = pros::E_CONTROLLER_DIGITAL_R2;
        rearIntake_toggle = pros::E_CONTROLLER_DIGITAL_Y;
        driverEnum = robot::JON;
    }
    };
}
}

#endif