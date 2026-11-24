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
        // Intake Mappings
        frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;
        toggle_mech = pros::E_CONTROLLER_DIGITAL_DOWN;
            
        // Field Oriented Drive
        fieldOriented_toggle = pros::E_CONTROLLER_DIGITAL_RIGHT;
        heading_reset = pros::E_CONTROLLER_DIGITAL_UP;
            
        // Elevator Mappings
        elevator_up = pros::E_CONTROLLER_DIGITAL_R1;
        elevator_down = pros::E_CONTROLLER_DIGITAL_R2;
        elevator_full_down = pros::E_CONTROLLER_DIGITAL_B;
            
        // Claw Mappings
        claw_toggle = pros::E_CONTROLLER_DIGITAL_RIGHT;
        manual_claw_up = pros::E_CONTROLLER_DIGITAL_L1;
        manual_claw_down = pros::E_CONTROLLER_DIGITAL_L2;
            
        // Arm Mappings
        //arm_low = pros:: E_CONTROLLER_DIGITAL_DOWN;
        //arm_high = pros::E_CONTROLLER_DIGITAL_RIGHT;

        driverEnum = robot::QUINN;
        }
    };

}
}

#endif