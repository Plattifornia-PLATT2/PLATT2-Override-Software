#ifndef EAUTONCONFIG_HPP
#define EAUTONCONFIG_HPP

/**
 * @brief The namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
#include "ERobotConfig.hpp"
namespace platt2{

/**
 * @brief The namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{
    
/**
 * @brief Enumeration to represent the 6 different autonomous modes the robot may operate in with a non auton option.
 * @author Dominic Young
 */
 enum AutonConfig
 {
    NO_AUTON = 0,
    PURPLE_COMP_WP = 1,
    PURPLE_SKILLS = 3,
    PINK_COMP_WP = 4,
    PINK_SKILLS = 6,

 };
}
}

#endif