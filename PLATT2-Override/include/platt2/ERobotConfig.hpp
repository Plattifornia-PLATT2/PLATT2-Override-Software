#ifndef EROBOTCONFIG_HPP
#define EROBOTCONFIG_HPP

/**
 * @brief The namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief The namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{
    /**
     * @brief Enumeration to represent the 2 different robot configurations with a default option.
     * @author Dominic Young
     */
    enum RobotConfig
    {
        NO_ROBOT = 0,
        PINK = 1,
        PURPLE = 2
        
    };

}
}

#endif