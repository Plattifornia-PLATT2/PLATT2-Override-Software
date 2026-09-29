#ifndef EDRIVERCONFIG_HPP
#define EDRIVERCONFIG_HPP

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
     * @brief Enumeration to represent different driver profiles with a default option.
     * @author Dominic Young
     */
    enum DriverProfile {
        NO_DRIVER = -1,
        JON = 0,
        QUINN = 1
    };
}
}

#endif