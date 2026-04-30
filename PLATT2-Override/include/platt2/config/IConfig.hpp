#ifndef ICONFIG_HPP
#define ICONFIG_HPP

#include <memory>

#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/EDriverConfig.hpp"
#include "platt2/robot/Robot.hpp"

/**
 * @brief Namespace for all PLATT2 Library Code
 * @authors PLATT2 Development team
 */
namespace platt2{

/**
 * @brief Namespace for all Robot configuration related code
 * @author Dominic Young
 */
namespace config{

    /**
     * @brief Interface for robot configuration classes
     * @author Dominic Young
     */
    class IConfig{
        public:

        virtual ~IConfig() = default;

    /**
     * @brief Builds a robot with given config values
     * 
     * @param auton The auton route to use
     * @param profile The driver profile to use
     * @param alliance The alliance to use
     * @return std::shared_ptr<robot::Robot> The built robot object
     */
        virtual std::shared_ptr<robot::Robot> buildRobot(robot::AutonConfig auton, robot::DriverProfile profile, robot::AllianceConfig alliance) = 0;
    };
}
}

#endif