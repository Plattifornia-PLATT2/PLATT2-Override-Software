#ifndef IAUTON_HPP
#define IAUTON_HPP

#include "platt2/robot/EAllianceConfig.hpp"

#include "platt2/robot/Robot.hpp"

#include <memory>
#include <string>

/**
 * @brief Namespace for all PLATT2 Library Code
 * @authors PLATT2 development team
 */
namespace platt2{

/**
 * @brief Namespace for all autonomous routines
 * @author Dominic Young
 */
namespace auton{

/**
 * @brief Interface for all autonomous routines
 * @author Dominic Young
 */
class IAuton{
    public:

    virtual ~IAuton() = default;

    /**
     * @brief Initalizes an auton routine with the needed subsystems
     * 
     * @param holonomic_subsytem The holonomic subsystem to use
     * @param odometry_subsystem The odometry subsystem to use
     * @param intake_subsystem The intake subsystem to use
     * @param color_sort_subsystem The color sort subsystem to use
     */
    virtual void init() = 0;

    virtual std::string getName() const = 0;

    /**
     * @brief Runs the autonomous routine.
     * 
     */
    virtual void run() = 0;

};

}}

#endif