#ifndef HOLONOMICCONTROL_HPP
#define HOLONOMICCONTROL_HPP

#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/pid/pid.hpp"

#include <memory>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */
namespace holonomicDrive{

/**
 * @brief Control class for holonomic drive systems.
 * @authors Dominic Young and Logan Wolf
 */
class HolonomicControl{
public:
    /**
     * @brief Construct a new Holonomic Control object
     * 
     * @param drive X drive chassis to control
     * @param odom Odometry system for position tracking
     * @param posPID PID controller for position control
     * @param headingPID PID controller for heading control
     */
    HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID);

    /**
     * @brief Moves the holonomic drive to a specified point with a target heading.
     * 
     * @param x_target Target X value to move to.
     * @param y_target Target Y value to move to.
     * @param heading_target Target heading to achieve at the end of the movement.
     */
    void moveToPoint(double x_target, double y_target, double heading_target, double rSpeed = 0.8, double wSpeed = 0.4, double timeout = 7);

    std::unique_ptr<pid::PID>& getHeadingPID();
    

    /**
     * @brief Used for debugging purposes.
     */

private:
    std::shared_ptr<XDrive> drivetrain;
    std::shared_ptr<odometry::Odometry> odometry;
    std::unique_ptr<pid::PID> positionPID;
    std::unique_ptr<pid::PID> headingPID;
    
    double velocityProfile(double TotalDistance, double remainingDistance, double currentVel, double maxVel, double maxAccel, bool& usePid);


};
}}}}

#endif