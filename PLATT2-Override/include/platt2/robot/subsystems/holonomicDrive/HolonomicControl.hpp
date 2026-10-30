#ifndef HOLONOMICCONTROL_HPP
#define HOLONOMICCONTROL_HPP

#include "autonControl/LQR.hpp"
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
    
    HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::shared_ptr<LQR> lqr);

    void tauTest();
    

private:
    std::shared_ptr<XDrive> drivetrain;
    std::shared_ptr<odometry::Odometry> odometry;
    std::shared_ptr<LQR> lqrController;


};
}}}}

#endif