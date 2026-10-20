#ifndef ROBOT_HPP
#define ROBOT_HPP

// ** PLATT2 Library Includes **
#include "platt2/profiles/DriverProfile.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/robot/subsystems/tankDrive/tankControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/colorsort/ColorSort.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/auton/IAuton.hpp"

// ** PROS API Includes **
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "subsystems/holonomicDrive/HolonomicControl.hpp"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "subsystems/tankDrive/tankControl.hpp"
#include "subsystems/tankDrive/tankDrive.hpp"

// ** Standard Library Includes **
#include <memory>
#include <algorithm>
#include <cmath>
#include <utility>

/**
 * @brief The namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2
{
    /**
     * @brief The namespace for robot-related code.
     * @authors PLATT2 Development team.
     */
    namespace robot
    {

        struct RobotSubsystems{
            std::shared_ptr<subsystems::holonomicDrive::XDrive> xdriveSubsystem;
            std::shared_ptr<subsystems::odometry::Odometry> odometrySubsystem;
            std::shared_ptr<subsystems::holonomicDrive::HolonomicControl> holonomicControlSubsystem;
            std::shared_ptr<subsystems::intake::IntakeSubsystem> intakeSubsystem;
        };

        /**
         * @brief The robot class containing all subsystems and control logic.
         * @authors Dominic Young & Logan Wolf
         */
        class Robot
        {
            private:
            RobotSubsystems subsystems;

            // Config Values
            std::unique_ptr<auton::IAuton> auton_routine;
            RobotConfig currentColor;
            AutonConfig currentAutonRoute;
            std::unique_ptr<profiles::DriverProfile> driverProfile;

            public:
            
            /**
             * @brief Runs the autonomous control routine.
             * 
             */
            void autonControl();

            /**
             * @brief Runs the driver control routine.
             * 
             */
            void driverControl();

            /**
             * @brief Initalizes robot sensors and subsystems.
             * 
             */
            void init();

            /**
             * @brief Construct a new Robot object with the given subsystems and parameters.
             * 
             */
            Robot(RobotSubsystems subsystems, RobotConfig robotColor, std::unique_ptr<auton::IAuton> autonRoutine, std::unique_ptr<profiles::DriverProfile> driverProfile);

        };
    };
};

#endif