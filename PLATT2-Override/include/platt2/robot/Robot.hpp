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
        /**
         * @brief The robot class containing all subsystems and control logic.
         * @authors Dominic Young & Logan Wolf
         */
        class Robot
        {
            private:

            // Subsystems
            std::shared_ptr<subsystems::odometry::Odometry> odom_subsystem;
            std::shared_ptr<subsystems::tankDrive::TankDrive> tankDrive_subsystem;
            std::shared_ptr<subsystems::tankDrive::TankControl> tank_controller;
            std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem;
            std::shared_ptr<subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem;

            // Config Values
            std::unique_ptr<auton::IAuton> auton_routine;
            AllianceConfig current_alliance;
            RobotConfig current_config;
            AutonConfig current_auton_route;
            std::unique_ptr<profiles::DriverProfile> driver_profile;

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
             * @param xdrive_subsystem Shared pointer to an Xdrive subsystem object
             * @param odometry_subsystem Shared pointer to an Odometry subsystem object
             * @param holonomic_controller Shared pointer to a HolonomicControl subsystem object
             * @param intake_subsystem Shared pointer to an IntakeSubsystem object
             * @param alliance_config Current alliance configuration
             * @param robot_config Current robot configuration
             * @param auton_config Current autonomous route configuration
             * @param driver_profile Current driver profile
             * @param auton_routine Current autonomous routine
             * @param color_sort_subsystem Shared pointer to a ColorSortSubsystem object
             */
            Robot(std::shared_ptr<subsystems::tankDrive::TankDrive>& tank_drive_subsystem, 
                std::shared_ptr<subsystems::odometry::Odometry>& odometry_subsystem, 
                std::shared_ptr<subsystems::tankDrive::TankControl>& tank_controller,
                std::shared_ptr<subsystems::intake::IntakeSubsystem>& intake_subsystem,
                platt2::robot::AllianceConfig alliance_config,
                platt2::robot::RobotConfig robot_config,
                platt2::robot::AutonConfig auton_config,
                std::unique_ptr<profiles::DriverProfile>& driver_profile,
                std::unique_ptr<auton::IAuton>& auton_routine,
                std::shared_ptr<subsystems::colorsort::ColorSortSubsystem>& color_sort_subsystem
            );

        };
    };
};

#endif