#ifndef MATCHCONTROLLER_HPP
#define MATCHCONTROLLER_HPP

// ** PLATT2 Library Includes **
#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/menu/menu.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/config/PinkConfig.hpp"
#include "platt2/config/PurpleConfig.hpp"
#include "platt2/menu/menu.hpp"


// ** PROS API Includes **
#include "pros/rtos.hpp"

// ** Standard Library Includes **
#include <memory>

/**
    @brief The namespace for platt2 library code.
    @authors PLATT2 Development team.
*/
namespace platt2{

/**
    @brief The class to run the different states of the match and set the robot configuration.
    @author Dominic Young
*/
class MatchController{
    private:
    /**
        @brief The enumeration for the current auton type.
    */
    robot::AutonConfig auton_config;

    /**
        @brief A unique pointer to the current robot config class object.
    */
    std::unique_ptr<config::IConfig> config{};

    /**
        @brief The enumeration for the current alliance configuration.
    */
    robot::AllianceConfig alliance_config; 

    /**
        @brief The menu object for setting config values via an on-screen GUI.
    */
    menu::RobotConfigMenu menu{};

    /**
        @brief The shared pointer to the robot object built with the current configuration.
    */
    std::shared_ptr<robot::Robot> robot{};

    public:

    /**
        @brief Initializes the match controller and runs the menu.
    */
    void init();

    /**
        @brief Runs the robot driver control logic.
    */
    void driveControl();

    /**
        @brief Runs the robot autonomous control logic.
    */
    void autonControl();

};
}

#endif