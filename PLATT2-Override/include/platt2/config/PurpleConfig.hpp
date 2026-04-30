#ifndef PURPLECONFIG_HPP
#define PURPLECONFIG_HPP

// ** PLATT2 Library Includes **
#include "platt2/config/IConfig.hpp"
#include "platt2/robot/Robot.hpp"
#include "platt2/robot/pid/pid.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/subsystems/colorsort/ColorSort.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/profiles/JonProfile.hpp"
#include "platt2/profiles/DriverProfile.hpp"
#include "platt2/profiles/QuinnProfile.hpp"
#include "platt2/auton/PinkSkillsAuton.hpp"
#include "platt2/auton/PurpleSkillsAuton.hpp"
#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/auton/PurpleCompAuton.hpp"

// ** Pros API Includes **
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// ** Standard C++ Library Includes **
#include <memory>
#include <vector>
#include <cmath>

/**
 * @brief Namespace for all PLATT2 library code
 * @authors PLATT2 devlopment team
 */
namespace platt2{

/**
 * @brief Namespace for all Robot configuration related code
 * @author Dominic Young
 */
namespace config{

/**
 * @brief Class for the purple robot's configuration
 * @author Dominic Young
 */
class PurpleConfig : public IConfig {
    private:
    static constexpr pros::MotorGears DRIVE_GEARSET{pros::MotorGears::blue};


    // ** Drive Ports **
    static constexpr int RIGHT_1_PORT{-20};
    static constexpr int RIGHT_2_PORT{19};
    static constexpr int RIGHT_3_PORT{2};
    static constexpr int RIGHT_4_PORT{-1};

    static constexpr int LEFT_1_PORT{3};
    static constexpr int LEFT_2_PORT{-4}; 
    static constexpr int LEFT_3_PORT{-5};
    static constexpr int LEFT_4_PORT{6};

    // ** Intake Ports **
    static constexpr int FRONT_INTAKE_MOTOR_PORT{-8};
    static constexpr int MIDDLE_INTAKE_MOTOR_PORT{12};
    static constexpr int REAR_INTAKE_MOTOR_PORT{-9};
    static constexpr int LOWER_ROLLER_MOTOR_PORT{-11};
    static constexpr int UPPER_CONVEYOR_MOTOR_PORT{15};
    static constexpr int REAR_INTAKE_LEFT_MOTOR_PORT{13};
    static constexpr int REAR_INTAKE_RIGHT_MOTOR_PORT{-14};
    //static constexpr int DISTANCE_SENSOR_PORT{14};
    static constexpr pros::MotorGears INTAKE_GEARSET{pros::MotorGears::blue};

    // ** Color Sort Ports **
    static constexpr int OPTICAL_SENSOR_PORT{10};

    // ** Pneumatics **
    static constexpr int ED_MECH_PISTON_PORT{1};
    static constexpr int UPPER_CONVEYOR_HEIGHT_PISTON_PORT{2};
    static constexpr int CONVEYOR_STOPPER_PISTON_PORT{3};
    static constexpr int RAKE_MECH_PISTON_PORT{4};
    static constexpr int DESCORE_PISTON_PORT{5};
    static constexpr int REAR_INTAKE_PISTON_PORT {6};

    // ** VEX IMU Port **
    static constexpr int VEX_IMU_PORT{18};

    // ** Odometry Offsets **
    static constexpr int HORIZONTAL_ENCODER_PORT{20};
    static constexpr int VERTICAL_ENCODER_PORT{-7};
    static constexpr double TRACKING_WHEEL_DIAMETER{2};
    static constexpr double HORIZONTAL_TRACKING_WHEEL_OFFSET{-.5};
    static constexpr double VERTICAL_TRACKING_WHEEL_OFFSET{-3.15};

    // ** Linear PID Tuning Values **
    static constexpr double position_dt{0.01};
    static constexpr double position_max{1};
    static constexpr double position_min{-1};
    static constexpr double position_Kp{0.075};
    static constexpr double position_Kd{0.0042};
    static constexpr double position_Ki{0};

    // ** Angular PID Tuning Values **
    static constexpr double heading_dt{0.01}; 
    static constexpr double heading_max{1};
    static constexpr double heading_min{-1};
    static constexpr double heading_Kp{0.17};
    static constexpr double heading_Kd{0.005};
    static constexpr double heading_Ki{0};

    
    public:

    /**
     * @brief Builds the purple robot with given config values
     * 
     * @param auton The auton route to use
     * @param profile The driver profile to use
     * @param alliance The alliance to use
     * @return std::shared_ptr<robot::Robot> The built robot object
     */
    std::shared_ptr<robot::Robot> buildRobot(robot::AutonConfig auton, robot::DriverProfile profile, robot::AllianceConfig alliance) override;
};
}
}

#endif