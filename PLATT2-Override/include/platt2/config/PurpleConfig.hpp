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
#include "platt2/hal/TrackingWheel.hpp"
#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"


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
    static constexpr int FRONT_RIGHT_TOP_MOTOR_PORT{-17};
    static constexpr int FRONT_RIGHT_BOTTOM_MOTOR_PORT{18};
    static constexpr int FRONT_LEFT_TOP_MOTOR_PORT{12};
    static constexpr int FRONT_LEFT_BOTTOM_MOTOR_PORT{-11};

    static constexpr int BACK_RIGHT_TOP_MOTOR_PORT{-20};
    static constexpr int BACK_RIGHT_BOTTOM_MOTOR_PORT{19};
    static constexpr int BACK_LEFT_TOP_MOTOR_PORT{15};
    static constexpr int BACK_LEFT_BOTTOM_MOTOR_PORT{-16};


    // ** Intake Ports **
    static constexpr int FRONT_INTAKE_MOTOR_PORT{-14};
    static constexpr int TOGGLE_MECH_MOTOR_PORT{-3};
    static constexpr pros::MotorGears INTAKE_GEARSET{pros::MotorGears::blue};
    static constexpr int ALIGNER_DEPLOY_PISTON{2};


    // ** VEX IMU Port **
    static constexpr int VEX_IMU_PORT{8};

    // ** Elevator Ports **
    static constexpr int LEFT_ELEVATOR_MOTOR_PORT{-2};
    static constexpr int RIGHT_ELEVATOR_MOTOR_PORT{13};
    static constexpr int ELEVATOR_ROTATION_SENSOR_PORT{3};
    static constexpr pros::MotorGears ELEVATOR_MOTOR_GEARS{pros::MotorGears::red};

    // **Arm Ports**
    static constexpr int LEFT_ARM_MOTOR_PORT{-4};
    static constexpr int RIGHT_ARM_MOTOR_PORT {6};
    static constexpr int CLAW_MOTOR_PORT{5};
    static constexpr int CLAW_PISTON_PORT{1};
    static constexpr pros::MotorGears ARM_MOTOR_GEARS{pros::MotorGears::red};
    static constexpr pros::MotorGears CLAW_MOTOR_GEARS{pros::MotorGears::green};

    // ** Elevator PID Tuning Values **
    static constexpr double ELEVATOR_DT{0.01};
    static constexpr double ELEVATOR_MAX{1};
    static constexpr double ELEVATOR_MIN{-1};
    static constexpr double ELEVATOR_Kp{0.054};
    static constexpr double ELEVATOR_Kd{0.005};
    static constexpr double ELEVATOR_Ki{0};

    // ** Odometry Offsets **
    static constexpr int HORIZONTAL_ENCODER_PORT{20};
    static constexpr int VERTICAL_ENCODER_PORT{-7};
    static constexpr double TRACKING_WHEEL_DIAMETER{2};
    static constexpr double HORIZONTAL_TRACKING_WHEEL_OFFSET{-.5};
    static constexpr double VERTICAL_TRACKING_WHEEL_OFFSET{-3.15};

    static constexpr double PI_PORT{7};

    // ** Linear PID Tuning Values **
    static constexpr double position_dt{0.01};
    static constexpr double position_max{1};
    static constexpr double position_min{-1};
    static constexpr double position_Kp{0.054};
    static constexpr double position_Kd{0.005};
    static constexpr double position_Ki{0};

    // ** Angular PID Tuning Values **
    static constexpr double heading_dt{0.01}; 
    static constexpr double heading_max{1};
    static constexpr double heading_min{-1};
    static constexpr double heading_Kp{0.15};
    static constexpr double heading_Kd{0.005};
    static constexpr double heading_Ki{0};


        // LQR tuning values
    static constexpr double TAU{0.1}; //NEEDS TO BE FOUND
    static constexpr double DT{0.01};
    const Eigen::Matrix<double, 6, 1> QDIAG{1.2, 1.2, 40, 0.01, 0.01, 1}; 
    const Eigen::Vector3d RDIAG{0.0025, 0.0025, 100};

    
    std::shared_ptr<robot::subsystems::odometry::Odometry> buildOdometrySubsystem();
    std::shared_ptr<robot::subsystems::odometry::Odometry> buildOdometrySubsystemWithPi(std::shared_ptr<platt2::robot::subsystems::piLink::PiLinkSubsystem> pi_subsystem);
    std::shared_ptr<platt2::robot::subsystems::holonomicDrive::XDrive> buildXDriveSubsystem();
    std::shared_ptr<platt2::robot::subsystems::holonomicDrive::HolonomicControl> buildControlSystem(std::shared_ptr<robot::subsystems::holonomicDrive::XDrive> xdriveSubsystem, std::shared_ptr<robot::subsystems::odometry::Odometry> odometrySubsystem);
    std::shared_ptr<platt2::robot::subsystems::intake::IntakeSubsystem> buildIntakeSubsystem();
    std::shared_ptr<platt2::robot::subsystems::piLink::PiLinkSubsystem> buildPiLinkSubsystem();
    std::shared_ptr<platt2::robot::subsystems::arm::ArmSubsystem> buildArmSubsystem();
    std::shared_ptr<robot::subsystems::elevator::ElevatorSubsystem> buildElevatorSubsystem();

    robot::RobotSubsystems buildSubsystems();



    public:

    /**
     * @brief Builds the pink robot with given config values
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