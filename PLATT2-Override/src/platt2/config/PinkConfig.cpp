#include "platt2/config/PinkConfig.hpp"


namespace platt2{
namespace config{
constexpr double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

std::shared_ptr<platt2::robot::subsystems::holonomicDrive::XDrive> PinkConfig::buildXDriveSubsystem(){
    // Right Module
    std::unique_ptr<pros::v5::Motor> fr_bottom{std::make_unique<pros::v5::Motor>(FRONT_RIGHT_TOP_MOTOR_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> fr_top{std::make_unique<pros::v5::Motor>(FRONT_RIGHT_BOTTOM_MOTOR_PORT, DRIVE_GEARSET)};

    std::unique_ptr<pros::v5::Motor> fl_bottom{std::make_unique<pros::v5::Motor>(FRONT_LEFT_TOP_MOTOR_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> fl_top{std::make_unique<pros::v5::Motor>(FRONT_LEFT_BOTTOM_MOTOR_PORT, DRIVE_GEARSET)};

    //Left Module
    std::unique_ptr<pros::v5::Motor> br_top{std::make_unique<pros::v5::Motor>(BACK_RIGHT_TOP_MOTOR_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> br_bottom{std::make_unique<pros::v5::Motor>(BACK_RIGHT_BOTTOM_MOTOR_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> bl_top{std::make_unique<pros::v5::Motor>(BACK_LEFT_TOP_MOTOR_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> bl_bottom{std::make_unique<pros::v5::Motor>(BACK_LEFT_BOTTOM_MOTOR_PORT, DRIVE_GEARSET)};

    

    //X drive modules

    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> front_right_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(fr_top, fr_bottom, deg_to_rad(45), deg_to_rad(135))};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> front_left_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(fl_top, fl_bottom, deg_to_rad(135), deg_to_rad(225))};

    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> back_right_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(br_top, br_bottom, deg_to_rad(315), deg_to_rad(45))};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> back_left_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(bl_top, bl_bottom, deg_to_rad(225), deg_to_rad(315))};

    // x drive system
    std::vector<std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule>> modules;
    modules.push_back(std::move(front_right_module));
    modules.push_back(std::move(front_left_module));
    modules.push_back(std::move(back_right_module));
    modules.push_back(std::move(back_left_module));

    std::shared_ptr<platt2::robot::subsystems::holonomicDrive::XDrive> xDrive_subsystem = std::make_shared<platt2::robot::subsystems::holonomicDrive::XDrive>(std::move(modules));

    return xDrive_subsystem;
}

std::shared_ptr<robot::subsystems::odometry::Odometry> PinkConfig::buildOdometrySubsystem(){
    // odom subsystem
    std::unique_ptr<pros::IMU> vex_imu = std::make_unique<pros::IMU>(VEX_IMU_PORT);
    std::unique_ptr<pros::Rotation> horiontal_encoder = std::make_unique<pros::Rotation>(HORIZONTAL_ENCODER_PORT);
    std::unique_ptr<pros::Rotation> vertical_encoder = std::make_unique<pros::Rotation>(VERTICAL_ENCODER_PORT);

    std::unique_ptr<hal::TrackingWheel> horizontal_tracking_wheel = std::make_unique<hal::TrackingWheel>(std::move(horiontal_encoder), TRACKING_WHEEL_DIAMETER);
    std::unique_ptr<hal::TrackingWheel> vertical_tracking_wheel = std::make_unique<hal::TrackingWheel>(std::move(vertical_encoder), TRACKING_WHEEL_DIAMETER);

    std::unique_ptr<robot::subsystems::odometry::TrackingWheelPositionTracker> position_tracker = std::make_unique<robot::subsystems::odometry::TrackingWheelPositionTracker>(std::move(horizontal_tracking_wheel), std::move(vertical_tracking_wheel), std::move(vex_imu));
    position_tracker->setOffsets(HORIZONTAL_TRACKING_WHEEL_OFFSET, VERTICAL_TRACKING_WHEEL_OFFSET);
    std::shared_ptr<robot::subsystems::odometry::Odometry> odom_subsystem;
    odom_subsystem = std::make_shared<robot::subsystems::odometry::Odometry>(std::move(position_tracker));
   
    robot::subsystems::odometry::Position startingPos = {0,0,270};
    odom_subsystem->setPos(startingPos);

    return odom_subsystem;
}

std::shared_ptr<robot::Robot> PinkConfig::buildRobot(robot::AutonConfig auton, robot::DriverProfile profile, robot::AllianceConfig alliance){

    std::shared_ptr<platt2::robot::subsystems::holonomicDrive::XDrive> xDrive_subsystem = buildXDriveSubsystem();

    std::shared_ptr<robot::subsystems::odometry::Odometry> odom_subsystem = buildOdometrySubsystem();
    
    // intake subsystem
    
    

    //holonomic control system
    std::unique_ptr<robot::pid::PID>position_pid = std::make_unique<robot::pid::PID>(position_dt, position_max, position_min, position_Kp, position_Kd, position_Ki);
    std::unique_ptr<robot::pid::PID>heading_pid = std::make_unique<robot::pid::PID>(heading_dt, heading_max, heading_min, heading_Kp, heading_Kd, heading_Ki);
   
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_control_subsystem = std::make_shared<robot::subsystems::holonomicDrive::HolonomicControl>(xDrive_subsystem, odom_subsystem, std::move(position_pid), std::move(heading_pid));
    
    
    std::unique_ptr<profiles::DriverProfile> driver_profile;
    //build driver profile
    if(profile == robot::JON){
        std::unique_ptr<profiles::JonProfile> jon_profile = std::make_unique<profiles::JonProfile>();
        driver_profile = std::move(jon_profile);
    }
    else{
        std::unique_ptr<profiles::QuinnProfile> quinn_profile = std::make_unique<profiles::QuinnProfile>();
        driver_profile = std::move(quinn_profile);
    }

    // Build auton routine
    std::unique_ptr<auton::IAuton> auton_routine;

    switch(auton){
        case robot::PINK_SKILLS:{
            std::unique_ptr<auton::PinkSkillsAuton> pink_skills_auton = std::make_unique<auton::PinkSkillsAuton>();
            auton_routine = std::move(pink_skills_auton);
            auton_routine->init();
            break;
        }
        case robot::PURPLE_SKILLS:{
            std::unique_ptr<auton::PurpleSkillsAuton> purple_skills_auton = std::make_unique<auton::PurpleSkillsAuton>();
            auton_routine = std::move(purple_skills_auton);
            auton_routine->init();
             break;
        }
        case robot::PINK_COMP_WP:{
            std::unique_ptr<auton::PinkCompAuton> pink_comp_auton = std::make_unique<auton::PinkCompAuton>();
            auton_routine = std::move(pink_comp_auton);
            auton_routine->init();
             break;
        }
        case robot::PURPLE_COMP_WP:{
            std::unique_ptr<auton::PurpleCompAuton> purple_comp_auton = std::make_unique<auton::PurpleCompAuton>();
            auton_routine = std::move(purple_comp_auton);
            auton_routine->init();
             break;
        }
        case robot::NO_AUTON:{
            break;
        }

    }   

    // build robot object
    std::shared_ptr<robot::Robot> robot{std::make_shared<robot::Robot>(
        xDrive_subsystem, 
        odom_subsystem, 
        holonomic_control_subsystem, 
        alliance, 
        platt2::robot::RobotConfig::PINK, 
        auton, 
        driver_profile
    )};

    return robot;

}

}}