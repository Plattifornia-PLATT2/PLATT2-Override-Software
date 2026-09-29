#include "platt2/robot/Robot.hpp"

namespace platt2{

namespace robot{

    Robot::Robot(
        std::shared_ptr<subsystems::holonomicDrive::XDrive>& xDrive_subsystem,
        std::shared_ptr<subsystems::odometry::Odometry>& odometry_subsystem,
        std::shared_ptr<subsystems::holonomicDrive::HolonomicControl>& holonomic_control_subsystem,
        platt2::robot::AllianceConfig alliance_config,
        platt2::robot::RobotConfig robot_config,
        platt2::robot::AutonConfig auton_config,
        std::unique_ptr<profiles::DriverProfile>& driver_profile
    ) : xDrive_subsystem{xDrive_subsystem},
        odom_subsystem{odometry_subsystem},
        holonomic_controller{holonomic_control_subsystem},
        driver_profile{std::move(driver_profile)}    
    {
        current_alliance = alliance_config;
        current_auton_route = auton_config;
        current_config = robot_config;
    }

    void Robot::driverControl(){

        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};      
        
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Exited");
        
        
        while(true){


            
            double leftX = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))/127;
            double leftY = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))/127;
            double rightX = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))/127;

            // right stick deadzone to eliminate stick drift issues with heading
            if(rightX < 0.03 && rightX > -0.03){
                rightX = 0;
            }

            // Create movement vector
            subsystems::holonomicDrive::MovementVector movement;  
            
            polar p = CtoP(leftX, leftY);   
            
            p.theta = p.theta - (odom_subsystem->getHeading()+(M_PI_2));

            if((driver_profile->driverEnum = JON)){
                movement.r = p.r;
                movement.theta = p.theta;
                movement.w = rightX/1.5;
            }
            else if((driver_profile->driverEnum = QUINN)){
                movement.r = p.r * 0.90;
                movement.theta = p.theta; 
                movement.w = rightX/3;
            }
    

            xDrive_subsystem->moveVector(movement);

            
            pros::delay(10);
        }
    
    }

    void Robot::autonControl(){

    }

    void Robot::init(){
        odom_subsystem->initImu();
    }

}
}