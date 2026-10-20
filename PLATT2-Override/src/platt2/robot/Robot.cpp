#include "platt2/robot/Robot.hpp"
#include <cmath>    
#include <math.h>
#include <memory>
#include "platt2/EDriverConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/helperFunctions.h"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

namespace platt2{

namespace robot{

    Robot::Robot(RobotSubsystems subsystems,
        RobotConfig robotColor,
        std::unique_ptr<auton::IAuton> autonRoutine,
        std::unique_ptr<profiles::DriverProfile> driverProfile
    ) :
    subsystems(std::move(subsystems)),
    auton_routine{std::move(autonRoutine)},
    driverProfile{std::move(driverProfile)}
    {
        currentColor = robotColor;
    }

    void Robot::driverControl(){

        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};
        //controller.print(0, 0, "Sorted Color: %d", color_sort_subsystem->getSortedColor());          
        
        
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
            
            p.theta = p.theta - (subsystems.odometrySubsystem->getHeading()+(M_PI_2));

            p.r = p.r*60;

            if(driverProfile->driverEnum == JON){
                movement.r = p.r;
                movement.theta = p.theta;
                movement.w = rightX*3.14;
            }
            else if(driverProfile->driverEnum == QUINN){
                movement.r = p.r * 0.90;
                movement.theta = p.theta; 
                movement.w = rightX;
            }

            subsystems.xdriveSubsystem->moveVector(movement);

            
            pros::delay(10);
        }
    
    }

    void Robot::autonControl(){
        auton_routine->run();
    }

    void Robot::init(){
        subsystems.odometrySubsystem->initImu();
    }

}
}