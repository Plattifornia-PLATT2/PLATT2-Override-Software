#include "platt2/robot/Robot.hpp"
#include <cmath>    
#include <coroutine>
#include <math.h>
#include <memory>
#include "platt2/EDriverConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/helperFunctions.h"
#include "pros/misc.h"
#include "subsystems/arm/arm.hpp"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"
#include "subsystems/intake/IntakeSubsystem.hpp"

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

        subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ONE_PIN);
        subsystems.armSubsystem->moveToLoadThenReady2();
        


        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};
        //controller.print(0, 0, "Sorted Color: %d", color_sort_subsystem->getSortedColor());          
        
        while(true){


            
            double vx = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))/127;
            double vy = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))/127;
            double vw = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))/127;

            // right stick deadzone to eliminate stick drift issues with heading
            if(vw < 0.03 && vw > -0.03){
                vw = 0;
            }

            
            
            subsystems::holonomicDrive::MovementVector movement;

        

            if(driverProfile->driverEnum == JON){
                movement.vx = vx*60;
                movement.vy = vy*60;
                movement.w = vw*3.14; // 1 rad/s turn speed
            }
            else if(driverProfile->driverEnum == QUINN){
                 movement.vx = vx*60;
                movement.vy = vy*60; 
                movement.w = vw*6.28;
            }
            movement.useVel = false;

            movement = subsystems.xdriveSubsystem->fieldToRobotOrientation(movement,subsystems.odometrySubsystem->getHeading());

            subsystems.xdriveSubsystem->moveVector(movement);

        

            // ********* Button Mappings *********

            if(controller.get_digital(driverProfile->toggle_mech)) subsystems.intakeSubsystem->moveToggleMech(subsystems::intake::IN);
            else subsystems.intakeSubsystem->moveToggleMech(subsystems::intake::STOP);

            if (controller.get_digital(driverProfile->elevator_up))
                subsystems.elevatorSubsystem->manualMove(200);
            else if (controller.get_digital(driverProfile->elevator_down))
                subsystems.elevatorSubsystem->manualMove(-200);
            else
                subsystems.elevatorSubsystem->manualMove(0);


            if (controller.get_digital_new_press(driverProfile->elevator_full_down))
                subsystems.elevatorSubsystem->moveToPosition(subsystems::elevator::FULL_DOWN);

            if(controller.get_digital_new_press(driverProfile->claw_toggle)) subsystems.armSubsystem->toggleClaw();

            if (controller.get_digital_new_press(driverProfile->arm_low))
                subsystems.armSubsystem->moveArmToPosition(subsystems::arm::READY2);

            if (controller.get_digital_new_press(driverProfile->arm_high))
                subsystems.armSubsystem->moveArmToPosition(subsystems::arm::SCORE);

            if (controller.get_digital_new_press(driverProfile->claw_toggle))
                subsystems.armSubsystem->toggleClaw();

            if(controller.get_digital(driverProfile->manual_claw_down)){
                subsystems.armSubsystem->manualClawMove(-1);
            } else if (controller.get_digital(driverProfile->manual_claw_up)){
                subsystems.armSubsystem->manualClawMove(1);
            }
            pros::screen::print(pros::E_TEXT_LARGE, 6, "Elevator Pos: %.3f",
            subsystems.elevatorSubsystem->getElevatorPositionRadians());
            
            pros::delay(10);
        }
    
    }

    void Robot::autonControl(){
        auton_routine->run();
    }

    void Robot::init(){
        subsystems.odometrySubsystem->initImu();
        subsystems.elevatorSubsystem->initElevator();
        subsystems.armSubsystem->init();
    }

}
}