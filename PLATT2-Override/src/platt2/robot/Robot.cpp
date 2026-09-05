#include "platt2/robot/Robot.hpp"
#include <cmath>    
#include <math.h>
#include <memory>
#include "platt2/EDriverConfig.hpp"
#include "platt2/helperFunctions.h"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

namespace platt2{

namespace robot{

    Robot::Robot(
        std::shared_ptr<subsystems::holonomicDrive::XDrive>& xDrive_subsystem,
        std::shared_ptr<subsystems::odometry::Odometry>& odometry_subsystem,
        std::shared_ptr<subsystems::holonomicDrive::HolonomicControl>& holonomic_control_subsystem,
        std::shared_ptr<subsystems::intake::IntakeSubsystem>& intake_subsystem,
        platt2::robot::AllianceConfig alliance_config,
        platt2::robot::RobotConfig robot_config,
        platt2::robot::AutonConfig auton_config,
        std::unique_ptr<profiles::DriverProfile>& driver_profile,
        std::unique_ptr<auton::IAuton>& auton_routine,
        std::shared_ptr<subsystems::colorsort::ColorSortSubsystem>& color_sort_subsystem
    ) : xDrive_subsystem{xDrive_subsystem},
        odom_subsystem{odometry_subsystem},
        holonomic_controller{holonomic_control_subsystem},
        intake_subsystem{intake_subsystem},
        driver_profile{std::move(driver_profile)},
        auton_routine{std::move(auton_routine)},
        color_sort_subsystem{color_sort_subsystem}        

    {
        current_alliance = alliance_config;
        current_auton_route = auton_config;
        current_config = robot_config;
    }

    void Robot::driverControl(){

        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};
        controller.print(0, 0, "Sorted Color: %d", color_sort_subsystem->getSortedColor());

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

            if(driver_profile->driverEnum = JON){
                movement.r = p.r;
                movement.theta = p.theta; 
                movement.w = rightX/1.5;
            }
            else if(driver_profile->driverEnum = QUINN){
                movement.r = p.r * 0.90;
                movement.theta = p.theta; 
                movement.w = rightX/3;
            }
    

            xDrive_subsystem->moveVector(movement);

            if(controller.get_digital(driver_profile->frontIntake_IN)){
                intake_subsystem->colorSortMode(color_sort_subsystem->isSortActive());
                intake_subsystem->move_intake(subsystems::intake::IntakeDirection::IN);
            }
            else if(controller.get_digital(driver_profile->frontIntake_OUT)){
                intake_subsystem->colorSortMode(color_sort_subsystem->isSortActive());
                intake_subsystem->move_intake(subsystems::intake::IntakeDirection::OUT_LOW_GOAL);
            }
            else{
                intake_subsystem->move_intake(subsystems::intake::IntakeDirection::STOP);
            }

            if(controller.get_digital_new_press(driver_profile->edMech_toggle)){
                intake_subsystem->toggle_ed_mech_piston();
            }

            if(controller.get_digital_new_press(driver_profile->highScore_toggle)){
                intake_subsystem->toggle_upper_conveyor_height_piston();
            }

            if(controller.get_digital_new_press(driver_profile->stopper_toggle)){
                intake_subsystem->toggle_conveyor_stopper_piston();
            }  
            
            if(controller.get_digital_new_press(driver_profile->rakeMech_toggle)){
                intake_subsystem->toggle_rake_mech_piston();
            }
            
            if(controller.get_digital_new_press(driver_profile->fieldOriented_toggle)){

            }

            if(controller.get_digital_new_press(driver_profile->colorSort_toggle)){
                color_sort_subsystem->toggleSortedColor();
                controller.print(0, 0, "Sorted Color: %d", color_sort_subsystem->getSortedColor());
            }

            if(controller.get_digital_new_press(driver_profile->descore_toggle)){
                intake_subsystem->toggle_descore_piston();
            }

            if(controller.get_digital_new_press(driver_profile->auto_unload_button)){
                if(!intake_subsystem->is_auto_unload_active()){
                    pros::Task([this]{ intake_subsystem->auto_unload(); });
                }
            }

            if(controller.get_digital_new_press(driver_profile->scoreHighMacro)){
                intake_subsystem->toggle_upper_conveyor_height_piston();
                intake_subsystem->toggle_conveyor_stopper_piston();  
            }

            if(controller.get_digital_new_press(driver_profile->scoreMiddleHighMacro)){
                intake_subsystem->toggle_ed_mech_piston();
                intake_subsystem->toggle_conveyor_stopper_piston();
            }
            
            if(controller.get_digital_new_press(driver_profile->rearIntake_toggle)){
                intake_subsystem->toggle_rear_intake_piston();
                if(intake_subsystem->get_rear_intake_position()){
                    controller.rumble("..");
                }
            }
            
            pros::delay(10);
        }
    
    }

    void Robot::autonControl(){
        auton_routine->run();
    }

    void Robot::init(){
        odom_subsystem->initImu();
    }

}
}