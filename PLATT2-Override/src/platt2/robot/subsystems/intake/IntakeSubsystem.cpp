#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <memory>

namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    IntakeSubsystem::IntakeSubsystem(
        std::unique_ptr<pros::Motor>front_intake, 
        std::unique_ptr<pros::Motor>rear_intake, 
        std::unique_ptr<pros::Motor>mid_intake, 
        std::unique_ptr<pros::Motor>upper_conveyor,
        std::unique_ptr<pros::Motor>lower_roller,
        std::unique_ptr<pros::Motor> matchload_left_motor,
        std::unique_ptr<pros::Motor> matchload_right_motor,
        std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston,
        std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston,
        std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston,
        std::unique_ptr<pros::adi::DigitalOut> rake_mech_piston,
        std::unique_ptr<pros::adi::DigitalOut> descore_piston,
        std::unique_ptr<pros::adi::DigitalOut> rear_intake_piston
    )
        {
        front_intake_motor = std::move(front_intake);
        rear_intake_motor = std::move(rear_intake);
        middle_intake_motor = std::move(mid_intake);
        upper_conveyor_motor = std::move(upper_conveyor);
        lower_roller_motor = std::move(lower_roller);
        this->matchload_left_motor = std::move(matchload_left_motor);
        this->matchload_right_motor = std::move(matchload_right_motor);
        this->ed_mech_piston = std::move(ed_mech_piston);
        this->upper_conveyor_height_piston = std::move(upper_conveyor_height_piston);
        this->conveyor_stopper_piston = std::move(conveyor_stopper_piston);
        this->rake_mech_piston = std::move(rake_mech_piston);
        this->descore_piston = std::move(descore_piston);
        this->rear_intake_piston = std::move(rear_intake_piston);

        if(rear_intake_motor){
            rear_intake_motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        }

    }

    void IntakeSubsystem::move_intake(IntakeDirection direction){
        switch(direction){
            case IN:{
                if(colorSort){
                    if(rear_intake_piston_state){
                        front_intake_motor->move_velocity(-intake_speed);
                        middle_intake_motor->move_velocity(-intake_speed);
                        rear_intake_motor->move_velocity(intake_speed);
                        upper_conveyor_motor->move_velocity(intake_speed);
                        lower_roller_motor->move_velocity(intake_speed);
                        matchload_left_motor->move_velocity(intake_speed);
                        matchload_right_motor->move_velocity(intake_speed);
                    }
                    else {
                        front_intake_motor->move_velocity(intake_speed);
                        middle_intake_motor->move_velocity(intake_speed);
                        rear_intake_motor->move_velocity(-intake_speed);
                        upper_conveyor_motor->move_velocity(intake_speed);
                        lower_roller_motor->move_velocity(-intake_speed);
                        matchload_left_motor->move_velocity(-intake_speed);
                        matchload_right_motor->move_velocity(-intake_speed);
                    }
                }
                else{
                    front_intake_motor->move_velocity(intake_speed);
                    middle_intake_motor->move_velocity(intake_speed);
                    rear_intake_motor->move_velocity(intake_speed);
                    upper_conveyor_motor->move_velocity(intake_speed);
                    lower_roller_motor->move_velocity(intake_speed);
                    matchload_left_motor->move_velocity(intake_speed);
                    matchload_right_motor->move_velocity(intake_speed);
                }

                break;
            }
            case OUT:{
                front_intake_motor->move_velocity(-intake_speed);
                middle_intake_motor->move_velocity(-intake_speed);
                rear_intake_motor->move_velocity(-intake_speed);
                upper_conveyor_motor->move_velocity(-intake_speed);
                lower_roller_motor->move_velocity(-intake_speed);
                matchload_left_motor->move_velocity(-intake_speed);
                matchload_right_motor->move_velocity(-intake_speed);
                break;   
            }

            case OUT_LOW_GOAL:{
                if(rear_intake_piston_state){
                    front_intake_motor->move_velocity(-intake_speed);
                    middle_intake_motor->move_velocity(-intake_speed);
                    rear_intake_motor->move_velocity(intake_speed/2);
                    upper_conveyor_motor->move_velocity(-intake_speed/2);   
                    lower_roller_motor->move_velocity(intake_speed);
                    matchload_left_motor->move_velocity(intake_speed);
                    matchload_right_motor->move_velocity(intake_speed);
                }

                else{
                    front_intake_motor->move_velocity(-intake_speed/2);
                    middle_intake_motor->move_velocity(-intake_speed/2);
                    rear_intake_motor->move_velocity(-intake_speed);
                    upper_conveyor_motor->move_velocity(-intake_speed);   
                    lower_roller_motor->move_velocity(intake_speed/2);
                    matchload_left_motor->move_velocity(-intake_speed);
                    matchload_right_motor->move_velocity(-intake_speed);
                }
                break;
            }

            case TOP_REV:{
                front_intake_motor->move_velocity(0);
                middle_intake_motor->move_velocity(0);
                rear_intake_motor->move_velocity(0);
                upper_conveyor_motor->move_velocity(intake_speed);
                lower_roller_motor->move_velocity(0);   
                matchload_left_motor->move_velocity(0);
                matchload_right_motor->move_velocity(0);
                break;
            }

            case STOP:{
                front_intake_motor->move_velocity(0);
                middle_intake_motor->move_velocity(0);
                rear_intake_motor->move_velocity(0);
                upper_conveyor_motor->move_velocity(0);
                lower_roller_motor->move_velocity(0);   
                matchload_left_motor->move_velocity(0);
                matchload_right_motor->move_velocity(0);
                break;
            }
        }
    }

    void IntakeSubsystem::toggle_ed_mech_piston(){
        ed_mech_piston_state = !ed_mech_piston_state;
        ed_mech_piston->set_value(ed_mech_piston_state);
    }

    void IntakeSubsystem::toggle_upper_conveyor_height_piston(){
        upper_conveyor_height_piston_state = !upper_conveyor_height_piston_state;
        upper_conveyor_height_piston->set_value(upper_conveyor_height_piston_state);
    }

    void IntakeSubsystem::toggle_conveyor_stopper_piston(){
        conveyor_stopper_piston_state = !conveyor_stopper_piston_state;
        conveyor_stopper_piston->set_value(conveyor_stopper_piston_state);
    }

    void IntakeSubsystem::toggle_descore_piston(){
        descore_piston_state = !descore_piston_state;
        descore_piston->set_value(descore_piston_state);
    }

    void IntakeSubsystem::toggle_rake_mech_piston(){
        rake_mech_piston_state = !rake_mech_piston_state;
        rake_mech_piston->set_value(rake_mech_piston_state);
    }

    void IntakeSubsystem::move_rear_motor(IntakeDirection direction){
        switch(direction){
            case IN:{
                if(rear_intake_piston_state){
                        front_intake_motor->move_velocity(-intake_speed);
                        middle_intake_motor->move_velocity(-intake_speed);
                        rear_intake_motor->move_velocity(intake_speed);
                        upper_conveyor_motor->move_velocity(intake_speed);
                        lower_roller_motor->move_velocity(intake_speed);
                        matchload_left_motor->move_velocity(intake_speed);
                        matchload_right_motor->move_velocity(intake_speed);
                    }
                    else {
                        front_intake_motor->move_velocity(intake_speed);
                        middle_intake_motor->move_velocity(intake_speed);
                        rear_intake_motor->move_velocity(-intake_speed);
                        upper_conveyor_motor->move_velocity(intake_speed);
                        lower_roller_motor->move_velocity(-intake_speed);
                        matchload_left_motor->move_velocity(-intake_speed);
                        matchload_right_motor->move_velocity(-intake_speed);
                    }
            }
            case OUT:{
                rear_intake_motor->move_velocity(-intake_speed);
                lower_roller_motor->move_velocity(-intake_speed);
                matchload_left_motor->move_velocity(-intake_speed);
                matchload_right_motor->move_velocity(-intake_speed);
                break;   
            }
            
            case OUT_LOW_GOAL:{
                if(rear_intake_piston_state){
                    front_intake_motor->move_velocity(-intake_speed);
                    middle_intake_motor->move_velocity(-intake_speed);
                    rear_intake_motor->move_velocity(-intake_speed/2);
                    upper_conveyor_motor->move_velocity(-intake_speed/2);   
                    lower_roller_motor->move_velocity(intake_speed);
                    matchload_left_motor->move_velocity(-intake_speed);
                    matchload_right_motor->move_velocity(-intake_speed);
                }

                else{
                    front_intake_motor->move_velocity(-intake_speed);
                    middle_intake_motor->move_velocity(-intake_speed);
                    rear_intake_motor->move_velocity(intake_speed);
                    upper_conveyor_motor->move_velocity(-intake_speed);   
                    lower_roller_motor->move_velocity(intake_speed);
                    matchload_left_motor->move_velocity(intake_speed);
                    matchload_right_motor->move_velocity(intake_speed);
                }
                break;
            }

            case STOP:{
                rear_intake_motor->move_velocity(0);
                lower_roller_motor->move_velocity(0);
                matchload_left_motor->move_velocity(0);
                matchload_right_motor->move_velocity(0);
                break;
            }
        }
    }

    void IntakeSubsystem::tare_rear_motor_position(){
        rear_intake_motor->tare_position();
    }

    double IntakeSubsystem::get_rear_motor_position(){
        return rear_intake_motor->get_position();
    }

    void IntakeSubsystem::tare_middle_motor_position(){
        middle_intake_motor->tare_position();
    }

    double IntakeSubsystem::get_middle_motor_position(){
        return middle_intake_motor->get_position();
    }  

    void IntakeSubsystem::auto_unload(){
        auto_unload_active = false;
    }

    bool IntakeSubsystem::is_auto_unload_active(){
        return auto_unload_active;
    }

    void IntakeSubsystem::colorSortMode(bool state){
        this->colorSort = state;
    }

    void IntakeSubsystem::toggle_rear_intake_piston(){
        rear_intake_piston_state = !rear_intake_piston_state;
        rear_intake_piston->set_value(rear_intake_piston_state);
    }

    bool IntakeSubsystem::get_rear_intake_position(){
        return rear_intake_piston_state;
    }
    
}
}
}
}