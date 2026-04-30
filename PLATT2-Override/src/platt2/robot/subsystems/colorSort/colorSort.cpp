#include "platt2/robot/subsystems/colorSort/colorSort.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace robot{
namespace subsystems{
namespace colorsort{

    static void taskFunction(void *p) {
        reinterpret_cast<ColorSortSubsystem*>(p)->run();
    }

    ColorSortSubsystem::ColorSortSubsystem(
        std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem,
        std::unique_ptr<pros::Optical> optical_sensor
    ) : 
    intake_subsystem(intake_subsystem), 
    optical_sensor(std::move(optical_sensor)),
    color_sort_task(taskFunction, this)
    {

    }

    void ColorSortSubsystem::setSortOption(BallColor color) {
        current_color = color;
    }

    BallColor ColorSortSubsystem::getSortedColor(){
        return current_color;
    }

    bool ColorSortSubsystem::isSortActive(){
        return isActive;
    }

    void ColorSortSubsystem::toggleSortedColor() {
        switch(current_color) {
            case OFF:
                current_color = RED;
                break;
            case RED:
                current_color = BLUE;
                break;
            case BLUE:
                current_color = OFF;
                isActive = false;
                break;
        }
    }

    void ColorSortSubsystem::run() {
        optical_sensor->set_led_pwm(100);
        while(true){
    

        pros::c::optical_rgb_s_t currentRGB = optical_sensor->get_rgb();
        double currentR = currentRGB.red;
        double currentB = currentRGB.blue;
        double currentDistance = optical_sensor->get_proximity();
        double rejection_threshold;
        if(intake_subsystem->get_rear_intake_position()){
            rejection_threshold = MATCHLOAD_BALL_REJECTION_THRESHOLD;
        }
        else {
            rejection_threshold = FRONT_LOAD_BALL_REJECTION_THRESHOLD;
        }

        switch(current_color)
        {
            case RED:
            {

                if(currentR > currentB && currentDistance > BALL_DISTANCE_THRESHOLD)
                {
                    isActive = true;
                    intake_subsystem->tare_rear_motor_position();
                    if(intake_subsystem->get_rear_intake_position()){
                        intake_subsystem->move_rear_motor(intake::IN);
                    }
                    else {
                        intake_subsystem->move_rear_motor(intake::OUT);
                    }
                    
                    while(std::abs(intake_subsystem->get_rear_motor_position()) < rejection_threshold) {
                        pros::delay(10);
                    }
                    intake_subsystem->move_rear_motor(intake::STOP);
                }
                isActive = false;
                break;
            }
            case BLUE:
            {
                if(currentB > currentR && currentDistance > BALL_DISTANCE_THRESHOLD)
                {
                    intake_subsystem->tare_rear_motor_position();
                    if(intake_subsystem->get_rear_intake_position()){
                        intake_subsystem->move_rear_motor(intake::IN);
                    }
                    else {
                        intake_subsystem->move_rear_motor(intake::OUT);
                    }
                    while(std::abs(intake_subsystem->get_rear_motor_position()) < rejection_threshold) {
                        std::cout<<" Distance: "<<intake_subsystem->get_rear_motor_position()<<std::endl;
                        pros::delay(10);
                    }
                    intake_subsystem->move_rear_motor(intake::STOP);
                    }
                    isActive = false;
                    break;
            }
            case OFF:
            {
                isActive = false;
                break;
            }
        }
            pros::delay(10);
    }

        }       
}}}}