#ifndef INTAKESUBSYSTEM_HPP
#define INTAKESUBSYSTEM_HPP

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

#include <memory>

/**
 * @brief Namespace for PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for intake subsystem related code.
 * @authors PLATT2 Development team.
 */
namespace intake{

    /**
     * @brief An enumeration to represent the direction of the intake subsystem.
     * @author Dominic Young
     */
    enum IntakeDirection{
        IN = 1,
        OUT = 2,
        STOP = 3,
        OUT_LOW_GOAL=4,
        TOP_REV=5
    };

    /**
    @brief A class to control the Intake subsytem
    @author Dominic Young
    */
    class IntakeSubsystem{

        private:
        double intake_speed{600};
        
        // ** Motors **
        std::unique_ptr<pros::Motor> front_intake_motor;
        std::unique_ptr<pros::Motor> rear_intake_motor;
        std::unique_ptr<pros::Motor> middle_intake_motor;
        std::unique_ptr<pros::Motor> upper_conveyor_motor;
        std::unique_ptr<pros::Motor> lower_roller_motor;
        std::unique_ptr<pros::Motor> matchload_left_motor;
        std::unique_ptr<pros::Motor> matchload_right_motor;
 
        // ** Solenoids **
        std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston;
        std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston;
        std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston;
        std::unique_ptr<pros::adi::DigitalOut> rake_mech_piston;
        std::unique_ptr<pros::adi::DigitalOut> descore_piston;
        std::unique_ptr<pros::adi::DigitalOut> rear_intake_piston;


        bool conveyor_stopper_piston_state = false;
        bool conveyor_stopper_piston_lastState = false;
        bool ed_mech_piston_state = false;
        bool ed_mech_piston_lastState = false;
        bool upper_conveyor_height_piston_state = false;
        bool upper_conveyor_height_piston_lastState = false;
        bool rake_mech_piston_state = false;
        bool auto_unload_active = false;
        bool descore_piston_state = false;
        bool colorSort = false;
        bool matchLoading = false;
        bool rear_intake_piston_state = false;
        bool rear_intake_piston_lastState = false;

        const double MAX_DISTANCE{100.0};

        public:
        /**
         * @brief Moves the intake motors in the specified direction.
         * 
         * @param direction The direction to move the intake motors.
         */
        void move_intake(IntakeDirection direction);

        /**
         * @brief Moves the rear intake motor in the specified direction.
         * 
         * @param direction The direction to move the rear intake motor.
         */
        void move_rear_motor(IntakeDirection direction);
        
        /**
         * @brief Automatically unloads the intake subsystem.
         */
        void auto_unload();

        /**
         * @brief Get the rear motor position object   
         * 
         * @return double 
         */
        double get_rear_motor_position();

        /**
         * @brief Tares the rear motor position.
         * 
         */
        void tare_rear_motor_position();

        /**
         * @brief Checks if the auto unload feature is active.
         * 
         * @return true When the auto unload feature is active and running.
         * @return false When the auto unload feature is not active.
         */
        bool is_auto_unload_active();

        /**
         * @brief Toggles the state of the ED mechanism piston.
         * 
         */
        void toggle_ed_mech_piston();

        /**
         * @brief Toggles the state of the upper conveyor height piston.
         * 
         */
        void toggle_upper_conveyor_height_piston();

        /**
         * @brief Toggles the state of the conveyor stopper piston.
         * 
         */
        void toggle_conveyor_stopper_piston();

        /**
         * @brief Toggles the state of the rake mechanism piston.
         * 
         */
        void toggle_rake_mech_piston();

        /**
         * @brief Toggles the state of the descore piston.
         * 
         */
        void toggle_descore_piston();

        /**
         * @brief Toggles the state of the rear intake (matchloader) pistons.
         * 
         */
        void toggle_rear_intake_piston();

        /**
         * @brief Gets the position of the rear intake piston (matchloader).
         * @return A boolean for the position of the rear intake state (T for out, F for in).
         */
        bool get_rear_intake_position();

        /**
         * @brief Sets the current position of the middle intake motor to 0.
         */
        void tare_middle_motor_position();

        /**
         * @brief gets the position of the middle intake motor.
         * @return A double of the current position of the middle intake motor from its internal encoder. 
         */
        double get_middle_motor_position();

        /**
         * @brief Spins the intake motors in an optimized configuration for matchloading.
         */
        void move_intake_matchload(IntakeDirection direction);

        /**
         * @brief Sets the intake subsystem to color sort mode, or disables it.
         * 
         * @param state True for color sort enabled, false for color sort disabled.
         */
        void colorSortMode(bool state);

        /**
         * @brief Construct a new Intake Subsystem object
         * 
         * @param front_intake Front intake motor
         * @param rear_intake Rear intake motor
         * @param mid_intake Middle intake motor
         * @param upper_conveyor Upper conveyor motor
         * @param lower_roller Lower roller motor
         * @param ed_mech_piston ED mechanism piston
         * @param upper_conveyor_height_piston Upper conveyor height piston
         * @param conveyor_stopper_piston Conveyor stopper piston
         * @param rake_mech_piston Rake mechanism piston
         * @param descore_piston Descore piston
         */
        IntakeSubsystem(
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
        );


    };
}
}
}
}

#endif