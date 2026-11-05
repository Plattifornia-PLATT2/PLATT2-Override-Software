#ifndef ELEVATOR_HPP
#define ELEVATOR_HPP

#include <cstdint>
#include <memory>
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "platt2/robot/pid/pid.hpp"

namespace platt2{

namespace robot{

namespace subsystems{

namespace elevator{

    enum ElevatorPosition{
        FULL_DOWN,
        MIDDLE_EXTENSION,
        FULL_EXTENSION

    };

    /**
     * @brief A class to control the Elevator on the robot.
     * 
     */
    class ElevatorSubsystem{
        public:

        /**
         * @brief Construct a new Elevator Subsystem object with the needed hardware
         * 
         * @param leftMotor Left motor for the elevator
         * @param rightMotor Right motor for the elevator
         * @param rotationSensor Rotation sensor on the elevator axle
         */
        ElevatorSubsystem(
            std::unique_ptr<pros::Motor> leftMotor,
            std::unique_ptr<pros::Motor> rightMotor,
            std::unique_ptr<pros::Rotation> rotationSensor,
            std::unique_ptr<robot::pid::PID> elevatorPID
        );

        /**
         * @brief Initalizes the elevator subsystem
         * 
         */
        void initElevator();

        /**
         * @brief Moves the elevator to the desired setpoint position
         * 
         * @param pos the enum for the desired position
         */
        void moveToPosition(ElevatorPosition pos);

        /**
         * @brief Gets the current position of the elevator in inches
         * 
         * @return std::int32_t The current elevator position in inches
         */
        std::int32_t getElevatorPositionInches();

        /**
         * @brief Background task for the elevator position. Runs in a pros::task.
         * 
         */
        void elevatorTaskLoop();

        pros::Task elevatorTask;
    
        private:
        uint32_t getPosInches(ElevatorPosition pos);

        const int32_t FULL_LIFT_HEIGHT = 30;

        std::unique_ptr<pros::Motor> leftMotor;
        std::unique_ptr<pros::Motor> rightMotor;
        std::unique_ptr<pros::Rotation> rotationSensor;
        std::unique_ptr<robot::pid::PID> elevatorPID;

        ElevatorPosition currentPosition;

    };
}
}
}
}

#endif