#ifndef ELEVATOR_HPP
#define ELEVATOR_HPP

#include <cstdint>
#include <memory>
#include <cmath>
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
        ZERO_PIN,
        ONE_PIN,
        TWO_PIN,
        THREE_PIN,
        FULL_EXTENSION,
        INTAKE_POSITION

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
        void moveToPosition(ElevatorPosition pos, double delay=0);

        /**
         * @brief Gets the current position of the elevator in inches
         * 
         * @return std::int32_t The current elevator position in inches
         */
        double getElevatorPositionRadians();

        /**
         * @brief Background task for the elevator position. Runs in a pros::task.
         * 
         */
        void elevatorTaskLoop();

        void manualMove(int32_t velocity);

        void stop();                                 // main loop: sets intent

        bool isElevatorAtMax();
        
        private:
        uint32_t getPosInches(ElevatorPosition pos);

        const double FULL_LIFT_HEIGHT = 55;
        const double BOTTOM_LIFT_HEIGHT = 0;
        const double ZERO_PIN_HEIGHT = 15.5;
        const double ONE_PIN_HEIGHT = 27.5;
        const double TWO_PIN_HEIGHT = 27.5;
        const double THREE_PIN_HEIGHT = 34.5;
        const double INTAKE_LOAD_HEIGHT = 11.5;

        const double ELEVATOR_VELOCITY = 200;

        std::unique_ptr<pros::Motor> leftMotor;
        std::unique_ptr<pros::Motor> rightMotor;
        std::unique_ptr<pros::Rotation> rotationSensor;
        std::unique_ptr<robot::pid::PID> elevatorPID;

        ElevatorPosition currentPosition;

        enum class Mode { IDLE, MANUAL, POSITION };
        std::atomic<Mode>    mode{Mode::IDLE};
        std::atomic<int32_t> manualVelocity{0};
        std::atomic<double>  targetTicks{0};
        std::atomic<bool>    newTarget{false};
        std::atomic<double> delayTime{0};
        std::atomic<bool> isElevatorMax{false};
        std::unique_ptr<pros::Task> task;
        void run();

    };
}
}
}
}

#endif