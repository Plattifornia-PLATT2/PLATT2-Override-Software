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

    enum IntakeDirection{
        STOP = 0,
        IN = 1,
        OUT = 2
    };

    /**
    @brief A class to control the Intake subsytem
    @author Dominic Young
    */
    class IntakeSubsystem{

        private:
        uint32_t intakeVoltage{127};

        std::unique_ptr<pros::Motor> frontMotor;

        public:
        IntakeSubsystem(
            std::unique_ptr<pros::Motor> frontMotor
        );

        void moveIntake(IntakeDirection direction);


    };
}
}
}
}

#endif