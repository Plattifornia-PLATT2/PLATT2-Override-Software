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
        std::unique_ptr<pros::Motor> frontMotor
    ) :
        frontMotor{std::move(frontMotor)}
    
    {
        frontMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    void IntakeSubsystem::moveIntake(IntakeDirection direction)
    {
        if(direction == OUT)
        {
            frontMotor->move(intakeVoltage);
        }   
        else if (direction == IN) 
        {
            frontMotor->move(-intakeVoltage);
        }
        else if (direction == STOP)
        {
            frontMotor->move(0);
        }
    }

    
}
}
}
}