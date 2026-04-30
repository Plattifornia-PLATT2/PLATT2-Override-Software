#ifndef TANKDRIVE_HPP
#define TANKDRIVE_HPP


/**
 * @brief Namespace containing all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
#include "platt2/hal/MotorGroup.hpp"
#include "pros/motors.hpp"
#include <memory>
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
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */

namespace tankDrive{

class TankDrive{
    
    
    
    
    public:
    class TankModule{
    
        private:    
           
        public: 
            hal::MotorGroup module_motors;
            TankModule(std::unique_ptr<pros::Motor>& motor_1, std::unique_ptr<pros::Motor>& motor_2, std::unique_ptr<pros::Motor>& motor_3, std::unique_ptr<pros::Motor>& motor_4);
    }; 
        struct MovementVector {
            double v;
            double w;
        };
        void moveVector(MovementVector v);
        TankDrive(std::vector<std::unique_ptr<TankModule>> drive_modules);
        std::vector<std::unique_ptr<TankModule>> drive_modules;

    private:        
    
        
};
}
}
}
}
#endif