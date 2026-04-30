#ifndef XDRIVEMODULE_HPP
#define XDRIVEMODULE_HPP

#include "platt2/hal/MotorGroup.hpp"
#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

/**
 * @brief Namespace for all PLATT2 Library Code
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
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */
namespace holonomicDrive{

    /**
     * @brief A class to represent an X Drive module
     * @authors Dominic Young and Logan Wolf
     */
    class XDriveModule {
        private:
        /**
         * @brief The motor group containing the motors for the module
         * 
         */
        hal::MotorGroup module_motors;
        
        double theta;
        double phi;

        public:
        /**
         * @brief Moves the X Drive module based on the given movement vector.
         * 
         * @param v The movement vector to apply to the module.
         */
        void move_vector(MovementVector v);

        /**
         * @brief Get the angle between N the turning center and the center of the wheel
         * 
         * @return double The offset angle (theta) of the X Drive module
         */
        double getTheta();

        /**
         * @brief Get the angle between N the turning center and the direction the wheel is facing.
         * 
         * @return double The angle (phi) of the X Drive module
         */
        double getPhi();

        //double get_rpm();
        //double getAngleFromZero();

        /**
         * @brief Construct a new XDriveModule object
         * 
         * @param motor_l Top motor object
         * @param motor_r Bottom motor object
         * @param theta The angle between N the turning center and the center of the wheel
         * @param phi The angle between N the turning center and the direction the wheel is facing
         */
        XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double theta, double phi);
        // theta refers to the angle between N the turning center and the center of the wheel
        // phi refers to the angle between N the turning center and the direction the wheel is faceing 
    };
}
}
}
}

#endif