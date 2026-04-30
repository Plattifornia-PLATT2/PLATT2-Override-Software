#ifndef XDRIVE_HPP
#define XDRIVE_HPP

#include "platt2/robot/subsystems/holonomicDrive/IHolonomicDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"

#include <memory>
#include <vector>

/**
 * @brief Namespace containing all PLATT2 library code.
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
 * @brief Class to control an X Drive chassis
 * @authors Dominic Young and Logan Wolf
 */
class XDrive : public IHolonomic {
    private:
    /**
     * @brief Vector containing all the robot's X Drive modules
     * 
     */
    std::vector<std::unique_ptr<XDriveModule>> drive_modules; 

    public:
        /**
         * @brief Moves the X Drive chassis based on the given movement vector.
         * 
         * @param v The movement vector to apply to the chassis.
         */
        void moveVector(MovementVector v);

        /**
         * @brief Turns the X Drive chassis to the specified heading.
         * 
         * @param heading The target heading to turn to.
         */
        void turnToHeading(double heading);

        /**
         * @brief Construct a new XDrive object
         * 
         * @param drive_modules Vector containing all the robot's X Drive modules
         */
        XDrive(std::vector<std::unique_ptr<XDriveModule>> drive_modules);
};

}
}
}
}

#endif