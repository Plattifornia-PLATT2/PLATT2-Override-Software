#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "platt2/robot/subsystems/odometry/IPositionTracker.hpp"
#include "platt2/hal/OpticalTrackingSensor.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"

#include "pros/imu.hpp"

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
 * @brief Namespace for odometry subsystem related code.
 * @author Dominic Young
 */
namespace odometry{
    
    /**
     * @brief Odometry subsystem class. 
     * @author Dominic Young
     */
    class Odometry{
        private:

        std::unique_ptr<IPositionTracker> position_tracker;
        pros::Task m_trackingTask;

        public:
        /**
         * @brief Gets the current position of the robot with x, y, and heading values.
         * 
         * @return OdometryPosition Struct containing x, y, and heading of the robot.
         */
        Position getPos();

        /**
        * @brief Sets the position of the odometry system.
        */
        void setPos(Position pos);

        /**
         * @brief Gets the current x position of the robot.
         * 
         * @return double The x position in inches.
         */
        double getX();

        /**
         * @brief Gets the current y position of the robot.
         * 
         * @return double The y position in inches.
         */
        double getY();

        /**
         * @brief Gets the current heading of the robot.
         * 
         * @return double The heading in radians.
         */
        double getHeading();

        double getVelocity();
        double getAngularVelocity();

        /**
         * @brief Resets the current heading of the robot to zero.
         * 
         */

        /**
         * @brief Gets the heading from the VEX IMU sensor.
         * 
         * @return double Vex IMU heading in radians.
         */

        void initImu();

        void setOffsets(double x, double y);

        void startTracking();

        /**
         * @brief Construct a new Odometry object with a vex IMU sensor.
         * 
         * @param vex_imu The unique pointer to a vex IMU sensor object.
         */
        Odometry(std::unique_ptr<IPositionTracker> position_tracker);
    };

};
};
};
};
#endif