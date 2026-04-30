#ifndef TRACKING_WHEEL_POSITION_TRACKER_HPP
#define TRACKING_WHEEL_POSITION_TRACKER_HPP

#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include "platt2/hal/TrackingWheel.hpp"
#include "platt2/robot/subsystems/odometry/IPositionTracker.hpp"
#include "platt2/helperFunctions.h"

#include "pros/screen.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

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
 * @authors Doiminic Young
 */
namespace subsystems{

/**
 * @brief Namespace for odometry subsystem related code.
 * @authors Dominic Young & Logan Wolf
 */
namespace odometry{

    class TrackingWheelPositionTracker : public IPositionTracker{
        private:
        
        std::unique_ptr<hal::TrackingWheel> x_wheel;
        std::unique_ptr<hal::TrackingWheel> y_wheel;
        std::unique_ptr<pros::IMU> imu;

        Position current_position;
        

        double x_offset{};
        double y_offset{};
        double heading_offset{};
        double velocity{};
        double angular_velocity{};

        public:
        /**
         * @brief Gets the current position of the robot with x, y, and heading values.
         * 
         * @return OdometryPosition Struct containing x, y, and heading of the robot.
         */
        Position getPos() override;

        /**
        * @brief Sets the postion of the odometry subsystem.
        * @param OdometryPosition the position struct of the new position.
        */
        void setPos(Position pos) override;

        /**
         * @brief Gets the current x position of the robot.
         * 
         * @return double The x position in inches.
         */
        double getX() override;

        /**
         * @brief Gets the current y position of the robot.
         * 
         * @return double The y position in inches.
         */
        double getY() override;

        /**
         * @brief Gets the current heading of the robot.
         * 
         * @return double The heading in degrees.
         */
        double getHeading() override;

        double getVelocity() override;
        double getAngularVelocity() override;

        /**
         * @brief Sets the offsets for the position tracker.
         * 
         * @param offsets The struct containing new offset values.
         */
        void setOffsets(double x_offset, double y_offset) override;

        /**
        * @brief Initializes the position tracker.
        */
        void init() override;

        /**
         * @brief Updates the position of the robot.
        */
        void updatePosition() override; 

        /**
         * @brief Constructs a new Tracking Wheel Position Tracker object.
         * 
         * @param x_wheel The tracking wheel for the x axis.
         * @param y_wheel The tracking wheel for the y axis.
         * @param imu The IMU sensor for heading data.
         */
        TrackingWheelPositionTracker(
            std::unique_ptr<hal::TrackingWheel> x_wheel, 
            std::unique_ptr<hal::TrackingWheel> y_wheel, 
            std::unique_ptr<pros::IMU> imu
        );
    };

}}}}
#endif