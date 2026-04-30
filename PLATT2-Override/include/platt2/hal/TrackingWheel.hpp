#ifndef TRACKING_WHEEL_HPP
#define TRACKING_WHEEL_HPP

#include "pros/rotation.hpp"

#include <memory>
#include <cmath>

/**
 * @brief Namespace for PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for hardware abstraction layer related code.
 * @authors PLATT2 Development team.
 */    
namespace hal{

    /**
     * @brief A class representing an odometry tracking wheel.
     * @author Dominic Young
     */
    class TrackingWheel{
        private:

        const int CENTIDEGREES_CONVERSION_FACTOR = 36000;

        std::unique_ptr<pros::Rotation> rotation_sensor;

        double wheel_diameter;
        double wheel_circumference;

        public:
        /**
         * @brief Gets the current position of the tracking wheel.
         * 
         * @return double The position in inches.
         */
        double getPosition();

        /**
         * @brief Sets the current position of the tracking wheel.
         * 
         * @param pos The new position in inches.
         */
        void setPosition(double pos);

        /**
         * @brief Initializes the tracking wheel sensor.
         */
        void init();

        /**
         * @brief Constructs a new Tracking Wheel object.
         * 
         * @param rot_sensor The rotation sensor used to measure wheel rotation.
         * @param offset The offset of the tracking wheel from the robot's center in inches.
         * @param wheel_diameter The diameter of the tracking wheel in inches.
         */
        TrackingWheel(std::unique_ptr<pros::Rotation> rot_sensor, double wheel_diameter);
    };

}}

#endif