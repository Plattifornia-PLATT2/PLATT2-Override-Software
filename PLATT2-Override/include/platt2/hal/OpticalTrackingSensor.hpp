#ifndef OPTICAL_TRACKING_SENSOR_HPP
#define OPTICAL_TRACKING_SENSOR_HPP

#include "pros/serial.hpp"
#include "pros/screen.hpp"
#include "pros/imu.hpp"

#include <memory>
#include <string>
#include <stdint.h>
#include <iostream>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for all hardware abstraction layer code.
 * @author Dominic Young
 */
namespace hal{

/**
 * @brief Class for interfacing with the sparkfun optical tracking odometry sensor over a serial interface
 * @author Dominic Young
 */
class OpticalTrackingSensor{

private:
    const int BAUD_RATE = 74880;

    pros::Serial m_serialInterface = pros::Serial(4, BAUD_RATE);

    std::unique_ptr<pros::IMU> vex_imu;

    pros::Task m_otosTask;

    double xOffset;

    double yOffset;

    double hOffset;

    double xPos;

    double yPos;

    double heading;

    double localHeading;

    /**
     * @brief Get the Bounded Heading of the VEX IMU sensor.
     * 
     * @return double current heading in radians from the VEX IMU sensor.
     */
    double getBoundedHeading();

public:

    /**
     * @brief Construct a new Optical Tracking Sensor object with given offset parameters
     * 
     * @param xOffset The starting X of the robot
     * @param yOffset The starting Y of the robot
     * @param hOffset The starting heading of the robot
     * @param vex_imu The VEX IMU sensor object
     */
    OpticalTrackingSensor(double xOffset, double yOffset, double hOffset, std::unique_ptr<pros::IMU> vex_imu);

    /**
     * @brief The main loop to read the data from the serial bus
     * 
     */
    void readData();

    /**
     * @brief Gets the current X position of the robot
     * 
     * @return double The current x position in inches
     */
    double getXPosition();

    /**
     * @brief Gets the current Y position of the robot
     * 
     * @return double The Y position in inches
     */
    double getYPosition();

    /**
     * @brief Gets the current heading of the robot from the OTOS sensor
     * 
     * @return double The current heading in radians from the OTOS sensor
     */
    double getHeading();

    /**
     * @brief Resets the heading on the VEX IMU
     * 
     */
    void resetHeading();

    /**
     * @brief Get the current heading from the VEX IMU degrees
     * 
     * @return double The current heading in degrees
     */
    double getVexHeading();
    
    /**
     * @brief Initalizes the VEX IMU sensor and sets the starting heading
     * 
     */
    void initVexImu();
};

} // namespace hal
} // namespace platt2
#endif