#ifndef MOTOR_GROUP_HPP
#define MOTOR_GROUP_HPP

#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <memory>
#include <vector>

/**
 * @brief Namespace for all PLATT2 library code
 * @authors PLATT2 development team
 */
namespace platt2{

/**
 * @brief Namespace for all hardware abstraction layer code
 * @author Dominic Young
 */
namespace hal{

/**
 * @brief A class to control a group of motors
 * @author Dominic Young
 */
class MotorGroup{
    private:
    std::vector<std::unique_ptr<pros::Motor>> motors {};

    public:
    /**
     * @brief Set the Voltage of the motor
     * 
     * @param voltage The voltage to set from -127 to 127
     */
    void setVoltage(double voltage);

    /**
     * @brief Sets the velocity of the motor groups
     * 
     * @param velocity The velocity to set from 0 to 600
     */
    void setVelocity(double velocity);

    /**
     * @brief Sets the brake mode of the motor group
     * 
     * @param brakeMode The brake mode to set to
     */
    void setBrakeMode(pros::MotorBrake brakeMode);

     /**
     * @brief Construct a new Motor Group object
     * 
     * @param motor_1 The first motor in the group
     * @param motor_2 The second motor in the group
     * @param motor_3 The third motor in the group
     * @param motor_4 The fourth motor in the group
     */

    /**
     * @brief Adds a motor to the motor group
     * 
     * @param motor The motor to add
     */
    void addMotor(std::unique_ptr<pros::Motor> &motor);

    /**
     * @brief Sets the gearset type for the motor group.
     * 
     * @param gears The gearset to set to
     */
    void setMotorGearing(pros::MotorGear gears);

    /**
     * @brief Sets the zero position of the motor encoders
     * 
     */
    void setZeroPostion();

    /**
     * @brief Gets the current voltage the motors are set to
     * 
     * @return double The current voltage the motors are set to
     */
    double getVoltage();

    /**
     * @brief Gets the current velocity the motors are set to
     * 
     * @return double Velocity from -600 to 600
     */
    double getVelocity();

    /**
     * @brief Get the Motor object for the motor at the specific intex in the group;
     * 
     * @param index Index of the motor to get
     * @return pros::Motor& The found motor
     */
    pros::Motor& getMotor(int index);

    /**
     * @brief Copy overload 
     * 
     * @param rhs The group to copy from
     * @return MotorGroup& New motor group
     */
    MotorGroup& operator=(MotorGroup& rhs);
}; 
} // namespace hal
} // namespace platt2

#endif