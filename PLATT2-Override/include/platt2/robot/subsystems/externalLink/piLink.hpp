#pragma once

#include "platt2\hal\serialPort.hpp"

#include "pros/imu.hpp"
#include "pros/rtos.hpp"
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
namespace piLink{

class PiLinkSubsystem{

    public:

    double getXPos();
    double getYPos();
    double getHPos();
    
    
    void linkLoop();

    PiLinkSubsystem(std::unique_ptr<hal::serialPort> port);



    private:
    pros::Task piLinkTask;
    std::unique_ptr<hal::serialPort> link;
    pros::Mutex packetMtx;
    hal::serialPort::sendPacket linkData;
         
    

};}}}}