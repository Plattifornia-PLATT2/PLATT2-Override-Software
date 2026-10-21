#pragma once

#include "platt2\hal\serialPort.hpp"

#include "pros/rtos.hpp"
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

class piLinkSubsystem{

    public:

    
    pros::Mutex packetMtx;
    hal::serialPort::sendPacket linkData;
    void linkLoop();

    


    private:
    pros::Task piLinkTask;
         
    piLinkSubsystem();


};}}}}