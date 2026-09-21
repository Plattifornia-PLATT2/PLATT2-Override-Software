#pragma once


#include "pros/rtos.hpp"


class piLink{

    public:

    struct sendPacket{

            double posX = 0;
            double posY = 0;
            double heading = 0;

    }; 
    
    pros::Mutex packetMtx;
    sendPacket linkData;
    void linkLoop();




    private:

         



};