
#include "platt2\hal\serialPort.hpp"
#include "platt2\robot\subsystems\externalLink\piLink.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <memory>

namespace platt2{
namespace robot{
namespace subsystems{
namespace piLink{

static void taskLink(void *p) {
    reinterpret_cast<PiLinkSubsystem*>(p)->linkLoop();

}

PiLinkSubsystem::PiLinkSubsystem(

        std::unique_ptr<hal::serialPort> port


) : 

link{std::move(port)},

piLinkTask(taskLink, this)

{
}


void PiLinkSubsystem::linkLoop(){

    hal::serialPort::sendPacket dataBuffer;
    hal::serialPort::sendPacket zeroPacket;

    hal::serialPort::sendPacket startPacket = {1, 0,0};
    hal::serialPort::sendPacket testPacket = {1, 0,0};

    short count = 0;
    short previousCount = count;

    int loopCount = 0;

    while(true){

        bool recived = link->receiveLine(dataBuffer, 0);



        if (recived){

            count = dataBuffer.resetCount;
            loopCount++;
            if (loopCount == 1){

                previousCount = count;

            }
        }

        

        if (previousCount != count){break;}

        bool sent = link->sendLine(startPacket);
        
        pros::delay(100);

        previousCount = count;


    }
    

    while (true) {
        
        

        bool recived = link->receiveLine(dataBuffer, 0);
        


        if (recived){
            
            packetMtx.take();
            linkData = dataBuffer;
            packetMtx.give();     
            
        }
        pros::delay(7);

        bool sent = link->sendLine(zeroPacket);


    }

}


double PiLinkSubsystem::getXPos(){
    packetMtx.take();
    double out = linkData.posX;
    packetMtx.give();
    return out;
}

double PiLinkSubsystem::getYPos(){
    packetMtx.take();
    double out = linkData.posY;
    packetMtx.give();
    return out;
}

double PiLinkSubsystem::getResetCount(){

    packetMtx.take();
    double out = linkData.resetCount;
    packetMtx.give();
    return out;
}




}}}}

        //piLink comlink;
        //comlink.linkLoop();