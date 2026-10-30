
#include "platt2\hal\serialPort.hpp"
#include "platt2\robot\subsystems\externalLink\piLink.hpp"
#include "pros/rtos.hpp"
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

    while (true) {
        
        bool sent = link->sendLine(zeroPacket);
        
        bool recived = link->receiveLine(dataBuffer, 100);


        if (recived){

            packetMtx.take();
            linkData = dataBuffer;
            packetMtx.give();
            
        }

        pros::delay(10);

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

double PiLinkSubsystem::getHPos(){

    packetMtx.take();
    double out = linkData.heading;
    packetMtx.give();
    return out;
}




}}}}

        //piLink comlink;
        //comlink.linkLoop();