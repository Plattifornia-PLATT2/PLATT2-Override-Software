
#include "platt2\hal\serialPort.hpp"
#include "platt2\robot\subsystems\externalLink\piLink.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace platt2{
namespace robot{
namespace subsystems{
namespace piLink{

static void taskLink(void *p) {
    reinterpret_cast<piLinkSubsystem*>(p)->linkLoop();

}

piLinkSubsystem::piLinkSubsystem(


) : 

piLinkTask(taskLink, this)

{
}


void piLinkSubsystem::linkLoop(){

    hal::serialPort::sendPacket dataBuffer;
    hal::serialPort::sendPacket zeroPacket;
    
    hal::serialPort link(5);

    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "working");

    while (true) {
        
        bool sent = link.sendLine(zeroPacket);
        
        bool recived = link.receiveLine(dataBuffer, 100);

        if (recived){
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "recived");

        }

        if (recived){

            packetMtx.take();
            linkData = dataBuffer;
            packetMtx.give();
            
        }

        pros::delay(10);

        packetMtx.take();
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, std::to_string(linkData.posX).c_str());
        packetMtx.give();
        
    }
    


}}}}}

        //piLink comlink;
        //comlink.linkLoop();