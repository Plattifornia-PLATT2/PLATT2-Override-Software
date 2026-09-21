#include "platt2\robot\subsystems\externalLink\piLink.hpp"
#include "platt2\robot\subsystems\externalLink\serialPort.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

void piLink::linkLoop(){

    sendPacket dataBuffer;
    sendPacket zeroPacket;
    
    serialPort link(5);

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
    


}

        //piLink comlink;
        //comlink.linkLoop();