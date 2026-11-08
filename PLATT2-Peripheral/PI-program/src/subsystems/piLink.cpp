#include "subsystems/piLink.hpp"
#include "utilities/gpio.hpp"


void piLink::linkLoop(std::stop_token stopToken, sharedData& shared){

    sendPacket dataBuffer;
    sendPacket zeroPacket;
    
    //UartPort link("/dev/ttyAMA0", B115200);
    UartPort link("/dev/ttyS0", B115200);

    if (!link.open()) {
    std::cerr << link.lastError() << std::endl;   
    return;}

    GpioOut pin(19); 

    while (!stopToken.stop_requested()) {
        
        //{
        //std::lock_guard<std::mutex> lock(shared.mtx);
        //    dataBuffer.pos = shared.OTOSPos.pose;
        //}
        dataBuffer.pos = {6,9,0};

        pin.low();
        bool sent = link.sendLine(dataBuffer);  
        
        pin.high();
        bool recived = link.receiveLine(zeroPacket, 100);
        
        //if (recived) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }    
}



//std::cout << "Hello " << std::endl;
//if (counter != 3){
//        {
//        std::lock_guard<std::mutex> lock(shared.mtx);
//            shared.restart = true;
//            shared.cv.notify_one();
//        }
//
//    }