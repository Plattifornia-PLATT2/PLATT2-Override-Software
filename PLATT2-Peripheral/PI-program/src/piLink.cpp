#include "piLink.hpp"


void piLink::linkLoop(std::stop_token stopToken, sharedData& shared){

    sendPacket dataBuffer;
    sendPacket zeroPacket;
    
    UartPort link("/dev/ttyAMA0", B115200);

    if (!link.open()) {
    std::cerr << link.lastError() << std::endl;   
    return;}


    {
        std::lock_guard<std::mutex> lock(shared.mtx);
        shared.linkData.posX = 700;
    }

    while (!stopToken.stop_requested()) {
        
        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            dataBuffer = shared.linkData;
        }
        
        bool sent = link.sendLine(dataBuffer);

        
        bool recived = link.receiveLine(zeroPacket, 100);

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