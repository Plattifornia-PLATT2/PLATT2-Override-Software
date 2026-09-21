#include "piLink.hpp"


void piLink::linkLoop(std::stop_token stopToken, sharedData& shared){

    sendPacket dataBuffer;
    
    UartPort link("/dev/ttyAMA0", B115200);

    while (!stopToken.stop_requested()) {

        
        //dataBuffer.
        


        
        bool recived = link.receiveLine(dataBuffer, 20);

        if (recived){

            bool sent = link.sendLine(dataBuffer);
            continue;

        }
        
        
        
        
    
    
    }

    int counter = 0;

    {
        std::lock_guard<std::mutex> lock(shared.mtx);
            //counter = shared.counter;
    }
    {
        std::lock_guard<std::mutex> lock(shared.mtx);
            //shared.counter++;
    }

    std::cout << "Hello " << std::endl;
    
    if (counter != 3){
        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            shared.restart = true;
            shared.cv.notify_one();
        }

    }
   
    
    
    
}



//std::cout << "Hello " << std::endl;
