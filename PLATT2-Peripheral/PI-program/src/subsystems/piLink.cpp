#include "subsystems/piLink.hpp"
#include "utilities/gpio.hpp"
#include "utilities/softSerialRx.hpp"


void piLink::linkLoop(std::stop_token stopToken, sharedData& shared){

    sendPacket dataBuffer;
    sendPacket zeroPacket;
    
    //UartPort link("/dev/ttyAMA0", B115200);
    UartPort link("/dev/ttyS0", B19200);

    if (!link.open()) {
    std::cerr << link.lastError() << std::endl;   
    return;}

    SoftSerialRx rx("/dev/gpiochip0", 17, 19200);   // RO wired to GPIO17
     if (!rx.open()) {
        std::cerr << rx.lastError() << std::endl;   
        return;}
    rx.setRealtimePriority(30); 
    std::cerr << rx.lastError() << std::endl; 

    GpioOut pin(19); 

    auto lastReport = std::chrono::steady_clock::now();
    int sentCount = 0;
    int recvCount = 0;

    while (!stopToken.stop_requested()) {
        
        zeroPacket.pos = {0,0,0};

        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            dataBuffer.pos = shared.OTOSPos.pose;
        }
        //dataBuffer.pos = {6,9,0};

                     // optional, needs root
            
        pin.high();
        link.sendLine(dataBuffer); 
        pin.low();
        //rx.clearBuffer();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        bool recived = rx.receiveLine(zeroPacket, 15);

        //sentCount++;
        //
        //if (recived) {
        //    recvCount++;
        //    //std::cout << zeroPacket.pos.theta << std::endl;
        //}
//
        //auto now = std::chrono::steady_clock::now();
        //if (now - lastReport >= std::chrono::seconds(1)) {
        //    double secs = std::chrono::duration<double>(now - lastReport).count();
        //    const auto& st = rx.stats();
        //    std::cout << "sent/s: " << sentCount / secs
        //              << "  recv/s: " << recvCount / secs
        //              << "  success: " << (sentCount ? 100.0 * recvCount / sentCount : 0.0) << "%"
        //              << "  bursts=" << st.bursts 
        //              << "  frames=" << st.frames
        //              << "  rejected=" << st.rejected
        //              << std::endl;
        //    sentCount = 0;
        //    recvCount = 0;
        //    lastReport = now;
        //}
        if (zeroPacket.pos.x == 1){
//
            std::cout << "recived reset command" << std::endl;
//
            {
            std::lock_guard<std::mutex> lock(shared.mtx);
                shared.restart = true;
                shared.cv.notify_all();
            }
//
        }

        //if (recived) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}

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