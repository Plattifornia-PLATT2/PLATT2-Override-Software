#ifndef SHAREDDATA_HPP
#define SHAREDDATA_HPP

#include <mutex>
#include <condition_variable>


struct sendPacket{

        double posX;
        double posY;
        double heading;

};




struct sharedData {
    
    std::mutex mtx;
    
    bool restart = false;
    bool shutdown = false;
    
    std::condition_variable cv;


    sendPacket linkData;





};



#endif