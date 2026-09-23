#ifndef SHAREDDATA_HPP
#define SHAREDDATA_HPP

#include <mutex>
#include <condition_variable>


struct sendPacket{

        double posX = 0;
        double posY = 0;
        double heading = 0;

};

struct Pos{

    double x = 0;
    double y = 0;


};


struct sharedData {
    
    std::mutex mtx;
    
    bool restart = false;
    bool shutdown = false;
    
    std::condition_variable cv;

    sendPacket linkData;

    double x = 0;
    double y = 0;



};



#endif