#ifndef SHAREDDATA_HPP
#define SHAREDDATA_HPP

#include <mutex>
#include <condition_variable>

struct Pos{

    double x = 0;
    double y = 0;
    double heading = 0;

};

struct sendPacket{

    Pos pos;

};


struct sharedData {
    
    std::mutex mtx;
    
    bool restart = false;
    bool shutdown = false;
    
    std::condition_variable cv;

    sendPacket sendData;

    Pos OTOSpos;

    Pos kalmanPos;


};



#endif