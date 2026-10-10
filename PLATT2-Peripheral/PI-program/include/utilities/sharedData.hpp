#ifndef SHAREDDATA_HPP
#define SHAREDDATA_HPP

#include <mutex>
#include <condition_variable>

struct Pose {
    double x;
    double y;
    double theta; // (from -pi to pi)
};

struct PoseReading {
    Pose pose; 
    double std_x;
    double std_y;
    double std_theta; 
    double latency = 0.0; // in mili seconds
    bool newData = false;
};

struct sendPacket{

    Pose pos;

};

struct sharedData {
    
    std::mutex mtx;
    
    bool restart = false;
    bool shutdown = false;
    
    std::condition_variable cv;

    PoseReading OTOSPos;
    PoseReading camPos;

    Pose kalmanPos;

};



#endif