#ifndef SHAREDDATA_HPP
#define SHAREDDATA_HPP

#include <mutex>
#include <condition_variable>

struct sharedData {
    
    std::mutex mtx;
    
    bool restart = false;
    bool shutdown = false;
    
    
    int counter = 0;
    std::condition_variable cv;








};

#endif