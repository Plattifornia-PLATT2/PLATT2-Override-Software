#ifndef SHAREDDATA_HPP
#define SHAREDDATA_HPP

#include <mutex>

struct sharedData {
    std::mutex mtx;
    int counter = 0;
    
};

#endif