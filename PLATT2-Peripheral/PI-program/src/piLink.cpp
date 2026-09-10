#include "piLink.hpp"

#include <iostream>
#include <mutex>


void piLink(sharedData& shared){




    std::cout << "Hello " << std::endl;

    {
        std::lock_guard<std::mutex> lock(shared.mtx);
            shared.counter++;
    }

    std::cout << shared.counter << std::endl;
}