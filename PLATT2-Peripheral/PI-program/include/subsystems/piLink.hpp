#ifndef LINK_HPP
#define LINK_HPP

#include "utilities/sharedData.hpp"
#include "utilities/uartPort.hpp"
#include <stop_token>
#include <iostream>
#include <mutex>

class piLink{
    public:

        void linkLoop(std::stop_token, sharedData&);




};






#endif