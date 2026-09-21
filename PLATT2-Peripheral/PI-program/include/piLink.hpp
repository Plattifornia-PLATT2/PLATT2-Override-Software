#ifndef LINK_HPP
#define LINK_HPP

#include "sharedData.hpp"
#include "uartPort.hpp"
#include <stop_token>
#include <iostream>
#include <mutex>

class piLink{
    public:

        void linkLoop(std::stop_token, sharedData&);




};






#endif