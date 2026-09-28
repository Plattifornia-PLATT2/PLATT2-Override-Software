#ifndef OTOS_HPP
#define OTOS_HPP

#include "utilities/OtosLinux.h"
#include "utilities/sfTkLinuxI2C.h"
#include "utilities/sfTk/sfDevOTOS.h"
#include <stop_token>
#include <utilities/sharedData.hpp>

class OTOS {
        private:

        OtosLinux otos;

        sfTkLinuxI2C i2c {0x17};

        public:
        
        OTOS();

        void calibrate();

        void sensorLoop(std::stop_token stopToken, sharedData& shared);

        void getPos();

        double getX();
        double getY();
        double getH();


    };

#endif