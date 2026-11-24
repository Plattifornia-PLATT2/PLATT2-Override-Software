#ifndef OTOS_HPP
#define OTOS_HPP

#include "utilities/OtosLinux.h"
#include "utilities/sfTk/sfTkLinuxI2C.h"
#include "utilities/sfTk/sfDevOTOS.h"
#include <stop_token>
#include <utilities/sharedData.hpp>

class OTOS {
    private:

        OtosLinux otos;

        sfTkLinuxI2C i2c {0x17};

        void calibrate();

        sfe_otos_pose2d_t pinkStartPos = {63.5,7.5,270};
        sfe_otos_pose2d_t purpleStartPos = {42.5,7.5,270};

    public:
        

        void sensorLoop(std::stop_token stopToken, sharedData& shared);

        void restart();

        void getPos();

        double getX();
        double getY();
        double getH();
        double getXstdDev();
        double getYstdDev();
        double getHstdDev();
        double getAcceleration();
        
        int selfTest();
        OTOS();

    };

#endif