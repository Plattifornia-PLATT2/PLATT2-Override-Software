#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "MovementVector.hpp"
#include <algorithm>
#include <vector>
#include "math.h"


namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    XDrive::XDrive(std::vector<std::unique_ptr<XDriveModule>> drive_modules){
        this->drive_modules = (std::move(drive_modules));
    }

    void XDrive::moveVector(MovementVector v){ 

        std::vector<double> pV;
        pV.reserve(drive_modules.size());
        double scale = 1.0;

        for (auto& m : drive_modules) {
            double s = m->desiredModuleVel(v);
            pV.push_back(s);
            if (std::abs(s) > m->maxModuleVel())
                scale = std::min(scale, m->maxModuleVel() / std::abs(s));
        }
        
        for (size_t i = 0; i < drive_modules.size(); ++i)
            
            if (v.useVel){
                drive_modules[i]->setModuleVel(pV[i] * scale);
            }else {
                drive_modules[i]->setModuleVoltageVel(pV[i] * scale);
            }
        }


    MovementVector XDrive::fieldToRobotOrientation(MovementVector v, double heading){



        double a = heading - M_PI / 2.0;
        double c = cos(a), s = sin(a);
        double vx =  c * v.vx + s * v.vy;
        double vy = -s * v.vx + c * v.vy;
        return {vx, vy, v.w};

    }

}
     
} // class

}} // namespaces
