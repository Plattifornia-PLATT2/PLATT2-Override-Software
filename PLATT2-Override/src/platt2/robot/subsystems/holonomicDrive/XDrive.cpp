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
            drive_modules[i]->setModuleVel(pV[i] * scale);
        }

}
     
} // class

}} // namespaces
