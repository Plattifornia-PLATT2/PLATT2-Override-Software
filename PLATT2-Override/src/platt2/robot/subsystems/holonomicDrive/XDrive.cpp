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

        double projection;
        double s;
        double test;

        std::vector<double> pV;

        for (auto& module : drive_modules) {
            
            projection = -sin(v.theta+(module->getPhi()))/sin((module->getTheta()));
            
            pV.push_back(std::abs(projection));
        }

        s = *std::max_element(pV.begin(), pV.end());
       
        v.normalization_scalar = s/v.r;

        for (auto& module : drive_modules){
            module->move_vector(v);
        }

         
    }

}
     
} // class

}} // namespaces
