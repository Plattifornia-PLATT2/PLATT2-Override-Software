#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include <math.h>

namespace platt2{
namespace auton{

void PinkCompAuton::init(const robot::RobotSubsystems& subsystems) {

    this->subsystems = subsystems;
}

std::string PinkCompAuton::getName() const {
    return AUTON_NAME;
}
void PinkCompAuton::run() {


    //subsystems.holonomicControlSubsystem->tauTest();

        TrajConstraints lim; 
        lim.maxVel = 40; 
        lim.maxAccel = 40; 
        lim.maxOmega = 2; 
        lim.maxAlpha = 6;

    subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {18,-24,M_PI+M_PI_2}, M_PI_2, M_PI+M_PI_2, lim);

    
}

}
}