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
        lim.maxVel = 60; 
        lim.maxAccel = 80; 
        lim.maxOmega = 2; 
        lim.maxAlpha = 6;

    //subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {0,36,M_PI_2}, M_PI_2, M_PI_2, lim);
    subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {-48,60,M_PI_2+M_PI}, M_PI_2, M_PI_2+M_PI, lim);
        //subsystems.holonomicControlSubsystem->tauTest();
}

}
}