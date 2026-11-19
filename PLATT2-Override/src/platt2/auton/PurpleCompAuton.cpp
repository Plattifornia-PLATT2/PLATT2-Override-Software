#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace auton{

void PurpleCompAuton::init(const robot::RobotSubsystems& subsystems) {

    this->subsystems = subsystems;

}


std::string PurpleCompAuton::getName() const {
    return AUTON_NAME;
}


void PurpleCompAuton::run() {
        TrajConstraints lim; 
        lim.maxVel = 45; 
        lim.maxAccel = 80; 
        lim.maxOmega = 2; 
        lim.maxAlpha = 6;
    subsystems.holonomicControlSubsystem->moveToPoint({42.5,7.5,M_PI_2}, {35.0,22.0,M_PI}, 0, 0, lim);


}

}
}