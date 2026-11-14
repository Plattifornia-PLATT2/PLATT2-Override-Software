#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace auton{

void PurpleCompAuton::init() {

}


std::string PurpleCompAuton::getName() const {
    return AUTON_NAME;
}


void PurpleCompAuton::run() {
}

}
}