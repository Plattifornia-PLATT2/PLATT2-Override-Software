#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"

namespace platt2{
namespace auton{

void PinkCompAuton::init(const robot::RobotSubsystems& subsystems) {

    this->subsystems = subsystems;
}

std::string PinkCompAuton::getName() const {
    return AUTON_NAME;
}
void PinkCompAuton::run() {


    subsystems.holonomicControlSubsystem->tauTest();
    
}

}
}