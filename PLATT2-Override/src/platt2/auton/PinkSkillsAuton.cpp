#include "platt2/auton/PinkSkillsAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/colorSort/colorSort.hpp"
#include "pros/rtos.hpp"

namespace platt2{
namespace auton{

void PinkSkillsAuton::init(const robot::RobotSubsystems& subsystems) {

    this->subsystems = subsystems;

}


std::string PinkSkillsAuton::getName() const {
    return AUTON_NAME;
}

void PinkSkillsAuton::run() {
        
}

}
}