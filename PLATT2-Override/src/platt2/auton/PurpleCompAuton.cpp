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

        subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::IN);
    pros::delay(500);
    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::STOP);

    //Score Matchload
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ZERO_PIN);
    pros::delay(400);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::SCORE_LOW);
    subsystems.holonomicControlSubsystem->moveToPoint({63.5,7.5,M_PI_2}, {56.5,23.5,M_PI}, 180, 180, lim);
}
    
}
}