#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/arm/arm.hpp"
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
        lim.kStraightLen = 6;

    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::IN);
    pros::delay(500);
    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::STOP);

    //Score Matchload
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::START);
    pros::delay(400);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::SCORE_LOW);
    subsystems.holonomicControlSubsystem->moveToPoint({63.75,7.5,M_PI_2}, {56.5,23.25,M_PI}, M_PI*1.5, 0, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(650);
    subsystems.armSubsystem->toggleClaw();

    //Grab off wall and score
    lim.maxVel = 45; 
    lim.maxAccel = 40; 
    lim.kStraightLen = 5;
    lim.maxOmega = 2; 
    lim.maxAlpha = 2;
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::PURPLE_LOW);
    subsystems.holonomicControlSubsystem->moveToPoint({56.5,23.50,0}, {51.25,10.25,M_PI+M_PI/3}, 0, M_PI/3, lim);
    subsystems.armSubsystem->toggleClaw();
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ONE_PIN);
    lim.kStraightLen = 1;
    subsystems.holonomicControlSubsystem->moveToPoint({51,11,M_PI*.25}, {52.5,14.25,M_PI*.75}, M_PI/3, M_PI*1.67, lim);
    //subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::LOAD);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ZERO_PIN);
    pros::delay(350);
    subsystems.armSubsystem->toggleClaw();
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::PURPLE_LOW);

    //Cross field to score to other side
    lim.maxOmega = 2; 
    lim.maxAlpha = 6;
    lim.kStraightLen = 4;
    lim.maxAccel = 40; 
    subsystems.holonomicControlSubsystem->moveToPoint({52.5,14.25,M_PI*1.6}, {87,40.50,M_PI*.25}, M_PI*1.67, M_PI*1.25, lim);
    subsystems.armSubsystem->toggleClaw();
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ONE_PIN);
    lim.kStraightLen = 4;
    subsystems.holonomicControlSubsystem->moveToPoint({87.0,41,M_PI*1.25}, {84.0,23.25,0}, M_PI*1.25, M_PI, lim);
    //subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::SCORE_LOW);
    pros::delay(500);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(450);
    subsystems.armSubsystem->toggleClaw();
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::PURPLE_LOW);

    lim.kStraightLen = 5;
    subsystems.holonomicControlSubsystem->moveToPoint({84.0,23.5,M_PI}, {90.25,9.5,M_PI*1.66}, M_PI, M_PI*.66, lim);
    
    subsystems.armSubsystem->toggleClaw();
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::THREE_PIN);
    subsystems.holonomicControlSubsystem->moveToPoint({89.75,10.5,M_PI*.75}, {87.60,30.25,M_PI*1.75}, M_PI*.66, M_PI*.75, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::TWO_PIN);
    pros::delay(350);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::PURPLE_LOW);
    subsystems.armSubsystem->toggleClaw();

    lim.kStraightLen = 1;
    lim.maxAccel = 80; 
    lim.maxVel = 60;
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    subsystems.holonomicControlSubsystem->moveToPoint({87.75,30.65,M_PI*.75}, {72,48,M_PI*75}, M_PI*.75, M_PI*.75, lim);

    


}
    
}
}