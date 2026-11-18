#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/elevator/elevator.hpp"
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

    //0=0, 90=M_PI_2, 180=M_PI, 270=M+PI_2+M_PI
    //subsystems.holonomicControlSubsystem->tauTest();
        TrajConstraints lim; 
        lim.maxVel = 45; 
        lim.maxAccel = 80; 
        lim.maxOmega = 2; 
        lim.maxAlpha = 6;
    
    //Spin toggle to color
    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::IN);
    pros::delay(500);
    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::STOP);

    //Score Matchload
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::MIDDLE_EXTENSION);
    pros::delay(400);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::SCORE);
    subsystems.holonomicControlSubsystem->moveToPoint({63.5,7.5,M_PI_2}, {56.5,23.5,M_PI}, 0, 0, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(750);
    subsystems.armSubsystem->toggleClaw();

    //Move 
    subsystems.holonomicControlSubsystem->moveToPoint({57.0,23.5,0}, {12.0,14.0,M_PI}, 0, 0, lim);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::LOAD);
    //subsystems.holonomicControlSubsystem->moveToPoint({75.0,12.5,M_PI}, {13.25,13.0,M_PI}, 0, 0, lim);
    subsystems.armSubsystem->toggleClaw();
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::MIDDLE_EXTENSION, 750);
    subsystems.holonomicControlSubsystem->moveToPoint({12.0,14.0,0}, {38.0,24.25,0}, 0, M_PI, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(250);
    subsystems.armSubsystem->toggleClaw();


    

    //subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {0,36,M_PI_2}, M_PI_2, M_PI_2, lim);
    //subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {-48,60,M_PI_2+M_PI}, M_PI_2, M_PI_2+M_PI, lim);
        //subsystems.holonomicControlSubsystem->tauTest();
}

}
}