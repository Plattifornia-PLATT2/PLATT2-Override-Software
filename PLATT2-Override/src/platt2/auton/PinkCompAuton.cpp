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
        lim.maxAccel = 50; 
        lim.maxOmega = 2; 
        lim.maxAlpha = 6;
        lim.kStraightLen = 8;
    
    //Spin toggle to color
    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::IN);
    pros::delay(500);
    subsystems.intakeSubsystem->moveToggleMech(platt2::robot::subsystems::intake::STOP);

    //Score Matchload
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ZERO_PIN);
    pros::delay(400);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::SCORE_LOW);
    subsystems.holonomicControlSubsystem->moveToPoint({63.5,7.5,M_PI_2}, {56.5,23.5,M_PI}, 0, 0, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(750);
    subsystems.armSubsystem->toggleClaw();


    //Move to pin in front of matchload
    //subsystems.holonomicControlSubsystem->moveToPoint({56.5,23.5,0}, {90.0,10.5,0}, 0, 0, lim);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::LOAD);
    lim.maxAccel =30; 
    subsystems.holonomicControlSubsystem->moveToPoint({56.5,23.5,M_PI*.3}, {28.25,31.75,M_PI*1.15}, 0, M_PI*.25, lim); //Supposed to be 315
    subsystems.armSubsystem->toggleClaw();
    lim.maxAccel = 50; 
    lim.kStraightLen = .1;
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ONE_PIN);
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::SCORE_LOW);
    pros::delay(200);
    subsystems.holonomicControlSubsystem->moveToPoint({28.25,31.75,M_PI}, {40.5,32.25,M_PI*.75}, M_PI*1.75, M_PI*.75, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(350);
    subsystems.armSubsystem->toggleClaw();
    pros::delay(100);

    lim.maxVel = 45; 
    lim.maxOmega = 2; 
    lim.maxAlpha = 6;
    lim.kStraightLen = 8;

    //Matchload Single Pin
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::LOAD);
    subsystems.holonomicControlSubsystem->moveToPoint({40.5,32.25,M_PI*1.25}, {11.0,14.5,M_PI}, M_PI*1.25, 0, lim);
    pros::delay(500);
    subsystems.armSubsystem->toggleClaw();
    lim.maxAccel = 40; 
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::ZERO_PIN);
    subsystems.holonomicControlSubsystem->moveToPoint({11.0,14.5,0}, {24.0,40,M_PI*.5}, 0, M_PI*1.5, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(350);
    subsystems.armSubsystem->toggleClaw();

    lim.kStraightLen = 8;
    lim.maxOmega = 30; 
    lim.maxAlpha = 24;
    //Matchload Single Pin
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::LOAD);
    subsystems.holonomicControlSubsystem->moveToPoint({24.0,40,M_PI*1.55}, {11.0,14.5,M_PI}, M_PI, 0, lim);
    subsystems.armSubsystem->toggleClaw();
    pros::delay(500);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::THREE_PIN);
    subsystems.holonomicControlSubsystem->moveToPoint({11.0,14.5,0}, {38.0,24.75,0}, 0, M_PI, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_DOWN);
    pros::delay(250);
    subsystems.armSubsystem->toggleClaw();

    //Matchload Single Pin
    subsystems.armSubsystem->moveArmToPosition(platt2::robot::subsystems::arm::LOAD);
    subsystems.holonomicControlSubsystem->moveToPoint({38.0,24.75,M_PI}, {11.0,14.5,M_PI}, M_PI, 0, lim);
    subsystems.armSubsystem->toggleClaw();
    pros::delay(500);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::FULL_EXTENSION);
    subsystems.holonomicControlSubsystem->moveToPoint({11.0,14.5,0}, {38.0,24.75,0}, 0, M_PI, lim);
    subsystems.elevatorSubsystem->moveToPosition(platt2::robot::subsystems::elevator::THREE_PIN);
    pros::delay(250);
    subsystems.armSubsystem->toggleClaw();
    
    //38.0,24.25,0

    

    //subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {0,36,M_PI_2}, M_PI_2, M_PI_2, lim);
    //subsystems.holonomicControlSubsystem->moveToPoint({0,0,M_PI_2}, {-48,60,M_PI_2+M_PI}, M_PI_2, M_PI_2+M_PI, lim);
        //subsystems.holonomicControlSubsystem->tauTest();
}

}
}