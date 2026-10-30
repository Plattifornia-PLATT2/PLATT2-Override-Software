#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/helperFunctions.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <math.h>
#include <memory>
#include <ostream>
#include <regex>
#include <utility>
#include <vector>

namespace platt2{
namespace robot{
namespace subsystems{
namespace holonomicDrive{


void HolonomicControl::tauTest(){


    pros::delay(500);
    uint32_t start = pros::millis();
    drivetrain->moveVector({-30, 0,0});


    while (pros::millis() - start < 2000) {
    std::cout<<pros::millis()-start<<","<<std::hypot(std::abs(odometry->getVelocity().x()), std::abs(odometry->getVelocity().y()))<<std::endl;

    pros::delay(20);
    
    }

    drivetrain->moveVector({0,0,0});


 
}




HolonomicControl::HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::shared_ptr<LQR> lqr)
{
    drivetrain = drive;
    odometry = odom;
    lqrController = lqr;

}

}}}}