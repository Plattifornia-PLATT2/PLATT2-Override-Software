#include "platt2/MatchController.hpp"

namespace platt2{

void MatchController::init(){
    menu.build();

    while(menu.menuInUse()){
        pros::delay(10);
    }

    menu::FullConfig roboConfig =  menu.getFullConfig();
    std::cout<<roboConfig.robotConfig<<std::endl;
    if(roboConfig.robotConfig == robot::PINK) {
        std::unique_ptr<config::PinkConfig> pink{std::make_unique<config::PinkConfig>()};
        config = std::move(pink);
    }
    else if (roboConfig.robotConfig == robot::PURPLE) {
        std::unique_ptr<config::PurpleConfig> purple{std::make_unique<config::PurpleConfig>()};
        config = std::move(purple);
    }

    robot = config->buildRobot(roboConfig.autonConfig, roboConfig.driverProfile, roboConfig.allianceColor);
    if(robot){
        robot->init();
    }
    pros::screen::erase();
}

void MatchController::driveControl(){
    pros::screen::erase();
   robot->driverControl();
}

void MatchController::autonControl(){
   pros::screen::erase();
   robot->autonControl();
}
}