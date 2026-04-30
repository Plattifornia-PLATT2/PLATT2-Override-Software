#include "platt2/robot/subsystems/tankDrive/tankDrive.hpp"
#include <vector>
#include "math.h"


namespace platt2{

namespace robot{

namespace subsystems{

namespace tankDrive{

    TankDrive::TankDrive(std::vector<std::unique_ptr<TankModule>> drive_modules){
        this->drive_modules = (std::move(drive_modules));
    }

    void TankDrive::moveVector(MovementVector vec) {

        //TODO: needs min voltage Scaling 
        
        drive_modules[0]->TankDrive::TankModule::module_motors.setVelocity(vec.v-vec.w);
        drive_modules[1]->TankDrive::TankModule::module_motors.setVelocity(vec.v+vec.w);
       
    }

    TankDrive::TankModule::TankModule(std::unique_ptr<pros::Motor>& motor_1, std::unique_ptr<pros::Motor>& motor_2, std::unique_ptr<pros::Motor>& motor_3, std::unique_ptr<pros::Motor>& motor_4){
    module_motors.addMotor(motor_1);
    module_motors.addMotor(motor_2);
    module_motors.addMotor(motor_3);
    module_motors.addMotor(motor_4);
    
}}}}}

