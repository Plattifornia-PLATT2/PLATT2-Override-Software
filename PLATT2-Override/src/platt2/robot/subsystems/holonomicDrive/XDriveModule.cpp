#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"
#include "MovementVector.hpp"
#include "platt2/helperFunctions.h"
#include <cmath>


namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    double deadzone = 0.085;
 
void XDriveModule::move_vector(MovementVector v){


    double p = -(sin(v.theta+getPhi())/sin(getTheta())); // should be expanded for readability

    double m = ((p/v.normalization_scalar)*(1-std::abs(v.w))); // should be expanded for readability
    
    double rot = -sgn(cos(getTheta()))*v.w;

    double combined = m+rot;

    double output = sgn(combined)*deadzone + combined*(1-deadzone);
    //std::cout<<output<<std::endl;

    module_motors.setVoltage((output));
    
}

XDriveModule::XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double theta, double phi){
    module_motors.addMotor(motor_l);
    module_motors.addMotor(motor_r);
    this->theta = theta;
    this->phi = phi;
    
}

double XDriveModule::getTheta(){

    return this->theta;

}
double XDriveModule::getPhi(){

    return this->phi;

}


}}}} // namespaces