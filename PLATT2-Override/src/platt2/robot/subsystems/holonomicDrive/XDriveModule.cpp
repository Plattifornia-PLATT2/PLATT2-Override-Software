#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"
#include "MovementVector.hpp"
#include "platt2/helperFunctions.h"
#include "pros/abstract_motor.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <ostream>


namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{


double XDriveModule::desiredModuleVel(const MovementVector& v) {
    double linear = -sin(v.theta + getPhi()) * sgn(sin(getTheta())) * v.r;
    double rot    = -sgn(cos(getTheta())) * v.w * getRadius();
    return linear + rot;   // in/s
}

double XDriveModule::maxModuleVel()  {
    
    double gear;
    
    switch(module_motors.getMotorGearing()){

        case pros::MotorGears::blue: 
        gear = 600;
        break;

        case pros::MotorGears::red: 
        gear = 100;
        break;

        case pros::MotorGears::green: 
        gear = 200;
        break;

        default:
        gear = 0;
        break;
    }

    return ((gear)*getGearRatio()) / 60.0 * M_PI * getWheelDiameter();

}

void XDriveModule::setModuleVel(double ips) {
    // Option A: motor-side velocity PID
    double rpm = ips / (M_PI * getWheelDiameter()) * 60.0 / getGearRatio();
    module_motors.setVelocity(rpm);   // new method on MotorGroup

    // Option B: voltage feedforward, using your existing setVoltage (-1..1)
    // double u = in_per_s / maxSpeed();                    // kV term, normalized
    // module_motors.setVoltage(sgn(u) * cfg.kS + u * (1.0 - cfg.kS));
}







XDriveModule::XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double theta, double phi, double radius, double wheelDiameter, double gearRatio){
    module_motors.addMotor(motor_l);
    module_motors.addMotor(motor_r);
    module_motors.setBrakeMode(pros::MotorBrake::brake);
    this->theta = theta;
    this->phi = phi;
    this->radius = radius;
    this->wheelDiameter = wheelDiameter;
    this->gearRatio = gearRatio;
    
}

double XDriveModule::getTheta(){

    return this->theta;

}
double XDriveModule::getPhi(){

    return this->phi;

}
double XDriveModule::getRadius(){

    return this->radius;

}
double XDriveModule::getWheelDiameter(){

    return this->wheelDiameter;

}
double XDriveModule::getGearRatio(){

    return this->gearRatio;

}



}}}} // namespaces