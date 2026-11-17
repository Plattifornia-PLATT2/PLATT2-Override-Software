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
    double linear = -(v.vx * sin(getPhi()) + v.vy * cos(getPhi())) * sgn(sin(getTheta()));
    double rot    = -sgn(cos(getTheta())) * v.w * getRadius();
    return linear + rot;   // in/s
}



void XDriveModule::setModuleVel(double ips) {

    double rpm = ips / (M_PI * getWheelDiameter()) * 56.7 / getGearRatio();
    module_motors.setVelocity(rpm);   

}

double XDriveModule::maxModuleVel(){

    return 60;


}

void XDriveModule::setModuleVoltageVel(double ips) {
    
    const double Ks = 800;                       // mV, tune per wheel direction
    const double Kv = (12000.0 - Ks) / 60.0;      // mV per ips
    const double blend = 0.3;                     // ips, avoids a jump/chatter near zero

    double sgnSmooth = std::clamp(ips / blend, -1.0, 1.0);
    double output = Kv * ips + Ks * sgnSmooth;

    module_motors.setVoltage(std::clamp((int)std::lround(output), -12000, 12000));

    //module_motors.setVoltage(std::clamp((int)std::lround(ips * mvPerIps), -12000, 12000));
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