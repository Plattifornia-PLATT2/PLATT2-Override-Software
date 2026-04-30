#include "platt2/hal/TrackingWheel.hpp"
#include <math.h>

namespace platt2{
namespace hal{

double TrackingWheel::getPosition(){
    return ((double)(rotation_sensor->get_position() / (double)CENTIDEGREES_CONVERSION_FACTOR) * wheel_circumference);
}

void TrackingWheel::setPosition(double pos){
    double rotations = pos / wheel_circumference;
    rotation_sensor->set_position(rotations / CENTIDEGREES_CONVERSION_FACTOR);
}

void TrackingWheel::init(){
    rotation_sensor->reset();
}

TrackingWheel::TrackingWheel(std::unique_ptr<pros::Rotation> rot_sensor, double wheel_diameter)
: 
rotation_sensor(std::move(rot_sensor)), 
wheel_diameter(wheel_diameter) 
{
    wheel_circumference = wheel_diameter * M_PI;   
}    

}} 