#include "platt2/robot/subsystems/odometry/OTOSPositionTracker.hpp"
#include "OdometryPosition.hpp"
#include "platt2/robot/subsystems/externalLink/piLink.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>

namespace platt2{
namespace robot{
namespace subsystems{
namespace odometry{

    Position OTOSPositionTracker::getPos(){
        return current_position;
    }

    void OTOSPositionTracker::setPos(Position pos){

    }

    double OTOSPositionTracker::getX(){
        return current_position.x;
    }

    double OTOSPositionTracker::getY(){
        return current_position.y;
    }

    double OTOSPositionTracker::getHeading(){
        return current_position.heading;
    }
    
    double OTOSPositionTracker::getVelocity(){
        return this->velocity;
    }

    double OTOSPositionTracker::getAngularVelocity(){
        return this->angular_velocity;
    }

    void OTOSPositionTracker::setOffsets(double x_offset, double y_offset){
        this->x_offset = x_offset;
        this->y_offset = y_offset;
    }

    void OTOSPositionTracker::init(){

    }



    void OTOSPositionTracker::updatePosition(){



 
    }
    

    OTOSPositionTracker::OTOSPositionTracker(

    std::shared_ptr<subsystems::piLink::piLinkSubsystem> piLink_subsystem


    ) : piLink_subsystem(piLink_subsystem)
    {

    }


}}}}