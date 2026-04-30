#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "IPositionTracker.hpp"
#include "OdometryPosition.hpp"
#include <memory>
#include "TrackingWheelPositionTracker.hpp"
#include "pros/rtos.hpp"
#include <math.h>

namespace platt2 {

namespace robot {

namespace subsystems {

namespace odometry {

static void taskThunk(void *p) {
    reinterpret_cast<Odometry*>(p)->startTracking();
}

Position Odometry::getPos() {
  return position_tracker->getPos();
}

double Odometry::getX() { return position_tracker->getX(); }
double Odometry::getY() { return position_tracker->getY(); }
double Odometry::getHeading() { return position_tracker->getHeading(); }
void Odometry::initImu() {position_tracker->init();}

void Odometry::setPos(Position pos){
  position_tracker->setPos(pos);
}

void Odometry::startTracking(){
  position_tracker->updatePosition();
}

void Odometry::setOffsets(double x, double y){
  position_tracker->setOffsets(x, y);
}

double Odometry::getVelocity(){
  return position_tracker->getVelocity();
}

double Odometry::getAngularVelocity(){
  return position_tracker->getAngularVelocity();
}

Odometry::Odometry(std::unique_ptr<IPositionTracker> position_tracker):
m_trackingTask(taskThunk, this)
{
   this->position_tracker = std::move(position_tracker);
}

} // namespace odometry
} // namespace subsystems
} // namespace robot
} // namespace platt2