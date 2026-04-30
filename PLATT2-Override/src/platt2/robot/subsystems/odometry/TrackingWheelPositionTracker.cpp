#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"
#include "OdometryPosition.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>

namespace platt2{
namespace robot{
namespace subsystems{
namespace odometry{

    Position TrackingWheelPositionTracker::getPos(){
        return current_position;
    }

    void TrackingWheelPositionTracker::setPos(Position pos){
        current_position.x = pos.x;
        current_position.y = pos.y;
        current_position.heading = pos.heading*(M_PI/180);
        imu->set_heading(360 - pos.heading);
    }

    double TrackingWheelPositionTracker::getX(){
        return current_position.x;
    }

    double TrackingWheelPositionTracker::getY(){
        return current_position.y;
    }

    double TrackingWheelPositionTracker::getHeading(){
        return current_position.heading;
    }

    void TrackingWheelPositionTracker::setOffsets(double x_offset, double y_offset){
        this->x_offset = x_offset;
        this->y_offset = y_offset;
    }

    void TrackingWheelPositionTracker::init(){
        pros::delay(500);
        x_wheel->init();
        y_wheel->init();
        imu->reset();

        while(imu->is_calibrating()){
            pros::delay(10);
        }
    }

    double TrackingWheelPositionTracker::getVelocity(){
        return this->velocity;
    }

    double TrackingWheelPositionTracker::getAngularVelocity(){
        return this->angular_velocity;
    }

    void TrackingWheelPositionTracker::updatePosition(){
        pros::delay(1000);

        current_position.heading  = ((360 - imu->get_heading())*M_PI/180);
        double oldTheta = current_position.heading ;
        double dTheta = (current_position.heading  - oldTheta);

        double newX = 0;//x_wheel->getPosition();
        double oldX = newX;
        double dX = newX - oldX;

        double newY = y_wheel->getPosition();
        double oldY = newY;
        double dY = newY - oldY;

        double localX = 0;
        double localY = 0;

        double globalX = 0;
        double globalY = 0;

        
        while(true){
            //print statments for debugging
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1,"X Pos: %.3f", current_position.x);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2,"Y Pos: %.3f", current_position.y);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3,"Heading: %.2f", current_position.heading*180/M_PI);

            //get new values and calculate deltas
            newX = 0;//x_wheel->getPosition();
            newY = y_wheel->getPosition();
            this->current_position.heading  = ((360 - imu->get_heading())*M_PI/180);

            dX = newX - oldX;
            dY = newY - oldY;
            dTheta = this->current_position.heading - oldTheta;

            // adjust deltas for angle wraparound
            if(dTheta > M_PI || dTheta < -M_PI){
                dTheta = -1 * sgn(dTheta) * (2*M_PI - std::abs(dTheta));
            }

            this->velocity = sqrt(pow(dX, 2) + pow(dY, 2))/0.01;
            this->angular_velocity = dTheta/0.01;

            // save new values as old values for next loop
            oldX = newX;
            oldY = newY;
            oldTheta = this->current_position.heading;
            // apply tracking wheel odometry algorithm to calculate local and global position deltas
            localX = dX - (x_offset*dTheta);
            localY = dY + (y_offset*dTheta);

            localX = (1-(pow(dTheta, 2)/24))*localX;
            localY = (1-(pow(dTheta, 2)/24))*localY;

            globalX = localY * cos(oldTheta-dTheta/2) - localX * sin(oldTheta-dTheta/2);
            globalY = localY * sin(oldTheta-dTheta/2) + localX * cos(oldTheta-dTheta/2);

            // if the global deltas are NaN, skip the update and try again in the next loop iteration
            if (std::isnan(globalX) or std::isnan(globalY)){
                pros::delay(10);
                continue;
            }
            // update current position with global deltas
            this->current_position.x += globalX;
            this->current_position.y += globalY;
 
            pros::delay(10);
        }
    }

    TrackingWheelPositionTracker::TrackingWheelPositionTracker(
        std::unique_ptr<hal::TrackingWheel> x_wheel,
        std::unique_ptr<hal::TrackingWheel> y_wheel,
        std::unique_ptr<pros::IMU> imu
    )
        : x_wheel(std::move(x_wheel)),
          y_wheel(std::move(y_wheel)),
          imu(std::move(imu)) 
    {

    }

}}}}