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

void HolonomicControl::moveToPoint(double x_target, double y_target, double target_heading, double rSpeed, double wSpeed, double timeout) {

    target_heading = target_heading*M_PI/180;

    MovementVector motionVector;
    polar p;
    
    double x_error;
    double y_error;
    double angle_error;

    std::vector<double> rArray(40,1);
    std::vector<double> wArray(40,1);

    double startTime = pros::millis();
    double startPosX = odometry->getX();
    double startPosY = odometry->getY();

    double moveDistance = CtoP(x_target-startPosX, y_target-startPosY).r;

    double currentVel = 0;

    bool usePID = false;

    headingPID->resetPID();
    positionPID->resetPID();
 
    while (true){
        
        // calculate position errors
        x_error = x_target - odometry->getX();
        y_error = y_target - odometry->getY();  

        p = CtoP(x_error, y_error);

        angle_error = target_heading - (odometry->getHeading());

        // adjust angle error for wraparound
        if(angle_error > M_PI || angle_error < -M_PI){
            angle_error = -1 * sgn(angle_error) * (2*M_PI - std::abs(angle_error));
        }
        //set current velocity based on profile and position error
        currentVel = velocityProfile(moveDistance, p.r, currentVel, rSpeed, 1.5, usePID);
        motionVector.r = currentVel;
        motionVector.theta = p.theta - (odometry->getHeading()-M_PI/2);

        // adjust theta for wraparound
        if( motionVector.theta > M_PI ||  motionVector.theta < -M_PI){
             motionVector.theta = -1 * sgn( motionVector.theta) * (2*M_PI - std::abs( motionVector.theta));
        }

        // calculate rotational velocity from heading error and PID
        motionVector.w = std::clamp(headingPID->calculate(0, angle_error), -wSpeed, wSpeed);

        //add current rotational velocity to rolling average for determining if the motion is finished
        double rAve = rollAverage(std::abs(p.r), rArray);
        double wAve = rollAverage(std::abs(motionVector.w), wArray);

        // if both the linear and rotational velocities have been low for a little while, we can assume we've reached the target and stop the motion
        if (rAve < 0.4 && wAve < 00.02){break;}
        if (pros::millis()-startTime>timeout*1000){break;}
        
        // send the movement command to the drivetrain
        drivetrain->moveVector(motionVector);
        pros::delay(10);

    } 

    motionVector.r = 0;
    motionVector.w = 0;
    motionVector.theta = 0;
    drivetrain->moveVector(motionVector);

}

double HolonomicControl::velocityProfile(double totalDistance, double remainingDistance, double currentVel, double maxVel, double maxAccel, bool& usePID) {

    double distanceTravelled = totalDistance - remainingDistance;
    bool reachedMaxVel       = currentVel >= maxVel;
    bool pastHalfway         = distanceTravelled >= totalDistance * 0.5;

    if ((!reachedMaxVel && !pastHalfway) && !usePID ) {
    //if (false) {
        // --- Acceleration phase ---
        double max_accel_delta = maxAccel * 0.01;
        double output = std::clamp(currentVel + max_accel_delta, 0.0, maxVel);
        return output;

    } else {
        // --- Handoff to decel/approach controller ---
        usePID = true;
        return std::clamp((positionPID->calculate(remainingDistance, 0)), 0.0, maxVel);
    }
}

std::unique_ptr<pid::PID>& HolonomicControl::getHeadingPID(){

    std::unique_ptr<pid::PID>& out = this->headingPID;

    return out;


}





HolonomicControl::HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    this->positionPID = std::move(posPID);
    this->headingPID = std::move(headingPID);
}

}
}
}
}