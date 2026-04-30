#ifndef TANKCONTROL_HPP
#define TANKCONTROL_HPP

#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include "platt2/robot/subsystems/tankDrive/tankDrive.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/pid/pid.hpp"
#include "platt2/helperFunctions.h"
#include "Eigen/Dense"
#include <memory>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */

/**
 * @brief Control class for holonomic drive systems.
 * @authors Dominic Young and Logan Wolf
 */
namespace tankDrive{

class TankControl{
public:
    /**
     * @brief Construct a new Holonomic Control object
     * 
     * @param drive X drive chassis to control
     * @param odom Odometry system for position tracking
     * @param posPID PID controller for position control
     * @param headingPID PID controller for heading control
     */
    TankControl(std::shared_ptr<TankDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID);

struct parameter {
    double qx     = 0.0;   
    double qy     = 5.0;   // Lower pull toward the line
    double qtheta = 45.0;  // VERY high penalty for heading (Damping)
    double rV     = 20000.0;  
    double rW     = 3000.0; // Massive penalty for steering to stop the "snapping"
};
//struct parameter {
//    double qx     = 0;   
//    double qy     = 0;   // Lower pull toward the line
//    double qtheta = 0;  // VERY high penalty for heading (Damping)
//    double rV     = 1000.0;  
//    double rW     = 400.0; // Massive penalty for steering to stop the "snapping"
//};

    /**
     * @brief Moves the holonomic drive to a specified point with a target heading.
     * 
     * @param x_target Target X value to move to.
     * @param y_target Target Y value to move to.
     * @param heading_target Target heading to achieve at the end of the movement.
     */
    void moveToPoint(odometry::Position target,bool reverse, double rSpeed, double scaler=4);
    void turnToHeading(double targetHeading, double maxAngularVel=0.3);

    /**
     * @brief Used for debugging purposes.
     */

private:
    std::shared_ptr<TankDrive> drivetrain;
    std::shared_ptr<odometry::Odometry> odometry;
    std::unique_ptr<pid::PID> positionPID;
    std::unique_ptr<pid::PID> headingPID;

    struct point {
        double x;
        double y;
    };


    struct waypoint {
        odometry::Position pos;
        double v; 
        double w;
        Eigen::Matrix<double, 2, 3> K;
    };

    parameter params;



    odometry::Position p0; 
    point p1; 
    point p2; 
    odometry::Position p3;

    double tangentAngle(double t);
    double x(double t);
    double y(double t);
    double dx(double t);    
    double dy(double t);
    double ddx(double t);
    double ddy(double t);
    
    double curvature(double t);

    Eigen::Matrix3d solveRiccati(const Eigen::Matrix3d& Ad,const Eigen::Matrix<double,3,2>& Bd, const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R, Eigen::Matrix3d& P);

    double trapezoidalVelocity(double t, double maxVel, double pathLength);
    double arcLength();
    std::vector<waypoint> generatePath(odometry::Position target, double scaler);
    double remainingPathDistance(std::vector<waypoint> path, odometry::Position currentPos);
    void advanceIndex(std::vector<waypoint>& path, odometry::Position current);
    
    double pathLength;
    int waypointIndex;
    bool finished;
    double maxVel;
    double maxAccel;
    bool reversed;

};
}}}}

#endif