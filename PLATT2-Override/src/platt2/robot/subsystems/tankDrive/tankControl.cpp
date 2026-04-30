#include "platt2/robot/subsystems/tankDrive/tankControl.hpp"
#include "Eigen/src/Core/Reverse.h"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include "platt2/helperFunctions.h"
#include "Eigen/Dense"


namespace platt2{
namespace robot{
namespace subsystems{
namespace tankDrive{

void TankControl::moveToPoint(odometry::Position target, bool reverse, double maxV, double c) {

    reversed = reverse;

    uint32_t now = pros::millis();

    maxVel = maxV;
    maxAccel = 0.012; //TODO: add as parameter and tune

    TankDrive::MovementVector motionVector;

    target.heading = target.heading*(M_PI/180);
    target.heading = wrapHeading(target.heading);
  
    std::vector<waypoint> path = generatePath(target, c);
    
    for(const auto& wp : path) {
        std::cout << "("<<wp.pos.x << ", " << wp.pos.y <<") V:"<<wp.v<<std::endl;
    }

    waypointIndex = 0;
    finished = false;

    std::cout<<params.qtheta<<std::endl;

    double lookAheadDistance = 4; 


    std::vector<double> velArray(30,100);
    std::vector<double> angArray(30,100);
    std::vector<double> endArray(30,100);

    Eigen::Vector3d error;
    Eigen::Vector2d correction;
    

    while (!finished) {

        odometry::Position currentPos = odometry->getPos();  

        if (reversed){
            currentPos.heading = wrapHeading(currentPos.heading + M_PI);
        }

        advanceIndex(path, currentPos);

        double distToEnd = std::hypot(currentPos.x - target.x,
                                      currentPos.y - target.y);
        if (distToEnd < lookAheadDistance) {  // within 4 inches, hand off
            break;
        }

        const waypoint wp = path[waypointIndex];
        
        double cosRef = std::cos(wp.pos.heading);
        double sinRef = std::sin(wp.pos.heading);

        double dx = (wp.pos.x - currentPos.x);
        double dy = (wp.pos.y - currentPos.y);

        
        error(0) =  (reverse ? -1 : 1) *(cosRef * dx - sinRef * dy);             
        error(1) = -sinRef * dx + cosRef * dy;                        // lateral
        error(2) =  wrapHeading(wp.pos.heading - currentPos.heading);

        correction =  wp.K * error;

;
        if (std::isnan(correction(0)) || std::isnan(correction(1))) {
            correction << 0, 0;
        }

        motionVector.v = (reversed ? -1 : 1) *(wp.v + correction(0));
        motionVector.w = -(wp.w + correction(1));

        //std::cout << "Target: (" << wp.pos.x << ", " << wp.pos.y << ", " << wp.pos.heading*(180/M_PI) << ")" << std::endl;
        //std::cout << "Current: (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.heading*(180/M_PI) << ")" << std::endl;
        //std::cout << "Error (x, y, theta): (" << error(0) << ", " << error(1) << ", " << error(2)*(180/M_PI) << ") Correction (v, w): (" << correction(0) << ", " << correction(1) << ") Ref (v, w): (" << wp.v << ", " << wp.w << ")"<< std::endl;

        drivetrain->moveVector(motionVector);
        pros::Task::delay_until(&now, 10); 

        double avgVel = rollAverage(std::abs(odometry->getVelocity()), endArray);

        if (avgVel < 0.025) {
            std::cout << "Average velocity below threshold, finishing path." << std::endl;
            finished = true;
        }

    }
    std::cout << "Approaching final point..." << std::endl;
    positionPID->resetPID();
    headingPID->resetPID();

    maxVel = (reversed ? -1 : 1) *motionVector.v;

    while (true) {

        odometry::Position currentPos = odometry->getPos();

        double globalDx = target.x - currentPos.x;
        double globalDy = target.y - currentPos.y;

        double distToEnd =  std::cos((reversed ? target.heading + M_PI : target.heading)) * globalDx
                          + std::sin((reversed ? target.heading + M_PI : target.heading)) * globalDy;

        //std::cout << "Distance to end: " << distToEnd << std::endl;

        double headingError = wrapHeading(target.heading - currentPos.heading);

       //double v = (reversed ? 1 : -1) * std::clamp(positionPID->calculate(0, distToEnd), -maxV, maxV);
        double v = (reversed ? -1 : 1) *((maxVel)/(lookAheadDistance*3)) * distToEnd * (std::abs(distToEnd)<0.25 ? 0:1);
        double w = std::clamp(headingPID->calculate(0, headingError), -0.15, 0.15);

        double avgVel    = rollAverage(std::abs(odometry->getVelocity()),        velArray);
        double avgAngVel = rollAverage(std::abs(odometry->getAngularVelocity()), angArray);

        //std::cout<<"DistToEnd: "<<distToEnd<< "v: "<<v<<std::endl;

        if (avgVel < 0.1 && avgAngVel < 0.1) {
            break;
        }
        //std::cout<< " Heading Error: "  << headingError << " W: " << w << std::endl;
        motionVector.v = v * std::abs(std::cos(headingError));
        motionVector.w = w;
        drivetrain->moveVector(motionVector);
        pros::delay(10);
    }

    std::cout << "Finished moving to point." << std::endl;
    
    motionVector.v = 0;
    motionVector.w = 0;

    drivetrain->moveVector(motionVector);
}

void TankControl::advanceIndex(std::vector<waypoint>& path, odometry::Position current) {
            constexpr double kSwitchRadius = 2;
    while (waypointIndex < static_cast<int>(path.size()) - 1) {
        double globalDx = path[waypointIndex].pos.x - current.x;
        double globalDy = path[waypointIndex].pos.y - current.y;
        
        // Forward component in robot frame
        double forward = std::cos(current.heading) * globalDx
                       + std::sin(current.heading) * globalDy;
        double dist    = std::hypot(globalDx, globalDy);

        // Skip if behind robot OR within switch radius
        if (forward < 0 || dist < kSwitchRadius) {
            ++waypointIndex;
        } else {
            break;
        }
    
        }
    }

std::vector<TankControl::waypoint> TankControl::generatePath(odometry::Position endPos, double scaler) {
    
    double tankWidth = 14.0; //TODO: add as parameter and tune

    std::vector<waypoint> path;
    p0 = odometry->getPos();

    if (reversed) {
        p0.heading = wrapHeading(p0.heading + M_PI);
    }

    p3 = endPos;

    if (reversed) {
        p3.heading = wrapHeading(p3.heading + M_PI);
    }

    //double angleDiff = std::min(std::abs(endPos.heading+M_PI - p0.heading), 2 * M_PI - std::abs(endPos.heading+M_PI - p0.heading));

    p1.x = p0.x + 5*cos(p0.heading);
    p1.y = p0.y + 5*sin(p0.heading);
    p2.x = p3.x + scaler*cos(p3.heading+M_PI);
    p2.y = p3.y + scaler*sin(p3.heading+M_PI);

    pathLength = arcLength();

    double pointsPerInch = 3;//TODO: add as parameter and tune

    

    Eigen::Matrix3d m_Q = Eigen::Matrix3d::Zero();
        m_Q(0, 0) = params.qx;
        m_Q(1, 1) = params.qy;
        m_Q(2, 2) = params.qtheta;
    
    Eigen::Matrix2d m_R = Eigen::Matrix2d::Zero();
        m_R(0, 0) = params.rV;
        m_R(1, 1) = params.rW;   
    
    Eigen::Matrix3d P = m_Q * 100.0;
    
    int n = static_cast<int>(pathLength * pointsPerInch);
    std::vector<double> vels(n + 1);
    std::vector<double> arcLengths(n + 1, 0.0);
    double maxCornerAccel = 0.0025;

    // Precompute arc lengths
    
    for (int i = 1; i <= n; i++) {
        double t0 = (double)(i-1) / n;
        double t1 = (double)i / n;
        double tm = (t0 + t1) / 2.0;
        arcLengths[i] = (t1-t0)/6.0 * (std::hypot(dx(t0), dy(t0))
                                      + 4*std::hypot(dx(tm), dy(tm))
                                      + std::hypot(dx(t1), dy(t1)));
    }

    // PASS 1: forward — trapezoidal profile + curvature cap
    for (int i = 0; i <= n; i++) {
        double t = (double)i / n;
        double kappa = curvature(t);
        double maxCornerVel = (std::abs(kappa) > 1e-6)
                              ? std::sqrt(maxCornerAccel / std::abs(kappa))
                              : maxVel;
        vels[i] = std::min(trapezoidalVelocity(t, maxVel, pathLength), maxCornerVel);
    }

    // PASS 2: backward — ensure robot can decelerate in time for corners
    for (int i = n - 1; i >= 0; i--) {
        double ds_actual = arcLengths[i + 1];
        double vMax = std::sqrt(vels[i+1]*vels[i+1] + 2.0 * maxAccel * ds_actual);
        vels[i] =  std::min(vels[i], vMax);
    }

    for (int i = 0; i <= n; i++) {
        double t = (double)i / n;

        double v = vels[i];

        double omega = vels[i] * curvature(t);              // rad/s — goes into Ac
        double w     = omega * (tankWidth / 2.0);
             // wheel diff — goes into waypoint for drivetrain
        
        Eigen::Matrix3d Ac = Eigen::Matrix3d::Zero();
        Ac(0, 1) =   omega;
        Ac(1, 0) =  -omega;
        Ac(1, 2) =  v;

        Eigen::Matrix<double, 3, 2> Bc = Eigen::Matrix<double, 3, 2>::Zero();
        Bc(0, 0) = 1.0;
        Bc(2, 1) = 1.0;

        Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity() + Ac * 0.01;
        Eigen::Matrix<double, 3, 2> Bd = Bc * 0.01;

        P = solveRiccati(Ad, Bd, m_Q, m_R, P);
        Eigen::Matrix2d S = m_R + Bd.transpose() * P * Bd;
        Eigen::Matrix<double, 2, 3> K = S.ldlt().solve(Bd.transpose() * P * Ad);

        waypoint wp = {{x(t), y(t), tangentAngle(t)}, v, w, K};
        path.push_back(wp);
    }

    return path;

}

double TankControl::arcLength() {
    
    auto dx = [&](double t){
        return 3*pow(1-t,2)*(p1.x-p0.x) + 6*(1-t)*t*(p2.x-p1.x) + 3*pow(t,2)*(p3.x-p2.x);
        };

    auto dy = [&](double t){
        return 3*pow(1-t,2)*(p1.y-p0.y) + 6*(1-t)*t*(p2.y-p1.y) + 3*pow(t,2)*(p3.y-p2.y);
    };

    auto vel = [&](double t){
        double vx = dx(t);
        double vy = dy(t);
        return sqrt(vx*vx + vy*vy);
    };

    int n = 100; 
    double length = 0;

    for(int i=0;i<n;i++)
    {
        double t0 = (double)i/n;
        double t1 = (double)(i+1)/n;
        double tm = (t0+t1)/2;

        length += (t1-t0)/6*(vel(t0) + 4*vel(tm) + vel(t1));
    }

    return length;
}


double TankControl::trapezoidalVelocity(double t, double maxVel, double pathLength) {

    double maxDecel = maxAccel; //TODO: add as parameter and tune

    t = std::clamp(t, 0.0, 1.0);

    

    double tMax = (pathLength+2)/pathLength;

    pathLength = pathLength + 2; //to ensure we can decelerate to the point

    double physicsPeak = std::sqrt(maxAccel * pathLength);
    double actualMaxVel = std::min(maxVel, physicsPeak);

    double accelDist  = (actualMaxVel * actualMaxVel) / (2.0 * maxAccel);
    double decelDist  = (actualMaxVel * actualMaxVel) / (2.0 * maxDecel);

    double rampUpEnd   = std::clamp(accelDist  / pathLength, 0.0, tMax);
    double rampDownStart = std::clamp(tMax - decelDist / pathLength, 0.0, tMax);

    if (t < rampUpEnd) {
        return actualMaxVel * (t / rampUpEnd);
    } else if (t > rampDownStart) {
        return actualMaxVel * ((tMax - t) / (tMax - rampDownStart));
    } else {
        return actualMaxVel;
    }
}


void TankControl::turnToHeading(double targetAngle, double maxAngularVel) {
    
    TankDrive::MovementVector motionVector;
    odometry::Position currentPos;
    std::vector<double> angleArray(50,10);

    targetAngle = targetAngle*(M_PI/180);

    headingPID->resetPID();

    while (true) {
        
        currentPos = odometry->getPos();

        double angleError = targetAngle - currentPos.heading;
    

        if(angleError > M_PI || angleError < -M_PI){
            angleError = -1 * sgn(angleError) * (2*M_PI - std::abs(angleError));
        }  

        double posAvg = rollAverage(std::abs(angleError), angleArray);
        if (posAvg < 0.025) {
            break; // Exit if the robot is within 0.025 radians of the target angle
        }

        double w = std::clamp(headingPID->calculate(0, angleError), -maxAngularVel, maxAngularVel);

        std::cout << angleError << " " << w << std::endl;
        motionVector.w = w;
        motionVector.v = 0;
        drivetrain->moveVector(motionVector);
        pros::delay(10);
    }

    motionVector.w = 0;
    motionVector.v = 0;
    drivetrain->moveVector(motionVector);

}

Eigen::Matrix3d TankControl::solveRiccati(const Eigen::Matrix3d& Ad,const Eigen::Matrix<double,3,2>& Bd, const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R, Eigen::Matrix3d& P){
    
    
    Eigen::Matrix3d Pprev;

    for (int i = 0; i < 50; ++i) {
        Eigen::Matrix2d S = R + Bd.transpose() * P * Bd;
        Pprev = P;
        P = Q + Ad.transpose() * P * Ad
              - Ad.transpose() * P * Bd
                * S.ldlt().solve(Bd.transpose() * P * Ad);

        // Check for convergence
        if ((P - Pprev).norm() < 1e-10) {
            break;
        }

        // Check for collapse
        if (!P.allFinite() || P.norm() < 1e-12) {
            std::cerr << "Riccati collapsed at iteration " << i << "\n";
            return Q;  // fallback: return Q so controller gets some gain
        }
    }
    return P;
}


double TankControl::tangentAngle(double t){
    return wrapHeading(atan2(dy(t), dx(t)));
}
double TankControl::x(double t){ 
    return pow((1-t),3)*p0.x + 3*pow((1-t),2)*t*p1.x + 3*(1-t)*pow(t,2)*p2.x + pow(t,3)*p3.x;
}
double TankControl::y(double t){
    return pow((1-t),3)*p0.y + 3*pow((1-t),2)*t*p1.y + 3*(1-t)*pow(t,2)*p2.y + pow(t,3)*p3.y;
}
double TankControl::dx(double t){
    return 3*pow(1-t,2)*(p1.x-p0.x) + 6*(1-t)*t*(p2.x-p1.x) + 3*pow(t,2)*(p3.x-p2.x);
}
double TankControl::dy(double t){
    return 3*pow(1-t,2)*(p1.y-p0.y) + 6*(1-t)*t*(p2.y-p1.y) + 3*pow(t,2)*(p3.y-p2.y);
}
double TankControl::ddx(double t){
    return -6*(1-t)*(p1.x-p0.x) + 6*(1-2*t)*(p2.x-p1.x) + 6*t*(p3.x-p2.x);
} 
double TankControl::ddy(double t){
    return -6*(1-t)*(p1.y-p0.y) + 6*(1-2*t)*(p2.y-p1.y) + 6*t*(p3.y-p2.y);
}
double TankControl::curvature(double t) {
    double num   = dx(t) * ddy(t) - dy(t) * ddx(t);
    double denom = std::pow(dx(t)*dx(t) + dy(t)*dy(t), 1.5);
    return (std::abs(denom) > 1e-9) ? num / denom : 0.0;
}



TankControl::TankControl(std::shared_ptr<TankDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    this->positionPID = std::move(posPID);
    this->headingPID = std::move(headingPID);

}

}
}
}}
