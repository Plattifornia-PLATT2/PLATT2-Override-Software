#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/holonomicDrive/autonControl/trajectoryGen.hpp"
#include "platt2/robot/subsystems/holonomicDrive/autonControl/LQR.hpp"

#include "platt2/EAllianceConfig.hpp"
#include "platt2/helperFunctions.h"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <math.h>
#include <memory>
#include <ostream>
#include <regex>
#include <thread>
#include <utility>
#include <vector>

namespace platt2{
namespace robot{
namespace subsystems{
namespace holonomicDrive{


void HolonomicControl::tauTest(){


    pros::delay(500);
    uint32_t start = pros::millis();
    drivetrain->moveVector({0, 40,0});


    while (pros::millis() - start < 1500) {
    std::cout<<pros::millis()-start<<","<<std::hypot(std::abs(odometry->getVelocity().x()), std::abs(odometry->getVelocity().y()))<<std::endl;

    pros::delay(20);
    
    }

    drivetrain->moveVector({0,0,0});




 
}

void HolonomicControl::moveToPoint(const Vec3& start, const Vec3& goal, double startHeading,
                                   double endHeading, TrajConstraints constraints)
{
    // ---- tunables ----
    const double dt           = 0.010;
    const double lookaheadSec = 0.1;   // s, feedforward lookahead
    const double fbDelay      = 0.10;   // s, feedback off until robot can respond
    const double fbRamp       = 0.15;   // s, then fades in

    const double posTol       = 0.15;   // in
    const double headTol      = 0.03;   // rad
    const double exitSpeed    = 2.0;    // ips
    const double settleTime   = 1.0;    // s hard timeout
    const double kI           = 1.0;    // ips per (in*s)
    const double iMax         = 1.0;    // in*s per axis
    const double iRange       = 10.0;   // in, integral only acts this close
    //const double kickRate     = 40.0;   // ips/s ramp while stuck
    //const double kickMax      = 7.0; 
    const double floorCmd = 12.0;   // ips of command, just above breakaway
    const double floorDist = 0.25;  // in, fade the floor to 0 at this distance   // ips, just above breakaway

    const size_t lookahead = static_cast<size_t>(std::lround(lookaheadSec / dt));
    const auto   period    = std::chrono::milliseconds(10);

    TrajPtr path = generatePath(start, goal, startHeading, endHeading, constraints, 0.25);
    path = applyMotionProfile(path, constraints, dt);
    if (path->size() < 2) { drivetrain->moveVector({0, 0, 0}); return; }

    auto drive = [&](const Vec3& u, const Vec3& pose) {
        drivetrain->moveVector(drivetrain->fieldToRobotOrientation(
            {u.x(), u.y(), -u.z()}, pose.z()));
    };
    auto getPose = [&] {
        odometry::Position p = odometry->getPos();
        return Vec3{p.x, p.y, p.heading};
    };

    auto next = std::chrono::steady_clock::now();

    // ---- trajectory loop ----
    for (size_t i = 0; i < path->size(); ++i) {
        const TrajPoint& ref = (*path)[i];
        const TrajPoint& ff  = (*path)[std::min(i + lookahead, path->size() - 1)];

        Vec3 pose = getPose();
        Vec3 vel  = odometry->getVelocity();

        double fbScale = std::clamp((i * dt - fbDelay) / fbRamp, 0.0, 1.0);
        Vec3 u = lqrController->lqrTrack(ref.pos, ref.vel, ff.vel, ff.acc, pose, vel, fbScale);

        drive(u, pose);
        next += period;
        std::this_thread::sleep_until(next);
    }

    // ---- settle loop ----
    const TrajPoint& end = path->back();
    const auto settleStart = std::chrono::steady_clock::now();
    const auto deadline = settleStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                            std::chrono::duration<double>(settleTime));
    next = settleStart;

    Eigen::Vector2d integ = Eigen::Vector2d::Zero();
    Eigen::Vector2d prevErr = Eigen::Vector2d::Zero();
    bool arrived = false;

    while (std::chrono::steady_clock::now() < deadline) {
        Vec3 pose = getPose();
        Vec3 vel  = odometry->getVelocity();

        const Eigen::Vector2d err = (end.pos - pose).head<2>();   // target minus robot
        const double headErr = std::remainder(pose.z() - end.pos.z(), 2.0 * M_PI);
        const double dist = err.norm();
        const double spd  = vel.head<2>().norm();

        if (dist < posTol && std::abs(headErr) < headTol && spd < exitSpeed) {
            arrived = true;
            break;
        }

        if (err.dot(prevErr) < 0) integ.setZero();
            prevErr = err;

        // integral, only when close
        if (dist < iRange)
            integ = (integ + err * dt).cwiseMax(-iMax).cwiseMin(iMax);
        else
            integ.setZero();

        Vec3 u = lqrController->lqrTrack(end.pos, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), pose, vel);
        u.head<2>() += kI * integ;

        // stiction kick: ramp toward target while stuck, drop it once moving
       if (dist > posTol) {
        Eigen::Vector2d dir = err / dist;
        double fade = std::clamp((dist - posTol) / floorDist, 0.0, 1.0);
        double have = u.head<2>().dot(dir);
        double want = floorCmd * fade;
        if (have < want) u.head<2>() += (want - have) * dir;
        }

        //printf("ux:%.3f, uy:%.3f, uw:%.3f ", u.x(), u.y(), u.z());
        drive(u, pose);
        next += period;
        std::this_thread::sleep_until(next);
    }
    drivetrain->moveVector({0, 0, 0});

    // ---- end-of-move report ----
    odometry::Position pf = odometry->getPos();
    double ex = pf.x - end.pos.x(), ey = pf.y - end.pos.y();
    printf("settle: %s after %.0f ms, err=(%.2f, %.2f) |e|=%.2f heading_err=%.3f\n",
           arrived ? "arrived" : "TIMEOUT",
           std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - settleStart).count(),
           ex, ey, std::hypot(ex, ey),
           std::remainder(pf.heading - end.pos.z(), 2.0 * M_PI));
}



void HolonomicControl::driverControl(double wantedX, double wantedY, double wantedW){

    Vec3 currentPos = {};
    Vec3 currentVel = {};
    Vec3 wantedVel = {80,80, 8};
    Vec3 wantedAccel = {80,80, 8};




    //Vec3 u = lqrController->lqrTrack(ref.pos, ref.vel, wantedAccel, wantedAccel, pose, vel, fbScale);



   
}



HolonomicControl::HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::shared_ptr<LQR> lqr)
{
    drivetrain = drive;
    odometry = odom;
    lqrController = lqr;

}

}}}}