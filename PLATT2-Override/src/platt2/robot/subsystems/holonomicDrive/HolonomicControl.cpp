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


    //pros::delay(500);
    //uint32_t start = pros::millis();
    //drivetrain->moveVector({0, 30,0});
//
//
    //while (pros::millis() - start < 2000) {
    //std::cout<<pros::millis()-start<<","<<std::hypot(std::abs(odometry->getVelocity().x()), std::abs(odometry->getVelocity().y()))<<std::endl;
//
    //pros::delay(20);
    //
    //}
//
    //drivetrain->moveVector({0,0,0});

    struct Sample { int ms; double v; };
    std::vector<Sample> buf; buf.reserve(400);
    auto t0 = std::chrono::steady_clock::now(), next = t0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(2)) {
        drivetrain->moveVector({0, 30, 0});   // constant, robot frame
        int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0).count();
        buf.push_back({ms, odometry->getVelocity().head<2>().norm()});
        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    }
drivetrain->moveVector({0, 0, 0});
for (auto& s : buf) printf("%d %.1f\n", s.ms, s.v);


 
}

void HolonomicControl::moveToPoint(const Vec3& start, const Vec3& goal, double startHeading,
                                   double endHeading, TrajConstraints constraints)
{
    // ---- tunables ----
    const double dt           = 0.010;
    const bool   debug        = true;    // main-loop and settle prints (turn off for competition)

    // feedforward timing / feedback ramp
    const double lookaheadSec = 0.08;    // s
    const double fbDelay      = 0.20;    // s, feedback off until the robot can respond
    const double fbRamp       = 0.30;    // s, then fades in

    // reference hold: pause the reference while the robot lags behind it
    const double lagMax       = 0.5;     // in
    const size_t extraTicks   = 150;     // hard stop: 1.5 s past the planned time

    // main-loop integral (x and y, field frame)
    const double kIm          = 4.0;     // ips per (in*s)
    const double imMax        = 1;     // in*s per axis

    // settle loop
    const double posTol       = 0.15;     // in
    const double headTol      = 0.01;    // rad
    const double exitSpeed    = 2.0;     // ips, must be nearly stopped to count as arrived
    const double settleTime   = 2.0;     // s hard timeout
    const double kI           = 1;     // ips per (in*s)
    const double iMax         = 1.5;     // in*s per axis
    const double iRange       = 10.0;     // in, settle integral only acts this close
    const double kickRate     = 10.0;    // ips/s ramp while stuck
    const double kickMax      = 7.0;     // ips, just above breakaway
    const double kickHold     = 0.05;    // s, wait after motion starts (covers the delay)
    const int    stallTicks   = 50;      // 0.5 s at full kick with no motion, then give up
    const double kickStart    = 2.5;
    const double creepCmd  = 5.5;   // ips of command (about 1 ips actual), just above breakaway
    const double creepDist = 4.0;   // in   

    const size_t lookahead = static_cast<size_t>(std::lround(lookaheadSec / dt));

    TrajPtr path = generatePath(start, goal, startHeading, endHeading, 0.25);
    path = applyMotionProfile(path, constraints, dt);
    if (path->size() < 2) { drivetrain->moveVector({0, 0, 0}); return; }

    Eigen::Vector2d integM = Eigen::Vector2d::Zero();
    auto next = std::chrono::steady_clock::now();

    // ---- main trajectory loop ----
    size_t i = 0;                                  // reference index (can pause)
    size_t k = 0;                                  // ticks elapsed (never pauses)
    const size_t maxTicks = path->size() + extraTicks;

    while (i < path->size() && k < maxTicks) {
        const TrajPoint& ref = (*path)[i];
        const TrajPoint& ff  = (*path)[std::min(i + lookahead, path->size() - 1)];

        odometry::Position pos = odometry->getPos();
        Vec3 pose = {pos.x, pos.y, pos.heading};
        Vec3 vel  = odometry->getVelocity();

        double fbScale = std::clamp((k * dt - fbDelay) / fbRamp, 0.0, 1.0);
        Vec3 u = lqrController->lqrTrack(ref.pos, ref.vel, ff.vel, ff.acc, pose, vel, fbScale);

        // integral on x and y, minus the along-track part (that is timing, not bias)
        if (fbScale >= 1.0) {
            integM += (ref.pos - pose).head<2>() * dt;
            Eigen::Vector2d d = ref.vel.head<2>();
            if (d.norm() > 1.0) { d.normalize(); integM -= integM.dot(d) * d; }
            integM = integM.cwiseMax(-imMax).cwiseMin(imMax);
            u.head<2>() += kIm * integM;
        }

        {
        Eigen::Vector2d toGoal = (path->back().pos - pose).head<2>();
        double dGoal = toGoal.norm();
        if (dGoal > posTol && dGoal < creepDist) {
            Eigen::Vector2d dir = toGoal / dGoal;
            double have = u.head<2>().dot(dir);
            if (have < creepCmd) u.head<2>() += (creepCmd - have) * dir;
        }
        }

        if (debug && k % 5 == 0)
            printf("t=%.2f i=%zu err_x=%.2f err_y=%.2f err_th=%.3f vref=%.1f vmeas=%.1f vmeas_x=%.1f u_x=%.2f u_y=%.2f u_z=%.2f\n",
                   k * dt, i,
                   ref.pos.x() - pose.x(), ref.pos.y() - pose.y(),
                   std::remainder(ref.pos.z() - pose.z(), 2.0 * M_PI),
                   ref.vel.head<2>().norm(), vel.head<2>().norm(), vel.x(),
                   u.x(), u.y(), u.z());

        // along-track lag: how far behind the reference the robot is
        Eigen::Vector2d tdir = (ref.pos - (*path)[i > 0 ? i - 1 : 0].pos).head<2>();
        double lag = (tdir.norm() > 1e-6)
                         ? (ref.pos - pose).head<2>().dot(tdir.normalized()) : 0.0;

        drivetrain->moveVector(drivetrain->fieldToRobotOrientation(
            {u.x(), u.y(), -u.z()}, pose.z()));

        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);

        ++k;
        if (lag <= lagMax) ++i;                    // otherwise hold the reference
    }

    // ---- settle loop ----
    enum class Exit { Arrived, Stalled, Timeout };
    Exit why = Exit::Timeout;

    const TrajPoint& end = path->back();
    const auto settleStart = std::chrono::steady_clock::now();
    const auto deadline = settleStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                            std::chrono::duration<double>(settleTime));
    next = settleStart;

    Vec3 integ = Vec3::Zero();
    integ.head<2>() = 0.5 * integM * kIm / kI;           // carry the main-loop correction over
    double kick = 0.0, hold = 0.0;
    int stuck = 0, sc = 0;
    Eigen::Vector2d prevErr = Eigen::Vector2d::Zero();

    while (std::chrono::steady_clock::now() < deadline) {
        odometry::Position pos = odometry->getPos();
        Vec3 pose = {pos.x, pos.y, pos.heading};
        Vec3 vel  = odometry->getVelocity();

        Vec3 e = pose - end.pos;
        e.z() = std::remainder(e.z(), 2.0 * M_PI);
        const Eigen::Vector2d err = -e.head<2>();  // target minus robot
        const double dist = err.norm();
        const double spd  = vel.head<2>().norm();

        // 1. done: inside tolerance, heading good, nearly stopped
        if (dist < posTol && std::abs(e.z()) < headTol && spd < exitSpeed) {
            why = Exit::Arrived;
            break;
        }

        // 2. crossed the target on an axis: drop that axis's integral and the push
        for (int a = 0; a < 2; ++a)
            if (std::abs(err(a)) > 0.05 && err(a) * prevErr(a) < 0.0) {
                integ(a) = 0.0;
                kick = 0.0;
            }
        prevErr = err;

        // 3. integral on x and y, only when close
        if (dist < iRange) {
            integ.head<2>() += err * dt;
            integ.head<2>() = integ.head<2>().cwiseMax(-iMax).cwiseMin(iMax);
        } else {
            integ.head<2>().setZero();
        }
        for (int a = 0; a < 2; ++a) if (std::abs(err(a)) < 0.2) integ(a) *= 0.9;

        // 4. LQR hold at the final point, plus integral (field frame)
        Vec3 u = lqrController->lqrTrack(end.pos, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), pose, vel);
        u.head<2>() += kI * integ.head<2>();

        // 5. stiction kick: ramp a push toward the target while stuck; once it moves,
        //    stop pushing and wait out the delay so it doesn't overshoot
        if (dist > posTol) {
            const Eigen::Vector2d dir = err / dist;
            if (spd > 1.0)       { kick = 0.0; hold = kickHold; }
            else if (hold > 0.0) { hold -= dt; }
            else                 { kick = std::min(kickMax, kick + kickRate * dt); }
            u.head<2>() += kick * dir;

            // 6. give up cleanly if full kick still doesn't move it
            if (kick >= kickMax && spd < 0.5) ++stuck; else stuck = 0;
            if (stuck >= stallTicks) { why = Exit::Stalled; break; }
        } else  { kick = std::min(kickMax, std::max(kick, kickStart) + kickRate * dt); }

        if (debug && ++sc % 5 == 0)
            printf("s: dist=%.2f spd=%.1f kick=%.1f hold=%.2f u=(%.2f %.2f)\n",
                   dist, spd, kick, hold, u.x(), u.y());

        drivetrain->moveVector(drivetrain->fieldToRobotOrientation(
            {u.x(), u.y(), -u.z()}, pose.z()));

        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    }
    drivetrain->moveVector({0, 0, 0});

    // ---- end-of-move report ----
    static const char* names[] = {"arrived", "STALLED", "TIMEOUT"};
    double settleMs = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - settleStart).count();
    odometry::Position pf = odometry->getPos();
    double ex = pf.x - end.pos.x(), ey = pf.y - end.pos.y();
    printf("settle: %s after %.0f ms (main loop %zu ticks of %zu planned), err=(%.2f, %.2f) |e|=%.2f heading_err=%.3f\n",
           names[static_cast<int>(why)], settleMs, k, path->size(), ex, ey, std::hypot(ex, ey),
           std::remainder(pf.heading - end.pos.z(), 2.0 * M_PI));
}




HolonomicControl::HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::shared_ptr<LQR> lqr)
{
    drivetrain = drive;
    odometry = odom;
    lqrController = lqr;

}

}}}}