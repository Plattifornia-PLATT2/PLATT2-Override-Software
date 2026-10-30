#include "platt2/robot/subsystems/odometry/PiPositionTracker.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "OdometryPosition.hpp"
#include "platt2/robot/subsystems/externalLink/piLink.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <deque>

namespace platt2{
namespace robot{
namespace subsystems{
namespace odometry{

    Position PiPositionTracker::getPos(){
        return current_position;
    }

    void PiPositionTracker::setPos(Position pos){

    }

    double PiPositionTracker::getX(){
        this->mtx.take();
        double out = this->current_position.x;
        this->mtx.give();
        return out;
    }

    double PiPositionTracker::getY(){
        this->mtx.take();
        double out = this->current_position.y;
        this->mtx.give();
        return out;
    }

    double PiPositionTracker::getHeading(){
        this->mtx.take();
        double out = this->current_position.heading;
        this->mtx.give();
        return out;
    }
    
    Eigen::Vector3d PiPositionTracker::getVelocity(){
        this->mtx.take();
        Eigen::Vector3d out = this->velocity;
        this->mtx.give();

        return out;
    }


    void PiPositionTracker::setOffsets(double x_offset, double y_offset){
        this->x_offset = x_offset;
        this->y_offset = y_offset;
    }

    void PiPositionTracker::init(){

    }

    void PiPositionTracker::updatePosition() {
        
        static constexpr uint32_t MIN_WINDOW_US = 40000;
        static constexpr uint32_t STALE_US      = 150000;
        static constexpr double   ALPHA         = 0.5;

        struct PosSample { double x, y; uint32_t t_us; };
        std::deque<PosSample> history;

        double lastX = 0, lastY = 0;
        double vx = 0, vy = 0;            // filtered state kept locally
        uint32_t last_change_us = 0;
        uint32_t t = pros::millis();

        while (true) {
            uint32_t now = pros::micros();

            double x = piLink_subsystem->getXPos();
            double y = piLink_subsystem->getYPos();
            double heading = imu->get_heading();
            auto gyroRate = imu->get_gyro_rate();

            bool headingOk  = (heading != PROS_ERR_F);
            bool gyroRateOk = (gyroRate.z != PROS_ERR_F);

            // ---- velocity estimate ----
            if (x != lastX || y != lastY) {
                lastX = x; lastY = y;
                last_change_us = now;
                history.push_back({x, y, now});

                while (history.size() > 2 && (now - history[1].t_us) >= MIN_WINDOW_US)
                    history.pop_front();

                const PosSample& oldest = history.front();
                double dt = (now - oldest.t_us) / 1e6;
                if (dt >= MIN_WINDOW_US / 1e6) {
                    vx = ALPHA * ((x - oldest.x) / dt) + (1 - ALPHA) * vx;
                    vy = ALPHA * ((y - oldest.y) / dt) + (1 - ALPHA) * vy;
                }
            } else if (now - last_change_us > STALE_US) {
                vx = vy = 0;
                history.clear();
            }

            mtx.take();
            
            current_position.x = x;
            current_position.y = y;
            if (headingOk) current_position.heading = (360 - heading) * M_PI / 180;
            
            velocity.x() = vx;
            velocity.y() = vy;
            if (gyroRateOk) velocity.z() = -gyroRate.z * M_PI / 180.0;
            
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1,"X Pos: %.3f", current_position.x);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2,"Y Pos: %.3f", current_position.y);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3,"Heading: %.2f", current_position.heading*180/M_PI);
            
            mtx.give();

            pros::Task::delay_until(&t, 10);
    }
}
    

    PiPositionTracker::PiPositionTracker(

    std::shared_ptr<subsystems::piLink::PiLinkSubsystem> piLink_subsystem,
    std::unique_ptr<pros::IMU> imu


    ) : piLink_subsystem(piLink_subsystem),imu(std::move(imu))
    {

    }


}}}}