#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"
#include "Eigen/src/Core/Matrix.h"
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

        imu->set_data_rate(5);

        while(imu->is_calibrating()){
            pros::delay(10);
        }
    }

    Eigen::Vector3d TrackingWheelPositionTracker::getVelocity(){
        return this->velocity;
    }


    void TrackingWheelPositionTracker::updatePosition(){
    
    imu->set_data_rate(5);
    pros::delay(1000);

    constexpr uint32_t LOOP_MS = 10;
    constexpr int WINDOW = 4;            // velocity window, in samples (3 x 10 ms = 30 ms)
    constexpr int RING = WINDOW + 1;



    struct Sample { double t, x, y, th; };
    Sample hist[RING] = {};
    int head = 0;
    int count = 0;

    current_position.heading = (360 - imu->get_heading()) * M_PI / 180;
    double oldTheta = current_position.heading;
    double unwrappedTheta = oldTheta;    // continuous heading, used only for velocity

    double oldX = 0;                     // x_wheel->getPosition() once the x wheel is used
    double oldY = y_wheel->getPosition();

    uint32_t now = pros::millis();

    while(true){
        // read sensors
        double newX = x_wheel->getPosition();                 // x_wheel->getPosition();
        double newY = y_wheel->getPosition();
        current_position.heading = (360 - imu->get_heading()) * M_PI / 180;

        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1,"X Pos: %.3f", current_position.x);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2,"Y Pos: %.3f", current_position.y);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3,"Heading: %.2f", current_position.heading*180/M_PI);

        // deltas
        double dX = newX - oldX;
        double dY = newY - oldY;
        double dTheta = current_position.heading - oldTheta;
        if (dTheta > M_PI || dTheta < -M_PI) {
            dTheta = -1 * sgn(dTheta) * (2 * M_PI - std::abs(dTheta));
        }

        oldX = newX;
        oldY = newY;
        oldTheta = current_position.heading;
        unwrappedTheta += dTheta;

        // tracking wheel odometry
        double localX = dX - (x_offset * dTheta);
        double localY = dY + (y_offset * dTheta);

        double scale = 1 - (dTheta * dTheta) / 24;
        localX *= scale;
        localY *= scale;

        double avgTheta = current_position.heading - dTheta / 2;
        double globalX = localY * cos(avgTheta) - localX * sin(avgTheta);
        double globalY = localY * sin(avgTheta) + localX * cos(avgTheta);

        if (!std::isnan(globalX) && !std::isnan(globalY)) {
            current_position.x += globalX;
            current_position.y += globalY;
        }

        // windowed velocity from the pose history
        double t = pros::micros() * 1e-6;
        hist[head] = { t, current_position.x, current_position.y, unwrappedTheta };
        if (count < RING) count++;

        if (count == RING) {
            const Sample& oldest = hist[(head + 1) % RING];
            double dtw = t - oldest.t;
            if (dtw > 1e-4) {
                velocity.x() = (current_position.x - oldest.x) / dtw;
                velocity.y() = (current_position.y - oldest.y) / dtw;
                velocity.z() = (unwrappedTheta - oldest.th) / dtw;
            }
        }
        head = (head + 1) % RING;

        

        // fixed-period loop, no drift
        pros::Task::delay_until(&now, LOOP_MS);
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