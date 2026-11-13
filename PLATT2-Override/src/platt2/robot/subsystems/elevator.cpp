#include "platt2/robot/subsystems/elevator/elevator.hpp"
#include "pros/motors.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <sys/_intsup.h>

namespace platt2 {

namespace robot {

namespace subsystems {

namespace elevator {

static int32_t encoderTicksToRadians(int32_t ticks){
    return ticks * ((2*M_PI)/300);
}

static void taskThunk(void *p) {
  reinterpret_cast<ElevatorSubsystem *>(p)->elevatorTaskLoop();
}

ElevatorSubsystem::ElevatorSubsystem(
    std::unique_ptr<pros::Motor> leftMotor,
    std::unique_ptr<pros::Motor> rightMotor,
    std::unique_ptr<pros::Rotation> rotationSensor,
    std::unique_ptr<robot::pid::PID> elevatorPID

    )
    : leftMotor{std::move(leftMotor)}, rightMotor{std::move(rightMotor)},
      rotationSensor{std::move(rotationSensor)},
      elevatorPID{std::move(elevatorPID)}, elevatorTask(taskThunk, this)

{}

void ElevatorSubsystem::initElevator() {
  currentPosition = FULL_DOWN;
  leftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotor->tare_position();
  rightMotor->tare_position();
  leftMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
  rightMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
}

uint32_t ElevatorSubsystem::getPosInches(ElevatorPosition pos) {
    return 0;
}

std::int32_t ElevatorSubsystem::getElevatorPositionInches(){
    int position = leftMotor->get_position() + rightMotor->get_position();
    position = position/2;
    return encoderTicksToRadians(position);
}

void ElevatorSubsystem::moveToPosition(ElevatorPosition pos) {
  currentPosition = pos;
}

void ElevatorSubsystem::manualMove(int32_t velocity) {
    double pos = getElevatorPositionInches();

    bool atBottom = pos <= 0 && velocity < 0;
    bool atTop    = pos >= FULL_LIFT_HEIGHT && velocity > 0;

    if (atBottom || atTop) {
        leftMotor->move_velocity(0);
        rightMotor->move_velocity(0);
    } else {
        leftMotor->move_velocity(velocity);
        rightMotor->move_velocity(velocity);
    }

    pros::screen::print(pros::E_TEXT_LARGE, 6, "Elevator Pos: %.3f", pos);
}

void ElevatorSubsystem::elevatorTaskLoop() {
//   ElevatorPosition prevPosition = currentPosition;
//   double target = getPosInches(currentPosition);

//   // dt must match the delay below: 20 ms = 0.02 s
//   // max/min are in millivolts to match move_voltage (+-12000)
//   // platt2::robot::pid::PID pid(0.02, 12000, -12000, Kp, Kd, Ki);

//   while (true) {
//     // New target requested: pick the new setpoint and clear old PID state
//     if (currentPosition != prevPosition) {
//       target = getPosInches(currentPosition);
//       elevatorPID->resetPID();
//       prevPosition = currentPosition;
//     }

//     double pv = rotationSensor->get_position(); // centidegrees
//     double output = elevatorPID->calculate(target, pv);

//     leftMotor->move_voltage(output);
//     rightMotor->move_voltage(output);

//     pros::delay(20);
//   }
}

} // namespace elevator
} // namespace subsystems
} // namespace robot
} // namespace platt2