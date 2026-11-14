#include "platt2/robot/subsystems/elevator/elevator.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdint>
#include <math.h>

namespace platt2 {
namespace robot {
namespace subsystems {
namespace elevator {

static double encoderTicksToRadians(double ticks) {
  return ticks * ((2 * M_PI) / 1800);
}

static double radiansToEncoderTicks(double rad) {
  return rad * 1800 / (2.0 * M_PI);
}

ElevatorSubsystem::ElevatorSubsystem(
    std::unique_ptr<pros::Motor> leftMotor,
    std::unique_ptr<pros::Motor> rightMotor,
    std::unique_ptr<pros::Rotation> rotationSensor,
    std::unique_ptr<robot::pid::PID> elevatorPID)
    : leftMotor{std::move(leftMotor)}, rightMotor{std::move(rightMotor)},
      rotationSensor{std::move(rotationSensor)},
      elevatorPID{std::move(elevatorPID)} {}

void ElevatorSubsystem::initElevator() {
  currentPosition = FULL_DOWN;
  leftMotor->set_encoder_units(
      pros::E_MOTOR_ENCODER_COUNTS); // units before tare
  rightMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
  leftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotor->tare_position();
  rightMotor->tare_position();
  pros::delay(20);

  task = std::make_unique<pros::Task>([this]() { run(); }, "elevator");
}

uint32_t ElevatorSubsystem::getPosInches(ElevatorPosition pos) { return 0; }

double ElevatorSubsystem::getElevatorPositionRadians() {
  double position =
      (leftMotor->get_position() + rightMotor->get_position()) / 2.0;
  return encoderTicksToRadians(position);
}

void ElevatorSubsystem::moveToPosition(ElevatorPosition pos) {
  currentPosition = pos;
  switch (pos) {
  case FULL_DOWN:
    targetTicks = radiansToEncoderTicks(BOTTOM_LIFT_HEIGHT);
    break;
  case MIDDLE_EXTENSION:
    targetTicks = radiansToEncoderTicks(MIDDLE_LIFT_HEIGHT);
    break;
  case FULL_EXTENSION:
    targetTicks = radiansToEncoderTicks(FULL_LIFT_HEIGHT);
    break;
  }
  newTarget = true;
  mode = Mode::POSITION;
}

void ElevatorSubsystem::manualMove(int32_t velocity) {
  if (velocity == 0) {
    // a zero must not cancel a position move in progress
    if (mode == Mode::MANUAL)
      mode = Mode::IDLE;
    return;
  }
  manualVelocity = velocity;
  mode = Mode::MANUAL;
}

void ElevatorSubsystem::stop() { mode = Mode::IDLE; }

// ---- The task: the only code that commands the motors ----

void ElevatorSubsystem::run() {
  Mode last = Mode::IDLE;

  while (true) {
    Mode m = mode.load();

    switch (m) {
    case Mode::MANUAL: {
      double pos = getElevatorPositionRadians();
      int32_t v = manualVelocity.load();
      bool blocked = (pos <= 0 && v < 0) || (pos >= FULL_LIFT_HEIGHT && v > 0);
      leftMotor->move_velocity(blocked ? 0 : v);
      rightMotor->move_velocity(blocked ? 0 : v);
      break;
    }
    case Mode::POSITION:
      if (newTarget.exchange(false)) { // send move_absolute once per request
        double t = targetTicks.load();
        leftMotor->move_absolute(t, ELEVATOR_VELOCITY);
        rightMotor->move_absolute(t, ELEVATOR_VELOCITY);
      }
      break;
    case Mode::IDLE:
      if (last == Mode::MANUAL) { // stop once when leaving manual
        leftMotor->move_velocity(0);
        rightMotor->move_velocity(0);
      }
      break;
    }

    last = m;
    pros::delay(10);
  }
}

} // namespace elevator
} // namespace subsystems
} // namespace robot
} // namespace platt2