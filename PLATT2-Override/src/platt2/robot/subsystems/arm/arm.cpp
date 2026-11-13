#include "platt2/robot/subsystems/arm/arm.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

namespace platt2 {

namespace robot {

namespace subsystems {

namespace arm {

ArmSubsystem::ArmSubsystem(std::unique_ptr<pros::Motor> armMotor,
                           std::unique_ptr<pros::Motor> clawMotor,
                           std::unique_ptr<pros::adi::DigitalOut> clawPiston)
    : armMotor{std::move(armMotor)}, clawMotor{std::move(clawMotor)},
      clawPiston{std::move(clawPiston)} {}

void ArmSubsystem::init() {
  armMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  clawMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  armMotor->tare_position();
  clawMotor->tare_position();
  moveArmToPosition(LOAD);
  setClaw(OPEN);
}

void ArmSubsystem::moveArmToPosition(ArmPosition pos) {
  currentPosition = pos;
  switch (pos) {
  case SCORE: {
    armMotor->move_absolute(ARM_MOTOR_SCORE_POSITION, ARM_VELOCITY);
    clawMotor->move_absolute(CLAW_MOTOR_SCORE_POSITION, CLAW_VELOCITY);
    break;
  }
  case LOAD: {
    armMotor->move_absolute(ARM_MOTOR_READY_POSITION, ARM_VELOCITY);
    clawMotor->move_absolute(CLAW_MOTOR_READY_POSITION, CLAW_VELOCITY);
    break;
  }
  case IN_SIZE: {
    armMotor->move_absolute(ARM_MOTOR_INSIZE_POSITION, ARM_VELOCITY);
    clawMotor->move_absolute(CLAW_MOTOR_INSIZE_POSITION, CLAW_VELOCITY);
    break;
  }
  }
}

std::int32_t ArmSubsystem::getArmPosition() { return armMotor->get_position(); }

void ArmSubsystem::setClaw(ClawPosition pos) { clawPiston->set_value(pos); }

void ArmSubsystem::toggleArmPosition() {}
} // namespace arm
} // namespace subsystems
} // namespace robot
} // namespace platt2