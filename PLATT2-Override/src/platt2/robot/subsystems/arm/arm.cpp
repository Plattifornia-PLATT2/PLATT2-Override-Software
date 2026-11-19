#include "platt2/robot/subsystems/arm/arm.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <memory>

namespace platt2 {
namespace robot {
namespace subsystems {
namespace arm {

ArmSubsystem::ArmSubsystem(std::unique_ptr<pros::Motor> leftArmMotor,
                           std::unique_ptr<pros::Motor> rightArmMotor,
                           std::unique_ptr<pros::Motor> clawMotor,
                           std::unique_ptr<pros::adi::DigitalOut> clawPiston)
    : leftArmMotor{std::move(leftArmMotor)},
      rightArmMotor{std::move(rightArmMotor)},
      clawMotor{std::move(clawMotor)},
      clawPiston{std::move(clawPiston)} {}

static double encoderTicksToRadians(double ticks, double countsPerRev) {
    return ticks * (2.0 * M_PI) / countsPerRev;
}

static double radiansToEncoderTicks(double rad, double countsPerRev) {
    return rad * countsPerRev / (2.0 * M_PI);
}

void ArmSubsystem::init() {
    if (task) return;   // already initialized

    leftArmMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightArmMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    clawMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    leftArmMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    rightArmMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    clawMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    leftArmMotor->tare_position();
    rightArmMotor->tare_position();
    clawMotor->tare_position();
    pros::delay(20);   // let the tare settle before the first move

    setClaw(OPEN);
    currentClawPosition = OPEN;

    task = std::make_unique<pros::Task>([this]() { run(); }, "arm");

}

// ---- Called from the main loop: only set state ----

void ArmSubsystem::moveArmToPosition(ArmPosition pos) {
    currentPosition = pos;
    newTarget = true;
    mode = Mode::POSITION;
}

void ArmSubsystem::releaseArm() { mode = Mode::RELEASE; }

void ArmSubsystem::setClaw(ClawPosition pos) {
    currentClawPosition = pos;
    clawPiston->set_value(pos);
}

void ArmSubsystem::toggleClaw() {
    setClaw(currentClawPosition == OPEN ? CLOSED : OPEN);
}

void ArmSubsystem::toggleArmPosition() {}

double ArmSubsystem::getArmPosition() {
    return encoderTicksToRadians(leftArmMotor->get_position(), REV_COUNTS_RED);
}

double ArmSubsystem::getClawPosition() {
    return encoderTicksToRadians(clawMotor->get_position(), REV_COUNTS_GREEN);
}

void ArmSubsystem::moveToLoadThenReady2() {
    mode = Mode::LOAD_THEN_READY2;
    sequenceStep = 0;          // restart the sequence
}

bool ArmSubsystem::armAtTarget(double targetTicks) {
    return std::fabs(leftArmMotor->get_position() - targetTicks) < ARM_TOLERANCE_TICKS
        && std::fabs(rightArmMotor->get_position() - targetTicks) < ARM_TOLERANCE_TICKS;
}
// ---- The task: the only code that commands the arm/claw motors ----

void ArmSubsystem::run() {
    while (true) {
        switch (mode.load()) {
        case Mode::POSITION:
            if (newTarget.exchange(false)) {   // send each move once per request
                double armRad = 0, clawRad = 0;
                switch (currentPosition.load()) {
                case SCORE:
                    armRad = ARM_MOTOR_SCORE_POSITION;
                    clawRad = CLAW_MOTOR_SCORE_POSITION;
                    break;
                case LOAD:
                    armRad = ARM_MOTOR_READY_POSITION;
                    clawRad = CLAW_MOTOR_READY_POSITION;
                    break;
                case IN_SIZE:
                    armRad = ARM_MOTOR_INSIZE_POSITION;
                    clawRad = CLAW_MOTOR_INSIZE_POSITION;
                    break;
                case READY2:
                    armRad = ARM_MOTOR_READY2_POSITION;
                    clawRad = ARM_MOTOR_READY2_POSITION;
                    break;
                
                }

                double armTicks = radiansToEncoderTicks(armRad, REV_COUNTS_RED);
                double clawTicks = radiansToEncoderTicks(clawRad, REV_COUNTS_GREEN);

                leftArmMotor->move_absolute(armTicks, ARM_VELOCITY);
                rightArmMotor->move_absolute(armTicks, ARM_VELOCITY);
                clawMotor->move_absolute(clawTicks, CLAW_VELOCITY);

            }
            break;

        case Mode::RELEASE:
            // A stop command lets the COAST brake mode take effect
            leftArmMotor->move_velocity(0);
            rightArmMotor->move_velocity(0);
            clawMotor->move_velocity(0);
            mode = Mode::IDLE;
            break;

        case Mode::IDLE:
            break;
        
        case Mode::LOAD_THEN_READY2: {
    const double loadArmTicks =
        radiansToEncoderTicks(ARM_MOTOR_READY_POSITION, REV_COUNTS_RED);
    const double loadClawTicks =
        radiansToEncoderTicks(CLAW_MOTOR_READY_POSITION, REV_COUNTS_GREEN);
    const double r2ArmTicks =
        radiansToEncoderTicks(ARM_MOTOR_READY2_POSITION, REV_COUNTS_RED);
    const double r2ClawTicks =
        radiansToEncoderTicks(CLAW_MOTOR_READY2_POSITION, REV_COUNTS_GREEN);

    if (sequenceStep == 0) {
        // Step 0: command LOAD (arm only, per your request)
        leftArmMotor->move_absolute(loadArmTicks, ARM_VELOCITY);
        rightArmMotor->move_absolute(loadArmTicks, ARM_VELOCITY);
        sequenceStep = 1;
    } else if (sequenceStep == 1) {
        // Step 1: wait until the arm reaches LOAD
        if (armAtTarget(loadArmTicks)) {
            sequenceStep = 2;
        }
    } else if (sequenceStep == 2) {
        // Step 2: command READY2 for arm and claw
        leftArmMotor->move_absolute(r2ArmTicks, ARM_VELOCITY);
        rightArmMotor->move_absolute(r2ArmTicks, ARM_VELOCITY);
        clawMotor->move_absolute(r2ClawTicks, CLAW_VELOCITY);
        currentPosition = READY2;
        mode = Mode::IDLE;
    }
    }
    break;
}
        pros::screen::print(pros::E_TEXT_LARGE, 8, "Arm Pos: %.3f", getArmPosition());
        pros::screen::print(pros::E_TEXT_LARGE, 10, "Claw Pos: %.3f", getClawPosition());
        pros::delay(10);
    }
}

} // namespace arm
} // namespace subsystems
} // namespace robot
} // namespace platt2