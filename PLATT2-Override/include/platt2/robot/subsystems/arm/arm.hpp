#ifndef ARM_HPP
#define ARM_HPP

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>
#include <memory>

namespace platt2 {
namespace robot {
namespace subsystems {
namespace arm {

enum ArmPosition { LOAD, SCORE, IN_SIZE, READY2 };

enum ClawPosition { CLOSED = 1, OPEN = 0 };

class ArmSubsystem {
public:
    ArmSubsystem(std::unique_ptr<pros::Motor> leftArmMotor,
                 std::unique_ptr<pros::Motor> rightArmMotor,
                 std::unique_ptr<pros::Motor> clawMotor,
                 std::unique_ptr<pros::adi::DigitalOut> clawPiston);

    void init();                               // call once; starts the task

    // These only set intent. The task commands the motors.
    void moveArmToPosition(ArmPosition pos);
    void releaseArm();                         // stop holding, motors coast

    // Piston is a direct call (single instant write)
    void setClaw(ClawPosition pos);
    void toggleClaw();

    void toggleArmPosition();

    double getArmPosition();                   // radians
    double getClawPosition();                  // radians
    void moveToLoadThenReady2();
    bool armAtTarget(double targetTicks);


private:
    enum class Mode { IDLE, POSITION, RELEASE, LOAD_THEN_READY2 };

    bool atTarget(double armTicks, double clawTicks, bool checkClaw);

    std::unique_ptr<pros::Motor> leftArmMotor;
    std::unique_ptr<pros::Motor> rightArmMotor;
    std::unique_ptr<pros::Motor> clawMotor;
    std::unique_ptr<pros::adi::DigitalOut> clawPiston;
    static constexpr double ARM_TOLERANCE_TICKS = 20.0;

    std::atomic<Mode> mode{Mode::IDLE};
    std::atomic<ArmPosition> currentPosition{LOAD};
    std::atomic<ClawPosition> currentClawPosition{OPEN};
    std::atomic<bool> newTarget{false};
    std::atomic<bool> isMoving{false};
    std::atomic<int> sequenceStep{0};

    std::unique_ptr<pros::Task> task;
    void run();

    const int32_t REV_COUNTS_RED = 1800;
    const int32_t REV_COUNTS_GREEN = 900;

    const double CLAW_MOTOR_SCORE_POSITION = -0.9;
    const double ARM_MOTOR_SCORE_POSITION = 2.00;
    const double CLAW_MOTOR_READY_POSITION = 0.0;
    const double ARM_MOTOR_READY_POSITION = 0.71;
    const double CLAW_MOTOR_READY2_POSITION = -2.4;
    const double ARM_MOTOR_READY2_POSITION = 0.71;
    const double CLAW_MOTOR_INSIZE_POSITION = 0;
    const double ARM_MOTOR_INSIZE_POSITION = 0;
    const double ARM_VELOCITY = 50;
    const double CLAW_VELOCITY = 50;
};

} // namespace arm
} // namespace subsystems
} // namespace robot
} // namespace platt2

#endif