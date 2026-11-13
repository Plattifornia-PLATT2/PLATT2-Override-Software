#ifndef ARM_HPP
#define ARM_HPP

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include <memory>
namespace platt2 {

namespace robot {

namespace subsystems {

namespace arm {

    enum ArmPosition{
        LOAD,
        SCORE,
        IN_SIZE
    };

    enum ClawPosition{
        CLOSED = 0,
        OPEN = 1
    };

    class ArmSubsystem{
        public:

        ArmSubsystem(
            std::unique_ptr<pros::Motor> armMotor,
            std::unique_ptr<pros::Motor> clawMotor,
            std::unique_ptr<pros::adi::DigitalOut> clawPiston
        );

        void init();

        void moveArmToPosition(ArmPosition pos);

        void setClaw(ClawPosition pos);

        void toggleArmPosition();

        std::int32_t getArmPosition();
        private:
            std::unique_ptr<pros::Motor> armMotor;
            std::unique_ptr<pros::Motor> clawMotor;
            std::unique_ptr<pros::adi::DigitalOut> clawPiston;

            ArmPosition currentPosition;

            const std::int32_t CLAW_MOTOR_SCORE_POSITION = 0;
            const std::int32_t ARM_MOTOR_SCORE_POSITION = 0;
            const std::int32_t CLAW_MOTOR_READY_POSITION = 0;
            const std::int32_t ARM_MOTOR_READY_POSITION = 0;
            const std::int32_t CLAW_MOTOR_INSIZE_POSITION = 0;
            const std::int32_t ARM_MOTOR_INSIZE_POSITION = 0;

            const std::int32_t ARM_VELOCITY = 100;
            const std::int32_t CLAW_VELOCITY = 200;
    };

}}}}

#endif