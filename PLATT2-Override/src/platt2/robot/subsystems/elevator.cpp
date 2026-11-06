#include "platt2/robot/subsystems/elevator/elevator.hpp"
#include "pros/motors.h"

namespace platt2{

namespace robot{

namespace subsystems{

namespace elevator{

    static void taskThunk(void *p) {
    reinterpret_cast<ElevatorSubsystem*>(p)->elevatorTaskLoop();
}

    ElevatorSubsystem::ElevatorSubsystem(
        std::unique_ptr<pros::Motor> leftMotor,
        std::unique_ptr<pros::Motor> rightMotor,
        std::unique_ptr<pros::Rotation> rotationSensor,
        std::unique_ptr<robot::pid::PID> elevatorPID

    ) :
        leftMotor{std::move(leftMotor)},
        rightMotor{std::move(rightMotor)},
        rotationSensor{std::move(rotationSensor)},
        elevatorTask(taskThunk, this)

    {

    }

    void ElevatorSubsystem::initElevator(){
        currentPosition = FULL_DOWN;
        leftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rotationSensor->reset_position();
    }

    uint32_t ElevatorSubsystem::getPosInches(ElevatorPosition pos){
        //TODO: implementation
        return 0;
    }

    void ElevatorSubsystem::moveToPosition(ElevatorPosition pos){
        currentPosition = pos;
    }

    void ElevatorSubsystem::elevatorTaskLoop(){
        ElevatorPosition prevPosition = currentPosition;
        double target = getPosInches(currentPosition);

    // dt must match the delay below: 20 ms = 0.02 s
    // max/min are in millivolts to match move_voltage (+-12000)
    //platt2::robot::pid::PID pid(0.02, 12000, -12000, Kp, Kd, Ki);

    while(true){
        // New target requested: pick the new setpoint and clear old PID state
        if(currentPosition != prevPosition){
            target = getPosInches(currentPosition);
            elevatorPID->resetPID();
            prevPosition = currentPosition;
        }

        double pv = rotationSensor->get_position();   // centidegrees
        double output = elevatorPID->calculate(target, pv);

        leftMotor->move_voltage(output);
        rightMotor->move_voltage(output);

        pros::delay(20);
    }
    }

}}}}