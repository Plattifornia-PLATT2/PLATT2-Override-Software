#ifndef ROBOTSUBSYSTEMS_HPP
#define ROBOTSUBSYSTEMS_HPP

#include <memory>
#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "robot/subsystems/externalLink/piLink.hpp"

namespace platt2 {
namespace robot {

struct RobotSubsystems {
    std::shared_ptr<subsystems::holonomicDrive::XDrive> xdriveSubsystem;
    std::shared_ptr<subsystems::odometry::Odometry> odometrySubsystem;
    std::shared_ptr<subsystems::holonomicDrive::HolonomicControl> holonomicControlSubsystem;
    std::shared_ptr<subsystems::intake::IntakeSubsystem> intakeSubsystem;
    std::shared_ptr<subsystems::piLink::PiLinkSubsystem> piLinkSubsystem;
};

}}
#endif