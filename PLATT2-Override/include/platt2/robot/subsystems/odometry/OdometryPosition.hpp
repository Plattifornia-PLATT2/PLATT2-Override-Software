#ifndef ODOMETRYPOSITION_HPP
#define ODOMETRYPOSITION_HPP

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for odometry subsystem related code.
 * @author Dominic Young
 */
namespace odometry{
    /**
     * @brief Struct representing the position of the robot on the field.
     * @author Dominic Young
     */
    struct Position{
        public:
        double x = 0;
        double y = 0;
        double heading = 90;
    };
}
}
}
}

#endif