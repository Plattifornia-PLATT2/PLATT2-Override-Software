#ifndef MOVEMENTVECTOR_HPP
#define MOVEMENTVECTOR_HPP

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
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */
namespace holonomicDrive{

    /**
     * @brief Struct representing a movement vector for holonomic drive.
     * @author Dominic Young
     */
    struct MovementVector {
    double r;
    double theta;
    double w;
    double normalization_scalar = 1;  
};
}
}
}
}
#endif