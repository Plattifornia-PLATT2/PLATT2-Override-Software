#ifndef IHOLONOMIC_HPP
#define IHOLONOMIC_HPP

#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

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
     * @brief Interface for a holonomic drive control system.
     * @author Dominic Young
     */
    class IHolonomic {
        public:

        virtual ~IHolonomic() = default;

        /**
         * @brief Moves the holonomic drive using a movement vector.
         * 
         * @param v The movement vector containing movement parameters.
         */
        virtual void moveVector(MovementVector v) = 0;

    };


};
};
};
};

#endif