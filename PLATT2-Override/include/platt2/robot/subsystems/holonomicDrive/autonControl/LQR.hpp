#pragma once 

#include "Eigen/Dense"

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
 * @brief Control class for holonomic drive systems.
 * @authors Dominic Young and Logan Wolf
 */

class LQR {


    private:
    Eigen::Matrix<double, 3, 6> K;
    double tau = 1;

    public:
        LQR(double tau, double dt, const Eigen::Matrix<double, 6, 1>& qDiag, const Eigen::Vector3d& rDiag)
            : K(computeLqrGain(tau, dt, qDiag, rDiag)), tau(tau)
        {}

        Eigen::Vector3d lqrTrack(const Eigen::Vector3d& posDesired,
                                 const Eigen::Vector3d& velDesired,
                                 const Eigen::Vector3d& accDesired,
                                 const Eigen::Vector3d& posMeasured,
                                 const Eigen::Vector3d& velMeasured);

        Eigen::Matrix<double, 3, 6> computeLqrGain(double tau, double dt,
                                                   const Eigen::Matrix<double, 6, 1>& qDiag,
                                                   const Eigen::Vector3d& rDiag);
};


   


}

}}}