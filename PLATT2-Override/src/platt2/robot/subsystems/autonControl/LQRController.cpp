#include "platt2\robot\subsystems\autonControl\LQRController.hpp"





namespace platt2{


namespace robot{

namespace subsystems{

namespace autonControl{

namespace LQRcontroller{


Eigen::Matrix<double, 3, 6> computeLqrGain(double tau, double dt, const Eigen::Matrix<double, 6, 1>& qDiag, const Eigen::Vector3d& rDiag)
{
    // Continuous model: state = [pos error; vel error], input = velocity correction
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity() / tau;

    Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
    B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() / tau;

    // Discretize (Euler is fine when dt << tau)
    Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Identity() + A * dt;
    Eigen::Matrix<double, 6, 3> Bd = B * dt;

    Eigen::Matrix<double, 6, 6> Q = qDiag.asDiagonal();
    Eigen::Matrix3d R = rDiag.asDiagonal();

    // Iterate the discrete Riccati equation to steady state
    Eigen::Matrix<double, 6, 6> P = Q;
    Eigen::Matrix<double, 3, 6> K = Eigen::Matrix<double, 3, 6>::Zero();
    for (int i = 0; i < 5000; ++i) {
        Eigen::Matrix3d S = R + Bd.transpose() * P * Bd;
        K = S.ldlt().solve(Bd.transpose() * P * Ad);
        Eigen::Matrix<double, 6, 6> Pn = Q + Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * K;
        if ((Pn - P).cwiseAbs().maxCoeff() < 1e-9) { P = Pn; break; }
        P = Pn;
    }
    return K;
}

// Run every control loop. All quantities are in the field frame.
Eigen::Vector3d lqrTrack(const Eigen::Matrix<double, 3, 6>& K, double tau,
              const Eigen::Vector3d& posDesired, const Eigen::Vector3d& velDesired, const Eigen::Vector3d& accDesired,
              const Eigen::Vector3d& posMeasured, const Eigen::Vector3d& velMeasured)
{
    Eigen::Matrix<double, 6, 1> e;
    e.head<3>() = posMeasured - posDesired;
    e(2) = std::remainder(e(2), 2.0 * M_PI);       // wrap heading error to [-pi, pi]
    e.tail<3>() = velMeasured - velDesired;

    Eigen::Vector3d uFF = velDesired + tau * accDesired;       // lag-compensating feedforward
    return uFF - K * e;                             // commanded (vx, vy, omega)
}


}}}}}