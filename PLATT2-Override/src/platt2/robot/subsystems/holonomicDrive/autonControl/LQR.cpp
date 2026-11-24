
#include "platt2/robot/subsystems/holonomicDrive/autonControl/LQR.hpp"

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

Eigen::Matrix<double, 3, 6> LQR::computeLqrGain( double tau, double dt, const Eigen::Matrix<double, 6, 1>& qDiag, const Eigen::Vector3d& rDiag)
{
   
    // Continuous model: state = [pos error; vel error], input = velocity correction
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity() / tau;

    Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
    B.block<3, 3>(3, 0) = kV.asDiagonal()  * (1.0 / tau);

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
Eigen::Vector3d LQR::lqrTrack(const Eigen::Vector3d& posDes, const Eigen::Vector3d& velDes,
                              const Eigen::Vector3d& velFF,  const Eigen::Vector3d& accFF,
                              const Eigen::Vector3d& posMeas, const Eigen::Vector3d& velMeas,
                              double fbScale)
{
    Eigen::Matrix<double, 6, 1> e;
    e.head<3>() = posMeas - posDes;
    e(2) = std::remainder(e(2), 2.0 * M_PI);
    e.tail<3>() = velMeas - velDes;               // error vs. the CURRENT reference

    Eigen::Vector3d uFF = (velFF + tau * accFF).cwiseQuotient(kV);   // look-ahead, feedforward only
    for (int k = 0; k < 2; ++k)
        if (std::abs(velFF(k)) > 0.5) uFF(k) += kS * (velFF(k) > 0 ? 1.0 : -1.0);
    
    static int n = 0;
    if (n++ % 25 == 0)
        printf("K11=%.3f K14=%.3f fb_x=%.3f b_y=%.3f scale=%.2f\n",
           K(1, 1), K(1, 4), -(K * e)(1), -(K * e)(0), fbScale);

    Eigen::Vector3d fb  = fbScale * (K * e);
    Eigen::Vector3d out = uFF - fb;
    static int z = 0;
    //if (z++ % 25 == 0)
    //    printf("uFF_y=%.3f fb_y=%.3f out_y=%.3f\n", uFF(1), -fb(1), out(1));
    return out;
}


}}}}