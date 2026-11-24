#include "utilities/KalmanFilterHolonomicPose.hpp"
#include <cmath>
#include <numbers>

KalmanFilterHolonomicPose::KalmanFilterHolonomicPose(double q_pos, double q_theta, double q_vel, double q_w) {
    
    P.setZero(); 
    P.diagonal().setConstant(1.0);
    //set initialize values for P. The filter will change these values later.

    state_vector.setZero();
    // sigma_points.setZero();

    Q.setZero();
    Q.diagonal() << q_pos, q_pos, q_theta, q_vel, q_vel, q_w; // set the state weights

    // Init Filter
    double scaling_parameter = (std::pow(alpha, 2))*(n+kappa) - n;
    Wm(0) = scaling_parameter / (n+scaling_parameter);
    Wc(0) = Wm(0) + (1.0 - std::pow(alpha, 2) + beta);
    double weight = 1.0 / (2.0 * (n+scaling_parameter));
    for (int i = 1; i < n_sigmas; ++i) {
        Wm(i) = weight;
        Wc(i) = weight;
    }
};
Eigen::Matrix<double, 6, 13> KalmanFilterHolonomicPose::generate_sigma_points(const Eigen::Matrix<double, 6, 1>& state, const Eigen::Matrix<double, 6, 6>& cov) {
    Eigen::Matrix<double, 6, 13> sigma_points;
    sigma_points.setZero();
    double scaling_parameter = (std::pow(alpha, 2))*(n+kappa) - n;

    // Cholsky decompostion using ldlt (more stable and handles positive semi-definite better)
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(cov);
    Eigen::Matrix<double, 6, 6> L = ldlt.transpositionsP().transpose() * Eigen::MatrixXd(ldlt.matrixL());
    Eigen::Vector<double, 6> D_sqrt = ldlt.vectorD().cwiseMax(0.0).cwiseSqrt();
    
    Eigen::Matrix<double, 6, 6> P_sqrt = L * D_sqrt.asDiagonal();

    sigma_points.col(0) = state;
    for (int i = 0; i < n; ++i) { 
        sigma_points.col(i + 1) = (state - std::sqrt(n + scaling_parameter) * P_sqrt.col(i));
        sigma_points.col(i + 1 + n) = (state + std::sqrt(n + scaling_parameter) * P_sqrt.col(i));
    }
    return sigma_points;
};

Eigen::Vector<double, 6> KalmanFilterHolonomicPose::transition_function(const Eigen::Vector<double, 6>& current_state, double dt) {
    double next_x = current_state(0) + dt*current_state(3);
    double next_y = current_state(1) + dt*current_state(4);
    double next_theta = current_state(2) + dt*current_state(5);
    next_theta = normalize_theta(next_theta);
    Eigen::Vector<double, 6> return_vector;
    return_vector << next_x, next_y, next_theta, current_state(3), current_state(4), current_state(5);
    return return_vector;
};
Eigen::Vector3d KalmanFilterHolonomicPose::measurementFunction(const Eigen::Matrix<double, 6, 1> &state_point) {
    return state_point.head<3>(); //get first 3 (x, y, theta) of state_vector
}
double KalmanFilterHolonomicPose::normalize_theta(double angle) {
    constexpr double pi = 3.14159265358979323846;
    double const min = -pi;
    double const max = pi;
    double value = angle;
    // Keeps the value within the interval min to max
    // This should be used on the rotation readings for consistency purposes

    double modulus = max - min;

    int numMax = (int)((value - min) / modulus);
    value -= numMax * modulus;

    int numMin = (int)((value - max) / modulus);
    value -= numMin * modulus;

    return value;
 
}
void KalmanFilterHolonomicPose::predict(double dt) {
    // Predicts the future state of the system
    Eigen::Matrix<double, 6, 13> sigma_points = generate_sigma_points(state_vector, P);
    Eigen::Matrix<double, 6, 13> sigmas_f; //sigma points passed through the transition function
    for (int i = 0; i < n_sigmas; ++i) {
        sigmas_f.col(i) = transition_function(sigma_points.col(i), dt);
    }

    state_vector.setZero();
    for (int i = 0; i < n_sigmas; ++i) {
        state_vector(0) += Wm(i) * sigmas_f(0, i);
        state_vector(1) += Wm(i) * sigmas_f(1, i);
        state_vector(3) += Wm(i) * sigmas_f(3, i);
        state_vector(4) += Wm(i) * sigmas_f(4, i);
        state_vector(5) += Wm(i) * sigmas_f(5, i);
    }

    // Special case to handle angle wrapping:
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (int i = 0; i < n_sigmas; ++i) {
        sum_sin += Wm(i) * std::sin(sigmas_f(2, i));
        sum_cos += Wm(i) * std::cos(sigmas_f(2, i));
    }
    state_vector(2) = std::atan2(sum_sin, sum_cos);

    P.setZero();

    for (int i = 0; i < n_sigmas; ++i) {
        Eigen::Matrix<double, 6, 1> diff; 
        diff(0) = sigmas_f(0, i) - state_vector(0);
        diff(1) = sigmas_f(1, i) - state_vector(1);
        diff(3) = sigmas_f(3, i) - state_vector(3);
        diff(4) = sigmas_f(4, i) - state_vector(4);
        diff(5) = sigmas_f(5, i) - state_vector(5);

        diff(2) = normalize_theta(sigmas_f(2, i) - state_vector(2));

        P += Wc(i) * diff * diff.transpose();
    }
    P += Q;
};
void KalmanFilterHolonomicPose::update_sensor(const Eigen::Vector3d& z, const Eigen::Vector3d& standard_deviations) {
    Eigen::Matrix<double, 6, 13> sigma_points = generate_sigma_points(state_vector, P);
    Eigen::Matrix<double, 3, 3> R;
    R.setZero();
    R.diagonal() << std::pow(standard_deviations(0), 2), std::pow(standard_deviations(1), 2), std::pow(standard_deviations(2), 2);
    // Converting standard deviations into variances

    Eigen::Matrix<double, 3, 13> sigma_z; //sigma points into measurement space
    for (int i = 0; i < n_sigmas; ++i) {
        sigma_z.col(i) = measurementFunction(sigma_points.col(i));
    }

    // Mean prediction of measurement
    Eigen::Vector3d z_pred;
    z_pred.setZero();
    double sum_sin = 0.0;
    double sum_cos = 0.0; 

    for (int i = 0; i < n_sigmas; ++i) {
        z_pred(0) += Wm(i) * sigma_z(0, i);
        z_pred(1) += Wm(i) * sigma_z(1, i);
        //z_pred += Wm(i) * sigma_z.col(i);
        sum_sin += Wm(i) * std::sin(sigma_z(2, i));
        sum_cos += Wm(i) * std::cos(sigma_z(2, i));
    }
    z_pred(2) = std::atan2(sum_sin, sum_cos);

    // Measurement Covariance (Py) and Cross-Covariance (Pxz)
    Eigen::Matrix3d Py;
    Eigen::Matrix<double, 6, 3> Pxy;
    Py.setZero();
    Pxy.setZero();
    for (int i = 0; i < n_sigmas; ++i) {
        Eigen::Vector3d z_diff = sigma_z.col(i) - z_pred;
        z_diff(2) = normalize_theta(z_diff(2));
        Py += Wc(i) * z_diff * z_diff.transpose(); 
        Eigen::Vector<double, 6> x_diff = sigma_points.col(i) - state_vector;  
        x_diff(2) = normalize_theta(x_diff(2));      
        Pxy += Wc(i) * x_diff * z_diff.transpose();
    }
    Py += R; // + R(k) for additive noise

    //Kalman Gain
    Eigen::Matrix<double, 6, 3> K = Pxy * Py.inverse();
    Eigen::Vector3d z_residual;
    z_residual(0) = z(0) - z_pred(0);
    z_residual(1) = z(1) - z_pred(1);
    z_residual(2) = normalize_theta(z(2) - z_pred(2)); 

    state_vector(0) += K.row(0) * z_residual;
    state_vector(1) += K.row(1) * z_residual;
    state_vector(3) += K.row(3) * z_residual;
    state_vector(4) += K.row(4) * z_residual;
    state_vector(5) += K.row(5) * z_residual;
    
    // Correctly apply heading update to the state vector
    state_vector(2) = normalize_theta(state_vector(2) + (K.row(2) * z_residual));
    
    P -= K * Py * K.transpose();

}
Eigen::Vector3d KalmanFilterHolonomicPose::get_pose_estimation() {
    return state_vector.head<3>();
}
void KalmanFilterHolonomicPose::reset_filter(Eigen::Vector<double, 6> old_state_vector, Eigen::Matrix<double, 6, 6> old_P) {
    P = old_P;
    state_vector = old_state_vector;
}
