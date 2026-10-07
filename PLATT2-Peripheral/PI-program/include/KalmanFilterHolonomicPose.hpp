#ifndef KALMAN_FILTER_POSE
#define KALMAN_FILTER_POSE

#include <Eigen/Dense>
#include <Eigen/Core>

class KalmanFilterHolonomicPose {
    public:
    
    const int n = 6; 
    // ----------------- UKF hyperparameters --------------------------
    double alpha = 0.001;
    double beta = 2.0;
    double kappa = 0.0;
    // ----------------- See Mathworks for good tuning guide ---------

    const int n_sigmas = n*2 + 1;
    Eigen::Vector<double, 6> state_vector; 
    // X
    // Y
    // Theta
    // Vx
    // Vy
    // w

    Eigen::Matrix<double, 6, 6> P;
    Eigen::Matrix<double, 6, 6> Q; 
    // Eigen::Matrix<double, 6, 6> R; R is specific to each sensor
    
    // Eigen::Matrix<double, 6, 13> sigma_points; //each row in the 2 by 5 matrix represents a sigma point. n by (n * 2) + 1
    Eigen::Vector<double, 13> Wm;
    Eigen::Vector<double, 13> Wc;

    /*
    Meaning behind variables names:
    state_vector (x) --> estimated values
    n --> dimensions
    P --> error covariance matrix
    f(x) --> state transition function (for angle wrapping), h(x) --> measurement function
    Q --> process noise covariance matrix (tell how much to trust constant linear model)
    R --> measurement noise covariance matrix
    Z --> actual measurement
    sigma_points (X) --> sigma points that capture the mean and covariance of the state
    Wm --> the weight associated with the i-th sigma point used for calculating the mean
    Wc --> the weight associated with the i-th sigma point used for calculating the covariance
    scaling_factor (lambda) --> used to spread sigma points
    alpha --> spread of the sigma points around the mean state (usually 10^-3)
    kappa --> secondary scaling parameter (usually 0 or 3 - n)
    beta --> incorporates prior knowledge of the state distribution (2 is optimal for Gaussian distributions)
    K (kalman gain) --> how much weight to give to the new sensor vs the prediciton
    
    */
    
    KalmanFilterHolonomicPose(double q_pos, double q_theta, double q_vel, double q_w); 
    // recommeded values: q_pos = 0.01, q_theta = 0.02, q_vel = 0.1, q_w = 0.2
    // lower numbers means you trust model a lot and rely less on the constant velocity model. 
    
    // a = 0.001, b = 2.0, k = 0

    Eigen::Matrix<double, 6, 13> generate_sigma_points(const Eigen::Matrix<double, 6, 1>& state, const Eigen::Matrix<double, 6, 6>& cov);

    Eigen::Vector<double, 6> transition_function(const Eigen::Vector<double, 6>& current_state, double dt);
    Eigen::Vector3d measurementFunction(const Eigen::Matrix<double, 6, 1>& state_point);
    void predict(double dt);
    void update_sensor(const Eigen::Vector3d& z, const Eigen::Vector3d& standard_deviations);
    double normalize_theta(double angle);
    Eigen::Vector3d get_pose_estimation();
    void reset_filter(Eigen::Vector<double, 6> old_state_vector, Eigen::Matrix<double, 6, 6> old_P);
};

#endif