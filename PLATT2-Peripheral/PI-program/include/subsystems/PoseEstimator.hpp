
#ifndef POSE_ESTIMATOR
#define POSE_ESTIMATOR

#include <Eigen/Dense>
#include <Eigen/Core>
#include <chrono>
#include "utilities/KalmanFilterHolonomicPose.hpp"
#include <map>
#include <vector>
#include <stop_token>
#include "utilities/sharedData.hpp"

class PoseEstimator {
    /*
    INFO:
    In order for UKF to properly work, the two sensors 
    must be in the same exact global coordinate system. 
    Otherwise, this can cause issues. 
    Some solutions: 
    1.) know the EXACT starting point of the robot (doesn't require UKF code modifications)
    2.) Anchor the odometry sensors inital frame to the camera's output (this is probably the best option)
    3.) State augmentation (more complication) require biases for x/y/theta
    - Using the odometry sensor for local changes
        4.) Treat the change in x/y/theta as velocity/delta measurements (require code modification)
        5.) Use the change in x/y/theta to drive the state transition model.  
    */
    private:
    std::chrono::steady_clock::time_point last_time;
    bool is_init = false;
    
    
    public:
    using TimeStamp = std::chrono::steady_clock::time_point;


    struct StateUKF {
        PoseReading pose_measurement;
        Eigen::Matrix<double, 6, 6> P; // covariance
        Eigen::Vector<double, 6> state_vector; 
    };

    using Buffer = std::multimap<TimeStamp, StateUKF>; 
    //multimap is better incase two sensors share the same exact timestamp
    Buffer filter_buffer_window;

    //std::map<TimeStamp, StateUKF> filter_buffer_window;
    const std::chrono::milliseconds window_ms{30}; //ms

    KalmanFilterHolonomicPose UKF{0.01, 0.02, 0.1, 0.2};
    // Recommeded Values: q_pos = 0.01, q_theta = 0.02, q_vel = 0.1, q_w = 0.2
    // These should get tuned later

    PoseEstimator(); 
    double get_delta_time(TimeStamp current_time);
    void update_sensor_reading(const PoseReading& pose_reading);
    Pose get_current_pose_estimation();

    void kalmanLoop(std::stop_token stopToken, sharedData& shared);

    //void add_measurement(const PoseReading& pose_reading, TimeStamp measurement_time); OLD: used for map<>
    PoseEstimator::Buffer::iterator add_measurement(const PoseReading& pose_reading, TimeStamp measurement_time);
    // void rewindUKF(TimeStamp measurement_time); OLD: used for map<>
    void rewindUKF(PoseEstimator::Buffer::iterator start_it);
    void cleanup_window(TimeStamp current_time);
};

#endif