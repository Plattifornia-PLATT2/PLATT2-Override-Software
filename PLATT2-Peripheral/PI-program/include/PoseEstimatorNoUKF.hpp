#ifndef POSE_ESTIMATOR_NO_UKF
#define POSE_ESTIMATOR_NO_UKF

#include <Eigen/Dense>
#include <Eigen/Core>

struct Pose {
    double x;
    double y;
    double theta;
};

class PoseEstimator { 
    public:
    PoseEstimator(Eigen::RowVector3d vect);
    
    //TimestampQueueMap class

    Eigen::RowVector3d vec;
    Pose get_pose_estimation();
    void add_vision_measurement(Pose pose, double timestamp, double std_dev); //timestamp or latency???
    void add_odometry_measurement(Pose pose, double timestamp = 0);
    void update();
    

};


#endif