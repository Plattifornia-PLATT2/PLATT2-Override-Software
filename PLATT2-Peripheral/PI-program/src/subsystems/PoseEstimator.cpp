#include "subsystems/PoseEstimator.hpp"
#include <iostream>


PoseEstimator::PoseEstimator() {

}
double PoseEstimator::get_delta_time(TimeStamp current_time) {
    if (!is_init) {
        last_time = current_time;
        is_init = true;
        return 0.0; 
    }
    std::chrono::duration<double> time_change = current_time - last_time;
    double dt = time_change.count();
    last_time = current_time;
    return dt; 
}
void PoseEstimator::update_sensor_reading(const PoseReading& pose_reading) {
    auto time_now = std::chrono::steady_clock::now();

    if (pose_reading.latency != 0.0) {
        const auto lat = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(pose_reading.latency));
        const TimeStamp meas_time = time_now - lat;
        if (filter_buffer_window.empty() || meas_time <= filter_buffer_window.begin()->first) {
            return;
        }
        auto iterator = add_measurement(pose_reading, meas_time); //this will just get overwritten with the rewind
        rewindUKF(iterator);
    }
    
    else {
        double dt = get_delta_time(time_now);
        if (dt > 0.0) UKF.predict(dt); 
        Eigen::Vector3d position_orientation(pose_reading.pose.x, pose_reading.pose.y, pose_reading.pose.theta);
        Eigen::Vector3d standard_deviations(pose_reading.std_x, pose_reading.std_y, pose_reading.std_theta);
        UKF.update_sensor(position_orientation, standard_deviations);
        add_measurement(pose_reading, time_now);
    }
    cleanup_window(time_now);
}
Pose PoseEstimator::get_current_pose_estimation() {
    Eigen::Vector3d current_pose_vec = UKF.get_pose_estimation();
    Pose current_pose{current_pose_vec(0), current_pose_vec(1), current_pose_vec(2)};
    return current_pose;
}
PoseEstimator::Buffer::iterator PoseEstimator::add_measurement(const PoseReading& pose_reading, TimeStamp measurement_time) {
    return filter_buffer_window.emplace(
        measurement_time,
        StateUKF{pose_reading, UKF.P, UKF.state_vector});
    
    // filter_buffer_window[measurement_time] = PoseEstimator::StateUKF{
    //     pose_reading,
    //     UKF.P,
    //     UKF.state_vector
    // }; OLD: used with map<>
}
void PoseEstimator::cleanup_window(TimeStamp current_time) {
    auto cutoff = filter_buffer_window.lower_bound(current_time - window_ms);
    if (cutoff != filter_buffer_window.begin()) --cutoff;  // keep one base entry
    filter_buffer_window.erase(filter_buffer_window.begin(), cutoff);
}

void PoseEstimator::rewindUKF(PoseEstimator::Buffer::iterator start_it) {

    // auto start_it = filter_buffer_window.lower_bound(measurement_time); OLD: used with map<>

    PoseEstimator::TimeStamp prev_time = start_it->first;

    if (start_it != filter_buffer_window.begin()) {
        auto prev = std::prev(start_it);
        UKF.reset_filter(prev->second.state_vector, prev->second.P);
        prev_time = prev->first;
    }

    for (auto it = start_it; it != filter_buffer_window.end(); ++it) {
        const double dt = std::chrono::duration<double>(it->first - prev_time).count();
        if (dt > 0.0) UKF.predict(dt);

        const auto& it_state = it -> second;

        Eigen::Vector3d position_orientation(it_state.pose_measurement.pose.x, it_state.pose_measurement.pose.y, it_state.pose_measurement.pose.theta);
        Eigen::Vector3d standard_deviations(it_state.pose_measurement.std_x, it_state.pose_measurement.std_y, it_state.pose_measurement.std_theta);
        UKF.update_sensor(position_orientation, standard_deviations);

        it -> second.state_vector = UKF.state_vector;
        it -> second.P = UKF.P;

        prev_time = it -> first;
    }

    last_time = prev_time;

}


void PoseEstimator::kalmanLoop(std::stop_token stopToken, sharedData& shared){

    PoseReading OTOSReading;
    PoseReading CamReading;

    while(!stopToken.stop_requested()){

        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            OTOSReading = shared.OTOSPos;
            CamReading = shared.camPos;
        }

        if(OTOSReading.newData){

            update_sensor_reading(OTOSReading);
            {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.OTOSPos.newData = false;
            }

        }

        if(CamReading.newData){

            update_sensor_reading(CamReading);
            {
            std::lock_guard<std::mutex> lock(shared.mtx);
                shared.camPos.newData = false;
            }
        }
        

        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            shared.kalmanPos =  get_current_pose_estimation();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

}