#include "imageProssesing.hpp"

Camera::tagPos Camera::getTagPos() {

    cv::Mat frame, gray;
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::imwrite("gray.jpg", gray);
    
    image_u8_t image = {
        .width  = gray.cols,
        .height = gray.rows,
        .stride = (int32_t)gray.step,
        .buf    = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(tagDetector, &image);

    tagPos best_result;
    double best_error = std::numeric_limits<double>::max();

    for (int i = 0; i < zarray_size(detections); ++i){
        
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        info.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        if (err < best_error) {
            best_error  = err;
            best_result = tagPos {
                .x            = MATD_EL(pose.t, 0, 0),
                .y            = MATD_EL(pose.t, 1, 0),
                .z            = MATD_EL(pose.t, 2, 0),
                .tag_id       = det->id,
                .reproj_error = err
            };
        }

        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }

    return best_result;
}