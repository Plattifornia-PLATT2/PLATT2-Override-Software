#include "imageProssesing.hpp"

Camera::tagPos Camera::getImagePos() {

    
    cv::Mat frame, gray;
    cap >> frame;
    if (frame.empty())
        throw std::runtime_error("Failed to capture frame from VideoCapture.");

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    image_u8_t image = {
        .width  = gray.cols,
        .height = gray.rows,
        .stride = (int32_t)gray.step,
        .buf    = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(tagDetector, &image);

    for (int i = 0; i < zarray_size(detections); ++i)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        info.det     = det;
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
    }

}
