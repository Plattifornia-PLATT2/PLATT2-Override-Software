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
    //cv::imwrite("gray_image.jpg", gray);
    zarray_t *detections = apriltag_detector_detect(tagDetector, &image);

    tagPos best_result;
    double best_error = std::numeric_limits<double>::max();
    std::cout << "Detected " << zarray_size(detections) << " tags.\n";
    
    for (int i = 0; i < zarray_size(detections); ++i)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        info.det     = det;

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

