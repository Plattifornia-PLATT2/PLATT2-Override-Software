#include "utilities/imageProssesing.hpp"
    //Camera camera("frontCam");
    //camera.printCameraInfo();
    //auto pos = camera.getTagPos();
    //std::cout << "Tag ID: " << pos.tag_id << "\n";
    //std::cout << "Position (x, y, z): (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    //std::cout << "Reprojection Error: " << pos.reproj_error << "\n";
std::vector<Camera::tagInfo> Camera::getTagPos() {

    cv::Mat frame, gray;
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    //cv::imwrite("gray.jpg", gray);

    image_u8_t image = {
        .width  = gray.cols,
        .height = gray.rows,
        .stride = (int32_t)gray.step,
        .buf    = gray.data
    };

    auto start = std::chrono::high_resolution_clock::now();
    zarray_t *detections = apriltag_detector_detect(tagDetector, &image);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: "<<duration.count()<<std::endl;

    
    std::vector<tagInfo> result;
    tagInfo stepRes;
    
    for (int i = 0; i < zarray_size(detections); ++i){

        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // --- draw overlay for this detection ---
        for (int j = 0; j < 4; j++) {
            cv::Point2d p1(det->p[j][0], det->p[j][1]);
            cv::Point2d p2(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]);
            cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
        }
        cv::Point2d center(det->c[0], det->c[1]);
        cv::circle(frame, center, 4, cv::Scalar(0, 0, 255), -1);
        cv::putText(frame, std::to_string(det->id),
                    center + cv::Point2d(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
        // ----------------------------------------

        info.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        
        if (err < 0.1){

            stepRes = tagInfo {
                .pos          = { MATD_EL(pose.t, 0, 0), MATD_EL(pose.t, 2, 0) },
                .z            = MATD_EL(pose.t, 1, 0),
                .angle        = getHorizontalAngle(pose.R),
                .tag_id       = det->id,
                .reproj_error = err
            };

            result.push_back(stepRes);
        }

        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }

    cv::imwrite("detections.jpg", frame);

    return result;
}

void Camera::capImg(){


    cv::Mat frame, gray;
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::imwrite("gray.jpg", gray);



    
}

double Camera::getHorizontalAngle(matd_t* R) {
    // Tag's local Z axis (its normal), expressed in camera coordinates,
    // is the third column of R: [R(0,2), R(1,2), R(2,2)]
    double r02 = MATD_EL(R, 0, 2);
    double r22 = MATD_EL(R, 2, 2);

    // Angle of that normal within the camera's horizontal plane,
    // relative to the camera's forward (z) axis
    double angleRad = std::atan2(r02, r22);
    return angleRad * 180.0 / M_PI; // convert to degrees
}

bool Camera::waitForExposureSettled(int maxFrames, double clippedFraction, int stableFramesRequired) {
    cv::Mat gray;
    int stableCount = 0;

    for (int i = 0; i < maxFrames; ++i) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        int total   = gray.rows * gray.cols;
        int clipped = cv::countNonZero(gray >= 250);
        double fracClipped = (double)clipped / total;

        //std::cout << "frame " << i << " clipped: " << fracClipped * 100.0 << "%" << std::endl;

        if (fracClipped < clippedFraction) {
            stableCount++;
            if (stableCount >= stableFramesRequired) return true;
        } else {
            stableCount = 0;
        }
    }
    return false;
}

