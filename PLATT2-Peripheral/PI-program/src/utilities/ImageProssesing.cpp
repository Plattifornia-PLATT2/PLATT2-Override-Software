#include "utilities/imageProssesing.hpp"
    //Camera camera("frontCam");
    //camera.printCameraInfo();
    //auto pos = camera.getTagPos();
    //std::cout << "Tag ID: " << pos.tag_id << "\n";
    //std::cout << "Position (x, y, z): (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    //std::cout << "Reprojection Error: " << pos.reproj_error << "\n";
Camera::tagInfo Camera::getTagPos() {
    auto frameTimeStamp = std::chrono::high_resolution_clock::now();
    cv::Mat frame, gray;
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    image_u8_t image = {
        .width  = gray.cols,
        .height = gray.rows,
        .stride = (int32_t)gray.step,
        .buf    = gray.data
    };

    zarray_t* detections = apriltag_detector_detect(tagDetector, &image);

    tagInfo best;               
    const apriltag_detection_t* bestDet = nullptr;
    double bestScore = -std::numeric_limits<double>::infinity();  // #include <limits>

    for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // --- overlay (unchanged) ---
        // ...

        info.det = det;
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        if (err < 0.1) {
            double score = -err;                 // higher is better, see below

            if (score > bestScore) {
                bestScore = score;
                bestDet   = det;
                best = tagInfo{
                    .pos          = { MATD_EL(pose.t, 0, 0), MATD_EL(pose.t, 2, 0),
                                      getHorizontalAngle(pose.R) },
                    .z            = MATD_EL(pose.t, 1, 0),
                    .tag_id       = det->id,
                    .reproj_error = err,
                    .timeStamp    = frameTimeStamp
                };
            }
        }

        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }

    
    apriltag_detections_destroy(detections);     // must come after bestDet is used

    return best;
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

Camera::tagStd Camera::estimateTagStd(const apriltag_detection_t* det,
                                      double sigmaPx, int N)
{
    const double h = info.tagsize / 2.0;
    const std::vector<cv::Point3d> obj = {
        {-h,  h, 0}, { h,  h, 0}, { h, -h, 0}, {-h, -h, 0}
    };

    const cv::Matx33d K(info.fx, 0,       info.cx,
                        0,       info.fy, info.cy,
                        0,       0,       1);

    thread_local std::mt19937 rng{42};
    std::normal_distribution<double> noise(0.0, sigmaPx);

    // Nominal angle, used to unwrap the perturbed angles
    double nominal = getHorizontalAngle(/* your pose.R */ nullptr); // see note below

    std::vector<double> xs, zs, angs;
    xs.reserve(N); zs.reserve(N); angs.reserve(N);

    for (int i = 0; i < N; ++i) {
        std::vector<cv::Point2d> img(4);
        for (int k = 0; k < 4; ++k)
            img[k] = { det->p[k][0] + noise(rng), det->p[k][1] + noise(rng) };

        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(obj, img, K, cv::noArray(), rvec, tvec,
                          false, cv::SOLVEPNP_IPPE_SQUARE))
            continue;

        cv::Matx33d R;
        cv::Rodrigues(rvec, R);

        xs.push_back(tvec[0]);
        zs.push_back(tvec[2]);
        angs.push_back(std::atan2(R(0, 2), R(2, 2)) * 180.0 / M_PI);
    }

    if (xs.size() < 10) return {1e3, 1e3, 1e3};   // fail safe: huge uncertainty

    auto stddev = [](const std::vector<double>& v) {
        double m = 0; for (double a : v) m += a; m /= v.size();
        double s = 0; for (double a : v) s += (a - m) * (a - m);
        return std::sqrt(s / (v.size() - 1));
    };

    // Angle: wrap deviations relative to the first sample so ±180 doesn't blow it up
    for (double& a : angs) {
        double d = a - angs[0];
        while (d >  180) d -= 360;
        while (d < -180) d += 360;
        a = angs[0] + d;
    }

    return { stddev(xs), stddev(zs), stddev(angs) };
}

