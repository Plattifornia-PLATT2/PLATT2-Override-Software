#include <iostream>
#include <apriltag/apriltag.h>
#include <apriltag/tagCircle21h7.h>
#include <opencv2/opencv.hpp>



int main() {
    // Create tag family
    apriltag_family_t *tf = tagCircle21h7_create();

    // Create detector
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[AprilTag] Failed to open camera " << 0<< "\n";
        return false;
    }
        // --- Set format ---
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // --- Discard first few frames (camera needs to warm up) ---
    cv::Mat frame;
    for (int i = 0; i < 5; i++)
        cap.grab();

    // --- Capture frame ---
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "Failed to capture frame\n";
        return false;
    }

    // --- Save to disk ---
    if (!cv::imwrite("capture.jpg", frame)) {
        std::cerr << "Failed to save image to: " << "capture.jpg" << "\n";
        return false;
    }

    std::cout << "Image saved to: " << "capture.jpg" << "\n";
    return true;

    
    // Cleanup
    apriltag_detector_destroy(td);
    tagCircle21h7_destroy(tf);


    return 0;
}