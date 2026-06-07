#include <iostream>
#include <apriltag/apriltag.h>
#include <apriltag/tagCircle21h7.h>
#include <opencv2/opencv.hpp>


class Camera {

    public:

        struct tagPos{
            int id;
            double x;
            double y;
            double z;
        };

        tagPos getImagePos();

    private:
        
        cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);

        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);


        cv::Mat frame;
        for (int i = 0; i < 5; i++){
            cap.grab();
        };

        apriltag_family_t *tagFamily = tagCircle21h7_create();
        apriltag_detector_t *tagDetector = apriltag_detector_create();
        apriltag_detector_add_family(tagDetector, tagFamily);

        tagDetector->quad_decimate   = 2.0f;  // Decimate image for speed; 1.0 = no decimation
        tagDetector->quad_sigma      = 0.0f;  // Gaussian blur sigma (0 = off)
        tagDetector->nthreads        = 2;     // CPU threads for detection
        tagDetector->debug           = 0;
        tagDetector->refine_edges    = 1;

        apriltag_detection_info_t info;
            info.tagsize = tag_size;
            info.fx      = fx;
            info.fy      = fy;
            info.cx      = cx;
            info.cy      = cy;

};