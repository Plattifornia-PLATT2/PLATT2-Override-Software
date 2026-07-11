#ifndef IMAGEPROSSESING_HPP
#define IMAGEPROSSESING_HPP

#include <iostream>
#include <apriltag/apriltag.h>
#include <apriltag/tagCircle21h7.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag_pose.h>
#include <string>
#include "json.hpp"
#include <fstream>

class Camera {
public:
    struct tagPos {
        double x = 0;
        double y = 0;
        double z = 0;
        int    tag_id = -1;
        double reproj_error = std::numeric_limits<double>::max();
    };

    struct cameraInfo {
        
        std::string name;

        std::string address;
        
        double fx;
        double fy;
        double cx;
        double cy;
    };

    Camera(std::string camName) {

        using json = nlohmann::json;

        cameraInfo camInfo;

        std::ifstream f("cameraConfig.json");
        json root = json::parse(f);

        for (auto& cam : root.at("cameras")) {
            if (cam.at("name") == camName) {
                cameraInfo info;
                camInfo.name    = cam.at("name");
                camInfo.address = cam.at("address");
                camInfo.fx      = cam.at("fx");
                camInfo.fy      = cam.at("fy");
                camInfo.cx      = cam.at("cx");
                camInfo.cy      = cam.at("cy");
                break;
            }
        }
        
        cap.open(camInfo.address, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        for (int i = 0; i < 5; i++)
            cap.grab();

        tagFamily   = tagCircle21h7_create();
        tagDetector = apriltag_detector_create();
        apriltag_detector_add_family(tagDetector, tagFamily);

        tagDetector->quad_decimate = 2.0f;
        tagDetector->quad_sigma    = 0.0f;
        tagDetector->nthreads      = 2;
        tagDetector->debug         = 0;
        tagDetector->refine_edges  = 1;

        cap >> frame;

        info.tagsize = 3; 
        info.fx      = camInfo.fx;
        info.fy      = camInfo.fy;
        info.cx      = camInfo.cx;
        info.cy      = camInfo.cy;
    }

    ~Camera() {
        apriltag_detector_destroy(tagDetector);
        tagCircle21h7_destroy(tagFamily);
        cap.release();
    }

    tagPos getImagePos();


private:
    cv::VideoCapture          cap;
    cv::Mat                   frame;
    apriltag_family_t*        tagFamily   = nullptr;
    apriltag_detector_t*      tagDetector = nullptr;
    apriltag_detection_info_t info;
};

#endif // IMAGEPROSSESING_HPP