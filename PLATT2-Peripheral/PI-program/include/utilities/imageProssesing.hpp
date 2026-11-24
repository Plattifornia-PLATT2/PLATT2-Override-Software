#ifndef IMAGEPROSSESING_HPP
#define IMAGEPROSSESING_HPP

#include <iostream>
#include <apriltag/apriltag.h>
#include <apriltag/tagCircle21h7.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag_pose.h>
#include <string>
#include "utilities/json.hpp"
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include "utilities/sharedData.hpp"
#include <random>
#include <opencv2/calib3d.hpp>

class Camera {
public:

    struct tagStd {

        double x;          
        double y;          
        double theta;  
        
    };


    struct tagInfo {
        Pose pos;
        double z = 0;
        int    tag_id = -1;
        double reproj_error = std::numeric_limits<double>::max();
        tagStd stdErr;
        std::chrono::high_resolution_clock::time_point timeStamp;
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

        std::ifstream f("config.json");
        json root = json::parse(f);

        for (auto& cam : root.at("cameras")) {
            if (cam.at("name") == camName) {
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
        
        //cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
        //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        tagFamily   = tagCircle21h7_create();
        tagDetector = apriltag_detector_create();
        apriltag_detector_add_family(tagDetector, tagFamily);

        tagDetector->quad_decimate = 4.0f;
        tagDetector->quad_sigma    = 0.0f;
        tagDetector->nthreads      = 4;
        tagDetector->debug         = 0;
        tagDetector->refine_edges  = 1;

        waitForExposureSettled();

        info.tagsize =  0.6875; 
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

    Camera::tagInfo getTagPos();
    void capImg();
    double getHorizontalAngle(matd_t* R);


private:
    cv::VideoCapture          cap;
    cv::Mat                   frame;
    apriltag_family_t*        tagFamily   = nullptr;
    apriltag_detector_t*      tagDetector = nullptr;
    apriltag_detection_info_t info;
   
    tagStd estimateTagStd(const apriltag_detection_t* det, double sigmaPx = 0.5, int N = 30);
    bool waitForExposureSettled(int maxFrames = 60, double clippedFraction = 0.02, int stableFramesRequired = 3);
};

#endif // IMAGEPROSSESING_HPP