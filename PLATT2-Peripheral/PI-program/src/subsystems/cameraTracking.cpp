#include "subsystems/cameraTracking.hpp"
#include "utilities/imageProssesing.hpp"


//struct PoseReading {
//    Pose pose; 
//    double std_x;
//    double std_y;
//    double std_theta; 
//    double latency = 0.0; // in mili seconds
//    bool newData = false;
//};



void cameraTracking::camTrackLoop(std::stop_token stopToken, sharedData& shared){

    Camera cam("rear");

    PoseReading buffer;
    Pose stdErrBuffer;

    while(!stopToken.stop_requested()){

        Camera::tagInfo out = cam.getTagPos();

        if (out.tag_id < 0){
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;

        }

        out.pos = getGlobalPos(out, shared);
        stdErrBuffer = getGlobalStdErr(out, shared);

        buffer.pose = out.pos;

        buffer.std_x = stdErrBuffer.x;
        buffer.std_y = stdErrBuffer.y;
        buffer.std_theta = 1E6; // tell kalman filter to disregard angle from camera
        
        buffer.latency = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - out.timeStamp).count();

        buffer.newData = true;
        
        {
        std::lock_guard<std::mutex> lock(shared.mtx);
        shared.camPos = buffer;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    
    // auto start = std::chrono::high_resolution_clock::now();
    //auto end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


}

Pose cameraTracking::getGlobalPos(Camera::tagInfo tag, sharedData& shared){

    Pose currentPos;
    bool farGoal;

    {
        std::lock_guard<std::mutex> lock(shared.mtx);
        currentPos = shared.OTOSPos.pose;
    }

    Pose camVec = rotateVector(tag.pos, currentPos.theta);
    
    currentPos.y+camVec.y>=72 ? farGoal=false : farGoal=true;
    
    Pose goalPos = getGoalPos(tag.tag_id, farGoal);
    Pose globalPos = {goalPos.x - camVec.x, goalPos.y - camVec.y};

    return globalPos;

}

Pose cameraTracking::getGlobalStdErr(Camera::tagInfo tag, sharedData& shared){

    Pose currentPos;
    bool farGoal;

    {
        std::lock_guard<std::mutex> lock(shared.mtx);
        currentPos = shared.OTOSPos.pose;
    }

    const double c = std::cos(currentPos.theta);
    const double s = std::sin(currentPos.theta);
    
    const double vx = tag.stdErr.x * tag.stdErr.x;
    const double vy = tag.stdErr.y * tag.stdErr.y;

    Pose globalStdErr = {c*c*vx + s*s*vy, s*s*vx + c*c*vy};

    return globalStdErr;

}

Pose cameraTracking::getGoalPos(int goal, bool far){

    Pose goalPos;

    switch (goal) {
        case 0:
            goalPos = {72,72};
            break;
        case 1:
            goalPos = {48,24};
            break;
        case 2:
            goalPos = {96,24};
            break;
        case 3:
            goalPos = {120,48};
            break;
        case 4:
            goalPos = {24,48};
            break;
        
        default:
            break; 

    }
    if (!far){

        goalPos = rotateVector(goalPos, M_PI);
        goalPos = {goalPos.x-144, goalPos.y-144};
    }

    return goalPos;

}


Pose cameraTracking::rotateVector(Pose vec, double theta){

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Pose rotated;
    rotated.x = vec.x * cos_theta + vec.y * sin_theta;
    rotated.y = vec.x * sin_theta - vec.y * cos_theta;

    return rotated;
}


