#include "subsystems/cameraTracking.hpp"
#include "utilities/imageProssesing.hpp"






void cameraTracking::camTrackLoop(std::stop_token stopToken, sharedData& shared){

    Camera cam("rear");

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Camera::tagInfo> out = cam.getTagPos();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "tOTAL time: "<<duration.count()<<std::endl;

    for (const auto& element : out) {
        std::cout << "X: "<<element.pos.x<<" Y: "<<element.pos.y << " angle: "<<element.angle<<" error: "<<element.reproj_error<< std::endl;
    }

    //std::cout <<out[0].x <<", "<< out[0].y<<", " << out[0].z<< ", "<< out[0].Hangle << std::endl;


 


   

    // 4. Calculate the difference (duration)
    // You can change 'microseconds' to milliseconds, nanoseconds, or seconds
    



}

Pos cameraTracking::getGlobalPos(Camera::tagInfo tag, sharedData& shared){

    Pos currentPos;
    bool farGoal;

    {
        std::lock_guard<std::mutex> lock(shared.mtx);
        Pos currentPos = shared.OTOSpos;
    }

    Pos camVec = rotateVector(tag.pos, currentPos.heading);
    
    currentPos.y+camVec.y>=72 ? farGoal=false : farGoal=true;
    
    Pos goalPos = getGoalPos(tag.tag_id, farGoal);
    Pos globalPos = {goalPos.x - camVec.x, goalPos.y - camVec.y};

    return globalPos;

}

Pos cameraTracking::getGoalPos(int goal, bool far){

    Pos goalPos;

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


    if (!far){

        goalPos = rotateVector(goalPos, M_PI);
        goalPos = {goalPos.x-144, goalPos.y-144};
    }

    return goalPos;

}
}

Pos cameraTracking::rotateVector(Pos vec, double theta){

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Pos rotated;
    rotated.x = vec.x * cos_theta + vec.y * sin_theta;
    rotated.y = vec.x * sin_theta - vec.y * cos_theta;

    return rotated;
}


