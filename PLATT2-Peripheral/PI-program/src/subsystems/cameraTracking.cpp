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

Pos cameraTracking::getGlobalPos(std::vector<Camera::tagInfo>){







}

Pos cameraTracking::getGoalPos(int goal){

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

    return goalPos;

}




}


