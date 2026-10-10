#ifndef CAMERATRACKING_HPP
#define CAMERATRACKING_HPP


#include "utilities/sharedData.hpp"
#include "utilities/imageProssesing.hpp"

#include <string>

class cameraTracking{

    public:

        void camTrackLoop(std::stop_token, sharedData& );
        


    private:
    Pose getGlobalPos(Camera::tagInfo, sharedData&);
    Pose getGoalPos(int, bool);
    Pose rotateVector(Pose , double);
    Pose getGlobalStdErr(Camera::tagInfo, sharedData&);

};



#endif