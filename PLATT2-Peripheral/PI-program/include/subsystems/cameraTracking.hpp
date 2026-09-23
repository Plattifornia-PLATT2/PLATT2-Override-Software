#ifndef CAMERATRACKING_HPP
#define CAMERATRACKING_HPP


#include "utilities/sharedData.hpp"
#include "utilities/imageProssesing.hpp"

#include <string>

class cameraTracking{

    public:

        void camTrackLoop(std::stop_token, sharedData& );
        


    private:
    Pos getGlobalPos(std::vector<Camera::tagInfo>);
    Pos getGoalPos(int);

};



#endif