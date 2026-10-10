#include "utilities/imageProssesing.hpp"
#include "utilities/uartPort.hpp"
#include "subsystems/piLink.hpp"
#include "utilities/sharedData.hpp"
#include "subsystems/otos.hpp"
#include "subsystems/cameraTracking.hpp"
#include "subsystems/PoseEstimator.hpp"
#include <thread>
#include <vector>
#include <stop_token>
#include <csignal>


std::jthread sigThread(sharedData &shared);

int main() {

    std::vector<std::jthread> comTask;
    std::vector<std::jthread> dependentTasks;
    
    sharedData shared;
    OTOS otos;
    
    piLink link;
    cameraTracking cam;
    PoseEstimator kalman;


    otos.calibrate();
    
    std::jthread signalThread = sigThread(shared);
    comTask.emplace_back([&link, &shared](std::stop_token st) {link.linkLoop(st, shared);});

    //comTask[0].request_stop(); 

    while (true){
        
        // add dependent tasks to the vector below following the format
        //dependentTasks.emplace_back([&link, &shared](std::stop_token st) {link.linkLoop(st, shared);});
        
        //dependentTasks.emplace_back([&otos, &shared](std::stop_token st) {otos.sensorLoop(st, shared);});
        //dependentTasks.emplace_back([&cam, &shared](std::stop_token st) {cam.camTrackLoop(st, shared);});
        dependentTasks.emplace_back([&kalman, &shared](std::stop_token st) {kalman.kalmanLoop(st, shared);});
        
        std::unique_lock<std::mutex> lock(shared.mtx);
        shared.cv.wait(lock, [&shared] { return shared.restart || shared.shutdown; });

        std::cout << "fuck" << std::endl;

        for (auto& t : dependentTasks) {
            t.request_stop();
        }

        if (shared.shutdown){
            break;
        }

        shared.restart = false;
        lock.unlock();

    }
}


std::jthread sigThread(sharedData &shared){

    sigset_t set;
    
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, nullptr);

    std::jthread signalThread([&set, &shared] {
        int sig;
        sigwait(&set, &sig); 
        {
            std::lock_guard lock(shared.mtx);
            shared.shutdown = true;
            shared.cv.notify_all();
        }
        
    });

    return signalThread;
}

    