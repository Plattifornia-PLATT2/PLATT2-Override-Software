#include "utilities/imageProssesing.hpp"
#include "utilities/uartPort.hpp"
#include "subsystems/piLink.hpp"
#include "utilities/sharedData.hpp"
#include "subsystems/cameraTracking.hpp"
#include <thread>
#include <vector>
#include <stop_token>
#include <csignal>


std::jthread sigThread(sharedData &shared);

int main() {

    std::vector<std::jthread> comTask;
    std::vector<std::jthread> dependentTasks;
    
    sharedData shared;
    
    cameraTracking camLoop;
    piLink link;

    
    //std::jthread signalThread = sigThread(shared);
    //comTask.emplace_back([&link, &shared](std::stop_token st) {link.linkLoop(st, shared);});
    //comTask[0].request_stop(); 

    //while (true){
        
        // add dependent tasks to the vector below following the format
        //dependentTasks.emplace_back([&link, &shared](std::stop_token st) {link.linkLoop(st, shared);});
        dependentTasks.emplace_back([&camLoop, &shared](std::stop_token st) {camLoop.camTrackLoop(st, shared);});
        //
        
        //std::unique_lock<std::mutex> lock(shared.mtx);
        //shared.cv.wait(lock, [&shared] { return shared.restart || shared.shutdown; });
//
        //std::cout << "fuck" << std::endl;
//
        //for (auto& t : dependentTasks) {
        //    t.request_stop();
        //}
//
        //if (shared.shutdown){
        //    break;
        //}
//
        //shared.restart = false;
        //lock.unlock();
//
    //}
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

    