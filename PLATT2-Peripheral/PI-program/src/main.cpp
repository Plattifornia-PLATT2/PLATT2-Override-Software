#include "utilities/imageProssesing.hpp"
#include "utilities/uartPort.hpp"
#include "subsystems/piLink.hpp"
#include "utilities/sharedData.hpp"
#include "subsystems/otos.hpp"
#include "subsystems/cameraTracking.hpp"
#include "subsystems/PoseEstimator.hpp"
#include <thread>
#include <vector>
#include <csignal>
#include <cerrno>
#include <pthread.h>
#include <thread>

std::jthread sigThread(sharedData &shared) {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);   // systemd sends this on shutdown/reboot/stop
    sigaddset(&set, SIGHUP);    // session teardown
    pthread_sigmask(SIG_BLOCK, &set, nullptr);

    return std::jthread([set, &shared](std::stop_token st) {   // set by value
        const timespec timeout{0, 200'000'000};                // 200 ms
        while (!st.stop_requested()) {
            int sig = sigtimedwait(&set, nullptr, &timeout);
            if (sig < 0) {
                if (errno == EAGAIN || errno == EINTR) continue;
                break;
            }
            {
                std::lock_guard lock(shared.mtx);
                shared.shutdown = true;
            }
            shared.cv.notify_all();
            return;
        }
    });
}

int main() {
    std::cout << "running" << std::endl;

    sharedData shared;
    std::jthread signalThread = sigThread(shared);   // FIRST, before any other thread exists

    std::vector<std::jthread> comTask;
    std::vector<std::jthread> dependentTasks;

    OTOS otos;
    piLink link;
    cameraTracking cam;
    PoseEstimator kalman;

    comTask.emplace_back([&link, &shared](std::stop_token st) { link.linkLoop(st, shared); });

    while (true) {
        dependentTasks.emplace_back([&otos, &shared](std::stop_token st) { otos.sensorLoop(st, shared); });
        //dependentTasks.emplace_back([&cam, &shared](std::stop_token st) { cam.camTrackLoop(st, shared); });
        //dependentTasks.emplace_back([&kalman, &shared](std::stop_token st) { kalman.kalmanLoop(st, shared); });

        std::unique_lock lock(shared.mtx);
        shared.cv.wait(lock, [&shared] { return shared.restart || shared.shutdown; });
        const bool quit = shared.shutdown;
        lock.unlock();                       // never join while holding the mutex

        for (auto& t : dependentTasks) t.request_stop();
        dependentTasks.clear();              // joins and removes the old threads

        if (quit) break;

        std::cout << "restarting" << std::endl;
        otos.restart();
        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.restartCount++;
            shared.restart = false;
        }
        
    }

    // Shutdown: stop the comms thread(s), then do your hardware cleanup
    for (auto& t : comTask) t.request_stop();
    comTask.clear();
    // e.g. stop motors, close UART/GPIO
}