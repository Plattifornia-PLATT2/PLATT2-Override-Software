#include "subsystems/otos.hpp"
#include <pwd.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <string>

void OTOS::calibrate(){
    
    sfTkError_t err = otos.calibrateImu(255, true);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS calibrateImu() failed, error code %d\n", static_cast<int>(err));
    }
    std::printf("OTOS IMU calibration complete.\n");

    sfe_otos_signal_process_config_t config;
    config.enAcc = false;
    config.enRot = true;
    config.enLut = true;
    otos.setSignalProcessConfig(config);
    otos.resetTracking();
}
std::string getHostName() {
    char buf[HOST_NAME_MAX + 1] = {};
    if (gethostname(buf, sizeof(buf)) != 0) {
        return "";
    }
    return std::string(buf);
}
void OTOS::restart(){
    std::string name = getHostName();

    if (name == "purple") {
        otos.setPosition(purpleStartPos);
    } else if (name == "pink") {
        otos.setPosition(pinkStartPos);
    } else {
        std::cout << "Unknown host: '" << name << "'" << std::endl;
        sfe_otos_pose2d_t zero = {0, 0, 0};
        otos.setPosition(zero);
    }

    this->calibrate();
}

void OTOS::getPos(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPosition(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPosition() failed, error code %d\n", static_cast<int>(err));
    }
    //std::printf("OTOS Position: X: %.2f, Y: %.2f, H: %.2f\n", pos.x, pos.y, pos.h);
}

double OTOS::getX(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPosition(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPosition() failed, error code %d\n", static_cast<int>(err));
    }

    return pos.x;
}

double OTOS::getY(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPosition(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPosition() failed, error code %d\n", static_cast<int>(err));
    }
    return pos.y;
}

double OTOS::getH(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPosition(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPosition() failed, error code %d\n", static_cast<int>(err));
    }
    return pos.h;
}



double OTOS::getXstdDev(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPositionStdDev(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPositionStdDev() failed, error code %d\n", static_cast<int>(err));
    }
    return pos.x;
}

double OTOS::getYstdDev(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPositionStdDev(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPositionStdDev() failed, error code %d\n", static_cast<int>(err));
    }
    return pos.y;
}

double OTOS::getHstdDev(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPositionStdDev(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPositionStdDev() failed, error code %d\n", static_cast<int>(err));
    }
 
   return pos.h;
}

int OTOS::selfTest(){
    return otos.selfTest();
}

double OTOS::getAcceleration(){
    return 0;
}

void OTOS::sensorLoop(std::stop_token stopToken, sharedData& shared){
    
    while (!stopToken.stop_requested()) {
        
        PoseReading buffer;
        auto t0 = std::chrono::steady_clock::now();
        buffer.pose.x = getX();
        buffer.pose.y = getY();
        buffer.pose.theta = getH();

        buffer.std_x = getXstdDev();
        buffer.std_y = getYstdDev();
        buffer.std_theta = getHstdDev();
        auto t1 = std::chrono::steady_clock::now();
        buffer.latency = std::chrono::duration<double, std::milli>(t1 - t0).count();

        buffer.newData = true;

        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            shared.OTOSPos = buffer;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }    
}



OTOS::OTOS(){

    if (!i2c.openBus("/dev/i2c-1"))
    {
        std::fprintf(stderr, "Could not open I2C bus. Is I2C enabled (raspi-config)?\n");
    }

    sfTkError_t err = otos.begin(&i2c);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS begin() failed, error code %d\n", static_cast<int>(err));
    }
    std::printf("OTOS connected.\n");

    this->restart();
}