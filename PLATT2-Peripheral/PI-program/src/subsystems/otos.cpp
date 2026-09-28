#include "subsystems/otos.hpp"


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

    sfe_otos_pose2d_t pos;
    pos = {0.0f, 0.0f, 0.0f};

    otos.setPosition(pos);

}

void OTOS::calibrate(){
    
    sfTkError_t err = otos.calibrateImu(255, true);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS calibrateImu() failed, error code %d\n", static_cast<int>(err));
    }
    std::printf("OTOS IMU calibration complete.\n");
}

void OTOS::getPos(){
    sfe_otos_pose2d_t pos;
    sfTkError_t err = otos.getPosition(pos);
    if (err != ksfTkErrOk)
    {
        std::fprintf(stderr, "OTOS getPosition() failed, error code %d\n", static_cast<int>(err));
    }
    std::printf("OTOS Position: X: %.2f, Y: %.2f, H: %.2f\n", pos.x, pos.y, pos.h);
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

void OTOS::sensorLoop(std::stop_token stopToken, sharedData& shared){
    while (!stopToken.stop_requested()) {
        
        double x = getX();
        double y = getY();

        {
        std::lock_guard<std::mutex> lock(shared.mtx);
            shared.linkData.posX = x;
            shared.linkData.posY = y; 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }    
}