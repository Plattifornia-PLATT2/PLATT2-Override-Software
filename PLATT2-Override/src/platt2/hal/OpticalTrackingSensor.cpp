#include "PLATT2/hal/OpticalTrackingSensor.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
//#include <cstdint>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <math.h>
#include <memory>
#include <string>
#include <sys/types.h>
#include <vector>

namespace platt2{

namespace hal{

static void taskThunk(void *p) {
    //reinterpret_cast<OpticalTrackingSensor*>(p)->readData();
}

OpticalTrackingSensor::OpticalTrackingSensor(double xOffset, double yOffset, double hOffset, std::unique_ptr<pros::IMU> vex_imu):
 m_otosTask(taskThunk, this),
 vex_imu(std::move(vex_imu))
{
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    this->hOffset = hOffset;
    
    xPos = 0;
    yPos = 0;
    heading = 0;
}

double OpticalTrackingSensor::getXPosition(){
    return xPos;
}

double OpticalTrackingSensor::getYPosition(){
    return yPos;
}

double OpticalTrackingSensor::getHeading(){
    return heading;
}

void OpticalTrackingSensor::resetHeading(){
    if(vex_imu){
        vex_imu->set_heading(90);
        //localHeading = 90;
    }
}

void OpticalTrackingSensor::initVexImu(){
     if(vex_imu){
        this->vex_imu->reset();
        while(vex_imu->is_calibrating()){
            pros::delay(10);
        }
        vex_imu->set_heading(90);
    }
}

double OpticalTrackingSensor::getBoundedHeading(){
// return otos.getHeading();
    double heading = vex_imu->get_heading();
    /*// Normalize in case heading is outside 0–360
    while (heading < 0)   heading += 360;
    while (heading >= 360) heading -= 360;

    // Convert to -180 to 180
    if (heading > 180)
        heading -= 360;*/
    heading = heading * M_PI / 180.0;        // convert to radians

    // wrap to [-pi, pi]
    heading = fmod(heading + M_PI, 2.0*M_PI);
    if (heading < 0) heading += 2.0*M_PI;
    heading -= M_PI;

    return -heading; // radians in [-pi, pi]
}

double OpticalTrackingSensor::getVexHeading(){
    return getBoundedHeading();
}

void OpticalTrackingSensor::readData() {

    double tempHeading;
    std::string buffer;
    std::string writeString = "";

    while (true) {
    // --- WRITE TO SERIAL ---
    writeString = "/H:000;";
    std::vector<uint8_t> writeData(writeString.begin(), writeString.end());
    if (m_serialInterface.get_write_free() > 0) {
        m_serialInterface.write(reinterpret_cast<uint8_t*>(writeData.data()), writeData.size());
        pros::delay(5);
    }

    // --- READ SERIAL INTO BUFFER ---
    while (!m_serialInterface.get_read_avail()){ 
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "VEX Heading: %f", vex_imu->get_heading());
        pros::delay(10);
    }
    while (m_serialInterface.get_read_avail()) {
        char byteRead = static_cast<char>(m_serialInterface.read_byte());
        buffer.push_back(byteRead);
    }

    // --- EXTRACT FULL PACKET ---
    size_t start = buffer.find('/');
    size_t end   = buffer.find_last_of(';');

    if (start != std::string::npos && end != std::string::npos && end > start) {
        std::string packet = buffer.substr(start, end - start + 1);
        buffer.erase(0, end + 1);  // remove the packet from buffer

        //std::cout << "Full Read: " << packet << std::endl;

        // --- PARSE VALUES ---
        auto getValue = [&](const std::string &label) -> std::string {
            size_t i = packet.find(label);
            if (i == std::string::npos) return "";
            size_t j = packet.find(";", i);
            if (j == std::string::npos) return "";
            return packet.substr(i + 2, j - (i + 2));
        };

        std::string xPosStr = getValue("X:");
        std::string yPosStr = getValue("Y:");
        std::string hPosStr = getValue("H:");

        try { xPos = std::stod(xPosStr) + xOffset; } catch (...) {}
        try { yPos = std::stod(yPosStr) + yOffset; } catch (...) {}
        try {
            tempHeading = (std::stod(hPosStr) * M_PI / 180) + ((hOffset * M_PI) / 180);
            if (tempHeading < 0) tempHeading += (2 * M_PI);
            heading = tempHeading;
        } catch (...) {}
    }

    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "X Pos: %f", xPos);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Y Pos: %f", yPos);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Heading: %f", (heading * 180) / M_PI);
    pros::delay(10);
}

}
}
} // namespace hal
 // namespace platt2