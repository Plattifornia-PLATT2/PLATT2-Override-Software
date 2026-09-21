#pragma once

#include "piLink.hpp"
#include "pros/serial.hpp"

#include <cstdint>
#include <string.h>

class serialPort {
public:
    // port: smart port number (1-21). baudrate: e.g. 115200.
    explicit serialPort(std::uint8_t port, std::int32_t baudrate = 115200);
    

    bool sendLine(const piLink::sendPacket&);


    bool receiveLine(piLink::sendPacket&, std::uint32_t timeoutMs = 100);
 
    
    const std::string& lastError() const;

private:

    static constexpr size_t SENSOR_PAYLOAD =
        sizeof(float) +   // id
        sizeof(float)    +   // temperature
        sizeof(float);   // x     // ok

    static constexpr size_t kSyncLen   = 2;
    static constexpr size_t kHeaderLen = kSyncLen + 1 + 2;   // sync + type + length
    static constexpr size_t kCrcLen    = 2;
    static constexpr size_t kFrameLen  = kHeaderLen + SENSOR_PAYLOAD + kCrcLen;
    
    static constexpr uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;
    enum class Type : uint8_t { Sensor = 1 };
    
    uint16_t crc16(const uint8_t* p, size_t n);

    size_t pack(const piLink::sendPacket&, std::span<uint8_t>);
    
    bool tryParse(piLink::sendPacket& d);
    std::array<uint8_t, 64> rx_;
    size_t                  rxLen_ = 0;

    static std::uint32_t msUntil(std::uint32_t deadline);

    pros::Serial serial_;
    std::string rxBuffer_;
    std::string lastError_;
};