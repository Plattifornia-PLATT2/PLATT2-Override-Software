// UartPort.hpp
//
// Single-class UART text-packet transport for Raspberry Pi 5 (Linux termios).
// Packets are plain text lines, delimited by '\n' (assumes payloads don't
// contain embedded newlines).
//
// Usage:
//   UartPort link("/dev/ttyAMA0", B115200);
//   if (!link.open()) { /* handle error */ }
//   link.sendLine("HELLO");
//   if (auto line = link.receiveLine(200)) { /* use *line */ }

#pragma once

#include <optional>
#include <string>
#include <termios.h>
#include <iostream>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <span>

class UartPort {
public:
    explicit UartPort(std::string device, speed_t baud = B115200);
    ~UartPort();

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;
    UartPort(UartPort&& other) noexcept;
    UartPort& operator=(UartPort&& other) noexcept;

    UartPort(){if (!this->open()) {std::cerr << "open failed: " << this->lastError() << "\n";}} 
    
    struct data{

        double test = 300;

    };


    bool open();
    void close();
    bool isOpen() const;
    const std::string& lastError() const;


    bool sendLine(const data&);
    bool receiveLine(data&, std::uint32_t timeoutMs = 100);

    std::optional<std::string> receiveLine(int timeoutMs = 100);



private:
    bool configurePort();
    uint64_t msUntil(std::chrono::steady_clock::time_point deadline);

    int fd_ = -1;

    static constexpr size_t SENSOR_PAYLOAD =
        sizeof(uint32_t) +   // id
        sizeof(float)    +   // temperature
        sizeof(int16_t)  +   // x
        sizeof(uint8_t);     // ok

    static constexpr size_t kSyncLen   = 2;
    static constexpr size_t kHeaderLen = kSyncLen + 1 + 2;   // sync + type + length
    static constexpr size_t kCrcLen    = 2;
    static constexpr size_t kFrameLen  = kHeaderLen + SENSOR_PAYLOAD + kCrcLen;
    
    static constexpr uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;
    enum class Type : uint8_t { Sensor = 1 };
    
    uint16_t crc16(const uint8_t* p, size_t n);

    size_t pack(const data&, std::span<uint8_t>);
    
    bool tryParse(data& d);
    std::array<uint8_t, 64> rx_;
    size_t                  rxLen_ = 0;

    uint64_t millis();
    int32_t get_read_avail(int fd);

    std::string device_;
    speed_t baud_;
    std::string rxBuffer_;
    std::string lastError_;
};