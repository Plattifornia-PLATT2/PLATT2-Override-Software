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

class UartPort {
public:
    explicit UartPort(std::string device, speed_t baud = B115200);
    ~UartPort();

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;
    UartPort(UartPort&& other) noexcept;
    UartPort& operator=(UartPort&& other) noexcept;

    UartPort(){if (!this->open()) {std::cerr << "open failed: " << this->lastError() << "\n";}}

    // Opens and configures the serial port. Returns false on failure
    // (see lastError() for details).
    bool open();
    void close();
    bool isOpen() const;
    const std::string& lastError() const;

    // Sends a text packet, appending a '\n' delimiter. Returns false if
    // not open or the write fails.
    bool sendLine(const std::string& text);

    // Waits up to timeoutMs for one complete '\n'-terminated line.
    // Returns std::nullopt on timeout or link error. Bytes read past the
    // delimiter are buffered internally for the next call.
    std::optional<std::string> receiveLine(int timeoutMs = 100);

private:
    bool configurePort();

    int fd_ = -1;
    std::string device_;
    speed_t baud_;
    std::string rxBuffer_;
    std::string lastError_;
};