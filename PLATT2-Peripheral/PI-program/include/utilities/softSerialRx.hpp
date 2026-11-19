// SoftSerialRx.hpp
//
// Receive-only "software serial" for the same binary frame UartPort uses
// (SYNC0 SYNC1 | type | len | payload | crc16, 8N1, LSB first), for a Linux
// board with a GPIO character device (Raspberry Pi 3 / 5).
//
// The pin is not sampled in a timed loop. The kernel timestamps every edge on
// the line, a burst of edges is collected, and the bytes are decoded afterwards
// from the timestamps, so thread scheduling jitter does not corrupt bits. It
// also copes with a line that idles LOW (the first start bit has no falling
// edge): candidate frame starts around the first edges are tried and the one
// where SYNC0/SYNC1 and the CRC check out is kept.
//
// Usage:
//   SoftSerialRx rx("/dev/gpiochip0", 17, 38400);
//   if (!rx.open()) { /* handle error */ }
//   rx.clearBuffer();
//   if (rx.receiveLine(packet, 50)) { /* use packet */ }

#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iterator>
#include <optional>
#include <poll.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <linux/gpio.h>
#include <sys/ioctl.h>

#include "utilities/sharedData.hpp"

class SoftSerialRx {
public:
    // device = GPIO chip, e.g. "/dev/gpiochip0" (see `gpiodetect`)
    // pin    = GPIO number on that chip that RO is wired to
    explicit SoftSerialRx(std::string device, unsigned pin, unsigned baud = 38400);
    ~SoftSerialRx();

    SoftSerialRx(const SoftSerialRx&) = delete;
    SoftSerialRx& operator=(const SoftSerialRx&) = delete;
    SoftSerialRx(SoftSerialRx&& other) noexcept;
    SoftSerialRx& operator=(SoftSerialRx&& other) noexcept;

    bool open();
    void close();
    bool isOpen() const;
    const std::string& lastError() const;

    bool receiveLine(sendPacket&, std::uint32_t timeoutMs = 100);
    void clearBuffer();

    // Optional: SCHED_FIFO for the calling thread (needs root / CAP_SYS_NICE).
    bool setRealtimePriority(int priority = 80);

    void dumpBytes();

    struct Stats {
        uint32_t bursts   = 0;   // edge bursts collected
        uint32_t frames   = 0;   // bursts that decoded to a valid frame
        uint32_t rejected = 0;   // bursts that did not (noise, bad CRC, ...)
    };
    const Stats& stats() const;



private:
    struct Edge {
        int64_t t;       // kernel timestamp, ns
        bool    level;   // line level AFTER this edge (true = high)
    };

    bool collectBurst(std::chrono::steady_clock::time_point deadline);
    size_t readEvents();
    bool waitReadable(int64_t timeoutNs) const;
    int64_t nsUntil(std::chrono::steady_clock::time_point deadline);

    bool decodeBurst(sendPacket& d);
    void decodeBytes(int64_t start);
    bool levelAt(int64_t t) const;
    std::optional<int64_t> nextFallingAfter(int64_t t) const;

    int fd_     = -1;          // GPIO line request (edge events)
    int chipFd_ = -1;          // GPIO chip

    // Frame layout: must match UartPort / the other device.
    static constexpr size_t SENSOR_PAYLOAD =
        sizeof(float) +   // x
        sizeof(float) +   // y
        sizeof(short);    // theta

    static constexpr size_t kSyncLen   = 2;
    static constexpr size_t kHeaderLen = kSyncLen + 1 + 2;   // sync + type + length
    static constexpr size_t kCrcLen    = 2;
    static constexpr size_t kFrameLen  = kHeaderLen + SENSOR_PAYLOAD + kCrcLen;

    static constexpr uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;
    enum class Type : uint8_t { Sensor = 1 };

    static constexpr int kQuietBits = 25;    // quiet time (bit-times) that ends a burst

    uint16_t crc16(const uint8_t* p, size_t n);

    bool tryParse(sendPacket& d);
    std::array<uint8_t, 64> rx_;
    size_t                  rxLen_ = 0;

    std::vector<Edge> edges_;
    Stats             stats_;

    std::string device_;
    unsigned    pin_;
    unsigned    baud_;
    int64_t     bitNs_;
    std::string lastError_;
};