#include "platt2/robot/subsystems/externalLink/serialPort.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>

serialPort::serialPort(std::uint8_t port, std::int32_t baudrate)
    : serial_(port, baudrate) {}

std::uint32_t serialPort::msUntil(std::uint32_t deadline) {
    std::uint32_t now = pros::millis();
    return (deadline > now) ? (deadline - now) : 0;
}

bool serialPort::sendLine(const data& d) {
    constexpr uint32_t kSendTimeoutMs = 20;

    std::array<uint8_t, 64> buf;              // must be >= frame size
    const size_t n = pack(d, buf);
    if (n == 0) return false;                 // buffer too small

    uint8_t* p = buf.data();
    size_t left = n;
    const uint32_t start = pros::millis();

    while (left > 0) {
        const int32_t w = serial_.write(p, static_cast<int32_t>(left));
        if (w == PROS_ERR) return false;      // port error

        if (w == 0) {                         // TX buffer full, wait and retry
            if (pros::millis() - start > kSendTimeoutMs) return false;
            pros::delay(1);
            continue;
        }
        p    += w;
        left -= static_cast<size_t>(w);
    }
    return true;
}

bool serialPort::receiveLine(data& d, std::uint32_t timeoutMs) {
    const std::uint32_t deadline = pros::millis() + timeoutMs;

    while(true) {
        if (tryParse(d)) return true;             // frame already buffered or just completed
        if (msUntil(deadline) == 0) return false; // timed out

        // Pull whatever the port has
        const int32_t avail = serial_.get_read_avail();
        if (avail > 0) {
            const size_t room = rx_.size() - rxLen_;
            const size_t want = std::min(static_cast<size_t>(avail), room);
            const int32_t got = serial_.read(rx_.data() + rxLen_, static_cast<int32_t>(want));
            if (got > 0) { rxLen_ += static_cast<size_t>(got); continue; }
        }
        pros::delay(1);                           // nothing arrived, yield to other tasks
    }
}

const std::string& serialPort::lastError() const { return lastError_; }

uint16_t serialPort::crc16(const uint8_t* p, size_t n) {   // CRC-16/CCITT-FALSE
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= static_cast<uint16_t>(*p++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

size_t serialPort::pack(const data& d, std::span<uint8_t> out) {
    static_assert(std::endian::native == std::endian::little,
                  "wire format is little-endian");

    constexpr size_t kSyncLen   = 2;
    constexpr size_t kHeaderLen = kSyncLen + 1 + 2;   // sync + type + length
    constexpr size_t kCrcLen    = 2;
    constexpr size_t kFrameLen  = kHeaderLen + SENSOR_PAYLOAD + kCrcLen;

    if (out.size() < kFrameLen) return 0;

    uint8_t* p = out.data();
    auto put = [&p](const auto& v) {          // append raw bytes, advance cursor
        memcpy(p, &v, sizeof v);
        p += sizeof v;
    };

    // Header
    put(uint8_t{SYNC0});
    put(uint8_t{SYNC1});
    put(static_cast<uint8_t>(Type::Sensor));
    put(static_cast<uint16_t>(SENSOR_PAYLOAD));

    // Payload
    //put(static_cast<uint32_t>(d.id));
    //put(static_cast<float>(d.temperature));
    //put(static_cast<uint16_t>(d.x));
    //put(static_cast<uint8_t>(d.ok ? 1 : 0));

    // CRC covers type..payload (everything after the sync bytes)
    put(crc16(out.data() + kSyncLen, static_cast<size_t>(p - out.data()) - kSyncLen));

    return static_cast<size_t>(p - out.data());
}

bool serialPort::tryParse(data& d) {
    
    auto drop = [this](size_t n) {
        memmove(rx_.data(), rx_.data() + n, rxLen_ - n);
        rxLen_ -= n;
    };

    while (rxLen_ > 0) {
        if (rx_[0] != SYNC0)                  { drop(1); continue; }
        if (rxLen_ >= 2 && rx_[1] != SYNC1)   { drop(1); continue; }
        if (rxLen_ < kHeaderLen)              return false;

        const uint8_t  type = rx_[2];
        const uint16_t len  = static_cast<uint16_t>(rx_[3] | (rx_[4] << 8));

        if (type != static_cast<uint8_t>(Type::Sensor) || len != SENSOR_PAYLOAD) {
            drop(1);
            continue;
        }

        if (rxLen_ < kFrameLen) return false;     // rest of frame still in flight

        const uint16_t calc = crc16(rx_.data() + kSyncLen, 1 + 2 + len);
        const uint16_t recv = static_cast<uint16_t>(
            rx_[kFrameLen - 2] | (rx_[kFrameLen - 1] << 8));

        if (calc != recv) { drop(1); continue; }

        const uint8_t* p = rx_.data() + kHeaderLen;
        auto get = [&p](auto& v) {
            memcpy(&v, p, sizeof v);
            p += sizeof v;
        };

        uint32_t id;  float temp;  uint16_t x;  uint8_t ok;
        get(id);  get(temp);  get(x);  get(ok);

        //d.id          = id;
        //d.temperature = temp;
        //d.x           = static_cast<decltype(d.x)>(x);
        //d.ok          = (ok != 0);

        drop(kFrameLen);
        return true;
    }
    return false;
}


