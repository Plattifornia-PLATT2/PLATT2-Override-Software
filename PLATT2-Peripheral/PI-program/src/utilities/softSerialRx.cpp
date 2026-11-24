#include <algorithm>
#include <iterator>
#include <thread>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstdint>

#include "utilities/sharedData.hpp"
#include "utilities/softSerialRx.hpp"
#include <iostream>
#include <iomanip>



SoftSerialRx::SoftSerialRx(std::string device, unsigned pin, unsigned baud)
    : device_(std::move(device)), pin_(pin), baud_(baud),
      bitNs_(1'000'000'000LL / static_cast<int64_t>(baud)) {
    edges_.reserve(512);
}

SoftSerialRx::~SoftSerialRx() { close(); }

SoftSerialRx::SoftSerialRx(SoftSerialRx&& other) noexcept {
    *this = std::move(other);
}

SoftSerialRx& SoftSerialRx::operator=(SoftSerialRx&& other) noexcept {
    if (this != &other) {
        close();
        fd_ = other.fd_;
        chipFd_ = other.chipFd_;
        device_ = std::move(other.device_);
        pin_ = other.pin_;
        baud_ = other.baud_;
        bitNs_ = other.bitNs_;
        edges_ = std::move(other.edges_);
        stats_ = other.stats_;
        other.fd_ = -1;
        other.chipFd_ = -1;
    }
    return *this;
}

bool SoftSerialRx::open() {
    close();

    chipFd_ = ::open(device_.c_str(), O_RDONLY | O_CLOEXEC);
    if (chipFd_ < 0) {
        lastError_ = "open() failed: " + std::string(std::strerror(errno));
        return false;
    }

    gpio_v2_line_request req{};
    req.offsets[0] = pin_;
    req.num_lines  = 1;
    std::strncpy(req.consumer, "softserial-rx", sizeof(req.consumer) - 1);
    req.config.flags = GPIO_V2_LINE_FLAG_INPUT |
                       GPIO_V2_LINE_FLAG_EDGE_RISING |
                       GPIO_V2_LINE_FLAG_EDGE_FALLING;
    req.event_buffer_size = 1024;   // a frame is ~100 edges

    if (::ioctl(chipFd_, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        lastError_ = "GPIO_V2_GET_LINE_IOCTL failed: " + std::string(std::strerror(errno));
        if (errno == EBUSY)
            lastError_ += " (pin is owned by another driver, e.g. the UART; free it or use a spare GPIO)";
        close();
        return false;
    }
    fd_ = req.fd;

    const int fl = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);
    return true;
}

void SoftSerialRx::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (chipFd_ >= 0) {
        ::close(chipFd_);
        chipFd_ = -1;
    }
    edges_.clear();
    rxLen_ = 0;
}

bool SoftSerialRx::isOpen() const { return fd_ >= 0; }

const std::string& SoftSerialRx::lastError() const { return lastError_; }

const SoftSerialRx::Stats& SoftSerialRx::stats() const { return stats_; }

void SoftSerialRx::clearBuffer(){

    if (fd_ < 0) return;
    gpio_v2_line_event ev[64];
    while (::read(fd_, ev, sizeof ev) > 0) {}
    edges_.clear();
    rxLen_ = 0;

}

bool SoftSerialRx::setRealtimePriority(int priority) {
    sched_param sp{};
    sp.sched_priority = priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
        lastError_ = "setRealtimePriority failed: " + std::string(std::strerror(rc));
        return false;
    }
    return true;
}

bool SoftSerialRx::receiveLine(sendPacket& d, std::uint32_t timeoutMs) {
    if (fd_ < 0) { lastError_ = "receiveLine: port not open"; return false; }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (true) {
        if (!collectBurst(deadline)) return false;   // timed out, no edges at all
        ++stats_.bursts;

        if (decodeBurst(d)) { ++stats_.frames; return true; }
        ++stats_.rejected;
        //dumpBytes();                           // noise / bad frame, keep waiting
    }
}

// Block for the first edge, then keep collecting until the line has been quiet
// for kQuietBits bit-times (or the burst is longer than any valid frame).
bool SoftSerialRx::collectBurst(std::chrono::steady_clock::time_point deadline) {
    edges_.clear();

    while (true) {
        const int64_t remain = nsUntil(deadline);
        if (remain == 0) return false;
        if (waitReadable(remain) && readEvents() > 0) break;
    }

    const int64_t gapNs   = bitNs_ * kQuietBits;
    const int64_t maxSpan = bitNs_ * 10 * static_cast<int64_t>(rx_.size() + 2);

    while (edges_.back().t - edges_.front().t < maxSpan) {
        if (waitReadable(gapNs)) {
            readEvents();
        } else if (readEvents() == 0) {
            break;                                   // genuinely quiet: burst is over
        }
    }
    return true;
}

size_t SoftSerialRx::readEvents() {
    gpio_v2_line_event ev[64];
    size_t added = 0;

    while (true) {
        const ssize_t n = ::read(fd_, ev, sizeof ev);
        if (n <= 0) break;                           // EAGAIN: nothing more pending
        const size_t count = static_cast<size_t>(n) / sizeof(gpio_v2_line_event);
        for (size_t i = 0; i < count; ++i) {
            edges_.push_back({static_cast<int64_t>(ev[i].timestamp_ns),
                              ev[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE});
        }
        added += count;
    }
    return added;
}

bool SoftSerialRx::waitReadable(int64_t timeoutNs) const {
    pollfd pfd{fd_, POLLIN, 0};
    timespec ts{};
    ts.tv_sec  = static_cast<time_t>(timeoutNs / 1'000'000'000LL);
    ts.tv_nsec = static_cast<long>(timeoutNs % 1'000'000'000LL);

    int r;
    do { r = ::ppoll(&pfd, 1, &ts, nullptr); } while (r < 0 && errno == EINTR);
    return r > 0 && (pfd.revents & POLLIN);
}

int64_t SoftSerialRx::nsUntil(std::chrono::steady_clock::time_point deadline) {
    auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    return remaining > 0 ? static_cast<int64_t>(remaining) : 0;
}

bool SoftSerialRx::levelAt(int64_t t) const {
    const auto it = std::upper_bound(
        edges_.begin(), edges_.end(), t,
        [](int64_t v, const Edge& e) { return v < e.t; });
    if (it == edges_.begin()) return !edges_.front().level;   // level before the first edge
    return std::prev(it)->level;
}

std::optional<int64_t> SoftSerialRx::nextFallingAfter(int64_t t) const {
    auto it = std::upper_bound(
        edges_.begin(), edges_.end(), t,
        [](int64_t v, const Edge& e) { return v < e.t; });
    for (; it != edges_.end(); ++it)
        if (!it->level) return it->t;
    return std::nullopt;
}

// Decode bytes into rx_ assuming a start bit begins at `start`. After each
// byte the decoder re-syncs on the next falling edge, so a few percent of baud
// error between the two ends does not accumulate. rxLen_ is 0 if the first byte
// has bad framing (a wrong candidate start).
void SoftSerialRx::decodeBytes(int64_t start) {
    const int64_t T = bitNs_;
    int64_t s = start;
    rxLen_ = 0;

    while (rxLen_ < rx_.size()) {
        if (levelAt(s + T / 2)) break;               // start bit must be low

        uint8_t v = 0;
        for (int i = 0; i < 8; ++i)                  // centre of data bit i = s + (1.5 + i) T
            if (levelAt(s + (T * (2 * i + 3)) / 2)) v |= static_cast<uint8_t>(1u << i);

        if (!levelAt(s + (T * 19) / 2)) break;       // stop bit (centre at 9.5 T) must be high

        rx_[rxLen_++] = v;

        const auto next = nextFallingAfter(s + (T * 19) / 2);
        if (!next) break;                            // no further start bit
        s = *next;
    }
}

bool SoftSerialRx::decodeBurst(sendPacket& d) {
    if (edges_.size() < 4) return false;

    const int64_t gapNs = bitNs_ * kQuietBits;

    for (size_t s = 0; s < edges_.size(); ++s) {
        // a frame can only start after a quiet gap (or at the very first edge)
        if (s > 0 && edges_[s].t - edges_[s - 1].t <= gapNs) continue;

        const size_t last = std::min(s + 12, edges_.size());
        for (size_t i = s; i < last; ++i) {
            for (int k = 0; k < 10; ++k) {
                decodeBytes(edges_[i].t - static_cast<int64_t>(k) * bitNs_);
                if (rxLen_ < kFrameLen) continue;
                if (rx_[0] != SYNC0 || rx_[1] != SYNC1) continue;
                if (tryParse(d)) return true;
            }
        }
    }
    rxLen_ = 0;
    return false;
}

uint16_t SoftSerialRx::crc16(const uint8_t* p, size_t n) {   // CRC-16/CCITT-FALSE
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= static_cast<uint16_t>(*p++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

bool SoftSerialRx::tryParse(sendPacket& d) {

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

        if (rxLen_ < kFrameLen) return false;     // frame is shorter than expected

        const uint16_t calc = crc16(rx_.data() + kSyncLen, 1 + 2 + len);
        const uint16_t recv = static_cast<uint16_t>(
            rx_[kFrameLen - 2] | (rx_[kFrameLen - 1] << 8));

        if (calc != recv) { drop(1); continue; }

        const uint8_t* p = rx_.data() + kHeaderLen;
        auto get = [&p](auto& v) {
            memcpy(&v, p, sizeof v);
            p += sizeof v;
        };

        float posX;
        float posY;
        short resetCount;

        get(posX);
        get(posY);
        get(resetCount);

        d.pos.x = posX;
        d.pos.y = posY;
        d.resetCount = resetCount;

        drop(kFrameLen);
        return true;
    }
    return false;
}

void SoftSerialRx::dumpBytes() {
    static int shown = 0;
    if (shown >= 8 || edges_.size() < 4) return;

    size_t best = 0;
    std::array<uint8_t, 64> bestBuf{};
    for (size_t i = 0; i < std::min<size_t>(edges_.size(), 12); ++i)
        for (int k = 0; k < 10; ++k) {
            decodeBytes(edges_[i].t - static_cast<int64_t>(k) * bitNs_);
            if (rxLen_ > best) { best = rxLen_; bestBuf = rx_; }
        }

    ++shown;
    std::cout << "burst edges=" << edges_.size() << " bytes=" << best << ":";
    for (size_t i = 0; i < best; ++i)
        std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0') << int(bestBuf[i]);
    std::cout << std::dec << std::endl;
}