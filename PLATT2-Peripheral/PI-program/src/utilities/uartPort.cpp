#include <bit>
#include <thread>
#include <poll.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstdint>

#include "utilities/sharedData.hpp"
#include "utilities/uartPort.hpp"


UartPort::UartPort(std::string device, speed_t baud)
    : device_(std::move(device)), baud_(baud) {}

UartPort::~UartPort() { close(); }

UartPort::UartPort(UartPort&& other) noexcept {
    *this = std::move(other);
}

UartPort& UartPort::operator=(UartPort&& other) noexcept {
    if (this != &other) {
        close();
        fd_ = other.fd_;
        device_ = std::move(other.device_);
        baud_ = other.baud_;
        rxBuffer_ = std::move(other.rxBuffer_);
        other.fd_ = -1;
    }
    return *this;
}

bool UartPort::open() {
    close();

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        lastError_ = "open() failed: " + std::string(std::strerror(errno));
        return false;
    }

    if (!configurePort()) {
        close();
        return false;
    }
    return true;
}

void UartPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    rxBuffer_.clear();
}

bool UartPort::isOpen() const { return fd_ >= 0; }

const std::string& UartPort::lastError() const { return lastError_; }


bool UartPort::sendLine(const sendPacket& d) {
    constexpr uint32_t kSendTimeoutMs = 20;

    if (fd_ < 0) { lastError_ = "sendLine: port not open"; return false; }

    std::array<uint8_t, 64> buf;
    const size_t n = pack(d, buf);
    if (n == 0) { lastError_ = "sendLine: buffer too small"; return false; }

    const uint8_t* p = buf.data();
    size_t left = n;
    const uint64_t start = millis();

    while (left > 0) {
        const ssize_t w = ::write(fd_, p, left);
        if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                if (millis() - start > kSendTimeoutMs) {
                    lastError_ = "sendLine: timeout";
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            lastError_ = "write() failed: " + std::string(std::strerror(errno));
            return false;
        }
        p    += w;
        left -= static_cast<size_t>(w);
    }
    return true;
}

bool UartPort::receiveLine(sendPacket& d, std::uint32_t timeoutMs) {
   const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while(true) {
        if (tryParse(d)) return true;             // frame already buffered or just completed
        if (msUntil(deadline) == 0) return false; // timed out

        // Pull whatever the port has
        const int32_t avail = get_read_avail(fd_);
        if (avail > 0) {
            const size_t room = rx_.size() - rxLen_;
            const size_t want = std::min(static_cast<size_t>(avail), room);
            const int32_t got = ::read(fd_, rx_.data() + rxLen_, static_cast<int32_t>(want));
            if (got > 0) { rxLen_ += static_cast<size_t>(got); continue; }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));                           // nothing arrived, yield to other tasks
    }
}

uint16_t UartPort::crc16(const uint8_t* p, size_t n) {   // CRC-16/CCITT-FALSE
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= static_cast<uint16_t>(*p++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}


size_t UartPort::pack(const sendPacket& d, std::span<uint8_t> out) {
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

    put(static_cast<float>(d.posX));
    put(static_cast<float>(d.posY));
    put(static_cast<float>(d.heading));

    // Payload
    //put(static_cast<uint32_t>(d.id));
    //put(static_cast<float>(d.temperature));
    //put(static_cast<uint16_t>(d.x));
    //put(static_cast<uint8_t>(d.ok ? 1 : 0));

    // CRC covers type..payload (everything after the sync bytes)
    put(crc16(out.data() + kSyncLen, static_cast<size_t>(p - out.data()) - kSyncLen));

    return static_cast<size_t>(p - out.data());
}

bool UartPort::tryParse(sendPacket& d) {
    
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



        float posX;
        float posY;
        float heading;

        get(posX);
        get(posY);
        get(heading);

        d.posX = posX;
        d.posY = posY;
        d.heading = heading;


        //uint32_t id;  float temp;  uint16_t x;  uint8_t ok;
        //get(id);  get(temp);  get(x);  get(ok);

        //d.id          = id;
        //d.temperature = temp;
        //d.x           = static_cast<decltype(d.x)>(x);
        //d.ok          = (ok != 0);

        drop(kFrameLen);
        return true;
    }
    return false;
}

bool UartPort::configurePort() {
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        lastError_ = "configurePort: tcgetattr failed: " + std::string(std::strerror(errno));
        return false;
    }

    cfsetispeed(&tty, baud_);
    cfsetospeed(&tty, baud_);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        lastError_ = "configurePort: tcsetattr failed: " + std::string(std::strerror(errno));
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

uint64_t UartPort::msUntil(std::chrono::steady_clock::time_point deadline) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    return remaining > 0 ? static_cast<uint64_t>(remaining) : 0;
}

uint64_t UartPort::millis() {
    static const auto start_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
}

int32_t UartPort::get_read_avail(int fd){
    int n = 0;
    if (ioctl(fd, FIONREAD, &n) < 0) return -1;
    return static_cast<int32_t>(n);
}