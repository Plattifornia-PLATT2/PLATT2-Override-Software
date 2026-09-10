#include "uartPort.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

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

bool UartPort::sendLine(const std::string& text) {
    if (!isOpen()) {
        lastError_ = "sendLine: link not open";
        return false;
    }

    std::string frame = text;
    frame.push_back('\n');

    size_t written = 0;
    while (written < frame.size()) {
        ssize_t n = ::write(fd_, frame.data() + written, frame.size() - written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            lastError_ = "sendLine: write() failed: " + std::string(std::strerror(errno));
            return false;
        }
        written += static_cast<size_t>(n);
    }
    return true;
}

std::optional<std::string> UartPort::receiveLine(int timeoutMs) {
    if (!isOpen()) {
        lastError_ = "receiveLine: link not open";
        return std::nullopt;
    }

    // Already have a full line buffered from a previous read.
    if (auto pos = rxBuffer_.find('\n'); pos != std::string::npos) {
        std::string line = rxBuffer_.substr(0, pos);
        rxBuffer_.erase(0, pos + 1);
        return line;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    char chunk[256];

    while (true) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) {
            lastError_ = "receiveLine: timed out";
            return std::nullopt;
        }

        pollfd pfd{fd_, POLLIN, 0};
        int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
        if (ready < 0) {
            if (errno == EINTR) continue;
            lastError_ = "receiveLine: poll() failed: " + std::string(std::strerror(errno));
            return std::nullopt;
        }
        if (ready == 0) {
            lastError_ = "receiveLine: timed out";
            return std::nullopt;
        }

        ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            lastError_ = "receiveLine: read() failed: " + std::string(std::strerror(errno));
            return std::nullopt;
        }
        if (n == 0) continue;

        rxBuffer_.append(chunk, static_cast<size_t>(n));

        if (auto pos = rxBuffer_.find('\n'); pos != std::string::npos) {
            std::string line = rxBuffer_.substr(0, pos);
            rxBuffer_.erase(0, pos + 1);
            return line;
        }
    }
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