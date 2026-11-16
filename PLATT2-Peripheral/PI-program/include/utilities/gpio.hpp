#pragma once
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

class GpioOut {
    int fd_ = -1;
public:
    explicit GpioOut(unsigned pin, bool initial = false,
                     const char *chip = "/dev/gpiochip0") {
        int cfd = open(chip, O_RDWR);
        if (cfd < 0) return;

        gpio_v2_line_request req{};
        req.offsets[0] = pin;
        req.num_lines = 1;
        std::strncpy(req.consumer, "gpio", sizeof(req.consumer) - 1);
        req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        req.config.num_attrs = 1;
        req.config.attrs[0].mask = 1;
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        req.config.attrs[0].attr.values = initial;

        if (ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req) == 0) fd_ = req.fd;
        close(cfd);
    }
    ~GpioOut() { if (fd_ >= 0) close(fd_); }

    GpioOut(const GpioOut &) = delete;
    GpioOut &operator=(const GpioOut &) = delete;

    bool ok() const { return fd_ >= 0; }

    bool write(bool high) {
        gpio_v2_line_values v{};
        v.mask = 1;
        v.bits = high;
        return ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) == 0;
    }
    bool high() { return write(true); }
    bool low()  { return write(false); }
};