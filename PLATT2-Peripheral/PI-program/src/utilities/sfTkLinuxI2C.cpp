#include "utilities/sfTkLinuxI2C.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

bool sfTkLinuxI2C::openBus(const std::string &i2cDevice)
{
    close();

    _fd = ::open(i2cDevice.c_str(), O_RDWR);
    if (_fd < 0)
    {
        std::fprintf(stderr, "sfTkLinuxI2C: failed to open %s: %s\n", i2cDevice.c_str(), std::strerror(errno));
        return false;
    }

    return true;
}

void sfTkLinuxI2C::close()
{
    if (_fd >= 0)
    {
        ::close(_fd);
        _fd = -1;
    }
}

sfTkError_t sfTkLinuxI2C::ping()
{
    if (_fd < 0)
        return ksfTkErrBusNotInit;

    // A zero-length write is enough to get an ACK/NACK from the device.
    i2c_msg msg;
    msg.addr = address();
    msg.flags = 0; // write
    msg.len = 0;
    msg.buf = nullptr;

    i2c_rdwr_ioctl_data packet;
    packet.msgs = &msg;
    packet.nmsgs = 1;

    if (ioctl(_fd, I2C_RDWR, &packet) < 0)
        return ksfTkErrBusNoResponse;

    return ksfTkErrOk;
}

sfTkError_t sfTkLinuxI2C::writeRegister(uint8_t *devReg, size_t regLength, const uint8_t *data, size_t length)
{
    if (_fd < 0)
        return ksfTkErrBusNotInit;

    // Build one contiguous buffer: [register address bytes][data bytes]
    // and send it as a single I2C write transaction.
    size_t totalLen = regLength + length;
    uint8_t stackBuf[64];
    uint8_t *buf = stackBuf;
    bool heapAlloc = false;

    if (totalLen > sizeof(stackBuf))
    {
        buf = new uint8_t[totalLen];
        heapAlloc = true;
    }

    if (regLength > 0 && devReg != nullptr)
        std::memcpy(buf, devReg, regLength);
    if (length > 0 && data != nullptr)
        std::memcpy(buf + regLength, data, length);

    i2c_msg msg;
    msg.addr = address();
    msg.flags = 0; // write
    msg.len = static_cast<uint16_t>(totalLen);
    msg.buf = buf;

    i2c_rdwr_ioctl_data packet;
    packet.msgs = &msg;
    packet.nmsgs = 1;

    sfTkError_t result = (ioctl(_fd, I2C_RDWR, &packet) < 0) ? ksfTkErrFail : ksfTkErrOk;

    if (heapAlloc)
        delete[] buf;

    return result;
}

sfTkError_t sfTkLinuxI2C::readRegister(uint8_t *devReg, size_t regLength, uint8_t *data, size_t numBytes,
                                        size_t &readBytes, uint32_t read_delay)
{
    readBytes = 0;

    if (_fd < 0)
        return ksfTkErrBusNotInit;

    // Two-message combined transaction: write the register address, then
    // read the data, using a repeated START (no STOP in between). This is
    // what the OTOS needs, and is exactly what ioctl(I2C_RDWR, ...) gives
    // you on Linux - the two messages share one bus transaction.
    i2c_msg msgs[2];
    int nmsgs = 0;

    if (regLength > 0 && devReg != nullptr)
    {
        msgs[nmsgs].addr = address();
        msgs[nmsgs].flags = 0; // write
        msgs[nmsgs].len = static_cast<uint16_t>(regLength);
        msgs[nmsgs].buf = devReg;
        nmsgs++;
    }

    msgs[nmsgs].addr = address();
    msgs[nmsgs].flags = I2C_M_RD; // read
    msgs[nmsgs].len = static_cast<uint16_t>(numBytes);
    msgs[nmsgs].buf = data;
    nmsgs++;

    i2c_rdwr_ioctl_data packet;
    packet.msgs = msgs;
    packet.nmsgs = nmsgs;

    // read_delay (ms) is used by some drivers to pause between the register
    // write and the read, e.g. for sensors that need conversion time. The
    // combined ioctl transaction above can't insert a delay mid-transaction,
    // so if the driver asks for one, do two separate transactions instead.
    if (read_delay > 0 && nmsgs == 2)
    {
        i2c_msg writeMsg = msgs[0];
        i2c_rdwr_ioctl_data writePacket;
        writePacket.msgs = &writeMsg;
        writePacket.nmsgs = 1;

        if (ioctl(_fd, I2C_RDWR, &writePacket) < 0)
            return ksfTkErrFail;

        usleep(read_delay * 1000);

        i2c_msg readMsg = msgs[1];
        i2c_rdwr_ioctl_data readPacket;
        readPacket.msgs = &readMsg;
        readPacket.nmsgs = 1;

        if (ioctl(_fd, I2C_RDWR, &readPacket) < 0)
            return ksfTkErrFail;

        readBytes = numBytes;
        return ksfTkErrOk;
    }

    if (ioctl(_fd, I2C_RDWR, &packet) < 0)
        return ksfTkErrFail;

    readBytes = numBytes;
    return ksfTkErrOk;
}
