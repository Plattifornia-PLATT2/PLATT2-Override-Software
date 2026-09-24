/**
 * @file sfTkLinuxI2C.h
 * @brief Linux (i2c-dev / ioctl) implementation of the SparkFun Toolkit
 *        sfTkII2C interface, so SparkFun's platform-independent device
 *        drivers (like sfDevOTOS) can be used on a Raspberry Pi / any
 *        Linux SBC without the Arduino Wire library.
 */
#pragma once

#include <string>
#include <utilities/sfTk/sfTkII2C.h>

class sfTkLinuxI2C : public sfTkII2C
{
  public:
    sfTkLinuxI2C() : sfTkII2C(), _fd(-1)
    {
    }

    sfTkLinuxI2C(uint8_t addr) : sfTkII2C(addr), _fd(-1)
    {
    }

    ~sfTkLinuxI2C()
    {
        close();
    }

    /// @brief Opens the given I2C bus device (e.g. "/dev/i2c-1") and binds
    ///        this object to the address already set (or pass one in).
    /// @return true on success
    bool init(const std::string &i2cDevice, uint8_t addr, bool /*bInit*/ = false)
    {
        setAddress(addr);
        return openBus(i2cDevice);
    }

    bool openBus(const std::string &i2cDevice);
    void close();

    // --- sfTkII2C / sfTkIBus required overrides ---

    sfTkError_t ping() override;

    sfTkError_t writeRegister(uint8_t *devReg, size_t regLength, const uint8_t *data, size_t length) override;

    sfTkError_t readRegister(uint8_t *devReg, size_t regLength, uint8_t *data, size_t numBytes, size_t &readBytes,
                              uint32_t read_delay = 0) override;

  private:
    int _fd;
};
