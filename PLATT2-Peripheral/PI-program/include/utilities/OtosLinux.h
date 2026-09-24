/**
 * @file OtosLinux.h
 * @brief Trivial concrete subclass of sfDevOTOS. sfDevOTOS is abstract - it
 *        requires a derived class to supply delayMs(). On Arduino, the
 *        library provides that via Arduino's delay(); here we use usleep().
 */
#pragma once

#include <unistd.h>
#include <sfTk/sfDevOTOS.h>

class OtosLinux : public sfDevOTOS
{
  protected:
    void delayMs(uint32_t ms) override
    {
        usleep(ms * 1000);
    }
};
