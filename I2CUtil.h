/* *******************************************************************************
 * MIT License
 *
 * Copyright (c) 2026 Nico Trost
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * **************************************************************************** */


#ifndef I2C_UTIL_H
#define I2C_UTIL_H

#include "PowerBoxLogging.h"
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace PowerBox
{
    /* Transfer timeout of the I2C adapter in 10 ms units. A hung bus (SDA held
     * low) would otherwise block every transfer for the adapter default (about
     * 1 s) while the hardware lock is held. Note: the value applies to the whole
     * adapter (all users of /dev/i2c-N) and stays set until reboot; whether the
     * bus driver honors it depends on the adapter (e.g. not for a mux child). */
    constexpr int I2C_TIMEOUT_10MS = 5;

    inline void SetI2CTimeout(int fd, const char* who)
    {
        if (ioctl(fd, I2C_TIMEOUT, I2C_TIMEOUT_10MS) < 0)
        {
            PB_DEBUG("%s: could not set the I2C timeout", who);
        }
    }
} /* namespace PowerBox */

#endif /* I2C_UTIL_H */
