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

#ifndef TCA953X_H
#define TCA953X_H

#include "ErrorThrottle.h"
#include <stdint.h>

template <uint8_t PINS>
class TCA953X_
{
    public:
        TCA953X_(const char *dev, uint8_t address);
        ~TCA953X_(void);

        uint8_t begin(void);

        uint8_t pinMode(uint8_t pin, uint8_t mode);
        uint8_t digitalWrite(uint8_t pin, uint8_t data);
        uint8_t digitalRead(uint8_t pin);

    private:
        bool writeRegister_(uint8_t reg, uint8_t data);
        bool readRegister_(uint8_t reg, uint8_t &data);

        bool init_;
        uint8_t lastInput_[2];  /* last good input port read per register, used when a read fails */
        PowerBox::ErrorThrottle ioError_;  /* rate limits the log while the bus keeps failing */
        int i2c_fd_;
        const char* dev_;
        uint8_t address_;
};

using TCA9534 = TCA953X_<8>;
using TCA9535 = TCA953X_<16>;

#endif // TCA953X_H
