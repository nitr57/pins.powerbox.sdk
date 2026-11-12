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


#ifndef INA219_H
#define INA219_H

#include "ErrorThrottle.h"
#include <stdint.h>

namespace PowerBox
{
    /* INA219 current/voltage monitor with a 0.1 ohm shunt (TI SBOS448).
     * Configured for +-320 mV shunt range (PGA /8, 3.2 A), shunt ADC averaging
     * 128 samples (68 ms), bus 12 bit, 16 V range, continuous. The current is
     * computed from the shunt voltage register, which does not depend on the
     * calibration register (that one is lost when the chip resets). */
    class INA219
    {
        public:
            enum Result
            {
                OK,
                ERROR,      // no answer (NACK) or other error, retried before
                TIMEOUT     // bus timeout, not retried (the bus may hang)
            };

            /* Shunt register value at the end of the range: it clamps there */
            static constexpr int16_t SHUNT_FULL_SCALE = 32000;

            INA219(const char* dev, uint8_t address);
            ~INA219(void);

            /* Opens the device (if needed) and writes the configuration; OK if it
             * was written and read back. Can be called again after a failure;
             * failures are logged rate limited. */
            Result begin(void);
            bool isConfigured(void) const { return this->configured_; }

            /* Reads the configuration and writes it again if the chip lost it
             * (power-on reset). rewritten is set if it was written; otherRange if
             * the value found had another shunt range (PGA) than +-320 mV, then the
             * shunt value of this measurement may be too low (a smaller range
             * clamps lower; the LSB is the same in every range). If writing it
             * again fails, the chip counts as not configured (begin() again). */
            Result checkConfig(bool& rewritten, bool& otherRange);

            /* Shunt register: 10 uV per LSB = 0.1 mA at 0.1 ohm, signed */
            Result readShunt(int16_t& raw);
            Result readBus_mV(uint16_t& mV);

            uint8_t getAddress(void) const { return this->address_; }

        private:
            Result configure_(void);
            Result readRegister_(uint8_t reg, uint16_t& val);
            Result writeRegister_(uint8_t reg, uint16_t val);

            // I2C
            int i2c_fd_;
            const char* dev_;
            uint8_t address_;
            bool configured_;

            // Rate limit the log of failing transfers, of a failing setup, of chip
            // resets and of unexpected configurations
            ErrorThrottle ioError_;
            ErrorThrottle setupError_{std::chrono::seconds(600)};
            ErrorThrottle resetLog_{std::chrono::seconds(600)};
            ErrorThrottle unexpectedLog_{std::chrono::seconds(600)};
    };
} /* namespace PowerBox */

#endif // INA219_H
