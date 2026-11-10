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

#ifndef USB_H
#define USB_H

#include "GPIOManager.h"
#include "ErrorThrottle.h"
#include <stdint.h>

namespace PowerBox
{
    class INA219;

    class USB
    {
        public:
            USB(const GPIOManager& gpio, const char* dev);
            ~USB(void);

            uint8_t begin(uint8_t pin, uint8_t address, float maxCurrent, uint8_t state = false);

            bool enable(void);
            bool disable(void);
            void toggle(void);

            void measureVoltage(void);
            void measureCurrent(void);
            void measurePower(void);

            uint16_t getVoltage_mV(void) const { return this->voltage_mV_; }
            uint16_t getCurrent_mA(void) const { return this->current_mA_; }
            uint16_t getPower_mW(void) const { return this->power_mW_; }
            uint8_t getState(void) const { return this->enabled_; }
            uint8_t getOverCurrent(void) const { return this->over_current_; }

            void resetOverCurrent(void);

        private:
            uint16_t voltage_mV_;
            uint16_t current_mA_;
            uint16_t power_mW_;

            // GPIO manager
            const GPIOManager* gpio_;

            // INA219 sensor
            const char* dev_;
            INA219* ina_;

            // Pin to enable / disable USB port
            uint8_t pin_;

            // Rate limits the log for a pin that keeps failing to switch
            ErrorThrottle switch_error_;

            // Current state
            uint8_t enabled_;

            // Max current
            float max_current_;

            // Over current flag
            uint8_t over_current_;
    };
} /* namespace PowerBox */

#endif // USB_H