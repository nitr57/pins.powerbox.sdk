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
#include <chrono>

namespace PowerBox
{
    class INA219;

    /* USB port of the PinsBox: enable pin plus INA219 current monitor.
     * Over current (software): immediately at the end of the measuring range
     * (3.2 A) or at IMMEDIATE_FACTOR x the limit, otherwise above the limit in
     * TRIP_COUNT measurements (invalid ones in between are skipped, the count
     * expires ABOVE_EXPIRY after the last one above the limit). The first
     * measurement after switching on, (re)configuring the INA219 or a chip
     * reset never trips immediately. Without a valid measurement (I2C error)
     * the port stays as it is and the current is unknown. */
    class USB
    {
        public:
            USB(const GPIOManager& gpio, const char* dev);
            ~USB(void);

            /* index: API port index, used in log messages */
            uint8_t begin(uint8_t pin, uint8_t address, float maxCurrent, uint8_t state = false, int index = -1);

            bool enable(void);
            bool disable(void);
            void toggle(void);

            /* I2C results of one measurement pass over all ports */
            struct BusPass
            {
                int timeouts = 0;       // transfers that timed out
                bool answered = false;  // at least one transfer succeeded
            };

            /* Measures voltage and current of a switched on port and checks for
             * over current. From BUS_TIMEOUT_LIMIT timeouts in the pass on, the
             * I2C bus is not accessed any more (the measurement is invalid). */
            void measure(BusPass& bus);

            static constexpr int BUS_TIMEOUT_LIMIT = 2;

            uint16_t getVoltage_mV(void) const { return this->voltage_mV_; }
            uint16_t getCurrent_mA(void) const { return this->current_mA_; }
            /* false: voltage and current unknown (port on, no valid measurement) */
            bool isValid(void) const { return this->valid_; }
            uint8_t getState(void) const { return this->enabled_; }
            uint8_t getOverCurrent(void) const { return this->over_current_; }

            void resetOverCurrent(void);

        private:
            void invalid_(void);
            static void count_(BusPass& bus, int res);

            static constexpr float IMMEDIATE_FACTOR = 1.5f;
            static constexpr int TRIP_COUNT = 2;
            static constexpr std::chrono::seconds ABOVE_EXPIRY{10};
            static constexpr std::chrono::seconds CONFIG_RETRY{30};

            uint16_t voltage_mV_;
            uint16_t current_mA_;
            bool valid_;

            // GPIO manager
            const GPIOManager* gpio_;

            // INA219 sensor
            const char* dev_;
            INA219* ina_;
            std::chrono::steady_clock::time_point next_config_;

            // Pin to enable / disable USB port
            uint8_t pin_;
            int index_;

            // Rate limits the log for a pin that keeps failing to switch
            ErrorThrottle switch_error_;

            // Current state
            uint8_t enabled_;

            // Max current
            float max_current_;

            // Over current flag
            uint8_t over_current_;

            // Valid measurements above the limit in a row (invalid ones are
            // skipped) and the time of the last one; no immediate trip on the
            // first measurement after switching on or (re)configuring
            int above_count_;
            std::chrono::steady_clock::time_point last_above_;
            bool settling_;
    };
} /* namespace PowerBox */

#endif // USB_H
