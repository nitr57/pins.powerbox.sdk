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

#ifndef MCP320X_H
#define MCP320X_H

#include <stdint.h>
#include "ErrorThrottle.h"

namespace PowerBox
{
    class SPIBus;
    class GPIOManager;

    /* Statistics of a burst of samples of one channel, for a current sense that
     * may be chopped by a PWM (on-phase / off-phase) */
    struct SenseStats
    {
        int n = 0;              // samples taken
        float mean_mV = 0.f;    // mean of all samples
        float min_mV = 0.f;
        float max_mV = 0.f;
        float on_mV = 0.f;      // median of the upper cluster (split at (min + max) / 2)
        int on_count = 0;       // samples in the upper cluster; 0 without a clear on/off structure
        int saturated = 0;      // samples at >= 95 % of the full scale
    };

    template <uint8_t NCH, uint8_t BIT>
    class MCP320X_
    {
        public:
            MCP320X_(const GPIOManager& gpio);
            ~MCP320X_(void);

            void begin(float vRef);
            uint8_t isConnected(void) const;

            uint16_t analogRead(uint8_t ch) const;
            uint16_t analogReadAverage(uint8_t ch, uint8_t n, uint16_t delay = 0) const;
            /* n samples (at most 255) with delay_us between them */
            void analogReadStats(uint8_t ch, uint8_t n, uint16_t delay_us, SenseStats& stats) const;
            float getFullScale_mV(void) const { return this->v_ref_ * 1e3f; }
            /* Log throttle of the shared sense line of this ADC (one stuck sense
             * output blocks it for all ports; used under the device's lock) */
            ErrorThrottle& senseBusThrottle(void) const { return this->sense_bus_throttle_; }

        private:
            /* Minimum spread (raw counts, ~10 mV) for an on/off structure */
            static constexpr uint16_t SPREAD_MIN = 12;

            SPIBus* spi_;
            const GPIOManager* gpio_;
            uint8_t init_;

            float v_ref_;
            mutable ErrorThrottle sense_bus_throttle_;
            mutable ErrorThrottle read_error_;
    };

    using MCP3202 = MCP320X_<2, 12>;
    using MCP3204 = MCP320X_<4, 12>;
} /* namespace PowerBox */

#endif // MCP320X_H