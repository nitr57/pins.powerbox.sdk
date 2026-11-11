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

#ifndef DS18B20_H
#define DS18B20_H

#include "GPIOManager.h"
#include <chrono>

#define DEVICE_DISCONNECTED_C -127.f

namespace PowerBox
{
    // How long the last good sensor value is reused when reads fail
    constexpr std::chrono::seconds SENSOR_GRACE_PERIOD{60};

    class DS18B20
    {
        public:
            DS18B20(const GPIOManager& gpio);
            ~DS18B20(void);

            bool begin(uint8_t pin);

            /* Reads the result of the conversion started by the previous call
             * (once, retried only on a missing presence pulse or a CRC error),
             * checks it and starts the next conversion. Called once per cycle
             * (the 12 bit conversion takes up to 750 ms). */
            void update(void);

            /* Last plausible value, for at most SENSOR_GRACE_PERIOD after it
             * was read; DEVICE_DISCONNECTED_C otherwise. Does not access the
             * bus. */
            float getTemperature(void) const;

        private:
            bool readScratchpad_(uint8_t* data);
            bool plausible_(const uint8_t* data, float& temperature);
            bool reset_(void);
            void write_(uint8_t byte);
            uint8_t read_(void);
            uint8_t crc_(const uint8_t* data);
            void delay_us_(int delay);

            const GPIOManager* gpio_;

            uint8_t pin_;

            // A conversion was started by the previous update()
            bool converting_;

            // Last plausible value, reused for a grace period on failed reads
            float last_temperature_;
            bool last_valid_;
            std::chrono::steady_clock::time_point last_time_;

            // A value far from the last one is only taken over after it was
            // read JUMP_CONFIRM times in a row (within JUMP_MATCH_K); without a
            // reference (first value, or none for SENSOR_GRACE_PERIOD)
            // NOREF_CONFIRM times
            static constexpr float JUMP_LIMIT_K = 10.f;
            static constexpr float JUMP_MATCH_K = 1.f;
            static constexpr int JUMP_CONFIRM = 3;
            static constexpr int NOREF_CONFIRM = 2;
            float jump_temperature_;
            int jump_count_;
    };
} /* namespace PowerBox */

#endif /* DS18B20_H */