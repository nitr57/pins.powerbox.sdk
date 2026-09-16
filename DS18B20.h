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

            void measureTemperature(void);
            float getTemperature(void);

        private:
            bool reset_(void);
            void write_(uint8_t byte);
            uint8_t read_(void);
            uint8_t crc_(uint8_t* data);
            void delay_us_(int delay);

            const GPIOManager* gpio_;

            uint8_t pin_;

            float current_temperature_;

            // Last good value, reused for a grace period on transient read failures
            float last_temperature_;
            bool last_valid_;
            std::chrono::steady_clock::time_point last_time_;
    };
} /* namespace PowerBox */

#endif /* DS18B20_H */