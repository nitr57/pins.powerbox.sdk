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


#ifndef BUZZER_H
#define BUZZER_H

#include "PWM.h"
#include "PowerBoxSDK.h"
#include "ErrorThrottle.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace PowerBox
{
    /* Piezo buzzer (FUET-7525) on a hardware PWM channel. beep() starts the tone
     * and returns at once; a worker thread ends it. A new beep() replaces a
     * running tone (volume and end time), volume 0 ends it. Thread safe. */
    class Buzzer
    {
        public:
            Buzzer(const char* chip, int ch);
            /* Ends a running tone and switches the buzzer off */
            ~Buzzer(void);

            /* Exports the channel and switches the buzzer off (a tone left on by
             * an earlier process that ended during a beep) */
            void begin(void);

            /* volume 0..100 (clamped), duration_ms > 0, at most MAX_DURATION_MS.
             * PB_ERROR_COMMUNICATION if the tone could not be started. */
            PB_ERROR_TYPE beep(int volume, int duration_ms);

            static constexpr int MAX_DURATION_MS = 10000;

        private:
            /* The FUET-7525 is loudest around 2700 Hz */
            static constexpr unsigned int FREQ_HZ = 2700;
            static constexpr unsigned int PERIOD_NS = 1000000000u / FREQ_HZ;
            static constexpr std::chrono::seconds OFF_RETRY{1};

            bool start_(unsigned int duty_ns);
            bool off_(void);
            void run_(void);

            PWM pwm_;
            std::string name_;
            std::mutex mutex_;
            std::condition_variable cv_;
            std::thread worker_;
            bool quit_ = false;
            bool on_ = false;
            std::chrono::steady_clock::time_point stopAt_;
            ErrorThrottle offError_;
    };
} /* namespace PowerBox */

#endif /* BUZZER_H */
