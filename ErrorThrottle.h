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

#ifndef ERROR_THROTTLE_H
#define ERROR_THROTTLE_H

#include <chrono>

namespace PowerBox
{
    /* Rate limiter for hardware errors that repeat every measurement cycle:
     * report the first failure, then at most once per interval, and report
     * once when the operation works again. Not thread safe, the callers run
     * under the device's hardware lock. */
    class ErrorThrottle
    {
        public:
            explicit ErrorThrottle(std::chrono::seconds interval = std::chrono::seconds(60))
                : interval_(interval) {}

            /* Records a failure, returns true if it should be logged now */
            bool fail(void)
            {
                auto now = std::chrono::steady_clock::now();
                ++this->failures_;
                if (!this->failing_ || now - this->lastLog_ >= this->interval_)
                {
                    this->failing_ = true;
                    this->lastLog_ = now;
                    return true;
                }
                return false;
            }

            /* Records a success, returns true if it ended a series of failures */
            bool succeed(void)
            {
                if (!this->failing_)
                {
                    return false;
                }
                this->failing_ = false;
                this->failures_ = 0;
                return true;
            }

            /* Number of failures since the last success */
            unsigned failures(void) const { return this->failures_; }

        private:
            std::chrono::seconds interval_;
            std::chrono::steady_clock::time_point lastLog_;
            bool failing_ = false;
            unsigned failures_ = 0;
    };
} /* namespace PowerBox */

#endif /* ERROR_THROTTLE_H */
