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

#ifndef PWM_H
#define PWM_H

#include <string>

class PWM
{
    public:
        PWM(const char* chip, int ch);
        ~PWM(void);

        /* Exports the channel. With wait = true a failing export is retried for
         * up to 2 s (chip not there yet), and after a successful export it waits
         * up to 2 s until udev made period/duty_cycle/enable writable. After the
         * first export timeout in the process further exports do not wait, after
         * the first udev timeout no further channel waits for udev. An already
         * exported channel is used as it is. */
        bool setExport(bool wait = true);

        /* The channel is exported (pwmN exists) */
        bool exists(void) const;

        bool setPeriod(unsigned int period_ns);
        bool setDutyCycle(unsigned int duty_ns);
        bool setState(bool state);

        unsigned int getPeriod(void) const;
        unsigned int getDutyCycle(void) const;
        int getState(void) const;

    private:
        bool writable_(void) const;
        std::string read_(const std::string& path) const;
        bool write_(const std::string& path, const std::string& val);

        std::string chip_;
        int channel_;
        bool exported_;
        bool exportErrorLogged_;
};

#endif /* PWM_H */