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

#ifndef DEW_H
#define DEW_H

#include "PWM.h"
#include "GPIOManager.h"
#include "MCP320X.h"
#include "BTS7XXX.h"
#include "DS18B20.h"
#include <cstdint>
#include <chrono>

namespace PowerBox
{
    template <typename MCP = MCP3204>
    class DewPortT
    {
        public:
            DewPortT(const GPIOManager& gpio, const MCP& mcp, const char* chip, int ch);
            ~DewPortT(void);

            /* keptIn: level of the power switch input taken over from an earlier
             * process (-1 = none). If >= 0 and the PWM channel already runs with
             * the requested period, the port is kept as it is (switch, duty,
             * enable); otherwise it is initialized off. */
            /* index: port number used in log messages (the API index); -1 = port */
            uint8_t begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t probe_pin, uint8_t port, unsigned int freq, bool mode, uint8_t adc_ch = 2, int keptIn = -1, int index = -1);
            void update(float dewPoint, float threshold);

            void enable(void) { this->bts_->enable(); }
            void disable(void) { this->bts_->disable(); }
            void toggle(void) { this->bts_->toggle(); }
            void setState(uint8_t state, uint8_t dc);
            void setAutoMode(uint8_t mode);

            uint16_t getCurrent_mA(void) const { return this->bts_->getCurrent_mA(); }
            uint8_t getState(void) const { return this->bts_->getState(); }
            uint8_t getAutoMode(void) const { return this->auto_mode_; }
            uint8_t getDutyCycle(void) const;
            uint8_t getOverCurrent(void) const { return this->bts_->getOverCurrent(); }
            /* PWM channel set up; without it the heater is never switched on */
            bool isPwmOk(void) const { return this->pwm_ok_; }
            float getTemperature(void) const { return this->probe_->getTemperature(); }

            void resetOverCurrent(void) { this->bts_->resetOverCurrent(); }
            /* See BTS7XXX_::setSenseOffset (PinsBox Mini) */
            void setSenseOffset(float offset_mV, float coupling = 0.f) { this->bts_->setSenseOffset(offset_mV, coupling); }

        private:
            void setState_(uint8_t state, uint8_t dutyCycle);
            bool initPwm_(bool wait);
            bool retryPwm_(void);

            const GPIOManager* gpio_;
            const MCP* mcp_;

            DS18B20* probe_;
            BTS7080<MCP>* bts_;
            PWM* pwm_;

            bool auto_mode_;

            /* PWM channel set up (taken over or initialized); without it the
             * heater switch is never turned on. A failed setup is retried while
             * the switch is off: every second (up to FAST_RETRIES times) while
             * the channel is exported but not writable yet, else at most every
             * 30 s. */
            static constexpr int FAST_RETRIES = 10;
            bool pwm_ok_;
            unsigned int period_;
            uint8_t port_;
            int fast_retries_;
            std::chrono::steady_clock::time_point next_retry_;

            uint8_t outputDuty_(void) const;
            /* Duty (0..1) the PWM channel really applies, 0 if it does not run */
            float appliedDuty_(void) const;
            void checkRunaway_(int dutyCycle, float temperature, std::chrono::steady_clock::time_point now);

            /* Auto mode without a valid probe temperature or dew point keeps the
             * average duty of the last DUTY_AVERAGE_TIME (exponential average of
             * the regulated duty; starts from the output when the port is set up
             * or auto mode is switched on) until the data is valid again. The
             * port is not switched off: no data is not a reason to let the
             * optics fog up. Logged as info during START_WINDOW after begin()
             * (sensors not read yet) or with a duty of 0, as error otherwise. */
            static constexpr std::chrono::seconds DUTY_AVERAGE_TIME{300};
            static constexpr std::chrono::seconds START_WINDOW = SENSOR_GRACE_PERIOD;
            float avg_duty_;
            std::chrono::steady_clock::time_point avg_time_;
            bool holding_;
            bool hold_error_logged_;
            std::chrono::steady_clock::time_point start_window_end_;

            /* Runaway warning (no limit): heater switched on with a regulated
             * duty >= RUNAWAY_DUTY, and the probe gets less than RUNAWAY_RISE_K
             * warmer within RUNAWAY_TIME (checked in successive periods): the
             * probe is not heated by the heater (detached) or the heater is too
             * weak. Logged once; logged again only after a period with a rise or
             * after the full power phase ended. */
            static constexpr int RUNAWAY_DUTY = 250;
            static constexpr std::chrono::minutes RUNAWAY_TIME{15};
            static constexpr float RUNAWAY_RISE_K = 1.f;
            bool runaway_running_;
            bool runaway_warned_;
            std::chrono::steady_clock::time_point runaway_start_;
            float runaway_temperature_;
    };

    /* Backward-compatible alias: the historical MCP3204-based dew port */
    using DewPort = DewPortT<MCP3204>;
} /* namespace PowerBox */

#endif /* DEW_H */