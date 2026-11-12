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

#include "DEW.h"
#include "common.h"
#include <algorithm>
#include <cmath>

namespace PowerBox
{
    template <typename MCP>
    DewPortT<MCP>::DewPortT(const GPIOManager& gpio, const MCP& mcp, const char* chip, int ch)
    {
        this->gpio_ = &gpio;
        this->mcp_ = &mcp;
        this->auto_mode_ = false;
        this->pwm_ok_ = false;
        this->avg_duty_ = 0.f;
        this->holding_ = false;
        this->hold_error_logged_ = false;
        this->runaway_running_ = false;
        this->runaway_warned_ = false;
        this->runaway_temperature_ = 0.f;
        this->period_ = 0;
        this->port_ = 0;
        this->fast_retries_ = 0;

        this->probe_ = new DS18B20(gpio);
        this->bts_ = new BTS7080<MCP>(gpio, mcp);
        this->pwm_ = new PWM(chip, ch);
    }

    template <typename MCP>
    DewPortT<MCP>::~DewPortT(void)
    {
        if(this->probe_) {
            delete this->probe_;
            this->probe_ = nullptr;
        }
        if(this->bts_) {
            delete this->bts_;
            this->bts_ = nullptr;
        }
        if(this->pwm_) {
            delete this->pwm_;
            this->pwm_ = nullptr;
        }

        this->auto_mode_ = false;
    }

    template <typename MCP>
    uint8_t DewPortT<MCP>::begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t probe_pin, uint8_t port, unsigned int freq, bool mode, uint8_t adc_ch, int keptIn, int index)
    {
        this->probe_->begin(probe_pin);

        // Take the running port over only if its PWM channel is configured as
        // expected, otherwise the duty cycle read back would be meaningless
        unsigned int period = (unsigned int)(1e9 / freq);
        bool keep = false;
        this->pwm_ok_ = false;
        this->period_ = period;
        // Log with the API index (the DSEL port differs from it on the PinsBox)
        this->port_ = index >= 0 ? (uint8_t)index : port;
        // A channel that is not exported cannot be taken over (and must not wait here)
        if(keptIn >= 0 && this->pwm_->exists() && this->pwm_->setExport())
        {
            keep = this->pwm_->getPeriod() == period;
            if(!keep)
            {
                PB_INFO("Dew port %u: PWM not configured as expected, initialized off", this->port_);
            }
        }

        this->bts_->setName("Dew port " + std::to_string(this->port_));
        this->bts_->begin(1200, diag_pin, pwr_pin, sel_pin, port, keep ? keptIn : 0);
        this->bts_->setMaxCurrent(3.f);
        this->bts_->setChannel(adc_ch);
        this->bts_->setSampling(100, 50);

        // Auto mode keeps the output until probe and dew point are valid (the
        // probe is read for the first time one cycle after begin)
        auto now = std::chrono::steady_clock::now();
        this->holding_ = false;
        this->hold_error_logged_ = false;
        this->start_window_end_ = now + START_WINDOW;
        this->runaway_running_ = false;
        this->runaway_warned_ = false;
        this->avg_time_ = now;

        if(keep)
        {
            this->pwm_ok_ = true;
            this->auto_mode_ = mode;

            // A taken over heater runs on with its duty
            this->avg_duty_ = this->outputDuty_();
            return true;
        }

        this->pwm_ok_ = this->initPwm_(true);
        this->avg_duty_ = 0.f;

        // Auto mode also without a working PWM: the pwm_ok_ guard keeps the switch
        // off, and auto mode then retries the PWM setup
        this->auto_mode_ = mode;

        if(!this->pwm_ok_)
        {
            PB_ERROR("Dew port %u: PWM could not be set up, the heater stays off (retried by auto mode or the next switch request: every second for a while if the channel is exported, else at most every 30 s)", this->port_);
            // Exported but not writable yet (udev slow at boot): retry soon
            this->fast_retries_ = 0;
            this->next_retry_ = std::chrono::steady_clock::now() +
                (this->pwm_->exists() ? std::chrono::seconds(1) : std::chrono::seconds(30));
        }

        return this->pwm_ok_;
    }

    template <typename MCP>
    bool DewPortT<MCP>::initPwm_(bool wait)
    {
        if(!this->pwm_->setExport(wait))
        {
            return false;
        }

        // Best effort first (the switch is already off): disable a channel that may
        // still run, and duty 0, because a duty above the new period would be
        // rejected. Right after an export the period may still be 0, then these
        // writes fail.
        this->pwm_->setState(0);
        this->pwm_->setDutyCycle(0);

        return this->pwm_->setPeriod(this->period_) &&
               this->pwm_->setDutyCycle(0) &&
               this->pwm_->setState(0);
    }

    template <typename MCP>
    bool DewPortT<MCP>::retryPwm_(void)
    {
        auto now = std::chrono::steady_clock::now();
        if(now < this->next_retry_)
        {
            return false;
        }
        this->next_retry_ = now + std::chrono::seconds(30);

        // Only with the switch off (it could not be switched off otherwise)
        if(this->bts_->getState() != 0)
        {
            return false;
        }

        if(!this->initPwm_(false))
        {
            // Exported, but udev has not made it writable yet: try again soon,
            // a limited number of times (a failed try logs no error)
            if(this->pwm_->exists() && this->fast_retries_ < FAST_RETRIES)
            {
                ++this->fast_retries_;
                this->next_retry_ = now + std::chrono::seconds(1);
            }
            return false;
        }

        PB_INFO("Dew port %u: PWM set up after a retry", this->port_);
        this->pwm_ok_ = true;
        return true;
    }

    template <typename MCP>
    void DewPortT<MCP>::update(float dewPoint, float threshold)
    {
        // Read the probe once per cycle (result of the conversion started by the
        // previous cycle) and start the next conversion
        this->probe_->update();

        // Update heater power, if auto mode is enabled.
        // After an over current the port stays off until the user intervenes
        // (auto mode off and on again, or switching the port manually), otherwise
        // auto mode would switch a faulty heater back on every second
        if(this->auto_mode_ && !this->bts_->getOverCurrent())
        {
            float temperature = this->probe_->getTemperature();
            bool probeOk = temperature != DEVICE_DISCONNECTED_C;
            bool dewPointOk = dewPoint != DEVICE_DISCONNECTED_C;
            auto now = std::chrono::steady_clock::now();

            if(!probeOk || !dewPointOk)
            {
                // Keep the average duty (see DUTY_AVERAGE_TIME)
                long avg = lroundf(this->avg_duty_);
                uint8_t hold = (uint8_t)std::min(255L, std::max(0L, avg));
                const char* what = !probeOk && !dewPointOk ? "probe temperature and dew point" :
                                   (!probeOk ? "probe temperature" : "dew point");

                // An error only while the heater runs without regulation; with
                // duty 0 (e.g. a port in auto mode without a probe) it is info
                if(!this->holding_)
                {
                    this->holding_ = true;
                    this->hold_error_logged_ = now >= this->start_window_end_;
                    if(!this->hold_error_logged_)
                    {
                        PB_INFO("Dew port %u: waiting for a valid %s, keeping duty %u", this->port_, what, hold);
                    }
                    else if(hold != 0)
                    {
                        PB_ERROR("Dew port %u: no valid %s, keeping duty %u (average of the last minutes) until the data is valid again", this->port_, what, hold);
                    }
                    else
                    {
                        PB_INFO("Dew port %u: no valid %s, the heater stays off until the data is valid", this->port_, what);
                    }
                    this->setState_(hold != 0, hold);
                }
                else
                {
                    if(!this->hold_error_logged_ && now >= this->start_window_end_)
                    {
                        if(hold != 0)
                        {
                            PB_ERROR("Dew port %u: still no valid %s, keeping duty %u until the data is valid", this->port_, what, hold);
                        }
                        else
                        {
                            PB_INFO("Dew port %u: still no valid %s, the heater stays off until the data is valid", this->port_, what);
                        }
                        this->hold_error_logged_ = true;
                    }

                    // Applied every cycle like the regulation does: a failed switch
                    // on is retried, and without a working PWM the PWM setup is
                    // retried (rate limited) with the switch off
                    this->setState_(hold != 0, hold);
                }

                this->runaway_running_ = false;
                this->runaway_warned_ = false;
            }
            else
            {
                if(this->holding_)
                {
                    this->holding_ = false;
                    PB_INFO("Dew port %u: probe temperature and dew point valid, regulating", this->port_);
                }

                // Temperture difference between current and dew point
                // e.g. Threshold ; Temp  ;  DP   ;  diff
                //              4      20      15     -1  ;  heater should be inactive
                //              4      12      11      3  ;  heater should be active
                float dT = std::max(threshold, 0.1f);
                float diff = (dewPoint + dT) - temperature;

                // Difference cannot exceed threshold or get negative
                diff = std::min(dT, std::max(0.f, diff));

                // Compute the required power duty cycle
                // e.g. for threshold of 4:
                // diff  -1 --> 0
                // diff   3 --> 191
                // diff 0.5 --> 32
                // diff   4 --> 255
                int dutyCycle = 255 * (diff / dT);

                this->setState_(dutyCycle != 0, dutyCycle);

                // Average duty, kept when the data gets invalid. The time step is
                // limited, so the first value after a long pause does not replace
                // the average.
                float dt = std::chrono::duration<float>(now - this->avg_time_).count();
                dt = std::min(std::max(dt, 0.f), 10.f);
                float alpha = std::min(1.f, dt / std::chrono::duration<float>(DUTY_AVERAGE_TIME).count());
                this->avg_duty_ += alpha * ((float)dutyCycle - this->avg_duty_);

                this->checkRunaway_(dutyCycle, temperature, now);
            }

            this->avg_time_ = now;
        }

        // Measure properties. The heater current is chopped by the low-side PWM
        // (400 Hz): the over current check uses the current while on
        this->bts_->measureCurrent(this->appliedDuty_(), SenseMode::ON_PHASE);

        // Over current, but the switch-off failed (retried every measurement):
        // stop the low-side PWM, so no current flows through the heater
        if(this->bts_->getOverCurrent() && this->bts_->getState() && this->pwm_ok_)
        {
            this->pwm_->setDutyCycle(0);
            this->pwm_->setState(0);
        }
    }

    template <typename MCP>
    float DewPortT<MCP>::appliedDuty_(void) const
    {
        if(!this->pwm_ok_ || this->pwm_->getState() != 1)
        {
            return 0.f;
        }

        unsigned int period = this->pwm_->getPeriod();
        unsigned int dutyCycle_ns = this->pwm_->getDutyCycle();
        return period > 0 ? std::min(1.f, (float)dutyCycle_ns / period) : 0.f;
    }

    template <typename MCP>
    void DewPortT<MCP>::checkRunaway_(int dutyCycle, float temperature, std::chrono::steady_clock::time_point now)
    {
        // Only while the heater really runs (not with the switch kept off
        // because the PWM is not set up)
        bool fullPower = dutyCycle >= RUNAWAY_DUTY && this->pwm_ok_ && this->bts_->getState() != 0;
        if(!fullPower)
        {
            this->runaway_running_ = false;
            this->runaway_warned_ = false;
            return;
        }

        if(!this->runaway_running_)
        {
            this->runaway_running_ = true;
            this->runaway_start_ = now;
            this->runaway_temperature_ = temperature;
            return;
        }

        if(now - this->runaway_start_ >= RUNAWAY_TIME)
        {
            // Only a warning: limiting the power would also hit a heater that is
            // just too weak (frost, wind), when it is needed most
            float rise = temperature - this->runaway_temperature_;
            if(rise < RUNAWAY_RISE_K)
            {
                if(!this->runaway_warned_)
                {
                    PB_ERROR("Dew port %u: heater at full power for %d min, but the probe got only %.1f K warmer: probe not attached to the heater, or heater too weak?",
                             this->port_, (int)RUNAWAY_TIME.count(), rise);
                    this->runaway_warned_ = true;
                }
            }
            else
            {
                // Heated: a later detached probe is reported again
                this->runaway_warned_ = false;
            }

            // Next period, so a probe that falls off later is noticed as well
            this->runaway_start_ = now;
            this->runaway_temperature_ = temperature;
        }
    }

    template <typename MCP>
    void DewPortT<MCP>::setAutoMode(uint8_t mode)
    {
        if(mode != 0 && !this->auto_mode_)
        {
            // Switching auto mode on is a user action that releases an over current lockout
            this->bts_->resetOverCurrent();

            // Bumpless: the regulation (and holding without valid data) starts
            // from the current output
            this->avg_duty_ = this->outputDuty_();
            this->avg_time_ = std::chrono::steady_clock::now();
            this->runaway_running_ = false;
            this->runaway_warned_ = false;
        }

        if(mode == 0 || !this->auto_mode_)
        {
            this->holding_ = false;
        }

        this->auto_mode_ = mode != 0;
    }

    template <typename MCP>
    uint8_t DewPortT<MCP>::outputDuty_(void) const
    {
        return this->bts_->getState() != 0 ? this->getDutyCycle() : 0;
    }

    template <typename MCP>
    void DewPortT<MCP>::setState(uint8_t state, uint8_t dc)
    {
        if(!this->auto_mode_)
        {
            this->setState_(state, state != 0 ? dc : 0);
        }
    }

    template <typename MCP>
    uint8_t DewPortT<MCP>::getDutyCycle(void) const
    {
        // A channel that is not set up says nothing about the port
        if(!this->pwm_ok_)
        {
            return 0;
        }

        unsigned int period = this->pwm_->getPeriod();
        unsigned int dutyCycle_ns = this->pwm_->getDutyCycle();
        
        if (period == 0) {
            return 0;
        }

        // Convert nanoseconds back to 0-255 scale. Rounded: setting truncates
        // (period * dc / 255), so truncating here as well would read back dc - 1
        uint64_t dc = ((uint64_t)dutyCycle_ns * 255 + period / 2) / period;
        return (uint8_t)std::min<uint64_t>(dc, 255);
    }

    template <typename MCP>
    void DewPortT<MCP>::setState_(uint8_t state, uint8_t dutyCycle)
    {
        // Without a working PWM the heater must not be switched on, neither by
        // auto mode (also when switched on later by the user) nor manually.
        // The PWM setup is retried now and then, with the switch off.
        if(!this->pwm_ok_)
        {
            this->bts_->setState(0);
            if(!this->retryPwm_())
            {
                return;
            }
        }

        // A failed over current shutdown keeps the switch on: no new duty, the PWM
        // stays stopped; switching off is still tried
        if(this->bts_->getOverCurrent() && this->bts_->getState())
        {
            this->pwm_->setDutyCycle(0);
            this->pwm_->setState(0);
            if(!state)
            {
                this->bts_->setState(0);
            }
            else
            {
                PB_ERROR("Dew port %u: over current shutdown pending, request rejected", this->port_);
            }
            return;
        }

        // Convert duty cycle to nano seconds
        unsigned int period = this->pwm_->getPeriod();
        unsigned int dc = (unsigned int)(((uint64_t)period * dutyCycle) / 255);

        // Set duty cycle
        this->pwm_->setDutyCycle(dc);
        this->pwm_->setState(dutyCycle != 0);
        this->bts_->setState(state);
    }

    /* Explicit instantiations for the supported ADC types */
    template class DewPortT<MCP3204>;
    template class DewPortT<MCP3202>;
} /* namespace PowerBox */