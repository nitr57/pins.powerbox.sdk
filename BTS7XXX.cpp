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

#include "BTS7XXX.h"
#include "MCP320X.h"
#include "GPIOManager.h"
#include "common.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <thread>
#include <chrono>

namespace PowerBox
{

    template <typename MCP, uint16_t kILIS>
    BTS7XXX_<MCP, kILIS>::BTS7XXX_(const GPIOManager& gpio, const MCP& mcp)
    {
        this->gpio_ = &gpio;

        this->current_mA_ = 0;

        this->mcp_       = &mcp;
        this->mcp_ch_    = 0;
        this->mcp_n_     = 10;
        this->mcp_delay_ = 0;

        this->init_ = false;

        this->pwr_pin_  = UINT8_MAX;
        this->diag_pin_ = UINT8_MAX;
        this->sel_pin_  = UINT8_MAX;
        this->rsense_   = 1;
        this->port_     = 0;

        this->enabled_      = false;
        this->max_current_  = 0.0f;
        this->over_current_ = false;

        this->offset_mV_ = 0.f;
        this->coupling_  = 0.f;
        this->above_count_ = 0;
        this->saturated_count_ = 0;
        this->settling_ = true;
        this->sense_dirty_ = false;
        this->per_duty_mA_ = 0.f;
        this->per_duty_valid_ = false;
    }

    template <typename MCP, uint16_t kILIS>
    BTS7XXX_<MCP, kILIS>::~BTS7XXX_(void)
    {
        this->mcp_       = nullptr;
        this->mcp_ch_    = 0;
        this->mcp_n_     = 10;
        this->mcp_delay_ = 0;

        this->init_ = false;

        this->pwr_pin_  = UINT8_MAX;
        this->diag_pin_ = UINT8_MAX;
        this->sel_pin_  = UINT8_MAX;
        this->rsense_   = 1;
        this->port_     = 0;

        this->enabled_      = false;
        this->max_current_  = 0.0f;
        this->over_current_ = false;
    }

    template <typename MCP, uint16_t kILIS>
    uint8_t BTS7XXX_<MCP, kILIS>::begin(uint32_t rsense, uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, uint8_t state)
    {
        this->pwr_pin_  = pwr_pin;
        this->diag_pin_ = diag_pin;
        this->sel_pin_  = sel_pin;
        this->rsense_   = rsense;
        this->port_     = port;

        // Port starts in disabled state
        this->gpio_->digitalWrite(this->pwr_pin_, state);
        this->enabled_ = state;

        // Default log name
        if(this->name_.empty())
        {
            this->name_ = this->pwr_pin_ != UINT8_MAX ? "BTS port (pin " + std::to_string(this->pwr_pin_) + ")"
                                                      : "BTS sense (DEN pin " + std::to_string(this->diag_pin_) + ")";
        }

        // Diagnosis is disabled, selection pin low (retried by measureCurrent if
        // it fails, a sense left on would block the shared line)
        this->sense_dirty_ = true;
        this->releaseSense_();

        // No limit below the end of the measuring range until setMaxCurrent()
        this->max_current_ = 0.95f * this->mcp_->getFullScale_mV();

        // A port that is on (taken over) does not trip at once on its first measurement
        this->resetTrip_(true);

        // Initialization done
        this->init_ = true;

        return true;
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::setChannel(uint8_t ch)
    {
        this->mcp_ch_ = ch;
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::setSampling(uint8_t n, uint16_t delay)
    {
        this->mcp_n_ = std::max((uint8_t)0, n);
        this->mcp_delay_ = std::max((uint16_t)0, delay);
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::setMaxCurrent(float maxCurrent)
    {
        // The limit must be inside the measuring range (95 % of the ADC full scale),
        // above it only the end of the range trips
        float limit = std::max(0.0f, maxCurrent);
        float rangeLimit = 0.95f * this->fullScale_mA_() * 1e-3f;
        if(rangeLimit > 0.f && limit > rangeLimit)
        {
            limit = rangeLimit;
        }
        if(limit < maxCurrent)
        {
            PB_ERROR("%s: limit %.2f A above the measuring range, %.2f A used", this->name_.c_str(), maxCurrent, limit);
        }

        // Convert according to rsense and kilis (limit as sense voltage in mV)
        this->max_current_ = limit / kILIS * this->rsense_ * 1e3f;
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::setSenseOffset(float offset_mV, float coupling)
    {
        this->offset_mV_ = std::max(0.f, offset_mV);
        this->coupling_ = std::max(0.f, coupling);
    }

    template <typename MCP, uint16_t kILIS>
    float BTS7XXX_<MCP, kILIS>::fullScale_mA_(void) const
    {
        return this->mcp_ ? this->mcp_->getFullScale_mV() * kILIS / this->rsense_ : 0.f;
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::resetTrip_(bool counts)
    {
        // Switching keeps the counts (they expire ABOVE_EXPIRY after the last
        // measurement above the limit): a port switched off and on again between
        // its measurements (dew auto mode at duty 0) must still trip
        if(counts)
        {
            this->above_count_ = 0;
            this->saturated_count_ = 0;
        }
        this->settling_ = true;
        this->per_duty_valid_ = false;
    }

    template <typename MCP, uint16_t kILIS>
    bool BTS7XXX_<MCP, kILIS>::releaseSense_(void)
    {
        if(!this->sense_dirty_)
        {
            return true;
        }

        bool ok = this->gpio_->digitalWrite(this->diag_pin_, LOW);
        ok = this->gpio_->digitalWrite(this->sel_pin_, LOW) && ok;
        this->sense_dirty_ = !ok;
        if(!ok)
        {
            if(this->sense_release_error_.fail())
            {
                PB_ERROR("%s: sense output could not be switched off (DEN pin %u), retried (%u times)",
                         this->name_.c_str(), this->diag_pin_, this->sense_release_error_.failures());
            }
            return false;
        }
        if(this->sense_release_error_.succeed())
        {
            PB_INFO("%s: sense output switched off again", this->name_.c_str());
        }
        return true;
    }

    template <typename MCP, uint16_t kILIS>
    bool BTS7XXX_<MCP, kILIS>::enable(void)
    {
        if(this->init_ == true && this->enabled_ == false && this->pwr_pin_ != UINT8_MAX)
        {
            // Only track the port as enabled if the pin was actually switched
            if(!this->gpio_->digitalWrite(this->pwr_pin_, HIGH))
            {
                // The write may have reached the pin although it reported an error;
                // drive it low again so the port cannot end up on but unmonitored
                this->gpio_->digitalWrite(this->pwr_pin_, LOW);

                if(this->switch_error_.fail())
                {
                    PB_ERROR("%s: failed to switch on (pin %u, %u failures)", this->name_.c_str(), this->pwr_pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("%s: switches again", this->name_.c_str());
            }

            // Wait 1ms
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Set port enabled
            this->enabled_ = true;

            // Reset over current state; inrush: the first measurement does not
            // trip at once
            this->over_current_ = false;
            this->resetTrip_(false);
        }

        return true;
    }

    template <typename MCP, uint16_t kILIS>
    bool BTS7XXX_<MCP, kILIS>::disable(void)
    {
        if(this->init_ == true && this->enabled_ == true && this->pwr_pin_ != UINT8_MAX)
        {
            // If the pin could not be switched the port is still on: keep it marked
            // enabled, so it stays monitored and the next over current check retries
            if(!this->gpio_->digitalWrite(this->pwr_pin_, LOW))
            {
                if(this->switch_error_.fail())
                {
                    PB_ERROR("%s: failed to switch off (pin %u, %u failures)", this->name_.c_str(), this->pwr_pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("%s: switches again", this->name_.c_str());
            }

            // Set port disabled
            this->enabled_ = false;
            this->resetTrip_(false);
        }

        // Reset over current state
        this->over_current_ = false;

        return true;
    }

    template <typename MCP, uint16_t kILIS>
    bool BTS7XXX_<MCP, kILIS>::toggle(void)
    {
        if(this->init_ == true)
        {
            if(this->enabled_ == true)
            {
                return this->disable();
            }
            else
            {
                return this->enable();
            }
        }

        return false;
    }

    template <typename MCP, uint16_t kILIS>
    bool BTS7XXX_<MCP, kILIS>::setState(uint8_t state)
    {
        if(this->enabled_ == state)
        {
            return true;
        }

        return this->toggle();
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::measureCurrent(float duty, SenseMode mode)
    {
        if(this->init_ != true)
        {
            this->current_mA_ = 0;
            return;
        }

        const bool senseOnly = this->pwr_pin_ == UINT8_MAX;

        // A sense output that could not be switched off blocks the shared line for
        // all ports: retried first, also for a port that is off
        bool released = this->releaseSense_();

        // A previous over current shutdown failed and the port is still on:
        // retry it on every measurement until it succeeds
        if(this->over_current_ == true && this->enabled_ == true && !senseOnly)
        {
            if(this->disable())
            {
                this->over_current_ = true;
                this->current_mA_ = 0;
                PB_INFO("%s: switched off (over current)", this->name_.c_str());
                return;
            }
        }

        if(this->enabled_ != true && !senseOnly)
        {
            this->current_mA_ = 0;
            return;
        }

        SenseStats st;
        if(!released || !this->sample_(st) || st.n == 0)
        {
            // Not evaluated: the last current is kept, the trip state too
            return;
        }

        const float mA_per_mV = (float)kILIS / this->rsense_;
        const float limit_mA = this->max_current_ * mA_per_mV;
        duty = std::max(0.f, std::min(1.f, duty));

        // Mean current (display, energy). The + output of a chopped port is always
        // live, a short to ground draws DC: the mean is always checked as well.
        const float mean_mA = std::max(0.f, st.mean_mV - this->offset_mV_) * mA_per_mV;
        float on_mA = mean_mA;
        const char* how = "";

        if(mode == SenseMode::ON_PHASE && duty > 0.f && st.on_count >= ON_SAMPLES_MIN)
        {
            // The offset (mean of the other sense) contains this port's mean current
            // share duty x coupling, the on-phase level its full share: exact for
            // any duty
            float on = std::max(0.f, st.on_mV - this->offset_mV_) / (1.f + (1.f - duty) * this->coupling_) * mA_per_mV;
            if(on > on_mA)
            {
                on_mA = on;
                how = " while on";
            }
        }
        else if(mode == SenseMode::PER_DUTY)
        {
            if(duty > 0.f)
            {
                auto now = std::chrono::steady_clock::now();
                float estimate = mean_mA / std::max(duty, PER_DUTY_MIN);
                if(!this->per_duty_valid_)
                {
                    // Start from the mean (a lower bound), so that one noisy first
                    // estimate cannot trip
                    this->per_duty_mA_ = mean_mA;
                    this->per_duty_valid_ = true;
                }
                else
                {
                    float dt = std::min(2.f, std::chrono::duration<float>(now - this->per_duty_time_).count());
                    float alpha = std::min(1.f, dt / PER_DUTY_TAU);
                    this->per_duty_mA_ += alpha * (estimate - this->per_duty_mA_);
                }
                this->per_duty_time_ = now;
                if(this->per_duty_mA_ > on_mA)
                {
                    on_mA = this->per_duty_mA_;
                    how = " while on (estimated)";
                }
            }
            else
            {
                this->per_duty_valid_ = false;
            }
        }

        // End of the measuring range: (almost) all samples saturated means a BTS
        // fault (its sense drives the fault current) or a DC current far above
        // the range; some saturated samples mean a current above the range in the
        // on-phase, or a short spike (inrush of a capacitive load): counted
        const bool fault = st.saturated >= FAULT_SHARE * st.n && st.saturated >= SATURATED_MIN;
        const bool saturated = st.saturated >= SATURATED_MIN;
        this->saturated_count_ = saturated ? this->saturated_count_ + 1 : 0;

        const bool above = saturated || on_mA > limit_mA;
        if(above)
        {
            auto now = std::chrono::steady_clock::now();
            if(this->above_count_ > 0 && now - this->last_above_ > ABOVE_EXPIRY)
            {
                this->above_count_ = 0;
                this->saturated_count_ = saturated ? 1 : 0;
            }
            ++this->above_count_;
            this->last_above_ = now;
        }
        else
        {
            this->above_count_ = 0;
        }

        const bool immediate = !this->settling_ && (fault || on_mA >= IMMEDIATE_FACTOR * limit_mA);
        const bool trip = immediate || this->above_count_ >= TRIP_COUNT;
        this->settling_ = false;

        char reason[160];
        if(fault)
        {
            snprintf(reason, sizeof(reason), "BTS fault or current far above the measuring range (%.0f mA)", this->fullScale_mA_());
        }
        else if(this->saturated_count_ >= TRIP_COUNT || (saturated && immediate))
        {
            snprintf(reason, sizeof(reason), "current above the measuring range (%.0f mA)%s", this->fullScale_mA_(),
                     duty < 1.f ? " while on" : "");
        }
        else
        {
            snprintf(reason, sizeof(reason), "%.0f mA%s", on_mA, how);
        }

        if(senseOnly)
        {
            // Nothing to switch off: flag per measurement, the real current is shown
            this->over_current_ = trip;
            if(trip)
            {
                if(this->sense_only_log_.fail())
                {
                    PB_ERROR("%s: over current, %s, limit %.0f mA (not switchable)", this->name_.c_str(), reason, limit_mA);
                }
            }
            else if(this->above_count_ == 0)
            {
                this->sense_only_log_.succeed();
            }
            this->current_mA_ = (uint16_t)std::min(65535.f, mean_mA);
            return;
        }

        if(trip)
        {
            // Turn off the port. If the flag is already set the port is still on
            // because the shutdown failed; it was retried above this cycle already
            if(this->over_current_ == false)
            {
                bool off = this->disable();
                char dutyText[32] = "";
                if(duty < 1.f)
                {
                    snprintf(dutyText, sizeof(dutyText), ", duty %.0f %%", duty * 100.f);
                }
                PB_ERROR("%s: over current, %s, limit %.0f mA%s, %s", this->name_.c_str(), reason, limit_mA, dutyText,
                         off ? "switched off" : "switch-off failed, retrying");
            }
            this->over_current_ = true;
            this->above_count_ = 0;
            this->saturated_count_ = 0;
        }

        // Report 0 for a port that was shut off, but the real current while a
        // failed shutdown keeps the port on
        this->current_mA_ = this->enabled_ ? (uint16_t)std::min(65535.f, mean_mA) : 0;
    }

    template <typename MCP, uint16_t kILIS>
    uint8_t BTS7XXX_<MCP, kILIS>::getState(void) const
    {
        if(this->init_ == true)
        {
            return this->enabled_;
        }

        return false;
    }

    template <typename MCP, uint16_t kILIS>
    uint8_t BTS7XXX_<MCP, kILIS>::getOverCurrent(void) const
    {
        if(this->init_ == true)
        {
            return this->over_current_;
        }

        return false;
    }

    template <typename MCP, uint16_t kILIS>
    bool BTS7XXX_<MCP, kILIS>::sample_(SenseStats& stats)
    {
        if(this->mcp_ == nullptr)
        {
            return false;
        }

        // The sense line is shared by all BTS: with the own DEN still low it must
        // be idle. If not, another sense output is active (a DEN that could not
        // be switched off, or a faulted BTS): do not evaluate this port
        SenseStats idle;
        std::this_thread::sleep_for(std::chrono::microseconds(IDLE_WAIT_US));
        this->mcp_->analogReadStats(this->mcp_ch_, 2, 0, idle);
        if(idle.max_mV > IDLE_LIMIT_mV)
        {
            // Second look: the sense of the port measured just before may still decay
            std::this_thread::sleep_for(std::chrono::milliseconds(IDLE_RECHECK_MS));
            this->mcp_->analogReadStats(this->mcp_ch_, 2, 0, idle);
        }
        if(idle.max_mV > IDLE_LIMIT_mV)
        {
            if(this->mcp_->senseBusThrottle().fail())
            {
                PB_ERROR("%s: sense line not idle (%.0f mV) before the measurement, another sense output active? Ports not evaluated while it lasts (%u times)",
                         this->name_.c_str(), idle.max_mV, this->mcp_->senseBusThrottle().failures());
            }
            return false;
        }
        if(this->mcp_->senseBusThrottle().succeed())
        {
            PB_INFO("%s: sense line idle again, current measurement works", this->name_.c_str());
        }

        // Enable sense output and select the channel (switched off again below; a
        // failed switch-off is retried by the next measurement of this port). If
        // a write fails, the line would show nothing (0 A) or the other channel:
        // not evaluated
        this->sense_dirty_ = true;
        bool selected = this->gpio_->digitalWrite(this->diag_pin_, HIGH);
        selected = this->gpio_->digitalWrite(this->sel_pin_, this->port_ == 0 ? LOW : HIGH) && selected;
        if(!selected)
        {
            this->releaseSense_();
            if(this->sense_select_error_.fail())
            {
                PB_ERROR("%s: sense output could not be switched on (DEN pin %u / DSEL pin %u), not evaluated (%u times)",
                         this->name_.c_str(), this->diag_pin_, this->sel_pin_, this->sense_select_error_.failures());
            }
            return false;
        }
        if(this->sense_select_error_.succeed())
        {
            PB_INFO("%s: sense output switches again", this->name_.c_str());
        }

        // 1ms delay
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Read sense output from adc
        this->mcp_->analogReadStats(this->mcp_ch_, this->mcp_n_, this->mcp_delay_, stats);

        // Disable sense output and put the selection pin back to low
        this->releaseSense_();

        return true;
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::resetOverCurrent(void)
    {
        // Not while a failed over current shutdown keeps the port on (the flag
        // keeps the shutdown retried)
        if(this->init_ == true && (this->enabled_ == false || this->pwr_pin_ == UINT8_MAX))
        {
            this->over_current_ = false;
            this->resetTrip_(true);
        }
    }

    template class BTS7XXX_<MCP3202, 17700>;
    template class BTS7XXX_<MCP3202, 4785>;
    template class BTS7XXX_<MCP3202, 1800>;
    template class BTS7XXX_<MCP3204, 17700>;
    template class BTS7XXX_<MCP3204, 4785>;
    template class BTS7XXX_<MCP3204, 1800>;
} /* namespace PowerBox */