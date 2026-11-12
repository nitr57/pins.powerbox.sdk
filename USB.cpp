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

#include "USB.h"
#include "INA219.h"
#include "PowerBoxLogging.h"
#include <cmath>

namespace PowerBox
{
    USB::USB(const GPIOManager& gpio, const char* dev)
    {
        this->gpio_ = &gpio;
        this->dev_ = dev;

        this->voltage_mV_ = 0;
        this->current_mA_ = 0;
        this->valid_ = true;

        this->pin_ = -1;
        this->index_ = -1;
        this->ina_ = nullptr;

        this->enabled_ = false;
        this->max_current_ = 0.9f;
        this->over_current_ = false;
        this->above_count_ = 0;
        this->settling_ = true;
    }

    USB::~USB(void)
    {
        this->pin_ = -1;

        delete this->ina_;
    }

    uint8_t USB::begin(uint8_t pin, uint8_t address, float maxCurrent, uint8_t state, int index)
    {
        this->pin_ = pin;
        this->index_ = index >= 0 ? index : pin;
        this->max_current_ = maxCurrent;

        // Set initial state
        this->enabled_ = state;
        this->above_count_ = 0;
        this->settling_ = true;

        this->gpio_->digitalWrite(this->pin_, state == true ? 1 : 0);

        // The limit must be inside the measuring range (3.2 A), above it only the
        // end of the range trips
        if(!(maxCurrent > 0.f && maxCurrent < 3.2f))
        {
            PB_ERROR("USB port %d: limit %.2f A outside the measuring range (0 - 3.2 A)", this->index_, maxCurrent);
        }

        // Get INA219 (configuration retried by measure() if it fails). The
        // retries of the ports are staggered, so that a hung bus delays only one
        // of them per measurement pass
        this->ina_ = new INA219(this->dev_, address);
        if(this->ina_->begin() != INA219::OK)
        {
            this->next_config_ = std::chrono::steady_clock::now() + CONFIG_RETRY +
                                 CONFIG_RETRY * (this->index_ % 8) / 8;
            return false;
        }

        return true;
    }

    bool USB::enable(void)
    {
        if(this->enabled_ == false)
        {
            // Only track the port as enabled if the pin was actually switched
            if(!this->gpio_->digitalWrite(this->pin_, 1))
            {
                // The write may have reached the pin although it reported an error;
                // drive it low again so the port cannot end up on but unmonitored
                this->gpio_->digitalWrite(this->pin_, 0);

                if(this->switch_error_.fail())
                {
                    PB_ERROR("USB port %d: failed to switch on (pin %u, %u failures)", this->index_, this->pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("USB port %d: switches again", this->index_);
            }

            // Inrush: the first measurement after switching on does not trip immediately
            this->settling_ = true;

            // Reset over current state. Not for a port that is still on: its
            // flag means a failed over current shutdown that is still retried
            this->over_current_ = false;
            this->above_count_ = 0;
        }

        this->enabled_ = true;

        return true;
    }

    bool USB::disable(void)
    {
        if(this->enabled_ == true)
        {
            // If the pin could not be switched the port is still on: keep it marked
            // enabled, so it stays monitored and the next over current check retries
            if(!this->gpio_->digitalWrite(this->pin_, 0))
            {
                if(this->switch_error_.fail())
                {
                    PB_ERROR("USB port %d: failed to switch off (pin %u, %u failures)", this->index_, this->pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("USB port %d: switches again", this->index_);
            }
        }

        this->enabled_ = false;
        this->above_count_ = 0;

        // Reset over current state
        this->over_current_ = false;

        return true;
    }

    void USB::toggle(void)
    {
        if(this->enabled_ == true)
        {
            this->disable();
        }
        else
        {
            this->enable();
        }
    }

    void USB::invalid_(void)
    {
        // Keeps above_count_: a flaky bus must not hide a lasting over current
        this->valid_ = false;
        this->voltage_mV_ = 0;
        this->current_mA_ = 0;
    }

    void USB::count_(BusPass& bus, int res)
    {
        if(res == INA219::TIMEOUT)
        {
            ++bus.timeouts;
        }
        else if(res == INA219::OK)
        {
            bus.answered = true;
        }
    }

    void USB::measure(BusPass& bus)
    {
        // A previous over current shutdown failed and the port is still on:
        // retry it on every measurement until it succeeds
        if(this->over_current_ == true && this->enabled_ == true)
        {
            if(this->disable())
            {
                this->over_current_ = true;
                PB_INFO("USB port %d: switched off (over current)", this->index_);
            }
        }

        // A port that is off is not measured: 0 V, 0 A
        if(this->enabled_ == false)
        {
            this->valid_ = true;
            this->voltage_mV_ = 0;
            this->current_mA_ = 0;
            this->above_count_ = 0;
            return;
        }

        // The bus timed out for other ports in this pass: do not block it further
        if(bus.timeouts >= BUS_TIMEOUT_LIMIT)
        {
            this->invalid_();
            return;
        }

        auto now = std::chrono::steady_clock::now();

        // INA219 not set up yet: retry now and then
        if(!this->ina_->isConfigured())
        {
            if(now >= this->next_config_)
            {
                this->next_config_ = now + CONFIG_RETRY;
                INA219::Result res = this->ina_->begin();
                count_(bus, res);
                if(res == INA219::OK)
                {
                    // Recovery is logged by the INA219 (only after a logged failure)
                    this->settling_ = true;
                    this->above_count_ = 0;
                }
            }

            // Values of the new configuration come with the next measurement
            this->invalid_();
            return;
        }

        // A chip reset (brown-out) loses the averaging, a bus fault reads a wrong
        // configuration: check it on every measurement
        bool otherRange = false;
        {
            bool rewritten = false;
            INA219::Result res = this->ina_->checkConfig(rewritten, otherRange);
            count_(bus, res);
            if(res != INA219::OK)
            {
                // Writing the configuration again failed: set it up again later
                if(!this->ina_->isConfigured())
                {
                    this->next_config_ = now + CONFIG_RETRY;
                }
                this->invalid_();
                return;
            }
            if(rewritten)
            {
                // No immediate trip on this measurement, but keep the count: an
                // overload that resets the chip every time must still trip
                this->settling_ = true;
            }
        }

        int16_t raw = 0;
        uint16_t mV = 0;
        INA219::Result res = this->ina_->readShunt(raw);
        if(res == INA219::OK)
        {
            res = this->ina_->readBus_mV(mV);
        }
        count_(bus, res);
        if(res != INA219::OK)
        {
            this->invalid_();
            return;
        }

        // 0.1 mA per LSB at 0.1 ohm; negative (back feeding) is shown as 0
        float current = raw > 0 ? raw * 0.1f : 0.f;
        float limit = this->max_current_ * 1000.f;
        bool rangeEnd = raw >= INA219::SHUNT_FULL_SCALE;

        // The chip had another shunt range before the rewrite (the power-on default
        // has the same one): a smaller range clamps lower, so this value may be too
        // low, but never too high. Above the limit it counts, below it is unknown
        if(otherRange && !(rangeEnd || current > limit))
        {
            this->invalid_();
            return;
        }

        this->valid_ = true;

        // Count above the limit; an old count (only invalid measurements since)
        // expires
        if(rangeEnd || current > limit)
        {
            if(this->above_count_ > 0 && now - this->last_above_ > ABOVE_EXPIRY)
            {
                this->above_count_ = 0;
            }
            ++this->above_count_;
            this->last_above_ = now;
        }
        else
        {
            this->above_count_ = 0;
        }
        bool immediate = !this->settling_ && (rangeEnd || current >= IMMEDIATE_FACTOR * limit);
        this->settling_ = false;

        if(immediate || this->above_count_ >= TRIP_COUNT)
        {
            // Turn off the port. If the flag is already set the port is still on
            // because the shutdown failed; it was retried above this cycle already
            if(this->over_current_ == false)
            {
                bool off = this->disable();
                PB_ERROR("USB port %d: over current, %.0f mA%s, limit %.0f mA, %s",
                         this->index_, current, rangeEnd ? " or more (end of the measuring range)" : "", limit,
                         off ? "switched off" : "switch-off failed, retrying");
            }
            this->over_current_ = true;
            this->above_count_ = 0;

            // Report 0 for a port that was shut off, but the real values while a
            // failed shutdown keeps the port on
            if(this->enabled_ == false)
            {
                this->voltage_mV_ = 0;
                this->current_mA_ = 0;
                return;
            }
        }

        this->voltage_mV_ = mV;
        this->current_mA_ = (uint16_t)lroundf(current);
    }

    void USB::resetOverCurrent(void)
    {
        this->over_current_ = false;
    }
} /* namespace PowerBox */
