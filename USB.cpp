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

namespace PowerBox
{
    USB::USB(const GPIOManager& gpio, const char* dev)
    {
        this->gpio_ = &gpio;
        this->dev_ = dev;

        this->voltage_mV_ = 0;
        this->current_mA_ = 0;
        this->power_mW_   = 0;

        this->pin_ = -1;
        this->ina_ = nullptr;

        this->enabled_ = false;
        this->max_current_ = 0.9f;
        this->over_current_ = false;
    }

    USB::~USB(void)
    {
        this->pin_ = -1;

        delete this->ina_;
    }

    uint8_t USB::begin(uint8_t pin, uint8_t address, float maxCurrent, uint8_t state)
    {
        this->pin_ = pin;
        this->max_current_ = maxCurrent;

        // Set initial state
        this->enabled_ = state;

        this->gpio_->digitalWrite(this->pin_, state == true ? 1 : 0);

        // Get INA219
        this->ina_ = new INA219(this->dev_, address);

        // Initialize INA219 with best gain
        if(maxCurrent > 3.2)
        {
            return false;
        }
        else if(maxCurrent > 1.6)
        {
            return this->ina_->begin(INA219_CONFIG_GAIN8, maxCurrent);
        }
        else if(maxCurrent > 0.8)
        {
            return this->ina_->begin(INA219_CONFIG_GAIN4, maxCurrent);
        }
        else if(maxCurrent > 0.4)
        {
            return this->ina_->begin(INA219_CONFIG_GAIN2, maxCurrent);
        }
        else
        {
            return this->ina_->begin(INA219_CONFIG_GAIN1, maxCurrent);
        }
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
                    PB_ERROR("USB: failed to enable port (pin %u, %u failures)", this->pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("USB: port (pin %u) switches again", this->pin_);
            }
        }

        // Reset over current state
        this->over_current_ = false;
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
                    PB_ERROR("USB: failed to disable port (pin %u, %u failures)", this->pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("USB: port (pin %u) switches again", this->pin_);
            }
        }

        this->enabled_ = false;

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

    void USB::measureVoltage(void)
    {
        this->voltage_mV_ = this->enabled_ == true ? this->ina_->getBusVoltage_mV() : 0;
    }

    void USB::measureCurrent(void)
    {
        // A previous over current shutdown failed and the port is still on:
        // retry it on every measurement until it succeeds
        if(this->over_current_ == true && this->enabled_ == true)
        {
            if(this->disable())
            {
                this->over_current_ = true;
                this->current_mA_ = 0;
                return;
            }
        }

        uint16_t current = (this->enabled_ == true ? this->ina_->getCurrent_mA() : 0);

        if(current > this->max_current_ * 1e3)
        {
            // Turn off the port. If the flag is already set the port is still on
            // because the shutdown failed; it was retried above this cycle already
            if(this->over_current_ == false)
            {
                this->disable();
            }
            this->over_current_ = true;

            // Report 0 for a port that was shut off, but the real current while a
            // failed shutdown keeps the port on
            this->current_mA_ = this->enabled_ == true ? current : 0;
        }
        else
        {
            this->current_mA_ = current;
        }
    }

    void USB::measurePower(void)
    {
        this->power_mW_ = this->enabled_ == true ? this->ina_->getPower_mW() : 0;
    }

    void USB::resetOverCurrent(void)
    {
        this->over_current_ = false;
    }
} /* namespace PowerBox */