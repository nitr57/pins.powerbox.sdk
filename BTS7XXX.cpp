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

        // Diagnosis is disabled
        this->gpio_->digitalWrite(this->diag_pin_, LOW);

        // Selection pin should be low
        this->gpio_->digitalWrite(this->sel_pin_, LOW);

        // Max current
        this->max_current_ = this->iis_fault_ * kILIS * kILIS / this->rsense_;

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
        // Upper limit
        float limit = this->iis_fault_ * kILIS * kILIS / this->rsense_;

        // Limit should stay within reasonable bounds
        limit = std::max(0.0f, std::min(limit, maxCurrent));

        // Convert according to rsense and kilis
        this->max_current_ = limit / kILIS * this->rsense_ * 1e3f;
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
                    PB_ERROR("BTS7XXX: failed to enable port (pin %u, %u failures)", this->pwr_pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("BTS7XXX: port (pin %u) switches again", this->pwr_pin_);
            }

            // Wait 1ms
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Set port enabled
            this->enabled_ = true;

            // Reset over current state
            this->over_current_ = false;
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
                    PB_ERROR("BTS7XXX: failed to disable port (pin %u, %u failures)", this->pwr_pin_, this->switch_error_.failures());
                }
                return false;
            }

            if(this->switch_error_.succeed())
            {
                PB_INFO("BTS7XXX: port (pin %u) switches again", this->pwr_pin_);
            }

            // Set port disabled
            this->enabled_ = false;
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
    void BTS7XXX_<MCP, kILIS>::measureCurrent(void)
    {
        if(this->init_ == true)
        {
            // A previous over current shutdown failed and the port is still on:
            // retry it on every measurement until it succeeds
            if(this->over_current_ == true && this->enabled_ == true && this->pwr_pin_ != UINT8_MAX)
            {
                if(this->disable())
                {
                    this->over_current_ = true;
                    this->current_mA_ = 0;
                    return;
                }
            }

            if(this->enabled_ == true || this->pwr_pin_ == UINT8_MAX)
            {
                // Get current
                float current = this->getCurrent_mA_raw();

                // Scale it with kILIS and sense resistor
                this->current_mA_ = current * kILIS / this->rsense_;

                return;
            }
        }

        this->current_mA_ = 0;
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
    uint16_t BTS7XXX_<MCP, kILIS>::getCurrent_mA_raw(void)
    {
        if(this->mcp_ == nullptr)
        {
            return 0;
        }

        // Enable sense output
        this->gpio_->digitalWrite(this->diag_pin_, HIGH);

        // Enable port
        this->gpio_->digitalWrite(this->sel_pin_, this->port_ == 0 ? LOW : HIGH);

        // 1ms delay
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Read sense output from adc
        uint16_t adc = this->mcp_->analogReadAverage(this->mcp_ch_, this->mcp_n_, this->mcp_delay_);

        // Check for over current. If the flag is already set the port is still on
        // because the shutdown failed; measureCurrent retried it this cycle already
        if(adc >= this->iis_fault_ * kILIS)
        {
            // Serious fault event triggered, disable port and set fault flag
            if(this->over_current_ == false)
            {
                this->disable();
            }
            this->over_current_ = true;
        }
        else if(adc > this->max_current_)
        {
            // Weak current violation
            if(this->over_current_ == false)
            {
                this->disable();
            }
            this->over_current_ = true;
        }

        // Disable sense output
        this->gpio_->digitalWrite(this->diag_pin_, LOW);

        // Put selection pin back to low
        this->gpio_->digitalWrite(this->sel_pin_, LOW);

        // Report 0 for a port that was shut off, but the real current while a
        // failed shutdown keeps the port on
        return (this->over_current_ == true && this->enabled_ == false ? 0 : adc);
    }

    template <typename MCP, uint16_t kILIS>
    void BTS7XXX_<MCP, kILIS>::resetOverCurrent(void)
    {
        if(this->init_ == true)
        {
            this->over_current_ = false;
        }
    }

    template class BTS7XXX_<MCP3202, 17700>;
    template class BTS7XXX_<MCP3202, 4785>;
    template class BTS7XXX_<MCP3202, 1800>;
    template class BTS7XXX_<MCP3204, 17700>;
    template class BTS7XXX_<MCP3204, 4785>;
    template class BTS7XXX_<MCP3204, 1800>;
} /* namespace PowerBox */