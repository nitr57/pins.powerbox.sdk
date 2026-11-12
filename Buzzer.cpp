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


#include "Buzzer.h"
#include "PowerBoxLogging.h"
#include <algorithm>
#include <system_error>

namespace PowerBox
{
    Buzzer::Buzzer(const char* chip, int ch)
        : pwm_(chip, ch), name_(std::string(chip) + "/pwm" + std::to_string(ch))
    {
    }

    Buzzer::~Buzzer(void)
    {
        {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->quit_ = true;
        }
        this->cv_.notify_all();
        if(this->worker_.joinable())
        {
            this->worker_.join();
        }

        std::lock_guard<std::mutex> lock(this->mutex_);
        if(this->pwm_.exists() && !this->off_())
        {
            PB_ERROR("Buzzer %s: could not be switched off", this->name_.c_str());
        }
    }

    void Buzzer::begin(void)
    {
        std::lock_guard<std::mutex> lock(this->mutex_);
        if(this->pwm_.setExport() && !this->off_())
        {
            PB_ERROR("Buzzer %s: could not be switched off", this->name_.c_str());
        }
    }

    bool Buzzer::start_(unsigned int duty_ns)
    {
        // The duty cycle must never exceed the period: set it to 0 before the
        // period changes (the channel may come with another period). A freshly
        // exported channel has period 0 and duty 0; older kernels (rpi-6.6)
        // reject a duty or enable write while the period is 0, so the duty is
        // only written there if it is not 0 already.
        if(this->pwm_.getPeriod() != PERIOD_NS)
        {
            if(this->pwm_.getDutyCycle() != 0 && !this->pwm_.setDutyCycle(0))
            {
                return false;
            }
            if(!this->pwm_.setPeriod(PERIOD_NS))
            {
                return false;
            }
        }
        return this->pwm_.setDutyCycle(duty_ns) && this->pwm_.setState(true);
    }

    bool Buzzer::off_(void)
    {
        // Duty 0 also silences it, if the channel cannot be disabled
        if(this->pwm_.setState(false) || this->pwm_.setDutyCycle(0))
        {
            return true;
        }

        // A fresh channel (period 0) rejects both writes on older kernels, but
        // cannot sound: an enabled channel never has period 0. Only a period
        // read as 0, not one that could not be read.
        unsigned int period = 1;
        return this->pwm_.readPeriod(period) && period == 0;
    }

    PB_ERROR_TYPE Buzzer::beep(int volume, int duration_ms)
    {
        volume = std::min(std::max(volume, 0), 100);

        std::lock_guard<std::mutex> lock(this->mutex_);
        auto now = std::chrono::steady_clock::now();

        // Volume 0: no tone, a running one ends now (any duration); also one
        // left on (a switch-off at begin() that failed)
        if(volume == 0)
        {
            if(this->on_)
            {
                this->stopAt_ = now;
                this->cv_.notify_all();
            }
            else if(this->pwm_.getState() == 1 && !this->off_())
            {
                return PB_ERROR_COMMUNICATION;
            }
            return PB_SUCCESS;
        }

        if(duration_ms <= 0)
        {
            return PB_ERROR_INVALID_PARAMETER;
        }
        duration_ms = std::min(duration_ms, MAX_DURATION_MS);

        // The channel may not have been there at begin() (retried, without
        // waiting: a beep must not block)
        if(!this->pwm_.exists() && !this->pwm_.setExport(false))
        {
            return PB_ERROR_COMMUNICATION;
        }

        // The worker ends every tone: no tone without it
        if(!this->worker_.joinable())
        {
            try
            {
                this->worker_ = std::thread(&Buzzer::run_, this);
            }
            catch(const std::system_error&)
            {
                return PB_ERROR_COMMUNICATION;
            }
        }

        // Duty = volume up to 50 %: a piezo is loudest at 50 % and gets quieter
        // above (100 % is DC, silent), so 50..100 are all the loudest
        unsigned int duty_ns = (unsigned int)((unsigned long long)PERIOD_NS * std::min(volume, 50) / 100);
        bool wasOn = this->on_ || this->pwm_.getState() == 1;
        if(!this->start_(duty_ns))
        {
            // The channel may sound now (a running tone, or a write that got
            // through): if it cannot be switched off, the worker retries. One
            // that never sounded does not need that (and must not report a
            // failed switch-off).
            bool maySound = wasOn || this->pwm_.getState() == 1;
            if(this->off_() || !maySound)
            {
                this->on_ = false;
            }
            else
            {
                this->on_ = true;
                this->stopAt_ = now;
                this->cv_.notify_all();
            }
            return PB_ERROR_COMMUNICATION;
        }

        this->on_ = true;
        this->stopAt_ = now + std::chrono::milliseconds(duration_ms);
        this->cv_.notify_all();
        return PB_SUCCESS;
    }

    void Buzzer::run_(void)
    {
        std::unique_lock<std::mutex> lock(this->mutex_);
        while(!this->quit_)
        {
            if(!this->on_)
            {
                this->cv_.wait(lock);
                continue;
            }

            if(std::chrono::steady_clock::now() < this->stopAt_)
            {
                this->cv_.wait_until(lock, this->stopAt_);
                continue;
            }

            if(this->off_())
            {
                this->on_ = false;
                if(this->offError_.succeed())
                {
                    PB_INFO("Buzzer %s: switched off again", this->name_.c_str());
                }
            }
            else
            {
                if(this->offError_.fail())
                {
                    PB_ERROR("Buzzer %s: could not be switched off, retrying", this->name_.c_str());
                }
                this->stopAt_ = std::chrono::steady_clock::now() + OFF_RETRY;
            }
        }
    }
} /* namespace PowerBox */
