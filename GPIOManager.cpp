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

#include "GPIOManager.h"
#include "common.h"
#include "PowerBoxLogging.h"

namespace PowerBox
{
    GPIOManager::GPIOManager(const char* gpiochip, const char* i2c, uint8_t addr, int ngpio_in, const uint8_t* gpio_in, int ngpio_out, const uint8_t* gpio_out)
    {
        PB_DEBUG("GPIOManager constructor starting with i2c=%s, addr=0x%02x", i2c, addr);
        this->init_ = false;
        this->gpiochip_ = gpiochip;
        this->req_ = nullptr;
        this->request_ = nullptr;
        this->settings_ = nullptr;
        this->line_ = nullptr;
        this->chip_ = nullptr;

        if(addr != -1 && i2c != nullptr)
        {
            this->tca_ = new TCA9535(i2c, addr);
        }
        else
        {
            this->tca_ = nullptr;
        }

        // Count and allocate GPIO arrays
        this->cm5_ngpio_ = 0;
        this->tca_ngpio_ = 0;
        for(int i = 0; i < ngpio_out; ++i)
        {
            if(gpio_out[i] < 40)
            {
                ++this->cm5_ngpio_;
            }
            else
            {
                ++this->tca_ngpio_;
            }
        }

        this->cm5_gpio_ = (this->cm5_ngpio_ > 0) ? new unsigned int[this->cm5_ngpio_] : nullptr;
        this->tca_gpio_ = (this->tca_ngpio_ > 0) ? new unsigned int[this->tca_ngpio_] : nullptr;
        int cm5_idx = 0, tca_idx = 0;
        for(int i = 0; i < ngpio_out; ++i)
        {
            if(gpio_out[i] < 40)
            {
                this->cm5_gpio_[cm5_idx++] = gpio_out[i];
            }
            else
            {
                this->tca_gpio_[tca_idx++] = gpio_out[i] - 40;
            }
        }
    }

    GPIOManager::~GPIOManager(void)
    {
        PB_DEBUG("GPIOManager destructor starting");

        // Free libgpiod resources
        if(this->request_) {
            gpiod_line_request_release(this->request_);
            this->request_ = nullptr;
        }
        if(this->req_) {
            gpiod_request_config_free(this->req_);
            this->req_ = nullptr;
        }
        if(this->line_) {
            gpiod_line_config_free(this->line_);
            this->line_ = nullptr;
        }
        if(this->settings_) {
            gpiod_line_settings_free(this->settings_);
            this->settings_ = nullptr;
        }
        if(this->chip_) {
            gpiod_chip_close(this->chip_);
            this->chip_ = nullptr;
        }

        // Free TCA953X object
        if(this->tca_) {
            delete this->tca_;
            this->tca_ = nullptr;
        }

        // Free allocated arrays
        if(this->cm5_gpio_) {
            delete[] this->cm5_gpio_;
            this->cm5_gpio_ = nullptr;
            this->cm5_ngpio_ = 0;
        }
        if(this->tca_gpio_) {
            delete[] this->tca_gpio_;
            this->tca_gpio_ = nullptr;
            this->tca_ngpio_ = 0;
        }

        this->init_ = false;
    }

    void GPIOManager::clear(void)
    {
    }

    bool GPIOManager::begin(void)
    {
        if(this->init_)
        {
            return true;
        }

        PB_DEBUG("GPIOManager::begin() starting");
        
        // Initialize libgpiod
        if(!this->InitGpiod_()) {
            PB_DEBUG("GPIOManager::begin() - InitGpiod_ failed");
            return false;
        }

        PB_DEBUG("GPIOManager::begin() - InitGpiod_ succeeded, calling tca_->begin()");

        // Initialize TCA9535
        if(this->tca_ != nullptr)
        {
            if(!this->tca_->begin()) {
                PB_DEBUG("GPIOManager::begin() - tca_->begin() failed");
                return false;
            }
        }

        PB_DEBUG("GPIOManager::begin() - tca_->begin() succeeded");

        // Set output pins. Keep going after a failure, so one failing pin
        // does not leave all following pins unconfigured
        bool pinsOk = true;
        for(int i = 0; i < this->tca_ngpio_; ++i)
        {
            if(!this->tca_->pinMode(this->tca_gpio_[i], OUTPUT)) {
                PB_ERROR("GPIOManager::begin() - pinMode OUTPUT failed for TCA pin %u", this->tca_gpio_[i]);
                pinsOk = false;
            }
        }

        if(!pinsOk) {
            return false;
        }

        PB_DEBUG("GPIOManager::begin() completed successfully");
        this->init_ = true;
        return true;
    }

    uint8_t GPIOManager::digitalWrite(uint8_t pin, uint8_t data) const
    {
        if(pin < 40)
        {
            if(!this->request_) {
                PB_DEBUG("GPIOManager::digitalWrite(%d) - request_ is null, operation failed", pin);
                return 0;
            }
            
            unsigned int gpio = pin;
            enum gpiod_line_value value = data == HIGH ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;

            return gpiod_line_request_set_values_subset(this->request_, 1, &gpio, &value) == 0;
        }
        else
        {
            return this->tca_->digitalWrite(pin - 40, data);
        }
    }

    uint8_t GPIOManager::digitalRead(uint8_t pin) const
    {
        if(pin < 40)
        {
            unsigned int gpio = pin;
            enum gpiod_line_value value;

            if(this->request_ && gpiod_line_request_get_values_subset(this->request_, 1, &gpio, &value) == 0)
            {
                return value == GPIOD_LINE_VALUE_ACTIVE ? HIGH : LOW;
            }

            return LOW;
        }
        else
        {
            return this->tca_->digitalRead(pin - 40);
        }
    }

    bool GPIOManager::InitGpiod_(void)
    {
        this->chip_ = gpiod_chip_open(this->gpiochip_);
        if (!this->chip_)
        {
            PB_ERROR("GPIOManager::InitGpiod_ - gpiod_chip_open failed for %s", this->gpiochip_);
            return false;
        }

        this->settings_ = gpiod_line_settings_new();
        if (!this->settings_) {
            PB_DEBUG("GPIOManager::InitGpiod_ - gpiod_line_settings_new failed");
            gpiod_chip_close(this->chip_);
            this->chip_ = nullptr;
            return false;
        }

        // Configure GPIO settings
        gpiod_line_settings_set_direction(this->settings_, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(this->settings_, GPIOD_LINE_VALUE_ACTIVE);

        this->line_ = gpiod_line_config_new();
        if (!this->line_) {
            PB_DEBUG("GPIOManager::InitGpiod_ - gpiod_line_config_new failed");
            gpiod_line_settings_free(this->settings_);
            this->settings_ = nullptr;
            gpiod_chip_close(this->chip_);
            this->chip_ = nullptr;
            return false;
        }

        if(this->cm5_ngpio_ > 0 && gpiod_line_config_add_line_settings(this->line_, this->cm5_gpio_, this->cm5_ngpio_, this->settings_)) {
            PB_DEBUG("GPIOManager::InitGpiod_ - gpiod_line_config_add_line_settings failed");
            gpiod_line_config_free(this->line_);
            this->line_ = nullptr;
            gpiod_line_settings_free(this->settings_);
            this->settings_ = nullptr;
            gpiod_chip_close(this->chip_);
            this->chip_ = nullptr;
            return false;
        }

        this->req_ = gpiod_request_config_new();
        if (!this->req_) {
            PB_DEBUG("GPIOManager::InitGpiod_ - gpiod_request_config_new failed");
            gpiod_line_config_free(this->line_);
            this->line_ = nullptr;
            gpiod_line_settings_free(this->settings_);
            this->settings_ = nullptr;
            gpiod_chip_close(this->chip_);
            this->chip_ = nullptr;
            return false;
        }

        gpiod_request_config_set_consumer(this->req_, "GPIOManager");

        // Request GPIO lines if there are any
        this->request_ = gpiod_chip_request_lines(this->chip_, this->req_, this->line_);
        if (!this->request_) {
            PB_ERROR("GPIOManager::InitGpiod_ - gpiod_chip_request_lines failed for %s (lines may be claimed by another process)", this->gpiochip_);
            gpiod_request_config_free(this->req_);
            this->req_ = nullptr;
            gpiod_line_config_free(this->line_);
            this->line_ = nullptr;
            gpiod_line_settings_free(this->settings_);
            this->settings_ = nullptr;
            gpiod_chip_close(this->chip_);
            this->chip_ = nullptr;
            return false;
        }

        return true;
    }
} /* namespace PowerBox */