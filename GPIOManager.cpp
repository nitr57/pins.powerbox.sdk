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
#include <unistd.h>
#include <vector>

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
        this->cm5KeptMask_ = 0;
        this->cm5KeptVal_ = 0;
        this->tcaKeptMask_ = 0;
        this->tcaKeptVal_ = 0;

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

    static bool containsPin(const uint8_t* pins, int n, uint8_t pin)
    {
        for(int i = 0; pins && i < n; ++i)
        {
            if(pins[i] == pin)
            {
                return true;
            }
        }
        return false;
    }

    bool GPIOManager::begin(bool keepState, const uint8_t* keepPins, int nKeepPins, const uint8_t* idleHigh, int nIdleHigh)
    {
        if(this->init_)
        {
            return true;
        }

        this->cm5KeptMask_ = 0;
        this->cm5KeptVal_ = 0;
        this->tcaKeptMask_ = 0;
        this->tcaKeptVal_ = 0;

        // CM5 lines first: a second process fails here before it touches the TCA
        if(!this->InitGpiodKeep_(keepState, keepPins, nKeepPins, idleHigh, nIdleHigh))
        {
            return false;
        }

        if(this->tca_ != nullptr)
        {
            if(!this->tca_->begin())
            {
                PB_ERROR("GPIOManager::begin() - TCA953X could not be opened");
                return false;
            }

            if(!this->InitTcaKeep_(keepState, keepPins, nKeepPins))
            {
                return false;
            }
        }

        this->init_ = true;
        return true;
    }

    int GPIOManager::keptValue(uint8_t pin) const
    {
        if(pin < 40)
        {
            uint64_t bit = (uint64_t)1 << pin;
            return (this->cm5KeptMask_ & bit) ? ((this->cm5KeptVal_ & bit) ? 1 : 0) : -1;
        }

        uint8_t tpin = pin - 40;
        if(tpin >= 16)
        {
            return -1;
        }

        uint16_t bit = (uint16_t)(1u << tpin);
        return (this->tcaKeptMask_ & bit) ? ((this->tcaKeptVal_ & bit) ? 1 : 0) : -1;
    }

    bool GPIOManager::InitTcaKeep_(bool keepState, const uint8_t* keepPins, int nKeepPins)
    {
        // Read config and output latch, a few more attempts than a single access
        uint16_t config = 0, outputs = 0;
        bool readOk = false;
        for(int attempt = 0; attempt < 5 && !readOk; ++attempt)
        {
            readOk = this->tca_->readConfig(config) && this->tca_->readOutputs(outputs);
            if(!readOk)
            {
                usleep(2000);
            }
        }

        if(!readOk)
        {
            // Cannot know what is running: leave the TCA untouched, nothing is
            // taken over (the caller applies its boot states, its writes are
            // read-modify-write and fail the same way while the bus is down)
            PB_ERROR("GPIOManager::begin() - TCA953X state could not be read, port states are not taken over");
            return false;
        }

        uint16_t ownMask = 0, keepMask = 0;
        for(int i = 0; i < this->tca_ngpio_; ++i)
        {
            if(this->tca_gpio_[i] < 16)
            {
                uint16_t bit = (uint16_t)(1u << this->tca_gpio_[i]);
                ownMask |= bit;
                if(containsPin(keepPins, nKeepPins, (uint8_t)(this->tca_gpio_[i] + 40)))
                {
                    keepMask |= bit;
                }
            }
        }

        // A switch pin that is already an output keeps its latch (no pulse, e.g. a
        // TCA that kept its state over a reboot). All other own pins start low
        // (switch pins that are still inputs after power on or a TCA reset, and
        // sense enables). Foreign bits stay as they are.
        uint16_t wasOutput = (uint16_t)(~config) & keepMask;
        uint16_t newOutputs = (outputs & ~ownMask) | (outputs & wasOutput);

        // Latch first, then switch the pins to output
        if(newOutputs != outputs && !this->tca_->writeOutputs(newOutputs))
        {
            PB_ERROR("GPIOManager::begin() - TCA953X output latch could not be written");
            return false;
        }

        uint16_t newConfig = config & (uint16_t)~ownMask;
        if(newConfig != config && !this->tca_->writeConfig(newConfig))
        {
            PB_ERROR("GPIOManager::begin() - TCA953X configuration could not be written");
            return false;
        }

        if(keepState)
        {
            this->tcaKeptMask_ = wasOutput;
            this->tcaKeptVal_ = newOutputs & wasOutput;
        }

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

    bool GPIOManager::InitGpiodKeep_(bool keepState, const uint8_t* keepPins, int nKeepPins, const uint8_t* idleHigh, int nIdleHigh)
    {
        this->chip_ = gpiod_chip_open(this->gpiochip_);
        if (!this->chip_)
        {
            PB_ERROR("GPIOManager::InitGpiodKeep_ - gpiod_chip_open failed for %s", this->gpiochip_);
            return false;
        }

        this->settings_ = gpiod_line_settings_new();
        this->line_ = gpiod_line_config_new();
        this->req_ = gpiod_request_config_new();
        if (!this->settings_ || !this->line_ || !this->req_)
        {
            PB_ERROR("GPIOManager::InitGpiodKeep_ - libgpiod allocation failed");
            return false;  // freed by the destructor
        }

        gpiod_request_config_set_consumer(this->req_, "GPIOManager");

        if (this->cm5_ngpio_ <= 0)
        {
            return true;
        }

        // Step 1: request the lines without changing them (no hardware access on RP1)
        gpiod_line_settings_set_direction(this->settings_, GPIOD_LINE_DIRECTION_AS_IS);
        if (gpiod_line_config_add_line_settings(this->line_, this->cm5_gpio_, this->cm5_ngpio_, this->settings_))
        {
            PB_ERROR("GPIOManager::InitGpiodKeep_ - gpiod_line_config_add_line_settings failed");
            return false;
        }

        this->request_ = gpiod_chip_request_lines(this->chip_, this->req_, this->line_);
        if (!this->request_)
        {
            PB_ERROR("GPIOManager::InitGpiodKeep_ - gpiod_chip_request_lines failed for %s (lines may be claimed by another process or an overlay)", this->gpiochip_);
            return false;
        }

        const int n = this->cm5_ngpio_;
        auto fail = [this](const char* what) {
            // State unknown: do not drive anything. Releasing the request leaves
            // the lines as they are (pinctrl-rp1 persist_gpio_outputs, default on)
            PB_ERROR("GPIOManager::InitGpiodKeep_ - %s, GPIO lines are not configured", what);
            gpiod_line_request_release(this->request_);
            this->request_ = nullptr;
            this->cm5KeptMask_ = 0;
            this->cm5KeptVal_ = 0;
            return false;
        };

        // Step 2: direction of every line (line info works for the own request)
        std::vector<char> isOutput(n, 0), isKeep(n, 0);
        bool needValues = false;
        for (int i = 0; i < n; ++i)
        {
            isKeep[i] = containsPin(keepPins, nKeepPins, (uint8_t)this->cm5_gpio_[i]);

            struct gpiod_line_info* info = gpiod_chip_get_line_info(this->chip_, this->cm5_gpio_[i]);
            if (!info)
            {
                if (isKeep[i])
                {
                    return fail("direction of a switch line not readable");
                }
                continue;  // other lines get their idle level anyway
            }
            isOutput[i] = gpiod_line_info_get_direction(info) == GPIOD_LINE_DIRECTION_OUTPUT;
            gpiod_line_info_free(info);

            if (isKeep[i] && isOutput[i])
            {
                needValues = true;
            }
        }

        // Step 3: levels, only needed for switch lines that already are outputs
        std::vector<enum gpiod_line_value> values(n, GPIOD_LINE_VALUE_INACTIVE);
        if (needValues)
        {
            bool valuesOk = false;
            for (int attempt = 0; attempt < 5 && !valuesOk; ++attempt)
            {
                valuesOk = gpiod_line_request_get_values_subset(this->request_, n, this->cm5_gpio_, values.data()) == 0;
                if (!valuesOk)
                {
                    usleep(2000);
                }
            }
            if (!valuesOk)
            {
                return fail("current line levels not readable");
            }
        }

        // Step 4: start value per line
        for (int i = 0; i < n; ++i)
        {
            uint8_t pin = (uint8_t)this->cm5_gpio_[i];

            if (isKeep[i] && isOutput[i])
            {
                // Already driven output (earlier process, firmware): keep its level
                bool high = values[i] == GPIOD_LINE_VALUE_ACTIVE;
                if (keepState)
                {
                    this->cm5KeptMask_ |= (uint64_t)1 << pin;
                    if (high)
                    {
                        this->cm5KeptVal_ |= (uint64_t)1 << pin;
                    }
                }
                values[i] = high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
            }
            else
            {
                values[i] = containsPin(idleHigh, nIdleHigh, pin) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
            }
        }

        // Step 5: switch all lines to output with these start values
        gpiod_line_config_reset(this->line_);
        gpiod_line_settings_set_direction(this->settings_, GPIOD_LINE_DIRECTION_OUTPUT);
        bool ok = gpiod_line_config_add_line_settings(this->line_, this->cm5_gpio_, n, this->settings_) == 0 &&
                  gpiod_line_config_set_output_values(this->line_, values.data(), n) == 0 &&
                  gpiod_line_request_reconfigure_lines(this->request_, this->line_) == 0;

        if (!ok)
        {
            return fail("lines could not be configured as outputs");
        }

        return true;
    }
} /* namespace PowerBox */