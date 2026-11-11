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

#include "DS18B20.h"
#include "common.h"
#include <thread>
#include <chrono>
#include <cmath>

#define SKIP_ROM        0xCC
#define CONVERT_T       0x44
#define READ_SCRATCHPAD 0xBE

namespace PowerBox
{
    DS18B20::DS18B20(const GPIOManager& gpio)
    {
        this->gpio_ = &gpio;
        this->pin_ = UINT8_MAX;
        this->converting_ = false;
        this->last_temperature_ = DEVICE_DISCONNECTED_C;
        this->last_valid_ = false;
        this->jump_temperature_ = DEVICE_DISCONNECTED_C;
        this->jump_count_ = 0;
    }

    DS18B20::~DS18B20(void)
    {
        this->pin_ = UINT8_MAX;
    }

    bool DS18B20::begin(uint8_t pin)
    {
        this->pin_ = pin;
        this->converting_ = false;
        this->last_valid_ = false;
        this->jump_count_ = 0;

        return true;
    }

    void DS18B20::update(void)
    {
        // The scratchpad is read once per conversion: a second read would
        // return the same content and could only repeat a wrong value
        uint8_t data[9];
        float temperature;
        if(this->converting_ && this->readScratchpad_(data) && this->plausible_(data, temperature))
        {
            this->last_temperature_ = temperature;
            this->last_valid_ = true;
            this->last_time_ = std::chrono::steady_clock::now();
        }

        // Start the next conversion, read by the next call
        this->converting_ = false;
        if(this->reset_())
        {
            this->write_(SKIP_ROM);
            this->write_(CONVERT_T);
            this->converting_ = true;
        }
    }

    float DS18B20::getTemperature(void) const
    {
        // Bit-banged 1-Wire reads fail now and then, keep the last good value for a while
        if(this->last_valid_ && std::chrono::steady_clock::now() - this->last_time_ < SENSOR_GRACE_PERIOD)
        {
            return this->last_temperature_;
        }

        return DEVICE_DISCONNECTED_C;
    }

    bool DS18B20::readScratchpad_(uint8_t* data)
    {
        // Retried only on transfer errors (no presence pulse, CRC)
        for(uint8_t retry = 0; retry < 3; ++retry)
        {
            if(!this->reset_())
            {
                continue;
            }

            this->write_(SKIP_ROM);
            this->write_(READ_SCRATCHPAD);

            for(uint8_t i = 0; i < 9; ++i)
            {
                data[i] = this->read_();
            }

            if(this->crc_(data) == data[8])
            {
                return true;
            }
        }

        return false;
    }

    bool DS18B20::plausible_(const uint8_t* data, float& temperature)
    {
        // A line held low reads as all zero bytes, and their CRC is 0 as well.
        // The configuration register (byte 4) has bit 7 = 0 and bits 0..4 = 1,
        // on the DS18B20 and on its common clones (the reserved bytes 5 and 7
        // differ on clones and are not checked).
        if((data[4] & 0x9F) != 0x1F)
        {
            PB_DEBUG("DS18B20 (pin %u): invalid configuration register 0x%02X, value ignored", this->pin_, data[4]);
            this->jump_count_ = 0;
            return false;
        }

        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        temperature = (float)raw * 0.0625f;

        if(temperature < -55.f || temperature > 125.f)
        {
            PB_DEBUG("DS18B20 (pin %u): %.2f C out of range, value ignored", this->pin_, temperature);
            this->jump_count_ = 0;
            return false;
        }

        // Reference: the last plausible value, while it is still used
        bool ref = this->last_valid_ &&
                   std::chrono::steady_clock::now() - this->last_time_ < SENSOR_GRACE_PERIOD;
        float delta = ref ? std::fabs(temperature - this->last_temperature_) : 0.f;

        // 85 C is the power-on value of the scratchpad (sensor plugged in or
        // reset after the conversion was started): only taken as a real value
        // close to the last one. This rule comes before the jump rule, so a
        // repeated 85 C is never confirmed as a jump.
        if(raw == 0x0550 && !(ref && delta <= JUMP_LIMIT_K))
        {
            PB_DEBUG("DS18B20 (pin %u): power-on value 85 C ignored", this->pin_);
            this->jump_count_ = 0;
            return false;
        }

        // A large jump is a read error, unless it is confirmed by the next reads
        // (e.g. a probe plugged into another place). Without a reference a single
        // read could be such an error too (it would drive the regulation for the
        // whole grace period), so it needs a matching second read.
        if(!ref || delta > JUMP_LIMIT_K)
        {
            int confirm = ref ? JUMP_CONFIRM : NOREF_CONFIRM;

            if(this->jump_count_ > 0 && std::fabs(temperature - this->jump_temperature_) <= JUMP_MATCH_K)
            {
                ++this->jump_count_;
            }
            else
            {
                this->jump_count_ = 1;
            }
            this->jump_temperature_ = temperature;

            if(this->jump_count_ < confirm)
            {
                if(ref)
                {
                    PB_DEBUG("DS18B20 (pin %u): jump from %.2f C to %.2f C, not taken over yet", this->pin_, this->last_temperature_, temperature);
                }
                return false;
            }
        }

        this->jump_count_ = 0;
        return true;
    }

    bool DS18B20::reset_(void)
    {
        this->gpio_->digitalWrite(this->pin_, HIGH);
        this->delay_us_(10);
        this->gpio_->digitalWrite(this->pin_, LOW);
        this->delay_us_(480);
        this->gpio_->digitalWrite(this->pin_, HIGH);
        this->delay_us_(60);
        uint8_t state = this->gpio_->digitalRead(this->pin_);
        this->delay_us_(420);

        return state == LOW;
    }

    void DS18B20::write_(uint8_t byte)
    {
        for(uint8_t i = 0; i < 8; ++i)
        {
            this->gpio_->digitalWrite(this->pin_, LOW);

            if((byte & (1 << i)) != 0)
            {
                this->delay_us_(1);
                this->gpio_->digitalWrite(this->pin_, HIGH);
                std::this_thread::sleep_for(std::chrono::microseconds(60));
            }
            else
            {
                this->delay_us_(60);
                this->gpio_->digitalWrite(this->pin_, HIGH);
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
            this->delay_us_(60);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    uint8_t DS18B20::read_(void)
    {
        uint8_t byte = 0;
        uint8_t mask = 1;
        for(uint8_t i = 0; i < 8; ++i)
        {
            this->gpio_->digitalWrite(this->pin_, LOW);
            this->delay_us_(1);
            this->gpio_->digitalWrite(this->pin_, HIGH);
            this->delay_us_(2);
            if(this->gpio_->digitalRead(this->pin_) == HIGH)
            {
                byte |= (1 << i);
            }
            this->delay_us_(60);            
        }

        return byte;
    }

    uint8_t DS18B20::crc_(const uint8_t* data)
    {
        uint8_t crc = 0;
        for(uint8_t i = 0; i < 8; ++i)
        {
            uint8_t byte = data[i];
            for(uint8_t j = 0; j < 8; ++j)
            {
                crc = (crc ^ byte) & 1 ? (crc >> 1) ^ 0x8C : crc >> 1;
                byte >>= 1;
            }
        }
        return crc;
    }

    void DS18B20::delay_us_(int delay)
    {
        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::microseconds(delay);

        // If the delay is long, give the CPU a tiny break first
        if (delay > 100) {
            std::this_thread::sleep_for(std::chrono::microseconds(delay - 50));
        }

        // Finish with a high-precision busy-wait loop
        while (std::chrono::steady_clock::now() - start < timeout) {
            // "Pause" hint tells the CPU this is a spin-loop
            #if defined(__aarch64__)
                asm volatile("yield" ::: "memory"); 
            #else
                asm volatile("rep; nop" ::: "memory");
            #endif
        }
    }
} /* namespace PowerBox */