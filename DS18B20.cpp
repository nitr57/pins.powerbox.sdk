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

#define SKIP_ROM        0xCC
#define CONVERT_T       0x44
#define READ_SCRATCHPAD 0xBE

namespace PowerBox
{
    DS18B20::DS18B20(const GPIOManager& gpio)
    {
        this->gpio_ = &gpio;
        this->pin_ = UINT8_MAX;
        this->current_temperature_ = DEVICE_DISCONNECTED_C;
        this->last_temperature_ = DEVICE_DISCONNECTED_C;
        this->last_valid_ = false;
    }

    DS18B20::~DS18B20(void)
    {
        this->pin_ = UINT8_MAX;
        this->current_temperature_ = DEVICE_DISCONNECTED_C;
    }

    bool DS18B20::begin(uint8_t pin)
    {
        this->pin_ = pin;
        this->current_temperature_ = DEVICE_DISCONNECTED_C;

        return true;
    }

    void DS18B20::measureTemperature(void)
    {
        if(!this->reset_())
        {
            this->current_temperature_ = DEVICE_DISCONNECTED_C;
            return;
        }

        this->write_(SKIP_ROM);
        this->write_(CONVERT_T);
    }

    float DS18B20::getTemperature(void)
    {
        const float ERROR_TEMP = DEVICE_DISCONNECTED_C;
        uint8_t data[9];

        for(uint8_t retry = 0; retry < 5; ++retry)
        {
            if(!this->reset_())
            {
                this->current_temperature_ = ERROR_TEMP;
                continue;
            }

            this->write_(SKIP_ROM);
            this->write_(READ_SCRATCHPAD);

            for(uint8_t i = 0; i < 9; ++i)
            {
                data[i] = this->read_();
            }

            // Verify CRC
            if(this->crc_(data) != data[8])
            {
                continue;
            }

            // Combine bytes
            int16_t raw_temp = (data[1] << 8) | data[0];

            // Convert to Celsius
            this->current_temperature_ = (float)raw_temp * 0.0625f;

            this->last_temperature_ = this->current_temperature_;
            this->last_valid_ = true;
            this->last_time_ = std::chrono::steady_clock::now();

            return this->current_temperature_;
        }

        // Bit-banged 1-Wire reads fail now and then, keep the last good value for a while
        if(this->last_valid_ && std::chrono::steady_clock::now() - this->last_time_ < SENSOR_GRACE_PERIOD)
        {
            return this->last_temperature_;
        }

        return ERROR_TEMP;
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

    uint8_t DS18B20::crc_(uint8_t* data)
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