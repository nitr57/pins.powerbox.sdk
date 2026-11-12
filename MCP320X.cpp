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

#include "MCP320X.h"
#include "common.h"
#include "SPIBus.h"
#include "GPIOManager.h"
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <algorithm>

namespace PowerBox
{
    template <uint8_t NCH, uint8_t BIT>
    MCP320X_<NCH, BIT>::MCP320X_(const GPIOManager& gpio)
    {
        this->gpio_ = &gpio;
        this->init_ = false;
        this->v_ref_ = 0.0f;

        this->spi_ = new SPIBus("/dev/spidev0.0");
    }

    template <uint8_t NCH, uint8_t BIT>
    MCP320X_<NCH, BIT>::~MCP320X_(void)
    {
        if(this->spi_) {
            delete this->spi_;
            this->spi_ = nullptr;
        }
        this->init_ = false;
        this->v_ref_ = 0.0f;
    }

    template <uint8_t NCH, uint8_t BIT>
    void MCP320X_<NCH, BIT>::begin(float vRef)
    {
        if (!this->spi_ || !this->spi_->isConnected()) {
            std::cerr << "Error: SPI bus not available, MCP320X cannot initialize" << std::endl;
            this->init_ = false;
            return;
        }

        this->v_ref_ = vRef;
        this->spi_->begin(1000000, 8, 0);
        this->init_ = true;
    }

    template <uint8_t NCH, uint8_t BIT>
    uint8_t MCP320X_<NCH, BIT>::isConnected(void) const
    {
        return this->init_;
    }

    template <uint8_t NCH, uint8_t BIT>
    uint16_t MCP320X_<NCH, BIT>::analogRead(uint8_t ch) const
    {
        if (!this->init_ || !this->spi_ || !this->spi_->isConnected()) {
            std::cerr << "Warning: Attempting to read from uninitialized MCP320X" << std::endl;
            return 0;
        }

        std::vector<uint8_t> tx;
        std::vector<uint8_t> rx;

        // Build the SPI command (matches Python daemon)
        tx.push_back(NCH == 2 ? 0x01 : 0x06);
        tx.push_back(NCH == 2 ? (ch == 0 ? 0xA0 : 0xE0) : (ch << 6));
        tx.push_back(0x00);

        // Perform single SPI transaction with all 3 bytes
        this->spi_->transfer(tx, rx);

        // Extract the 12-bit result from bytes 1 and 2
        if (rx.size() >= 3) {
            uint8_t msb = rx[1];
            uint8_t lsb = rx[2];
            return ((msb << 8) | lsb) & 0xFFF;
        }

        return 0;
    }

    template <uint8_t NCH, uint8_t BIT>
    uint16_t MCP320X_<NCH, BIT>::analogReadAverage(uint8_t ch, uint8_t n, uint16_t delay_us) const
    {
        if(n == 0)
        {
            return 0;
        }

        float adc = 0.f;

        for(uint8_t i = 0; i < n; ++i)
        {
            adc += this->analogRead(ch);
            std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
        }

        // Divide by n to average the samples
        adc /= n;

        // Obtain the analog value
        adc = adc * this->v_ref_ * 1e3f / (1 << BIT);

        // Convert it back to the analog value using digital range
        // and reference voltage
        return adc;
    }

    template <uint8_t NCH, uint8_t BIT>
    void MCP320X_<NCH, BIT>::analogReadStats(uint8_t ch, uint8_t n, uint16_t delay_us, SenseStats& stats) const
    {
        stats = SenseStats();
        if(n == 0)
        {
            return;
        }

        uint16_t raw[255];
        uint32_t sum = 0;
        uint16_t lo = 0xFFFF;
        uint16_t hi = 0;
        for(uint8_t i = 0; i < n; ++i)
        {
            raw[i] = this->analogRead(ch);
            sum += raw[i];
            lo = std::min(lo, raw[i]);
            hi = std::max(hi, raw[i]);
            if(delay_us > 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
            }
        }

        const uint16_t full = (1 << BIT) - 1;
        const uint16_t satLevel = (uint16_t)(full * 95 / 100);
        const float mV = this->v_ref_ * 1e3f / (1 << BIT);

        stats.n = n;
        stats.mean_mV = (float)sum / n * mV;
        stats.min_mV = lo * mV;
        stats.max_mV = hi * mV;

        // Upper cluster (on-phase of a chopped current), only with a clear spread
        // above the noise. Its median, so that a single spike does not set it.
        uint16_t upper[255];
        int nUpper = 0;
        if(hi - lo >= SPREAD_MIN)
        {
            uint16_t split = (uint16_t)((lo + hi) / 2);
            for(uint8_t i = 0; i < n; ++i)
            {
                if(raw[i] > split)
                {
                    upper[nUpper++] = raw[i];
                }
            }
        }
        if(nUpper > 0)
        {
            std::nth_element(upper, upper + nUpper / 2, upper + nUpper);
            stats.on_mV = upper[nUpper / 2] * mV;
            stats.on_count = nUpper;
        }

        for(uint8_t i = 0; i < n; ++i)
        {
            if(raw[i] >= satLevel)
            {
                ++stats.saturated;
            }
        }
    }

    template class MCP320X_<2, 12>;
    template class MCP320X_<4, 12>;
} /* namespace PowerBox */