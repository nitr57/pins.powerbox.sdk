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

#ifndef DEW_H
#define DEW_H

#include "PWM.h"
#include "GPIOManager.h"
#include "MCP320X.h"
#include "BTS7XXX.h"
#include "DS18B20.h"
#include <cstdint>

namespace PowerBox
{
    template <typename MCP = MCP3204>
    class DewPortT
    {
        public:
            DewPortT(const GPIOManager& gpio, const MCP& mcp, const char* chip, int ch);
            ~DewPortT(void);

            uint8_t begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t probe_pin, uint8_t port, unsigned int freq, bool mode, uint8_t adc_ch = 2);
            void update(float dewPoint, float threshold);

            void enable(void) { this->bts_->enable(); }
            void disable(void) { this->bts_->disable(); }
            void toggle(void) { this->bts_->toggle(); }
            void setState(uint8_t state, uint8_t dc);
            void setAutoMode(uint8_t mode);

            uint16_t getCurrent_mA(void) const { return this->bts_->getCurrent_mA(); }
            uint8_t getState(void) const { return this->bts_->getState(); }
            uint8_t getAutoMode(void) const { return this->auto_mode_; }
            uint8_t getDutyCycle(void) const;
            uint8_t getOverCurrent(void) const { return this->bts_->getOverCurrent(); }
            float getTemperature(void) const { return this->probe_->getTemperature(); }

            void resetOverCurrent(void) { this->bts_->resetOverCurrent(); }

        private:
            void setState_(uint8_t state, uint8_t dutyCycle);

            const GPIOManager* gpio_;
            const MCP* mcp_;

            DS18B20* probe_;
            BTS7080<MCP>* bts_;
            PWM* pwm_;

            bool auto_mode_;
    };

    /* Backward-compatible alias: the historical MCP3204-based dew port */
    using DewPort = DewPortT<MCP3204>;
} /* namespace PowerBox */

#endif /* DEW_H */