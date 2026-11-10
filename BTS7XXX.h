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

#ifndef BTS7XXX_H
#define BTS7XXX_H

#include "ErrorThrottle.h"
#include <stdint.h>

namespace PowerBox
{
    class GPIOManager;

    template <typename MCP, uint16_t kILIS>
    class BTS7XXX_
    {
        public:
            BTS7XXX_(const GPIOManager& gpio, const MCP& mcp);
            ~BTS7XXX_(void);

            uint8_t begin(uint32_t rsense, uint8_t diag_pin, uint8_t pwr_pin = UINT8_MAX, uint8_t sel_pin = UINT8_MAX, uint8_t port = 0, uint8_t state = 0);
            uint8_t isConnected(void) const { return this->init_; }

            void setChannel(uint8_t ch);
            void setSampling(uint8_t n, uint16_t delay = 0);
            void setMaxCurrent(float maxCurrent);

            bool enable(void);
            bool disable(void);
            bool toggle(void);
            bool setState(uint8_t state);

            void measureCurrent(void);

            uint16_t getCurrent_mA(void) const { return this->current_mA_; }
            uint8_t getState(void) const;
            uint8_t getOverCurrent(void) const;

            void resetOverCurrent(void);

        private:
            uint16_t getCurrent_mA_raw(void);

            // GPIO Manager
            const GPIOManager* gpio_;

            // Current in mA
            uint16_t current_mA_;

            // MCP device
            const MCP* mcp_;

            // ADC channel
            uint8_t mcp_ch_;
            // Number of analog samples
            uint8_t mcp_n_;
            // Delay between samples in ms
            uint16_t mcp_delay_;

            // Initialization flag
            uint8_t init_;

            // Power pin
            uint8_t pwr_pin_;
            // Diagnosis pin
            uint8_t diag_pin_;
            // Port selection pin
            uint8_t sel_pin_;
            // Port
            uint8_t port_;

            // Sense resistor
            uint32_t rsense_;

            // Current state
            uint8_t enabled_;

            // Max current
            float max_current_;

            // Over current flag
            uint8_t over_current_;

            // Rate limits the log for a pin that keeps failing to switch
            ErrorThrottle switch_error_;

            // Fault current
            static constexpr float iis_fault_ = 4.4f;
    };

    template <typename MCP>
    using BTS7006 = BTS7XXX_<MCP, 17700>;
    template <typename MCP>
    using BTS7012 = BTS7XXX_<MCP, 4785>;
    template <typename MCP>
    using BTS7080 = BTS7XXX_<MCP, 1800>;

    /* Abstract base for a BTS power port — allows heterogeneous chip types in a single array */
    template<typename MCP>
    class BTSPort {
    public:
        virtual ~BTSPort() = default;
        virtual uint8_t  begin(uint32_t rsense, uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, uint8_t state) = 0;
        virtual void     setChannel(uint8_t ch) = 0;
        virtual void     setSampling(uint8_t n, uint16_t delay) = 0;
        virtual void     setMaxCurrent(float maxCurrent) = 0;
        virtual bool     setState(uint8_t state) = 0;
        virtual void     measureCurrent() = 0;
        virtual uint16_t getCurrent_mA() const = 0;
        virtual uint8_t  getState() const = 0;
        virtual uint8_t  getOverCurrent() const = 0;
    };

    template<typename MCP, uint16_t kILIS>
    class BTSPortImpl : public BTSPort<MCP> {
        BTS7XXX_<MCP, kILIS> bts_;
    public:
        BTSPortImpl(const GPIOManager& gpio, const MCP& mcp) : bts_(gpio, mcp) {}
        uint8_t  begin(uint32_t rsense, uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, uint8_t state) override
                     { return bts_.begin(rsense, diag_pin, pwr_pin, sel_pin, port, state); }
        void     setChannel(uint8_t ch) override           { bts_.setChannel(ch); }
        void     setSampling(uint8_t n, uint16_t delay) override { bts_.setSampling(n, delay); }
        void     setMaxCurrent(float maxCurrent) override  { bts_.setMaxCurrent(maxCurrent); }
        bool     setState(uint8_t state) override          { return bts_.setState(state); }
        void     measureCurrent() override                 { bts_.measureCurrent(); }
        uint16_t getCurrent_mA() const override            { return bts_.getCurrent_mA(); }
        uint8_t  getState() const override                 { return bts_.getState(); }
        uint8_t  getOverCurrent() const override           { return bts_.getOverCurrent(); }
    };

} /* namespace PowerBox */
#endif // BTS7XXX_H
