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
#include "MCP320X.h"
#include <stdint.h>
#include <string>
#include <chrono>

namespace PowerBox
{
    class GPIOManager;

    /* How the load current of a port flows (for the over current check):
     * STATIC   - DC load (12 V ports, supply, buck input)
     * ON_PHASE - chopped by a low-side switch at a low PWM frequency (dew,
     *            400 Hz): the sense follows the current, the on-phase level of
     *            the samples is the current while on
     * PER_DUTY - chopped at a high PWM frequency (PWM port, 30 kHz): the
     *            current while on is estimated as mean / duty (duty at least
     *            PER_DUTY_MIN), averaged over PER_DUTY_TAU */
    enum class SenseMode
    {
        STATIC,
        ON_PHASE,
        PER_DUTY
    };

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
            /* Name for log messages, e.g. "12V port 3" */
            void setName(const std::string& name) { this->name_ = name; }
            /* Sense of another channel that adds to this one's (PinsBox Mini
             * supply DEN on DSEL), as its mean in mV: subtracted before the
             * evaluation. coupling: this port's own current in that other sense
             * relative to its own sense (supply contains the port current:
             * kILIS_port / kILIS_supply); used for the on-phase level */
            void setSenseOffset(float offset_mV, float coupling = 0.f);

            bool enable(void);
            bool disable(void);
            bool toggle(void);
            bool setState(uint8_t state);

            /* Over current (switchable port): at once at the end of the
             * measuring range when (almost) all samples are saturated (BTS fault)
             * or when the current while on is >= IMMEDIATE_FACTOR x the limit;
             * otherwise above the limit (or saturated) in TRIP_COUNT measurements
             * in a row. Never at once in the first measurement after switching
             * on. duty: applied duty (0..1) of a low-side PWM of the port. A
             * port without a switch (sense only) reports the real current and
             * flags an over current per measurement. */
            void measureCurrent(float duty = 1.f, SenseMode mode = SenseMode::STATIC);

            uint16_t getCurrent_mA(void) const { return this->current_mA_; }
            uint8_t getState(void) const;
            uint8_t getOverCurrent(void) const;

            void resetOverCurrent(void);

        private:
            bool sample_(SenseStats& stats);
            bool releaseSense_(void);
            void resetTrip_(bool counts);
            float fullScale_mA_(void) const;

            static constexpr float IMMEDIATE_FACTOR = 1.5f;
            static constexpr int TRIP_COUNT = 2;
            static constexpr float FAULT_SHARE = 0.9f;        // share of saturated samples: BTS fault
            static constexpr int SATURATED_MIN = 2;           // saturated samples: above the range
            static constexpr int ON_SAMPLES_MIN = 3;          // on-phase samples needed (ON_PHASE)
            static constexpr float PER_DUTY_MIN = 0.2f;       // duty floor of the PER_DUTY estimate
            static constexpr float PER_DUTY_TAU = 5.f;        // s, averaging of the PER_DUTY estimate
            static constexpr float IDLE_LIMIT_mV = 100.f;     // sense line with the own DEN low
            static constexpr int IDLE_WAIT_US = 100;          // decay of the previous measurement
            static constexpr int IDLE_RECHECK_MS = 1;         // second look before rejecting
            static constexpr std::chrono::seconds ABOVE_EXPIRY{10}; // count of a port switched off and on

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
            // Delay between samples in us
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

            // Log name
            std::string name_;

            // Offset of another sense on the line (mV) and its coupling
            float offset_mV_;
            float coupling_;

            // Trip state: measurements above the limit / with saturated samples
            // in a row, first measurement after switching on
            int above_count_;
            int saturated_count_;
            std::chrono::steady_clock::time_point last_above_;
            bool settling_;

            // The last switch-off of the sense output (DEN / DSEL low) failed: the
            // sense may still drive the shared line, retried on every measurement
            bool sense_dirty_;

            // PER_DUTY: averaged estimate of the current while on
            float per_duty_mA_;
            bool per_duty_valid_;
            std::chrono::steady_clock::time_point per_duty_time_;

            // Sense only: over the limit (log on the edge)
            ErrorThrottle sense_only_log_{std::chrono::seconds(600)};
            // Sense output could not be switched on (DEN / DSEL write failed)
            ErrorThrottle sense_select_error_;
            // Own sense output could not be switched off
            ErrorThrottle sense_release_error_;
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
        virtual void     setName(const std::string& name) = 0;
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
        void     setName(const std::string& name) override { bts_.setName(name); }
        bool     setState(uint8_t state) override          { return bts_.setState(state); }
        void     measureCurrent() override                 { bts_.measureCurrent(); }
        uint16_t getCurrent_mA() const override            { return bts_.getCurrent_mA(); }
        uint8_t  getState() const override                 { return bts_.getState(); }
        uint8_t  getOverCurrent() const override           { return bts_.getOverCurrent(); }
    };

} /* namespace PowerBox */
#endif // BTS7XXX_H
