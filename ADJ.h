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

#ifndef ADJ_H
#define ADJ_H

#include "PWM.h"
#include "GPIOManager.h"
#include "MCP320X.h"
#include "MCP4725.h"
#include "BTS7XXX.h"
#include <cstdint>

namespace PowerBox
{
    class BuckPort
    {
        public:
            BuckPort(const GPIOManager& gpio, const MCP3204& mcp, const char* dev, uint8_t address);
            ~BuckPort(void);

            /* keptEn: level of the enable line taken over from an earlier process
             * (-1 = none, use the boot state). 0: stays off. 1: the converter keeps
             * running with the voltage its DAC holds (no DAC/EEPROM write); it is
             * switched off if that value cannot be read or is not plausible. */
            uint8_t begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, float target, uint8_t state, int keptEn = -1);

            bool disable(void) { return this->setState(0, 0.f); }

            /* Switches the port and sets the target voltage [V] in one step.
             * - The port is only switched on once the DAC verifiably holds the
             *   value for the target. A valid target (0..100 V) that rounds to
             *   0 mV switches off and keeps the last target.
             * - Switching off is never blocked by the target; an invalid target
             *   (NaN, outside VMIN..VMAX_EFF) is then not stored and false is
             *   returned, even though the port is off. If switching off fails,
             *   the target is not taken over, the DAC is parked if reachable
             *   (lowest output, about 0.9 V, below VMIN) and false is returned.
             *   A failed DAC park after a successful switch off is only logged
             *   (the result depends on the target check alone).
             * - Requests that switch on or keep the port on: a rejected target
             *   (NaN, out of range, above the supply limit) leaves the port
             *   untouched; they are also rejected while a failed over current
             *   shutdown is pending (port on, over current set). If the DAC
             *   cannot be set or the switch does not turn on, the port is
             *   switched off, even if it was on before; if that switch off
             *   fails too, the port stays on, with the DAC parked if reachable,
             *   otherwise with an undefined output voltage.
             * - On success the target is stored and true is returned. */
            bool setState(uint8_t state, float targetVoltage);
            void setSupplyVoltage(float voltage);

            void measureCurrent(void);

            uint16_t getCurrent_mA(void) const { return this->bts_->getCurrent_mA(); }
            uint8_t getState(void) const { return this->bts_->getState(); }
            float getVoltage(void) const;
            float getTargetVoltage(void) const { return this->voltage_target_ * 0.001f; }
            float getMinVoltage(void) const { return this->VMIN * 0.001f; }
            float getMaxVoltage(void) const { return this->voltage_max_ * 0.001f; }
            uint8_t getOverCurrent(void) const { return this->bts_->getOverCurrent(); }
            /* Set by range/validity rejections (and by begin() for an invalid
             * stored target), cleared on success. DAC or switch errors, the
             * pending over current rejection and a failed switch off leave it
             * unchanged. */
            uint8_t getOverVoltage(void) const { return this->over_voltage_; }
            /* begin() found an invalid stored target and did not switch on */
            bool getBootTargetInvalid(void) const { return this->boot_target_invalid_; }

            void resetOverCurrent(void) { this->bts_->resetOverCurrent(); }

        private:
            void updateMaxVoltage_(void);
            bool inRange_(int voltage) const;
            bool checkVoltage_(int voltage);
            bool setVoltage_(int target);
            bool parkDac_(void);
            bool takeOverRunning_(void);
            static float dacVoltageFor_(int target);

            const GPIOManager* gpio_;
            const MCP3204* adc_;
            MCP4725* dac_;
            const char* dev_;
            uint8_t address_;

            BTS7080<MCP3204>* bts_;

            int voltage_adc_;
            int voltage_max_;
            int voltage_target_;
            int voltage_supply_;
            bool over_voltage_;
            bool boot_target_invalid_;

            const BTS7006<MCP3204>* supply_;

            static constexpr float ADJ_R1 = 82e3f;
            static constexpr float ADJ_R2 = 24.3e3f;
            static constexpr float ADJ_R3 = 5.23e3f;
            static constexpr float ADJ_REF = 0.6f;
            static constexpr float SUPPLY_R1 = 35.7e3f;
            static constexpr float SUPPLY_R2 = 10e3f;
            static constexpr float SUPPLY_REF = 3.3f;

            /* Measured calibration: the DAC value is calculated for a nominal
             * output of target * ADJ_CAL, which compensates the measured
             * deviation of the real output voltage */
            static constexpr float ADJ_CAL = 1.015f;

            static constexpr int VMAX = 1e3 * (ADJ_REF + ADJ_REF * (ADJ_R1 / ADJ_R3 + ADJ_R1 / ADJ_R2));
            static constexpr int VMIN = std::max(1000, (int)(VMAX - ADJ_R1 * SUPPLY_REF / ADJ_R2 * 1e3));
            /* Highest target the output can follow: above it the DAC would be
             * clamped to 0 V and the output stays at VMAX */
            static constexpr int VMAX_EFF = (int)(VMAX / ADJ_CAL);
    };

    template <typename MCP = MCP3204, uint16_t kILIS = 1800>
    class PWMPortT
    {
        public:
            PWMPortT(const GPIOManager& gpio, const MCP& mcp, const char* chip = "pwmchip0", int ch = 2);
            ~PWMPortT(void);

            /* keptIn: level of the power switch input taken over from an earlier
             * process (-1 = none). If >= 0 and the PWM channel already runs with
             * the requested period, the port is kept as it is (see isTakenOver()). */
            uint8_t begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, unsigned int freq, uint8_t adc_ch = 2, float maxCurrent = 3.0f, int keptIn = -1);
            bool isTakenOver(void) const { return this->taken_over_; }
            /* PWM channel set up; without it the switch is never turned on */
            bool isPwmOk(void) const { return this->pwm_ok_; }

            void enable(void) { this->bts_->enable(); }
            void disable(void) { this->bts_->disable(); }
            void toggle(void) { this->bts_->toggle(); }
            void setState(uint8_t state, uint8_t dc);

            void measureCurrent(void) { this->bts_->measureCurrent(); }

            uint16_t getCurrent_mA(void) const { return this->bts_->getCurrent_mA(); }
            uint8_t getState(void) const { return this->bts_->getState(); }
            uint8_t getDutyCycle(void) const;
            uint8_t getOverCurrent(void) const { return this->bts_->getOverCurrent(); }

            void resetOverCurrent(void) { this->bts_->resetOverCurrent(); }

        private:
            const GPIOManager* gpio_;
            const MCP* mcp_;

            BTS7XXX_<MCP, kILIS>* bts_;
            PWM* pwm_;
            bool taken_over_;
            bool pwm_ok_;
    };

    /* Backward-compatible alias: the historical MCP3204 / BTS7080 PWM port */
    using PWMPort = PWMPortT<MCP3204, 1800>;
} /* namespace PowerBox */

#endif /* ADJ_H */