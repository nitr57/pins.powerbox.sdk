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

#include "ADJ.h"
#include "common.h"
#include <algorithm>
#include <cmath>

namespace PowerBox
{
    BuckPort::BuckPort(const GPIOManager& gpio, const MCP3204& mcp, const char* dev, uint8_t address)
    {
        this->gpio_ = &gpio;
        this->dev_ = dev;
        this->address_ = address;
        this->adc_ = &mcp;

        this->voltage_adc_ = 0;
        this->voltage_max_ = VMAX_EFF;
        this->voltage_target_ = VMIN;
        this->voltage_supply_ = 0;
        this->over_voltage_ = false;
        this->boot_target_invalid_ = false;

        this->dac_ = new MCP4725(dev, address, this->SUPPLY_REF);
        this->bts_ = new BTS7080<MCP3204>(gpio, mcp);
    }

    BuckPort::~BuckPort(void)
    {
        delete this->bts_;
        delete this->dac_;
    }

    uint8_t BuckPort::begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, float target, uint8_t state, int keptEn)
    {
        // A converter that is still running (taken over) keeps its enable line,
        // otherwise the enable pin is driven low and the converter is off
        bool running = keptEn == 1;
        this->bts_->begin(1200, diag_pin, pwr_pin, sel_pin, port, running ? 1 : 0);
        this->bts_->setMaxCurrent(3.0f);
        this->bts_->setChannel(2);
        this->bts_->setSampling(10, 1);

        this->voltage_supply_ = this->adc_->analogReadAverage(1, 5, 0) * (36e3 + 4.7e3) / 4.7e3;
        this->updateMaxVoltage_();

        // Take the stored target first, so it is kept (and persisted again)
        // even if the DAC cannot be reached or the supply is too low right now
        int bootTarget = (target >= 0.f && target <= 100.f) ? (int)lroundf(target * 1000.f) : 0;

        // Older versions accepted targets up to VMAX. Above VMAX_EFF the DAC is
        // clamped to 0 V, so such a target really produced VMAX_EFF: keep that.
        if (bootTarget > VMAX_EFF && bootTarget <= VMAX)
        {
            PB_INFO("Buck: stored target voltage %.3f V limited to %.3f V", bootTarget * 0.001f, VMAX_EFF * 0.001f);
            bootTarget = VMAX_EFF;
        }

        this->boot_target_invalid_ = !this->inRange_(bootTarget);
        if (!this->boot_target_invalid_)
        {
            this->voltage_target_ = bootTarget;
        }
        else
        {
            // E.g. 0 V stored by an older version, or a rejected target that
            // was persisted. Never switch on with a voltage nobody asked for.
            if (running)
            {
                PB_ERROR("Buck: stored target voltage %.2f V is invalid, the running converter's DAC value is checked", target);
            }
            else
            {
                PB_ERROR("Buck: stored target voltage %.2f V is invalid, using %.2f V%s",
                         target, this->voltage_target_ * 0.001f, state ? ", the buck stays off" : "");
            }
            this->over_voltage_ = true;
        }

        if (!this->dac_->begin())
        {
            // Without a working DAC the output voltage is unknown, stay off
            if (running)
            {
                PB_ERROR("Buck: DAC not readable, running converter switched off");
            }
            this->bts_->setState(0);
            return false;
        }

        if (running)
        {
            bool ok = this->takeOverRunning_();
            if (!ok && this->boot_target_invalid_)
            {
                // Switched off; setState(0) cleared the flag, the stored target was still rejected
                this->over_voltage_ = true;
            }
            return ok;
        }

        // Taken over as off: the boot state does not apply
        if (keptEn == 0)
        {
            state = 0;
        }

        // At power-on the DAC loads its EEPROM value. Keep the lowest output
        // voltage there, so a converter that is enabled before the SDK has
        // set the DAC never gets a previously used (higher) voltage.
        // Only writes the EEPROM if it holds a different value.
        if (!this->dac_->setPowerOnVoltage(SUPPLY_REF))
        {
            PB_ERROR("Buck: failed to set DAC power-on value");
        }

        // Off first: parks the DAC (on a warm restart it still holds the old value)
        bool ok = this->setState(0, 0.f);

        if (this->boot_target_invalid_)
        {
            // setState(0) cleared the flag, the stored target was still rejected
            this->over_voltage_ = true;
            return false;
        }

        if (state)
        {
            ok = this->setState(1, this->voltage_target_ * 0.001f);
        }

        return ok;
    }

    bool BuckPort::setState(uint8_t state, float targetVoltage)
    {
        // Negated check, so NaN is invalid as well
        bool valid = (targetVoltage >= 0.f && targetVoltage <= 100.f);
        int target = valid ? (int)lroundf(targetVoltage * 1000.f) : this->voltage_target_;

        // 0 V means off, the last valid target is kept
        if (valid && target == 0)
        {
            state = 0;
            target = this->voltage_target_;
        }

        if (!state)
        {
            // Off is never blocked, not even by an invalid target.
            // setState(0) is a no-op if the port is already off, so an over
            // current flag is kept (disable() would always clear it).
            // Note: after a failed over current shutdown the port is still on;
            // if switching it off here succeeds, disable() clears the over
            // current flag.
            if (!this->bts_->setState(0))
            {
                // Port still on: drop the output to the minimum as emergency
                // measure, the target is not taken over
                PB_ERROR("Buck: failed to switch off");
                this->parkDac_();
                return false;
            }

            // Park the DAC at the lowest output voltage, so a later enable
            // (or a glitch on the enable line) cannot apply an old voltage
            if (!this->parkDac_())
            {
                PB_ERROR("Buck: failed to park DAC");
            }

            // Remember the target for the next switch on. Only the absolute
            // range is checked here, the supply limit applies when switching on.
            if (!valid || !this->inRange_(target))
            {
                PB_ERROR("Buck: target voltage %.3f V is out of range (%.2f - %.2f V)", targetVoltage, VMIN * 0.001f, VMAX_EFF * 0.001f);
                this->over_voltage_ = true;
                return false;
            }

            this->voltage_target_ = target;
            this->over_voltage_ = false;
            return true;
        }

        // On: reject the request before touching the hardware
        if (!valid)
        {
            PB_ERROR("Buck: invalid target voltage %f", targetVoltage);
            this->over_voltage_ = true;
            return false;
        }

        if (!this->checkVoltage_(target))
        {
            return false;
        }

        // A failed over current shutdown leaves the port on with the over
        // current flag set (the listener retries the shutdown). Do not set a
        // new voltage on it and report success.
        if (this->bts_->getState() && this->bts_->getOverCurrent())
        {
            PB_ERROR("Buck: over current shutdown pending, request rejected");
            return false;
        }

        // DAC first, then switch on. If the DAC does not hold the target,
        // the output voltage would differ from the requested one.
        // setState(1) is a no-op if the port is already on.
        if (!this->setVoltage_(target) || !this->bts_->setState(1))
        {
            // The output voltage is not the requested one: switch off, even if
            // the port was on before. Note: if the switch off succeeds and an
            // over current flag was set, disable() clears it.
            this->bts_->setState(0);
            this->parkDac_();
            return false;
        }

        this->voltage_target_ = target;
        this->over_voltage_ = false;
        return true;
    }

    void BuckPort::setSupplyVoltage(float voltage)
    {
        this->voltage_supply_ = voltage * 1000;
        this->updateMaxVoltage_();
    }

    void BuckPort::measureCurrent(void)
    {
        uint8_t wasOn = this->bts_->getState();

        this->bts_->measureCurrent();

        // Over current shutdown: keep "off => DAC parked"
        if (wasOn && !this->bts_->getState())
        {
            this->parkDac_();
        }
    }

    float BuckPort::getVoltage(void) const
    {
        return this->adc_->analogReadAverage(3, 10, 1) * (SUPPLY_R1 + SUPPLY_R2) / SUPPLY_R2 * 0.001f;
    }

    void BuckPort::updateMaxVoltage_(void)
    {
        // We only support up to 95% of supply voltage
        // e.g. on 12.0V supply, we can have 11.4V at most
        this->voltage_max_ = std::min((int)(this->voltage_supply_ * 0.95f), VMAX_EFF);
    }

    bool BuckPort::inRange_(int voltage) const
    {
        return voltage >= VMIN && voltage <= VMAX_EFF;
    }

    bool BuckPort::checkVoltage_(int voltage)
    {
        // We need to check whether supply voltage is still able to power
        // the requested output voltage
        this->updateMaxVoltage_();

        if(voltage > this->voltage_max_ || voltage < VMIN)
        {
            this->over_voltage_ = true;

            PB_ERROR("Target voltage %d is out of range (%d - %d)", voltage, VMIN, this->voltage_max_);

            return false;
        }

        return true;
    }

    float BuckPort::dacVoltageFor_(int target)
    {
        // Required DAC voltage. ADJ_CAL is a measured calibration of the output
        // voltage (see ADJ.h).
        return std::min(SUPPLY_REF, ADJ_R2 * std::max(0.f, VMAX - target * ADJ_CAL) / ADJ_R1 * 0.001f);
    }

    bool BuckPort::setVoltage_(int target)
    {
        // Set DAC accordingly, verified by reading it back
        return this->dac_->setVoltage(dacVoltageFor_(target));
    }

    bool BuckPort::takeOverRunning_(void)
    {
        // The converter was left running by an earlier process. Keep the voltage
        // its DAC holds: no DAC write (the stored target may be stale), no EEPROM
        // write (would set the DAC to the minimum for a moment; done at the next
        // cold start). Switch off if the DAC value is not plausible.
        uint16_t code = 0;
        bool poweredDown = true;
        if (!this->dac_->readDac(code, poweredDown) || poweredDown)
        {
            PB_ERROR("Buck: DAC of the running converter not readable or powered down, switched off");
            this->setState(0, 0.f);
            return false;
        }

        // Target the DAC value stands for (inverse of dacVoltageFor_)
        float vdac_mV = code * SUPPLY_REF / 4095.f * 1000.f;
        int derived = (int)lroundf((VMAX - vdac_mV * ADJ_R1 / ADJ_R2) / ADJ_CAL);

        if (!this->inRange_(derived))
        {
            // E.g. the parked value (enable line high but DAC at the minimum)
            PB_ERROR("Buck: running converter has an implausible DAC value (%u, %d mV), switched off", code, derived);
            this->setState(0, 0.f);
            return false;
        }

        uint16_t storedCode = (uint16_t)lroundf(dacVoltageFor_(this->voltage_target_) * 4095.f / SUPPLY_REF);
        if (this->boot_target_invalid_ || code != storedCode)
        {
            if (this->boot_target_invalid_)
            {
                PB_INFO("Buck: running converter taken over at %.3f V (DAC value), stored target was invalid",
                        derived * 0.001f);
            }
            else
            {
                PB_INFO("Buck: running converter taken over at %.3f V (DAC value), stored target was %.3f V",
                        derived * 0.001f, this->voltage_target_ * 0.001f);
            }
            this->voltage_target_ = derived;
            this->boot_target_invalid_ = false;
        }

        this->over_voltage_ = false;
        return true;
    }

    bool BuckPort::parkDac_(void)
    {
        // Full scale DAC voltage gives the lowest output voltage
        return this->dac_->setVoltage(SUPPLY_REF);
    }

    template <typename MCP, uint16_t kILIS>
    PWMPortT<MCP, kILIS>::PWMPortT(const GPIOManager& gpio, const MCP& mcp, const char* chip, int ch)
    {
        this->gpio_ = &gpio;
        this->mcp_ = &mcp;

        this->bts_ = new BTS7XXX_<MCP, kILIS>(gpio, mcp);
        this->pwm_ = new PWM(chip, ch);
        this->taken_over_ = false;
        this->pwm_ok_ = false;
        this->period_ = 0;
        this->fast_retries_ = 0;
    }

    template <typename MCP, uint16_t kILIS>
    PWMPortT<MCP, kILIS>::~PWMPortT(void)
    {
        delete this->bts_;
        delete this->pwm_;
    }

    template <typename MCP, uint16_t kILIS>
    uint8_t PWMPortT<MCP, kILIS>::begin(uint8_t diag_pin, uint8_t pwr_pin, uint8_t sel_pin, uint8_t port, unsigned int freq, uint8_t adc_ch, float maxCurrent, int keptIn)
    {
        // Take the running port over only if its PWM channel is configured as
        // expected, otherwise the duty cycle read back would be meaningless
        unsigned int period = (unsigned int)(1e9 / freq);
        this->taken_over_ = false;
        this->pwm_ok_ = false;
        this->period_ = period;
        // A channel that is not exported cannot be taken over (and must not wait here)
        if(keptIn >= 0 && this->pwm_->exists() && this->pwm_->setExport())
        {
            this->taken_over_ = this->pwm_->getPeriod() == period;
            if(!this->taken_over_)
            {
                PB_INFO("PWM port: PWM not configured as expected, initialized off");
            }
        }

        this->bts_->begin(1200, diag_pin, pwr_pin, sel_pin, port, this->taken_over_ ? keptIn : 0);
        this->bts_->setMaxCurrent(maxCurrent);
        this->bts_->setChannel(adc_ch);
        this->bts_->setSampling(100, 50);

        if(this->taken_over_)
        {
            this->pwm_ok_ = true;
            return true;
        }

        this->pwm_ok_ = this->initPwm_(true);
        if(!this->pwm_ok_)
        {
            PB_ERROR("PWM port: PWM could not be set up, the port stays off (retried on the next switch request: every second for a while if the channel is exported, else at most every 30 s)");
            // Exported but not writable yet (udev slow at boot): retry soon
            this->fast_retries_ = 0;
            this->next_retry_ = std::chrono::steady_clock::now() +
                (this->pwm_->exists() ? std::chrono::seconds(1) : std::chrono::seconds(30));
        }

        return this->pwm_ok_;
    }

    template <typename MCP, uint16_t kILIS>
    bool PWMPortT<MCP, kILIS>::initPwm_(bool wait)
    {
        if(!this->pwm_->setExport(wait))
        {
            return false;
        }

        // Best effort first (the switch is already off): disable a channel that may
        // still run, and duty 0, because a duty above the new period would be
        // rejected. Right after an export the period may still be 0, then these
        // writes fail.
        this->pwm_->setState(0);
        this->pwm_->setDutyCycle(0);

        return this->pwm_->setPeriod(this->period_) &&
               this->pwm_->setDutyCycle(0) &&
               this->pwm_->setState(0);
    }

    template <typename MCP, uint16_t kILIS>
    bool PWMPortT<MCP, kILIS>::retryPwm_(void)
    {
        auto now = std::chrono::steady_clock::now();
        if(now < this->next_retry_)
        {
            return false;
        }
        this->next_retry_ = now + std::chrono::seconds(30);

        // Only with the switch off (it could not be switched off otherwise)
        if(this->bts_->getState() != 0)
        {
            return false;
        }

        if(!this->initPwm_(false))
        {
            // Exported, but udev has not made it writable yet: try again soon,
            // a limited number of times (a failed try logs no error)
            if(this->pwm_->exists() && this->fast_retries_ < FAST_RETRIES)
            {
                ++this->fast_retries_;
                this->next_retry_ = now + std::chrono::seconds(1);
            }
            return false;
        }

        PB_INFO("PWM port: PWM set up after a retry");
        this->pwm_ok_ = true;
        return true;
    }

    template <typename MCP, uint16_t kILIS>
    void PWMPortT<MCP, kILIS>::setState(uint8_t state, uint8_t dc)
    {
        // Without a working PWM the switch must not be turned on. The PWM setup is
        // retried now and then, with the switch off.
        if(!this->pwm_ok_)
        {
            this->bts_->setState(0);
            if(!this->retryPwm_())
            {
                return;
            }
        }

        // Off never leaves the channel running (like the dew port)
        if(!state)
        {
            dc = 0;
        }

        // Convert duty cycle to nano seconds
        unsigned int period = this->pwm_->getPeriod();
        unsigned int dutyCycle = (unsigned int)(((uint64_t)period * dc) / 255);

        // Like the dew port: the PWM channel only runs with a duty above 0, the
        // switch follows the requested state. "On with 0" stays logically on
        // (output 0), so a following power change takes effect (e.g. NINA sets
        // the flat panel light on, then the brightness).
        this->pwm_->setDutyCycle(dutyCycle);
        this->pwm_->setState(dutyCycle != 0);
        this->bts_->setState(state);
    }

    template <typename MCP, uint16_t kILIS>
    uint8_t PWMPortT<MCP, kILIS>::getDutyCycle(void) const
    {
        // A channel that is not set up says nothing about the port
        if(!this->pwm_ok_)
        {
            return 0;
        }

        unsigned int period = this->pwm_->getPeriod();
        unsigned int dutyCycle_ns = this->pwm_->getDutyCycle();

        if (period == 0) {
            return 0;
        }

        // Convert nanoseconds back to 0-255 scale. Rounded: setting truncates
        // (period * dc / 255), so truncating here as well would read back dc - 1
        uint64_t value = ((uint64_t)dutyCycle_ns * 255 + period / 2) / period;
        return (uint8_t)std::min<uint64_t>(value, 255);
    }

    /* Explicit instantiations for the supported (ADC, BTS) combinations */
    template class PWMPortT<MCP3204, 1800>;  // PinsBox: BTS7080 PWM port
    template class PWMPortT<MCP3202, 4785>;  // PinsBoxMini: BTS7012 PWM port
} /* namespace PowerBox */