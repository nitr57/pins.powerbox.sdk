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

#ifndef PINS_BOX_CUSTOM_DEVICE_H
#define PINS_BOX_CUSTOM_DEVICE_H

#include "Device.h"
#include "GPIOManager.h"
#include "PWM.h"
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace PowerBox
{
    /** Configuration for one PWM output port.
     *
     *  The pin is driven via the Linux sysfs PWM subsystem (not raw GPIO),
     *  so "gpio" is informational only — the kernel overlay must already map
     *  that BCM pin to a PWM channel before the device is opened.
     */
    struct CustomPWMPortCfg
    {
        uint8_t      gpio;        ///< BCM GPIO number (informational)
        unsigned int freq_hz;    ///< Carrier frequency [Hz]
        std::string  pwm_chip;   ///< sysfs chip name, e.g. "pwmchip0"
        int          pwm_channel;///< PWM channel index within the chip
    };

    /** Configuration for one simple 12 V power output port. */
    struct CustomPowerPortCfg
    {
        uint8_t gpio;             ///< BCM GPIO number (output, active-high)
    };

    /** Full hardware description parsed from /etc/pinsCustomDevice. */
    struct PinsBoxCustomHWConfig
    {
        std::string                     name;
        std::vector<CustomPWMPortCfg>   pwm_ports;   ///< Exposed as DEW ports (per-port PWM control)
        std::vector<CustomPowerPortCfg> power_ports; ///< Exposed as POWER ports (on/off GPIO)
    };

    /**
     * PinsBoxCustomDevice — a fully user-configurable GPIO power box.
     *
     * Hardware layout is read from /etc/pinsCustomDevice (JSON) at scan time.
     * - power_ports  → mapped to the SDK's power port API (on/off via GPIO)
     * - pwm_ports    → mapped to the SDK's dew-heater port API so callers get
     *                  per-port on/off and 8-bit duty-cycle control at a
     *                  user-specified carrier frequency (e.g. 50 Hz for servos)
     */
    class PinsBoxCustomDevice : public Device
    {
        public:
            explicit PinsBoxCustomDevice(const PinsBoxCustomHWConfig& hw);
            virtual ~PinsBoxCustomDevice(void);

            virtual PB_ERROR_TYPE Open(void);
            virtual void Close(void);

            virtual std::string GetSerial(void);
            virtual int         GetUpTime(void);
            virtual float       GetCoreTemp(void);

            /* No environment sensors on a custom box */
            virtual float GetTemperature(void) { return -127.f; }
            virtual float GetHumidity(void)    { return -127.f; }
            virtual float GetDewPoint(void)    { return -127.f; }
            virtual int   GetExtSensor(void)   { return 0; }

            /* No supply monitoring without dedicated ADC */
            virtual float GetSupply12V(void)  { return 0.f; }
            virtual float GetSupply5V(void)   { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12A(void)  { return 0.f; }
            virtual float GetSupply12Ah(void) { return 0.f; }
            virtual float GetSupply12Wh(void) { return 0.f; }

            /* No environment configuration */
            virtual float GetTemperatureOffset()          { return 0.f; }
            virtual bool  SetTemperatureOffset(float)     { return true; }
            virtual bool  SetExtTemperature(float)        { return true; }
            virtual float GetHumidityOffset()             { return 0.f; }
            virtual bool  SetHumidityOffset(float)        { return true; }
            virtual bool  SetExtHumidity(float)           { return true; }
            virtual int   GetEnvUpdateRate()              { return 1; }
            virtual bool  SetEnvUpdateRate(int)           { return true; }
            virtual int   GetUpdateRate()                 { return 1; }
            virtual bool  SetUpdateRate(int)              { return true; }

            /* Port counts come from the parsed config */
            virtual int GetNumPowerPorts(void) const;
            virtual int GetNumUSBPorts(void)   const { return 0; }
            virtual int GetNumDewPorts(void)   const;
            virtual int GetNumBuckPorts(void)  const { return 0; }
            virtual int GetNumPWMPorts(void)   const { return 0; }

            /* Power ports — simple GPIO on/off */
            virtual int  GetPowerState(int i);
            virtual bool SetPowerState(int i, int state);
            virtual bool ResetPowerOvercurrent(int i)          { return false; }
            virtual int  GetPowerBootState(int i)              { return powerBootstrap[i]; }
            virtual bool SetPowerBootState(int i, int state);

            /* USB — not present */
            virtual int  GetUSBState(int i)                    { return 0; }
            virtual bool SetUSBState(int i, int state)         { return true; }
            virtual bool ResetUSBOvercurrent(int i)            { return false; }
            virtual int  GetUSBBootState(int i)                { return 0; }
            virtual bool SetUSBBootState(int i, int state)     { return true; }

            /* Dew / PWM ports — carrier at freq_hz, 8-bit duty cycle */
            virtual int   GetDewState(int i);
            virtual bool  SetDewState(int i, int state, int power);
            virtual int   GetDewAutoMode(int i)                { return 0; }
            virtual bool  SetDewAutoMode(int i, int state)     { return true; }
            virtual float GetDewAutoThreshold(int i)           { return std::numeric_limits<float>::quiet_NaN(); }
            virtual bool  SetDewAutoThreshold(int i, float)    { return true; }
            virtual bool  ResetDewOvercurrent(int i)           { return false; }
            virtual int   GetDewPWMPower(int i);

            /* Buck — not present */
            virtual int   GetBuckState()                       { return 0; }
            virtual bool  SetBuckState(int, float)             { return true; }
            virtual int   GetBuckBootState()                   { return 0; }
            virtual bool  SetBuckBootState(int)                { return true; }
            virtual float GetBuckSetVoltage()                  { return 0.f; }
            virtual bool  ResetBuckOvercurrent(void)           { return false; }

            /* Single PWM port slot — not used; PWM ports are exposed as dew ports */
            virtual int  GetPWMState(void)                     { return 0; }
            virtual bool SetPWMState(int, int)                 { return true; }
            virtual int GetPWMPower(void)                     { return 0; }
            virtual bool ResetPWMOvercurrent(void)             { return false; }

            virtual int         GetFirmwareVersion()           { return 1; }
            virtual std::string GetModelType()                 { return "Raspberry Pi"; }
            virtual std::string GetUUID()                      { return this->GetFullSerial(); }

            virtual PB_ERROR_TYPE FactoryReset(void)           { return PB_SUCCESS; }

            virtual void StartStatusListener(void);
            virtual void StopStatusListener(void);

        protected:
            virtual void StatusListenerThreadFunc();

        private:
            std::string GetFullSerial(void);
            void LoadSettings(void);
            void SaveSettings(void);

            PinsBoxCustomHWConfig hw_;

            GPIOManager*              gpio_;         ///< Manages power-port output GPIOs
            std::vector<PWM*>         pwm_;          ///< One PWM object per PWM port
            std::vector<unsigned int> pwm_period_ns_;///< Pre-computed period [ns] per PWM port

            std::thread                            statusListenerThread_;
            std::chrono::steady_clock::time_point  sessionStart_;
    };

#ifdef HAVE_LIBGPIOD
    bool ScanPinsBoxCustom(int *ids);
#else
    /* Stub when libgpiod / GPIO support is not available */
    inline bool ScanPinsBoxCustom(int *ids) { (void)ids; return false; }
#endif

} /* namespace PowerBox */

#endif /* PINS_BOX_CUSTOM_DEVICE_H */
