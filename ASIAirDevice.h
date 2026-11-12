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

#ifndef ASIAIR_DEVICE_H
#define ASIAIR_DEVICE_H

#include "Device.h"
#include "GPIOManager.h"
#include "PWM.h"
#include "Buzzer.h"
#include <limits>
#include <string>
#include <thread>

namespace PowerBox
{
    #define ASIAIR_NUM_POWER_PORTS 4

    class ASIAirDevice : public Device
    {
        public:
            ASIAirDevice(void);
            virtual ~ASIAirDevice(void);

            virtual PB_ERROR_TYPE Open(void);
            virtual void Close(void);

            virtual std::string GetSerial(void);

            virtual int GetUpTime(void);
            virtual float GetCoreTemp(void);
            virtual float GetTemperature(void) { return -127.f; }
            virtual float GetHumidity(void) { return -127.f; }
            virtual float GetDewPoint(void) { return -127.f; }
            virtual int GetExtSensor(void) { return 0; }

            virtual float GetSupply12V(void)  { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply5V(void)   { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12A(void)  { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12Ah(void) { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12Wh(void) { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12AverageA(void) { return std::numeric_limits<float>::quiet_NaN(); }

            virtual float GetTemperatureOffset() { return 0.f; }
            virtual bool SetTemperatureOffset(float val) { return true; }
            virtual bool SetExtTemperature(float val) { return true; }
            virtual float GetHumidityOffset() { return 0.f; }
            virtual bool SetHumidityOffset(float val) { return true; }
            virtual bool SetExtHumidity(float val) { return true; }
            virtual int GetEnvUpdateRate() { return 1; }
            virtual bool SetEnvUpdateRate(int val) { return true; }
            virtual int GetUpdateRate() { return 1; }
            virtual bool SetUpdateRate(int val) { return true; }

            virtual int GetNumPowerPorts(void) const;
            virtual int GetNumUSBPorts(void) const;
            virtual int GetNumDewPorts(void) const;
            virtual int GetNumBuckPorts(void) const;
            virtual int GetNumPWMPorts(void) const;

            virtual int GetPowerState(int i);
            virtual bool SetPowerState(int i, int state);
            virtual bool ResetPowerOvercurrent(int i)             { return false; }
            virtual int GetPowerBootState(int i)                  { return powerBootstrap[i]; }
            virtual bool SetPowerBootState(int i, int state);
            virtual int GetUSBState(int i)                        { return 1; }
            virtual bool SetUSBState(int i, int state)            { return true; }
            virtual bool ResetUSBOvercurrent(int i)               { return false; }
            virtual int GetUSBBootState(int i)                    { return 1; }
            virtual bool SetUSBBootState(int i, int state)        { return true; }
            virtual int GetDewState(int i)                        { return 0; }
            virtual bool SetDewState(int i, int state, int power) { return true; }
            virtual int GetDewAutoMode(int i)                     { return 0; }
            virtual bool SetDewAutoMode(int i, int state)         { return true; }
            virtual float GetDewAutoThreshold(int i)              { return std::numeric_limits<float>::quiet_NaN(); }
            virtual bool SetDewAutoThreshold(int i, float val)    { return true; }
            virtual bool ResetDewOvercurrent(int i)               { return false; }
            virtual int GetDewPWMPower(int i)                     { return 0; }

            virtual int GetBuckState()                            { return 0; }
            virtual bool SetBuckState(int state, float target)    { return true; }
            virtual int GetBuckBootState()                        { return 0; }
            virtual bool SetBuckBootState(int state)              { return true; }
            virtual float GetBuckSetVoltage()                     { return 0; }
            virtual bool ResetBuckOvercurrent(void)               { return false; }

            virtual int GetPWMState(void)                         { return 0; }
            virtual bool SetPWMState(int state, int power)        { return true; }
            virtual int GetPWMPower(void)                        { return 0; }
            virtual bool ResetPWMOvercurrent(void)                { return false; }

            virtual int GetFirmwareVersion()                      { return 1; }
            virtual std::string GetModelType()                    { return "ZWO ASIAir"; }
            virtual std::string GetUUID()                         { return this->GetFullSerial(); }

            virtual PB_ERROR_TYPE FactoryReset(void)              { return PB_SUCCESS; }

            virtual PB_ERROR_TYPE Beep(int volume, int duration_ms);

            virtual void StartStatusListener(void);
            virtual void StopStatusListener(void);

        protected:
            virtual void StatusListenerThreadFunc();

        private:
            void ResetProperties(void);
            std::string GetFullSerial(void);
            void LoadSettings(void);
            void SaveSettings(void);

            GPIOManager *gpio_;
            Buzzer      *buzzer_ = nullptr;

            std::thread statusListenerThread_;
    };

#ifdef HAVE_LIBGPIOD
    bool ScanASIAir(int *ids);
#else
    /* Stub when libgpiod / ASIAir support is not available */
    inline bool ScanASIAir(int *ids) { (void)ids; return false; }
#endif

} /* namespace PowerBox */

#endif /* ASIAIR_DEVICE_H */
