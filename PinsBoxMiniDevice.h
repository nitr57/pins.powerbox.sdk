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

#ifndef PINS_BOX_MINI_DEVICE_H
#define PINS_BOX_MINI_DEVICE_H

#include "Device.h"
#include "GPIOManager.h"
#include "USB.h"
#include "BTS7XXX.h"
#include "MCP320X.h"
#include "DEW.h"
#include "DHT22.h"
#include "ADJ.h"
#include "PWM.h"
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <limits>

namespace PowerBox
{
    /*
     * PinsBoxMiniDevice — Raspberry Pi 5 + HAT power box.
     *
     *   3x 12V power out      (BTS7012, on/off)
     *   1x 12V PWM out        (BTS7012, 2nd channel shared with 12V #3; on/off + 8-bit duty cycle)
     *   2x dew heater out     (BTS7080, on/off + 8-bit duty cycle, DS18B20 probe each)
     *   4x USB               (read-only, non-controllable: 2x USB2, 2x USB3)
     *   1x DHT22             (ambient temperature / humidity)
     *   1x BTS7006           (total 12V input current sense)
     *
     * A single MCP3202 ADC is used: channel 0 is the (DSEL-multiplexed) current
     * sense shared by every BTS chip, channel 1 is the 12V supply sense.
     *
     * Hardware quirk: the BTS7006 supply DEN is wired to DSEL (GPIO26) rather
     * than a dedicated diag line. The supply is therefore read by driving DSEL
     * high in isolation; as a side effect the BTS7006 sense also bleeds onto the
     * shared IS line while any "sel 1" port is measured, so the current of those
     * ports (12V #2, the PWM port, dew #2) is corrected in software.
     */
    class PinsBoxMiniDevice : public Device
    {
        public:
            PinsBoxMiniDevice(void);
            virtual ~PinsBoxMiniDevice(void);

            virtual PB_ERROR_TYPE Open(void);
            virtual void Close(void);

            virtual std::string GetSerial(void);

            virtual int GetUpTime(void);
            virtual float GetCoreTemp(void);
            virtual float GetTemperature(void) { return this->temperature;}
            virtual float GetHumidity(void) {return this->humidity;}
            virtual float GetDewPoint(void){return this->dewPoint;}
            virtual int GetExtSensor(void) {return this->extSensor ? 1 : 0; }

            virtual float GetSupply12V(void)  { return this->supply12V; }
            virtual float GetSupply5V(void) { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12A(void) { return this->supply12A; }
            virtual float GetSupply12Ah(void) { return this->supply12Ah; }
            virtual float GetSupply12Wh(void) { return this->supply12Wh; }
            virtual float GetSupply12AverageA(void);

            virtual float GetTemperatureOffset() {return this->temperatureOffset;}
            virtual bool SetTemperatureOffset(float val);
            virtual bool SetExtTemperature(float val);
            virtual float GetHumidityOffset() { return this->humidityOffset;}
            virtual bool SetHumidityOffset(float val);
            virtual bool SetExtHumidity(float val);
            virtual int GetEnvUpdateRate() { return this->envUpdateRate; }
            virtual bool SetEnvUpdateRate(int val) { this->envUpdateRate = val; return true; }
            virtual int GetUpdateRate() { return 1; }
            virtual bool SetUpdateRate(int val) { return true; }

            virtual int GetNumPowerPorts(void) const;
            virtual int GetNumUSBPorts(void) const;
            virtual int GetNumDewPorts(void) const;
            virtual int GetNumBuckPorts(void) const;
            virtual int GetNumPWMPorts(void) const;

            virtual int GetPowerState(int i);
            virtual bool SetPowerState(int i, int state);
            virtual bool ResetPowerOvercurrent(int i)                               { return false; }
            virtual int GetPowerBootState(int i) { return powerBootstrap[i]; }
            virtual bool SetPowerBootState(int i, int state);

            /* USB ports are read-only / always on */
            virtual int GetUSBState(int i)                                          { return 1; }
            virtual bool SetUSBState(int i, int state)                              { return true; }
            virtual bool ResetUSBOvercurrent(int i)                                 { return false; }
            virtual int GetUSBBootState(int i)                                      { return 1; }
            virtual bool SetUSBBootState(int i, int state)                          { return true; }

            virtual int GetDewState(int i);
            virtual bool SetDewState(int i, int state, int power);
            virtual int GetDewAutoMode(int i) { return dewAuto[i]; }
            virtual bool SetDewAutoMode(int i, int state);
            virtual float GetDewAutoThreshold(int i) { return dewThreshold[i]; }
            virtual bool SetDewAutoThreshold(int i, float val);
            virtual bool ResetDewOvercurrent(int i)                                 { return false; }
            virtual int GetDewPWMPower(int i) { return this->dewPWM[i]; }

            /* No buck port on the Mini */
            virtual int GetBuckState()                                              { return 0; }
            virtual bool SetBuckState(int state, float target)                      { return true; }
            virtual int GetBuckBootState()                                          { return 0; }
            virtual bool SetBuckBootState(int state)                                { return true; }
            virtual float GetBuckSetVoltage()                                       { return 0; }
            virtual bool ResetBuckOvercurrent(void)                                 { return false; }

            virtual int GetPWMState(void);
            virtual bool SetPWMState(int state, int power);
            virtual bool GetPWMPower(void) { return this->pwmPWM; }
            virtual bool ResetPWMOvercurrent(void)                                  { return false; }

            virtual int GetFirmwareVersion() { return 1; }
            virtual std::string GetModelType() { return "Raspberry Pi 5"; }
            virtual std::string GetUUID() { return this->GetFullSerial(); }

            virtual PB_ERROR_TYPE Beep(int volume, int duration_ms);
            virtual PB_ERROR_TYPE FactoryReset(void)                                { return PB_SUCCESS; }

            virtual void StartStatusListener(void);
            virtual void StopStatusListener(void);

        protected:
            virtual void StatusListenerThreadFunc();
            virtual void DHT22ThreadFunc();

        private:
            void ResetProperties(void);
            std::string GetFullSerial(void);
            void GetDHT22Data(void);
            void GetMCP3202Data(void);
            void LoadSettings(void);
            void SaveSettings(void);

            float temperatureOffset = 0.0f;
            float humidityOffset = 0.0f;

            DHT22* dht_;

            GPIOManager *gpio_;
            MCP3202* adc_;

            BTS7006<MCP3202>* supply_;
            BTS7012<MCP3202>** pwr_;
            DewPortT<MCP3202>** dew_;
            PWMPortT<MCP3202, 4785>* pwm_;
            PWM* buzzer_;
            std::thread statusListenerThread_;
            std::thread dht22Thread_;
            std::atomic<bool> dht22ThreadRunning_{false};
            /* Serializes access to the shared SPI/GPIO hardware between the
             * listener thread and SDK API calls */
            std::mutex hwMutex_;
            std::chrono::steady_clock::time_point lastEnergyUpdate_;
            std::chrono::steady_clock::time_point sessionStart_;
            bool energyUpdateInitialized_ = false;
    };

#ifdef HAVE_LIBGPIOD
    bool ScanPinsBoxMini(int *ids);
#else
    /* Stub when libgpiod / PinsBox support is not available */
    inline bool ScanPinsBoxMini(int *ids) { (void)ids; return false; }
#endif

} /* namespace PowerBox */

#endif /* PINS_BOX_MINI_DEVICE_H */
