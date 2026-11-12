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

#ifndef PINS_BOX_DEVICE_H
#define PINS_BOX_DEVICE_H

#include "Device.h"
#include "GPIOManager.h"
#include "MCP4725.h"
#include "USB.h"
#include "BTS7XXX.h"
#include "MCP320X.h"
#include "ADJ.h"
#include "DEW.h"
#include "DHT22.h"
#include "PWM.h"
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace PowerBox
{
    /* Exclusive lock on the PinsBox hardware between processes (flock on a lock
     * file). Released when the object is destroyed (or the process ends). */
    class ProcessLock
    {
        public:
            enum Result { Acquired, Busy, Unavailable };

            ProcessLock(void) = default;
            ~ProcessLock(void);
            ProcessLock(const ProcessLock&) = delete;
            ProcessLock& operator=(const ProcessLock&) = delete;
            ProcessLock(ProcessLock&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
            ProcessLock& operator=(ProcessLock&& other) noexcept;

            Result acquire(const char* path);

        private:
            int fd_ = -1;
    };

    class PinsBoxDevice : public Device
    {
        public:
            PinsBoxDevice(void);
            virtual ~PinsBoxDevice(void);

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
            virtual float GetSupply5V(void) { return this->supply5V; }
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
            virtual int GetUSBState(int i);
            virtual bool SetUSBState(int i, int state);
            virtual bool ResetUSBOvercurrent(int i)                               { return false; }
            virtual int GetUSBBootState(int i) { return usbBootstrap[i]; }
            virtual bool SetUSBBootState(int i, int state);
            virtual int GetDewState(int i);
            virtual bool SetDewState(int i, int state, int power);
            virtual int GetDewAutoMode(int i) { return dewAuto[i]; }
            virtual bool SetDewAutoMode(int i, int state);
            virtual float GetDewAutoThreshold(int i) { return dewThreshold[i]; }
            virtual bool SetDewAutoThreshold(int i, float val);
            virtual bool ResetDewOvercurrent(int i)                               { return false; }
            virtual int GetDewPWMPower(int i) { return this->dewPWM[i]; }

            virtual int GetBuckState();
            virtual bool SetBuckState(int state, float target);
            virtual int GetBuckBootState() { return this->buckBootstrap; }
            virtual bool SetBuckBootState(int state);
            virtual float GetBuckSetVoltage() { return this->buckVset; }
            virtual bool ResetBuckOvercurrent(void)                                { return false; }

            virtual int GetPWMState(void);
            virtual bool SetPWMState(int state, int power);
            virtual int GetPWMPower(void) { return this->pwmPWM; }
            virtual bool ResetPWMOvercurrent(void)                                { return false; }

            virtual int GetFirmwareVersion() { return 1; }
            virtual std::string GetModelType() { return "Compute Model 5"; }
            virtual std::string GetUUID() { return this->GetFullSerial(); }

            virtual PB_ERROR_TYPE Beep(int volume, int duration_ms);
            virtual PB_ERROR_TYPE FactoryReset(void)                                { return PB_SUCCESS; }

            virtual void StartStatusListener(void);
            virtual void StopStatusListener(void);

            /* False if the GPIO initialization failed: nothing else was set up
             * and no output was changed, the device must not be used */
            bool IsInitialized(void) const { return this->initOk_; }

            /* Keeps the process lock for the lifetime of the device */
            void AdoptLock(ProcessLock&& lock) { this->lock_ = std::move(lock); }

        protected:
            virtual void StatusListenerThreadFunc();
            virtual void DHT22ThreadFunc();

        private:
            void ResetProperties(void);
            std::string GetFullSerial(void);
            void GetDHT22Data(void);
            void GetMCP3208Data(void);
            void LoadSettings(void);
            void SaveSettings(void);

            float temperatureOffset = 0.0f;
            float humidityOffset = 0.0f;

            MCP4725 *mcp4725 = nullptr;

            // All null until set up, the destructor copes with a constructor
            // that stopped early
            DHT22* dht_ = nullptr;

            GPIOManager *gpio_ = nullptr;
            MCP3204* adc_ = nullptr;

            BTS7006<MCP3204>* supply_ = nullptr;
            BTS7012<MCP3204>** pwr_ = nullptr;
            USB** usb_ = nullptr;
            DewPort** dew_ = nullptr;
            BuckPort* buck_ = nullptr;
            PWMPort* pwm_ = nullptr;
            PWM* buzzer_ = nullptr;

            bool initOk_ = false;
            // Destroyed after the destructor body, i.e. after the GPIO lines and
            // the I/O expander were released
            ProcessLock lock_;
            std::thread statusListenerThread_;
            std::thread dht22Thread_;
            std::atomic<bool> dht22ThreadRunning_{false};
            /* Serializes access to the shared SPI/I2C/GPIO hardware between the
             * listener thread and SDK API calls */
            std::mutex hwMutex_;

            // Rate limits the log of a hung I2C bus during the USB measurement
            ErrorThrottle usbBusError_;
            std::chrono::steady_clock::time_point lastEnergyUpdate_;
            std::chrono::steady_clock::time_point sessionStart_;
            bool energyUpdateInitialized_ = false;
    };

    /* 1: PinsBox found (ids[0] = 0), 0: no PinsBox, -1: PinsBox present but not
     * usable (used by another process, GPIO / I/O expander init failed, or id 0
     * is used by another device) */
#ifdef HAVE_LIBGPIOD
    int ScanPinsBox(int *ids);
#else
    /* Stub when libgpiod / PinsBox support is not available */
    inline int ScanPinsBox(int *ids) { (void)ids; return 0; }
#endif

} /* namespace PowerBox */

#endif /* PINS_BOX_DEVICE_H */
