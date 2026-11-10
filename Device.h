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

#ifndef DEVICE_H
#define DEVICE_H

#include "PowerBoxSDK.h"
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <string>

namespace PowerBox
{
    /**
     * Device represents a Power Box device with its current state.
     */
    class Device
    {
        public:
            virtual PB_ERROR_TYPE Open(void) = 0;
            virtual void Close(void) = 0;

            virtual std::string GetSerial(void) = 0;
            virtual int GetUpTime(void) = 0;
            virtual float GetCoreTemp(void) { return -127.f; }
            virtual float GetTemperature(void) = 0;
            virtual float GetHumidity(void) = 0;
            virtual float GetDewPoint(void) = 0;
            virtual int GetExtSensor(void) = 0;
            virtual int HasWiFi(void) { return 0; }

            virtual float GetSupply12V(void) = 0;
            virtual float GetSupply5V(void) = 0;
            virtual float GetSupply12A(void) = 0;
            virtual float GetSupply12Ah(void) = 0;
            virtual float GetSupply12Wh(void) = 0;
            virtual float GetSupply12AverageA(void) { return 0.0f; }

            virtual float GetTemperatureOffset(void) = 0;
            virtual bool SetTemperatureOffset(float val) = 0;
            virtual bool SetExtTemperature(float val) = 0;
            virtual float GetHumidityOffset(void) = 0;
            virtual bool SetHumidityOffset(float val) = 0;
            virtual bool SetExtHumidity(float val) = 0;
            virtual int GetEnvUpdateRate() = 0;
            virtual bool SetEnvUpdateRate(int val) = 0;
            virtual int GetUpdateRate(void) = 0;
            virtual bool SetUpdateRate(int val) = 0;

            virtual int GetNumPowerPorts(void) const = 0;
            virtual int GetNumUSBPorts(void) const = 0;
            virtual int GetNumDewPorts(void) const = 0;
            virtual int GetNumBuckPorts(void) const = 0;
            virtual int GetNumPWMPorts(void) const = 0;

            virtual int GetPowerState(int i) = 0;
            virtual bool SetPowerState(int i, int state) = 0;
            virtual int GetPowerBootState(int i) = 0;
            virtual bool SetPowerBootState(int i, int state) = 0;
            virtual bool ResetPowerOvercurrent(int i) = 0;

            virtual int GetUSBState(int i) = 0;
            virtual bool SetUSBState(int i, int state) = 0;
            virtual int GetUSBBootState(int i) = 0;
            virtual bool SetUSBBootState(int i, int state) = 0;
            virtual bool ResetUSBOvercurrent(int i) = 0;

            virtual int GetDewState(int i) = 0;
            virtual bool SetDewState(int i, int state, int power) = 0;
            virtual int GetDewAutoMode(int i) = 0;
            virtual bool SetDewAutoMode(int i, int state) = 0;
            virtual float GetDewAutoThreshold(int i) = 0;
            virtual bool SetDewAutoThreshold(int i, float val) = 0;
            virtual bool ResetDewOvercurrent(int i) = 0;
            virtual int GetDewPWMPower(int i) = 0;

            virtual int GetBuckState(void) = 0;
            virtual bool SetBuckState(int state, float target) = 0;
            virtual int GetBuckBootState(void) = 0;
            virtual bool SetBuckBootState(int state) = 0;
            virtual float GetBuckSetVoltage(void) = 0;
            virtual bool ResetBuckOvercurrent(void) = 0;

            virtual int GetPWMState(void) = 0;
            virtual bool SetPWMState(int state, int power) = 0;
            virtual bool GetPWMPower(void) = 0;
            virtual bool ResetPWMOvercurrent(void) = 0;

            PB_ERROR_TYPE GetPowerPortStatus(PB_POWER_PORT_STATUS *status) const;
            PB_ERROR_TYPE GetUSBPortStatus(PB_USB_PORT_STATUS *status) const;
            PB_ERROR_TYPE GetDewPortStatus(PB_DEW_PORT_STATUS *status) const;
            PB_ERROR_TYPE GetBuckPortStatus(PB_BUCK_PORT_STATUS *status) const;
            PB_ERROR_TYPE GetPWMPortStatus(PB_PWM_PORT_STATUS *status) const;

            virtual PB_ERROR_TYPE ScanWiFi(PB_WIFI_SCAN_RESULT *result) { return PB_ERROR_NOT_AVAILABLE; }
            virtual PB_ERROR_TYPE GetWiFiConfig(PB_WIFI_CONFIG *config) { return PB_ERROR_NOT_AVAILABLE; }
            virtual PB_ERROR_TYPE SetWiFiConfig(const PB_WIFI_CONFIG *config) { return PB_ERROR_NOT_AVAILABLE; }
            virtual PB_ERROR_TYPE GetWiFiStatus(PB_WIFI_STATUS *status) { return PB_ERROR_NOT_AVAILABLE; }

            virtual int GetFirmwareVersion(void) = 0;
            virtual std::string GetModelType(void) = 0;
            virtual std::string GetUUID(void) = 0;

            virtual PB_ERROR_TYPE Beep(int volume, int duration_ms) { return PB_ERROR_NOT_AVAILABLE; }
            virtual PB_ERROR_TYPE Restart(void) { return PB_ERROR_NOT_AVAILABLE; }
            virtual PB_ERROR_TYPE FactoryReset(void) = 0;

            virtual ~Device(void) = default;
            bool IsOpen(void) { return isOpen; }

            virtual void StartStatusListener(void) = 0;
            virtual void StopStatusListener(void) = 0;

        protected:
            virtual void StatusListenerThreadFunc() = 0;

        std::string modelType;
        std::string serial;

        // Status
        int upTime = 0;
        float temperature = -127.0f;
        float humidity = -127.0f;
        float dewPoint = -127.0f;
        int extSensor = 0;
        float extTemperature = -127.0f;
        float extHumidity = -127.0f;
        std::chrono::steady_clock::time_point extTime;  // last external temperature/humidity update

        // Supply status
        float supply12V = 0.0f;
        float supply12A = 0.0f;
        float supply5V = 0.0f;
        float supply12Ah = 0.0f;
        float supply12Wh = 0.0f;

        // Port status
        float powerCurrent[PB_MAX_POWER_PORTS] = {0.0f};
        int powerState[PB_MAX_POWER_PORTS] = {0};
        int powerBootstrap[PB_MAX_POWER_PORTS] = {0};
        int powerOvercurrent[PB_MAX_POWER_PORTS] = {0};
        int powerReadOnly[PB_MAX_POWER_PORTS] = {0};
        float usbCurrent[PB_MAX_USB_PORTS] = {0.0f};
        float usbVoltage[PB_MAX_USB_PORTS] = {0.0f};
        int usbState[PB_MAX_USB_PORTS] = {0};
        int usbBootstrap[PB_MAX_USB_PORTS] = {0};
        int usbOvercurrent[PB_MAX_USB_PORTS] = {0};
        int usbReadOnly[PB_MAX_USB_PORTS] = {0};
        float dewCurrent[PB_MAX_DEW_PORTS] = {0.0f};
        float dewProbe[PB_MAX_DEW_PORTS] = {-127.0f};
        int dewPWM[PB_MAX_DEW_PORTS] = {0};
        int dewState[PB_MAX_DEW_PORTS] = {0};
        int dewPwmResolution = 0;
        float dewThreshold[PB_MAX_DEW_PORTS] = {0};
        int dewAuto[PB_MAX_DEW_PORTS] = {0};
        int dewOvercurrent[PB_MAX_DEW_PORTS] = {0};
        float buckCurrent = 0.0f;
        float buckVoltage = 0.0f;
        float buckVmin = 1.0f;
        float buckVmax = 1.0f;
        float buckVset = 1.0f;
        int buckBootstrap = 0;
        int buckState = 0;
        int buckOvercurrent = 0;
        int buckOvervoltage = 0;
        float pwmCurrent = 0.0f;
        int pwmPWM = 0;
        int pwmState = 0;
        int pwmOvercurrent = 0;
        int pwmPwmResolution = 0;

        // Config
        float temperatureOffset = 0.0f;
        float humidityOffset = 0.0f;
        int envUpdateRate = 3;
        int updateRate = 1;

        /* Listener thread state - don't store thread, just the flag */
        std::atomic<bool> isOpen{false};
        std::atomic<bool> statusListenerRunning{false};
    };

    /**
     * Global device registry mapping device IDs to Device objects.
     */
    extern std::map<int, std::shared_ptr<Device>> g_devices;

    /**
     * Global mutex protecting access to g_devices.
     */
    extern std::mutex g_globalMutex;

} /* namespace PowerBox */

#endif /* DEVICE_H */
