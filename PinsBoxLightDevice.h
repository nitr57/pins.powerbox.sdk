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

#ifndef PINS_BOX_LIGHT_DEVICE_H
#define PINS_BOX_LIGHT_DEVICE_H

#include "Device.h"
#include "GPIOManager.h"
#include "BTS7XXX.h"
#include "MCP320X.h"
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <limits>

namespace PowerBox
{
    #define PINSBOX_LIGHT_NUM_POWER_PORTS 4

    /* Chip variant for each power port */
    enum class PinsBoxChip { BTS7006, BTS7012, BTS7080 };

    /* Per-port hardware configuration */
    struct PinsBoxLightPowerPortConfig {
        PinsBoxChip chip;
        uint8_t     pwr_pin;      ///< GPIO pin driving the load switch
        uint8_t     den_pin;      ///< Diagnostic enable / DEN pin
        uint8_t     dsel_pin;     ///< DSEL pin (shared across ports)
        uint8_t     dsel_port;    ///< Port selection value (0 or 1) passed to begin()
        uint32_t    rsense;       ///< Sense resistor [mΩ]
        float       max_current;  ///< Over-current threshold [A]
        uint8_t     adc_channel;  ///< MCP3202 channel for current sense
    };

    /* Full board hardware configuration for one PinsBoxLight variant */
    struct PinsBoxLightHWConfig {
        const char*                  name;
        const uint8_t*               gpio_pins;        ///< Flat array of all GPIO pins managed by GPIOManager
        unsigned int                 num_gpio_pins;
        uint8_t                      supply_inden_pin;  ///< Supply input DEN pin
        uint8_t                      supply_dsel_pin;   ///< Supply DSEL pin
        PinsBoxLightPowerPortConfig  power_ports[PINSBOX_LIGHT_NUM_POWER_PORTS];
    };

    /* Declared hardware variants — definitions are in PinsBoxLightDevice.cpp */
    extern const PinsBoxLightHWConfig PINSBOX_LIGHT_HW_V1;  ///< Ports 0-1: BTS7012 (6A), ports 2-3: BTS7080 (3A)
    extern const PinsBoxLightHWConfig PINSBOX_LIGHT_HW_V2;  ///< All ports: BTS7012 (6A)

    class PinsBoxLightDevice : public Device
    {
        public:
            explicit PinsBoxLightDevice(const PinsBoxLightHWConfig& hw);
            virtual ~PinsBoxLightDevice(void);

            virtual PB_ERROR_TYPE Open(void);
            virtual void Close(void);

            virtual std::string GetSerial(void);

            virtual int GetUpTime(void);
            virtual float GetCoreTemp(void);
            virtual float GetTemperature(void) { return -127.f; }
            virtual float GetHumidity(void) {return -127.f; }
            virtual float GetDewPoint(void){ return -127.f; }
            virtual int GetExtSensor(void) { return 0; }

            virtual float GetSupply12V(void)  { return this->supply12V; }
            virtual float GetSupply5V(void)   { return std::numeric_limits<float>::quiet_NaN(); }
            virtual float GetSupply12A(void)  { return this->supply12A; }
            virtual float GetSupply12Ah(void) { return this->supply12Ah; }
            virtual float GetSupply12Wh(void) { return this->supply12Wh; }
            virtual float GetSupply12AverageA(void);

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
            virtual bool ResetPowerOvercurrent(int i)           { return false; }
            virtual int GetPowerBootState(int i)                { return powerBootstrap[i]; }
            virtual bool SetPowerBootState(int i, int state);
            virtual int GetUSBState(int i)                      { return 1; }
            virtual bool SetUSBState(int i, int state)          { return true; }
            virtual bool ResetUSBOvercurrent(int i)             { return false; }
            virtual int GetUSBBootState(int i)                  { return 1; }
            virtual bool SetUSBBootState(int i, int state)      { return true; }
            virtual int GetDewState(int i)                      { return 0; }
            virtual bool SetDewState(int i, int state, int power) { return true; }
            virtual int GetDewAutoMode(int i)                   { return 0; }
            virtual bool SetDewAutoMode(int i, int state)       { return true; }
            virtual float GetDewAutoThreshold(int i)            { return std::numeric_limits<float>::quiet_NaN(); }
            virtual bool SetDewAutoThreshold(int i, float val)  { return true; }
            virtual bool ResetDewOvercurrent(int i)             { return false; }
            virtual int GetDewPWMPower(int i)                   { return 0; }

            virtual int GetBuckState()                          { return 0; }
            virtual bool SetBuckState(int state, float target)  { return true; }
            virtual int GetBuckBootState()                      { return 0; }
            virtual bool SetBuckBootState(int state)            { return true; }
            virtual float GetBuckSetVoltage()                   { return 0; }
            virtual bool ResetBuckOvercurrent(void)             { return false; }

            virtual int GetPWMState(void)                       { return 0; }
            virtual bool SetPWMState(int state, int power)      { return true; }
            virtual int GetPWMPower(void)                      { return 0; }
            virtual bool ResetPWMOvercurrent(void)              { return false; }

            virtual int GetFirmwareVersion()                    { return 1; }
            virtual std::string GetModelType()                  { return "Raspberry Pi"; }
            virtual std::string GetUUID()                       { return this->GetFullSerial(); }

            virtual PB_ERROR_TYPE FactoryReset(void)            { return PB_SUCCESS; }

            virtual void StartStatusListener(void);
            virtual void StopStatusListener(void);

        protected:
            virtual void StatusListenerThreadFunc();

        private:
            void ResetProperties(void);
            std::string GetFullSerial(void);
            void GetMCP3202Data(void);
            void LoadSettings(void);
            void SaveSettings(void);

            GPIOManager *gpio_;
            MCP3202* adc_;

            BTS7006<MCP3202>* supply_;
            BTSPort<MCP3202>* pwr_[PINSBOX_LIGHT_NUM_POWER_PORTS];

            std::thread statusListenerThread_;
            /* Serializes access to the shared SPI/GPIO hardware between the
             * listener thread and SDK API calls */
            std::mutex hwMutex_;
            std::chrono::steady_clock::time_point sessionStart_;
    };

#ifdef HAVE_LIBGPIOD
    bool ScanPinsBoxLight(int *ids);
#else
    /* Stub when libgpiod / PinsBox support is not available */
    inline bool ScanPinsBoxLight(int *ids) { (void)ids; return false; }
#endif

} /* namespace PowerBox */

#endif /* PINS_BOX_LIGHT_DEVICE_H */
