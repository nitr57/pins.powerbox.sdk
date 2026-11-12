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

#include "StellaVitaDevice.h"
#include <limits>
#include "PowerBoxLogging.h"
#include "Device.h"
#include "DeviceUtil.h"
#include <filesystem>
#include <mutex>
#include <fstream>
#include <sys/sysinfo.h>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <json/json.h>

#define STELLAVITA_NUM_USB_PORTS  4
#define STELLAVITA_NUM_DEW_PORTS  0
#define STELLAVITA_NUM_BUCK_PORTS 0
#define STELLAVITA_NUM_PWM_PORTS  0

namespace PowerBox
{
    /* GPIO pin numbers for power ports (BCM numbering) */
    static constexpr uint8_t STELLAVITA_PWR_PINS[STELLAVITA_NUM_POWER_PORTS] = {
        18, // PWR port 1
        10, // PWR port 2
        17, // PWR port 3
         4, // PWR port 4
    };

    /* GPIO pin numbers for USB ports — ports 0+1 share GPIO 9 (USB 2.0),
     * ports 2+3 share GPIO 11 (USB 3.0).  Writing either port switches both. */
    static constexpr uint8_t STELLAVITA_USB_PINS[STELLAVITA_NUM_USB_PORTS] = {
         9, // USB 2.0 port 0
         9, // USB 2.0 port 1
        11, // USB 3.0 port 2
        11, // USB 3.0 port 3
    };

    /* All GPIO output pins managed by GPIOManager */
    static constexpr uint8_t STELLAVITA_ALL_PINS[] = {
        18, // PWR port 1
        10, // PWR port 2
        17, // PWR port 3
         4, // PWR port 4
         9, // USB 2.0
        11, // USB 3.0
    };

    StellaVitaDevice::StellaVitaDevice(void)
    {
        this->statusListenerRunning = false;

        // Initial reset
        this->ResetProperties();

        // Load settings from file at construction time
        this->LoadSettings();

        this->gpio_ = new GPIOManager("/dev/gpiochip0",
                                      nullptr,
                                      -1,
                                      0,
                                      nullptr,
                                      (int)sizeof(STELLAVITA_ALL_PINS),
                                      STELLAVITA_ALL_PINS);

        this->gpio_->begin();

        /* Apply power port bootstrap states */
        for (int i = 0; i < STELLAVITA_NUM_POWER_PORTS; ++i)
        {
            this->gpio_->digitalWrite(STELLAVITA_PWR_PINS[i], this->powerBootstrap[i] ? 1 : 0);
            this->powerState[i] = this->powerBootstrap[i];
        }

        /* Apply USB bootstrap states (both ports sharing a GPIO are set together) */
        for (int i = 0; i < STELLAVITA_NUM_USB_PORTS; ++i)
        {
            this->gpio_->digitalWrite(STELLAVITA_USB_PINS[i], this->usbBootstrap[i] ? 1 : 0);
            this->usbState[i] = this->usbBootstrap[i];
        }
    }

    StellaVitaDevice::~StellaVitaDevice(void)
    {
        this->StopStatusListener();

        if (this->gpio_)
        {
            delete this->gpio_;
            this->gpio_ = nullptr;
        }
    }

    void StellaVitaDevice::ResetProperties(void)
    {
        for (int i = 0; i < STELLAVITA_NUM_POWER_PORTS; ++i)
        {
            this->powerBootstrap[i] = 0;
            this->powerState[i]     = 0;
        }

        for (int i = 0; i < STELLAVITA_NUM_USB_PORTS; ++i)
        {
            this->usbBootstrap[i] = 1; // USB ports default on
            this->usbState[i]     = 1;
            // No current/voltage sensor: unknown
            this->usbCurrent[i]   = std::numeric_limits<float>::quiet_NaN();
            this->usbVoltage[i]   = std::numeric_limits<float>::quiet_NaN();
        }
    }

    void StellaVitaDevice::StartStatusListener(void)
    {
        /* Stop any existing listener by setting the flag */
        this->statusListenerRunning = false;

        /* Wait for old thread to exit if it's still running */
        if (this->statusListenerThread_.joinable()) {
            this->statusListenerThread_.join();
        }

        /* Start new listener thread */
        this->statusListenerRunning = true;
        this->statusListenerThread_ = std::thread(&StellaVitaDevice::StatusListenerThreadFunc, this);
        PB_DEBUG("StartStatusListener: Listener thread started");
    }

    void StellaVitaDevice::StopStatusListener(void)
    {
        /* Signal listener thread to stop */
        this->statusListenerRunning = false;
        PB_DEBUG("StopStatusListener: Listener stop requested");

        /* Wait for the listener thread to actually exit */
        if(this->statusListenerThread_.joinable()) {
            this->statusListenerThread_.join();
            PB_DEBUG("StopStatusListener: Listener thread joined");
        }
    }

    /* Background listener thread function for status messages */
    void StellaVitaDevice::StatusListenerThreadFunc()
    {
        PB_DEBUG("StatusListener: starting");

        while(this->statusListenerRunning)
        {
            /* No ADC / sensors on the StellaVita — just keep the thread alive */
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        PB_DEBUG("StatusListener: exiting");
    }

    PB_ERROR_TYPE StellaVitaDevice::Open(void)
    {
        PB_DEBUG("PBOpen: Found ToupTek StellaVita device");

        // Fetch power ports config
        for (int i = 0; i < STELLAVITA_NUM_POWER_PORTS; ++i) {
            this->GetPowerState(i);
        }

        this->isOpen = true;
        PB_INFO("[OK] Device opened");
        return PB_SUCCESS;
    }

    void StellaVitaDevice::Close(void)
    {
        this->isOpen = false;
        PB_DEBUG("[OK] Device closed");
    }

    int StellaVitaDevice::GetNumPowerPorts(void) const
    {
        return STELLAVITA_NUM_POWER_PORTS;
    }

    int StellaVitaDevice::GetNumUSBPorts(void) const
    {
        return STELLAVITA_NUM_USB_PORTS;
    }

    int StellaVitaDevice::GetNumDewPorts(void) const
    {
        return STELLAVITA_NUM_DEW_PORTS;
    }

    int StellaVitaDevice::GetNumBuckPorts(void) const
    {
        return STELLAVITA_NUM_BUCK_PORTS;
    }

    int StellaVitaDevice::GetNumPWMPorts(void) const
    {
        return STELLAVITA_NUM_PWM_PORTS;
    }

    std::string StellaVitaDevice::GetFullSerial(void)
    {
        std::ifstream file("/proc/device-tree/serial-number");
        if(!file.is_open())
        {
            return "";
        }

        std::string serial;
        std::getline(file, serial);
        // device-tree strings are null-terminated; strip null bytes and any
        // other non-printable characters so the UUID is a valid filename.
        serial.erase(std::remove_if(serial.begin(), serial.end(),
                                    [](unsigned char c){ return !std::isprint(c); }),
                     serial.end());
        return serial;
    }

    std::string StellaVitaDevice::GetSerial(void)
    {
        std::string serial = this->GetFullSerial();
        if (serial.length() >= 7)
        {
            return serial.substr(serial.length() - 7);
        }
        return serial;
    }

    int StellaVitaDevice::GetUpTime(void)
    {
        struct sysinfo info;
        if(sysinfo(&info) == 0)
        {
            return info.uptime;
        }

        return 0;
    }

    float StellaVitaDevice::GetCoreTemp(void)
    {
        return ReadCoreTemp(-127.f);
    }

    int StellaVitaDevice::GetPowerState(int i)
    {
        if (i < 0 || i >= STELLAVITA_NUM_POWER_PORTS)
            return 0;

        uint8_t state = this->gpio_->digitalRead(STELLAVITA_PWR_PINS[i]);
        this->powerState[i] = state;
        return state;
    }

    bool StellaVitaDevice::SetPowerState(int i, int state)
    {
        if (i < 0 || i >= STELLAVITA_NUM_POWER_PORTS)
            return false;

        if (!this->gpio_->digitalWrite(STELLAVITA_PWR_PINS[i], state ? 1 : 0))
            return false;
        this->powerState[i] = state;
        return true;
    }

    bool StellaVitaDevice::SetPowerBootState(int i, int state)
    {
        if (i < 0 || i >= STELLAVITA_NUM_POWER_PORTS)
            return false;

        this->powerBootstrap[i] = state;
        this->SaveSettings();
        return true;
    }

    int StellaVitaDevice::GetUSBState(int i)
    {
        if (i < 0 || i >= STELLAVITA_NUM_USB_PORTS)
            return 0;

        uint8_t state = this->gpio_->digitalRead(STELLAVITA_USB_PINS[i]);
        this->usbState[i] = state;
        return state;
    }

    bool StellaVitaDevice::SetUSBState(int i, int state)
    {
        if (i < 0 || i >= STELLAVITA_NUM_USB_PORTS)
            return false;

        uint8_t pin = STELLAVITA_USB_PINS[i];
        if (!this->gpio_->digitalWrite(pin, state ? 1 : 0))
            return false;
        /* Sync all ports that share the same GPIO */
        for (int j = 0; j < STELLAVITA_NUM_USB_PORTS; ++j)
            if (STELLAVITA_USB_PINS[j] == pin)
                this->usbState[j] = state;
        return true;
    }

    int StellaVitaDevice::GetUSBBootState(int i)
    {
        if (i < 0 || i >= STELLAVITA_NUM_USB_PORTS)
            return 0;
        return this->usbBootstrap[i];
    }

    bool StellaVitaDevice::SetUSBBootState(int i, int state)
    {
        if (i < 0 || i >= STELLAVITA_NUM_USB_PORTS)
            return false;

        this->usbBootstrap[i] = state;
        this->SaveSettings();
        return true;
    }

    void StellaVitaDevice::LoadSettings(void)
    {
        Json::Value root;
        SettingsLoad result = LoadSettingsFile("/home/pi/pins/stellavitaSettings.json", root);
        if (result == SettingsLoad::Missing)
        {
            PB_DEBUG("Settings file not found, creating with defaults");
            SaveSettings();
            return;
        }
        if (result != SettingsLoad::Ok)
            return;

        /* LoadSettings runs inside PBScan, unexpected value types must not
         * throw across the C API, fall back to defaults instead */
        try
        {
            if (root["powerPorts"].isArray())
            {
                const Json::Value &powerPorts = root["powerPorts"];
                for (int i = 0; i < STELLAVITA_NUM_POWER_PORTS && i < (int)powerPorts.size(); i++)
                {
                    int idx = powerPorts[i].get("port", i).asInt();
                    if (idx >= 0 && idx < STELLAVITA_NUM_POWER_PORTS)
                        this->powerBootstrap[idx] = powerPorts[i].get("bootState", 0).asInt();
                }
            }

            if (root["usbPorts"].isArray())
            {
                const Json::Value &usbPorts = root["usbPorts"];
                for (int i = 0; i < STELLAVITA_NUM_USB_PORTS && i < (int)usbPorts.size(); i++)
                {
                    int idx = usbPorts[i].get("port", i).asInt();
                    if (idx >= 0 && idx < STELLAVITA_NUM_USB_PORTS)
                        this->usbBootstrap[idx] = usbPorts[i].get("bootState", 1).asInt();
                }
            }
        }
        catch (const std::exception &e)
        {
            PB_ERROR("LoadSettings: failed to read settings, using defaults (%s)", e.what());
            this->ResetProperties();
        }
    }

    void StellaVitaDevice::SaveSettings(void)
    {
        Json::Value root;

        for (int i = 0; i < STELLAVITA_NUM_POWER_PORTS; i++)
        {
            root["powerPorts"][i]["port"]      = i;
            root["powerPorts"][i]["bootState"] = this->powerBootstrap[i];
        }

        for (int i = 0; i < STELLAVITA_NUM_USB_PORTS; i++)
        {
            root["usbPorts"][i]["port"]      = i;
            root["usbPorts"][i]["bootState"] = this->usbBootstrap[i];
        }

        if (SaveSettingsFile("/home/pi/pins/stellavitaSettings.json", root))
            PB_DEBUG("Settings saved");
    }

    static bool IsRPI(void)
    {
        std::ifstream file("/proc/device-tree/model");
        if (!file.is_open())
            return false;

        std::string model;
        std::getline(file, model);
        return model.find("Raspberry Pi Compute Module 4") != std::string::npos;
    }

    bool ScanStellaVita(int *ids)
    {
        // Check if this is a Raspberry PI CM4
        if(!ids || !IsRPI())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        PB_DEBUG("Valid device found!");

        // Check if device already exists, reuse it instead of creating a new one
        if(g_devices.find(0) == g_devices.end())
        {
            auto device = std::make_shared<StellaVitaDevice>();
            g_devices[0] = device;

            // Start status listener thread
            g_devices[0]->StartStatusListener();
        }
        else
        {
            PB_DEBUG("Device at id=0 already exists, reusing it");
        }

        ids[0] = 0;
        return true;
    }

} /* namespace PowerBox */
