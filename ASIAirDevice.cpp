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

#include "ASIAirDevice.h"
#include "PowerBoxLogging.h"
#include "Device.h"
#include "DeviceUtil.h"
#include "PWM.h"
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

#define ASIAIR_NUM_USB_PORTS  0
#define ASIAIR_NUM_DEW_PORTS  0
#define ASIAIR_NUM_BUCK_PORTS 0
#define ASIAIR_NUM_PWM_PORTS  0

namespace PowerBox
{
    /* GPIO pin numbers for power ports (BCM numbering).
     * Identical to the PinsBoxLight power port layout. */
    static constexpr uint8_t ASIAIR_PWR_PINS[ASIAIR_NUM_POWER_PORTS] = {
        12, // PWR port 1
        13, // PWR port 2
        26, // PWR port 3
        18, // PWR port 4
    };

    ASIAirDevice::ASIAirDevice(void)
    {
        this->statusListenerRunning = false;

        this->ResetProperties();
        this->LoadSettings();

        this->gpio_ = new GPIOManager("/dev/gpiochip0",
                                      nullptr,
                                      -1,
                                      0,
                                      nullptr,
                                      ASIAIR_NUM_POWER_PORTS,
                                      ASIAIR_PWR_PINS);
        this->gpio_->begin();

        /* Initialize buzzer PWM on GPIO19 (pwmchip0, channel 1) */
        this->buzzer_ = new PWM("pwmchip0", 1);
        this->buzzer_->setExport();

        /* Apply bootstrap states */
        for (int i = 0; i < ASIAIR_NUM_POWER_PORTS; ++i)
        {
            this->gpio_->digitalWrite(ASIAIR_PWR_PINS[i], this->powerBootstrap[i] ? 1 : 0);
            this->powerState[i] = this->powerBootstrap[i];
        }
    }

    ASIAirDevice::~ASIAirDevice(void)
    {
        this->StopStatusListener();

        if (this->buzzer_)
        {
            delete this->buzzer_;
            this->buzzer_ = nullptr;
        }

        if (this->gpio_)
        {
            delete this->gpio_;
            this->gpio_ = nullptr;
        }
    }

    void ASIAirDevice::ResetProperties(void)
    {
        for (int i = 0; i < ASIAIR_NUM_POWER_PORTS; ++i)
        {
            this->powerBootstrap[i] = 0;
            this->powerState[i]     = 0;
        }
    }

    void ASIAirDevice::StartStatusListener(void)
    {
        this->statusListenerRunning = false;

        if (this->statusListenerThread_.joinable())
            this->statusListenerThread_.join();

        this->statusListenerRunning = true;
        this->statusListenerThread_ = std::thread(&ASIAirDevice::StatusListenerThreadFunc, this);
        PB_DEBUG("StartStatusListener: Listener thread started");
    }

    void ASIAirDevice::StopStatusListener(void)
    {
        this->statusListenerRunning = false;
        PB_DEBUG("StopStatusListener: Listener stop requested");

        if (this->statusListenerThread_.joinable())
        {
            this->statusListenerThread_.join();
            PB_DEBUG("StopStatusListener: Listener thread joined");
        }
    }

    void ASIAirDevice::StatusListenerThreadFunc()
    {
        PB_DEBUG("StatusListener: starting");

        // Beep on initial listener launch
        this->Beep(20, 400);

        while (this->statusListenerRunning)
        {
            /* No ADC / sensors on the ASIAir — just keep the thread alive */
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        PB_DEBUG("StatusListener: exiting");
    }

    PB_ERROR_TYPE ASIAirDevice::Open(void)
    {
        PB_DEBUG("PBOpen: Found ZWO ASIAir device");

        for (int i = 0; i < ASIAIR_NUM_POWER_PORTS; ++i)
            this->GetPowerState(i);

        this->isOpen = true;
        PB_INFO("[OK] Device opened");
        return PB_SUCCESS;
    }

    void ASIAirDevice::Close(void)
    {
        this->isOpen = false;
        PB_DEBUG("[OK] Device closed");
    }

    int ASIAirDevice::GetNumPowerPorts(void) const { return ASIAIR_NUM_POWER_PORTS; }
    int ASIAirDevice::GetNumUSBPorts(void)   const { return ASIAIR_NUM_USB_PORTS;   }
    int ASIAirDevice::GetNumDewPorts(void)   const { return ASIAIR_NUM_DEW_PORTS;   }
    int ASIAirDevice::GetNumBuckPorts(void)  const { return ASIAIR_NUM_BUCK_PORTS;  }
    int ASIAirDevice::GetNumPWMPorts(void)   const { return ASIAIR_NUM_PWM_PORTS;   }

    std::string ASIAirDevice::GetFullSerial(void)
    {
        std::ifstream file("/proc/device-tree/serial-number");
        if (!file.is_open())
            return "";

        std::string serial;
        std::getline(file, serial);
        // device-tree strings are null-terminated; strip null bytes and any
        // other non-printable characters so the UUID is a valid filename.
        serial.erase(std::remove_if(serial.begin(), serial.end(),
                                    [](unsigned char c){ return !std::isprint(c); }),
                     serial.end());
        return serial;
    }

    std::string ASIAirDevice::GetSerial(void)
    {
        std::string serial = this->GetFullSerial();
        if (serial.length() >= 7)
            return serial.substr(serial.length() - 7);
        return serial;
    }

    int ASIAirDevice::GetUpTime(void)
    {
        struct sysinfo info;
        if (sysinfo(&info) == 0)
            return info.uptime;
        return 0;
    }

    float ASIAirDevice::GetCoreTemp(void)
    {
        return ReadCoreTemp(-127.f);
    }

    int ASIAirDevice::GetPowerState(int i)
    {
        if (i < 0 || i >= ASIAIR_NUM_POWER_PORTS)
            return 0;

        uint8_t state = this->gpio_->digitalRead(ASIAIR_PWR_PINS[i]);
        this->powerState[i] = state;
        return state;
    }

    bool ASIAirDevice::SetPowerState(int i, int state)
    {
        if (i < 0 || i >= ASIAIR_NUM_POWER_PORTS)
            return false;

        if (!this->gpio_->digitalWrite(ASIAIR_PWR_PINS[i], state ? 1 : 0))
            return false;
        this->powerState[i] = state;
        return true;
    }

    bool ASIAirDevice::SetPowerBootState(int i, int state)
    {
        if (i < 0 || i >= ASIAIR_NUM_POWER_PORTS)
            return false;

        this->powerBootstrap[i] = state;
        this->SaveSettings();
        return true;
    }

    void ASIAirDevice::LoadSettings(void)
    {
        Json::Value root;
        SettingsLoad result = LoadSettingsFile("/home/pi/pins/asiairSettings.json", root);
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
                for (int i = 0; i < ASIAIR_NUM_POWER_PORTS && i < (int)powerPorts.size(); i++)
                {
                    int idx = powerPorts[i].get("port", i).asInt();
                    if (idx >= 0 && idx < ASIAIR_NUM_POWER_PORTS)
                        this->powerBootstrap[idx] = powerPorts[i].get("bootState", 0).asInt();
                }
            }
        }
        catch (const std::exception &e)
        {
            PB_ERROR("LoadSettings: failed to read settings, using defaults (%s)", e.what());
            this->ResetProperties();
        }
    }

    void ASIAirDevice::SaveSettings(void)
    {
        Json::Value root;

        for (int i = 0; i < ASIAIR_NUM_POWER_PORTS; i++)
        {
            root["powerPorts"][i]["port"]      = i;
            root["powerPorts"][i]["bootState"] = this->powerBootstrap[i];
        }

        if (SaveSettingsFile("/home/pi/pins/asiairSettings.json", root))
            PB_DEBUG("Settings saved");
    }

    PB_ERROR_TYPE ASIAirDevice::Beep(int volume, int duration_ms)
    {
        if (!this->buzzer_ || duration_ms <= 0)
        {
            return PB_ERROR_INVALID_PARAMETER;
        }

        // Clamp volume to 0-100 range
        volume = (volume < 0) ? 0 : (volume > 100) ? 100 : volume;

        // If volume is 0, no beep
        if (volume == 0)
        {
            return PB_SUCCESS;
        }

        // Use 2700 Hz as on the PinsBox buzzer
        const unsigned int BUZZER_FREQ_HZ = 2700;
        const unsigned int PERIOD_NS = 1000000000 / BUZZER_FREQ_HZ;

        if (this->buzzer_->getPeriod() != PERIOD_NS)
        {
            this->buzzer_->setPeriod(PERIOD_NS);
        }

        unsigned int duty_ns = (PERIOD_NS * volume) / 100;
        this->buzzer_->setDutyCycle(duty_ns);

        this->buzzer_->setState(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
        this->buzzer_->setState(false);

        return PB_SUCCESS;
    }

    static bool IsRPI(void)
    {
        std::ifstream file("/proc/device-tree/model");
        if (!file.is_open())
            return false;

        std::string model;
        std::getline(file, model);
        return (model.find("Raspberry Pi Compute Module 4") != std::string::npos ||
                model.find("Raspberry Pi 4 Model B") != std::string::npos);
    }

    bool ScanASIAir(int *ids)
    {
        // Check if this is a Raspberry PI CM4
        if(!ids || !IsRPI())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        PB_DEBUG("Valid ASIAir device found!");

        // Check if device already exists, reuse it instead of creating a new one
        if(g_devices.find(0) == g_devices.end())
        {
            auto device = std::make_shared<ASIAirDevice>();
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
