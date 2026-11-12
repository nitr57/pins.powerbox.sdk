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

#include "PinsBoxLightDevice.h"
#include "PowerBoxLogging.h"
#include "MCP320X.h"
#include "Device.h"
#include "DeviceUtil.h"
#include <filesystem>
#include <mutex>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <json/json.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define PINSBOX_NUM_POWER_PORTS PINSBOX_LIGHT_NUM_POWER_PORTS
#define PINSBOX_NUM_USB_PORTS 4
#define PINSBOX_NUM_DEW_PORTS 0
#define PINSBOX_NUM_BUCK_PORTS 0
#define PINSBOX_NUM_PWM_PORTS 0

namespace PowerBox
{
    /* -------------------------------------------------------------------------
     * Hardware variant definitions
     * Add a new entry here for each PinsBoxLight board revision.
     * The /etc/pinsLightDevice file should contain the variant name (e.g. "v1").
     * ------------------------------------------------------------------------- */

    static constexpr uint8_t PINS_V1[] = {
        12, // PWR0
        13, // PWR1
        26, // PWR2
        18, // PWR3
        16, // DEN12V_12
        24, // DEN12V_34
        23, // DEN12V_IN
        25, // DSEL
    };

    static constexpr uint8_t PINS_V2[] = {
        12, // PWR0
        13, // PWR1
        26, // PWR2
        18, // PWR3
        16, // DEN12V_12
        24, // DEN12V_34
        23, // DEN12V_IN
        25, // DSEL
    };

    const PinsBoxLightHWConfig PINSBOX_LIGHT_HW_V1 = {
        .name            = "PinsBoxLight v1",
        .gpio_pins       = PINS_V1,
        .num_gpio_pins   = sizeof(PINS_V1),
        .supply_inden_pin = 23,
        .supply_dsel_pin  = 25,
        .power_ports = {
            { PinsBoxChip::BTS7012, /*pwr*/12, /*den*/16, /*dsel*/25, /*port*/1, 1200, 6.0f, /*ch*/0 },
            { PinsBoxChip::BTS7012, /*pwr*/13, /*den*/16, /*dsel*/25, /*port*/0, 1200, 6.0f, /*ch*/0 },
            { PinsBoxChip::BTS7012, /*pwr*/26, /*den*/24, /*dsel*/25, /*port*/1, 1200, 6.0f, /*ch*/0 },
            { PinsBoxChip::BTS7012, /*pwr*/18, /*den*/24, /*dsel*/25, /*port*/0, 1200, 6.0f, /*ch*/0 },
        },
    };

    /* v2: same pin layout as v1, but ports 0-1 use BTS7080 instead of BTS7012 */
    const PinsBoxLightHWConfig PINSBOX_LIGHT_HW_V2 = {
        .name            = "PinsBoxLight v2",
        .gpio_pins       = PINS_V2,
        .num_gpio_pins   = sizeof(PINS_V2),
        .supply_inden_pin = 23,
        .supply_dsel_pin  = 25,
        .power_ports = {
            { PinsBoxChip::BTS7080, /*pwr*/12, /*den*/16, /*dsel*/25, /*port*/0, 1200, 3.0f, /*ch*/0 },
            { PinsBoxChip::BTS7080, /*pwr*/13, /*den*/16, /*dsel*/25, /*port*/1, 1200, 3.0f, /*ch*/0 },
            { PinsBoxChip::BTS7012, /*pwr*/26, /*den*/24, /*dsel*/25, /*port*/0, 1200, 6.0f, /*ch*/0 },
            { PinsBoxChip::BTS7012, /*pwr*/18, /*den*/24, /*dsel*/25, /*port*/1, 1200, 6.0f, /*ch*/0 },
        },
    };

    /* Factory: create the correct BTSPortImpl for a given chip type */
    static BTSPort<MCP3202>* MakeBTSPort(PinsBoxChip chip, const GPIOManager& gpio, const MCP3202& adc)
    {
        switch (chip) {
            case PinsBoxChip::BTS7006: return new BTSPortImpl<MCP3202, 17700>(gpio, adc);
            case PinsBoxChip::BTS7012: return new BTSPortImpl<MCP3202,  4785>(gpio, adc);
            case PinsBoxChip::BTS7080: return new BTSPortImpl<MCP3202,  1800>(gpio, adc);
        }
        return nullptr;
    }

    PinsBoxLightDevice::PinsBoxLightDevice(const PinsBoxLightHWConfig& hw)
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
                                      hw.num_gpio_pins,
                                      hw.gpio_pins);

        this->gpio_->begin();

        // ADC
        this->adc_ = new MCP3202(*this->gpio_);
        this->adc_->begin(3.3f);

        // Supply
        this->supply_ = new BTS7006<MCP3202>(*this->gpio_, *this->adc_);
        this->supply_->begin(1200, hw.supply_inden_pin, 255, hw.supply_dsel_pin, 0);
        this->supply_->setMaxCurrent(12.f);
        this->supply_->setChannel(0);
        this->supply_->setSampling(100, 50);

        // Power ports — instantiate the correct chip type per port
        for (int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            const PinsBoxLightPowerPortConfig& pc = hw.power_ports[i];
            this->pwr_[i] = MakeBTSPort(pc.chip, *this->gpio_, *this->adc_);
            this->pwr_[i]->begin(pc.rsense, pc.den_pin, pc.pwr_pin, pc.dsel_pin, pc.dsel_port, this->powerBootstrap[i]);
            this->pwr_[i]->setMaxCurrent(pc.max_current);
            this->pwr_[i]->setChannel(pc.adc_channel);
            this->pwr_[i]->setSampling(10, 1);
        }

        // All power ports are controllable
        for (int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
            this->powerReadOnly[i] = false;

        // All USB ports are readonly and always on
        for (int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i)
        {
            this->usbReadOnly[i] = true;
            this->usbState[i] = 1;
        }
    }

    PinsBoxLightDevice::~PinsBoxLightDevice(void)
    {
        // Stop listener if running
        this->StopStatusListener();

        // Clean up supply monitoring
        if(this->supply_) {
            delete this->supply_;
            this->supply_ = nullptr;
        }

        // Clean up power ports
        for (int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i) {
            delete this->pwr_[i];
            this->pwr_[i] = nullptr;
        }

        // Clean up ADC
        if(this->adc_) {
            delete this->adc_;
            this->adc_ = nullptr;
        }

        // Clean up GPIO manager (must be last)
        if(this->gpio_) {
            delete this->gpio_;
            this->gpio_ = nullptr;
        }
    }

    void PinsBoxLightDevice::ResetProperties(void)
    {
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            this->powerBootstrap[i] = 0;
        }

        for(int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i)
        {
            this->usbCurrent[i] = std::numeric_limits<float>::quiet_NaN();
            this->usbVoltage[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    void PinsBoxLightDevice::StartStatusListener(void)
    {
        /* Stop any existing listener by setting the flag */
        this->statusListenerRunning = false;

        /* Wait for old thread to exit if it's still running */
        if(this->statusListenerThread_.joinable()) {
            this->statusListenerThread_.join();
        }

        /* Start new listener thread */
        this->statusListenerRunning = true;
        this->sessionStart_ = std::chrono::steady_clock::now();
        this->statusListenerThread_ = std::thread(&PinsBoxLightDevice::StatusListenerThreadFunc, this);
        PB_DEBUG("StartStatusListener: Listener thread started");
    }

    void PinsBoxLightDevice::StopStatusListener(void)
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
    void PinsBoxLightDevice::StatusListenerThreadFunc()
    {
        PB_DEBUG("StatusListener: starting");

        while(this->statusListenerRunning)
        {
            // An exception escaping a std::thread terminates the whole process
            try
            {
                // Fetch MCP3202 data
                this->GetMCP3202Data();
            }
            catch (const std::exception &e)
            {
                PB_ERROR("StatusListener: %s", e.what());
            }
            catch (...)
            {
                PB_ERROR("StatusListener: unknown exception");
            }

            // 1 sec delay before we query again
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        PB_DEBUG("StatusListener: exiting");
    }

    PB_ERROR_TYPE PinsBoxLightDevice::Open(void)
    {
        PB_DEBUG("PBOpen: Found PI'N'Stars power box light device");

        // Fetch power ports config
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i) {
            this->GetPowerState(i);
        }

        this->isOpen = true;
        PB_INFO("[OK] Device opened");
        return PB_SUCCESS;
    }

    void PinsBoxLightDevice::Close(void)
    {
        this->isOpen = false;
        PB_DEBUG("[OK] Device closed");
    }

    int PinsBoxLightDevice::GetNumPowerPorts(void) const
    {
        return PINSBOX_NUM_POWER_PORTS;
    }

    int PinsBoxLightDevice::GetNumUSBPorts(void) const
    {
        return PINSBOX_NUM_USB_PORTS;
    }

    int PinsBoxLightDevice::GetNumDewPorts(void) const
    {
        return PINSBOX_NUM_DEW_PORTS;
    }

    int PinsBoxLightDevice::GetNumBuckPorts(void) const
    {
        return PINSBOX_NUM_BUCK_PORTS;
    }

    int PinsBoxLightDevice::GetNumPWMPorts(void) const
    {
        return PINSBOX_NUM_PWM_PORTS;
    }

    std::string PinsBoxLightDevice::GetFullSerial(void)
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

    std::string PinsBoxLightDevice::GetSerial(void)
    {
        return ShortSerial(this->GetFullSerial());
    }

    int PinsBoxLightDevice::GetUpTime(void)
    {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - this->sessionStart_).count());
    }

    float PinsBoxLightDevice::GetCoreTemp(void)
    {
        return ReadCoreTemp(-127.f);
    }

    float PinsBoxLightDevice::GetSupply12AverageA(void)
    {
        int uptimeSeconds = this->GetUpTime();
        if (uptimeSeconds <= 0)
        {
            return 0.0f;
        }

        float uptimeHours = uptimeSeconds / 3600.0f;
        return this->supply12Ah / uptimeHours;
    }

    static const char *SETTINGS_FILE = "/home/pi/pins/deviceSettings.json";

    void PinsBoxLightDevice::LoadSettings(void)
    {
        Json::Value root;
        SettingsLoad result = LoadSettingsFile(SETTINGS_FILE, root);
        if (result == SettingsLoad::Missing) {
            PB_DEBUG("Settings file not found, creating with defaults");
            SaveSettings();
            return;
        }
        if (result != SettingsLoad::Ok) {
            return;
        }

        // LoadSettings runs inside PBScan, unexpected value types must not
        // throw across the C API, fall back to defaults instead
        try {
            // Load power port bootstrap states
            const Json::Value &powerPorts = root["powerPorts"];
            if (powerPorts.isArray()) {
                for (int i = 0; i < PINSBOX_NUM_POWER_PORTS && i < (int)powerPorts.size(); i++) {
                    if (this->powerReadOnly[i] == true) {
                        this->powerBootstrap[i] = true;
                    } else {
                        int idx = powerPorts[i].get("port", 0).asInt();
                        if (idx >= 0 && idx < PINSBOX_NUM_POWER_PORTS) {
                            this->powerBootstrap[idx] = powerPorts[i].get("bootState", 0).asInt();
                        }
                    }
                }
            }
        } catch (const std::exception &e) {
            PB_ERROR("LoadSettings: failed to read settings, using defaults (%s)", e.what());
            this->ResetProperties();
        }
    }

    void PinsBoxLightDevice::SaveSettings(void)
    {
        Json::Value root;

        // Power port bootstrap states
        for (int i = 0; i < PINSBOX_NUM_POWER_PORTS; i++) {
            root["powerPorts"][i]["port"] = i;
            root["powerPorts"][i]["bootState"] = this->powerBootstrap[i];
        }

        if (SaveSettingsFile(SETTINGS_FILE, root)) {
            PB_DEBUG("Settings saved");
        }
    }

    void PinsBoxLightDevice::GetMCP3202Data(void)
    {
        // hwMutex_ is taken per measurement block instead of for the whole pass,
        // so an API call waits for at most one block

        // Supply
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->supply_->measureCurrent();
            this->supply12A = this->supply_->getCurrent_mA() * 0.001f;
            this->supply12V = this->adc_->analogReadAverage(1, 20, 0) * (36e3 + 4.7e3) / 4.7e3 * 0.001f;
        }

        // Accumulate energy (called every 1 second)
        // Time delta: 1 second = 1/3600 hours
        float timeDeltaHours = 1.0f / 3600.0f;

        // Ah = Amps * Hours
        this->supply12Ah += this->supply12A * timeDeltaHours;

        // Wh = Volts * Amps * Hours
        this->supply12Wh += this->supply12V * this->supply12A * timeDeltaHours;

        // PWR ports
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->pwr_[i]->measureCurrent();
            this->powerCurrent[i] = this->pwr_[i]->getCurrent_mA() * 0.001f;
            this->powerOvercurrent[i] = this->pwr_[i]->getOverCurrent() ? 1 : 0;
            // Track the driver state, it changes on its own on an over current shutdown
            this->powerState[i] = this->powerReadOnly[i] ? 1 : this->pwr_[i]->getState();
        }
    }

    int PinsBoxLightDevice::GetPowerState(int i)
    {
        if (i < 0 || i >= PINSBOX_NUM_POWER_PORTS) {
            return 0;
        }

        // Read only port is always on
        if (this->powerReadOnly[i] == true) {
            return 1;
        }

        std::lock_guard<std::mutex> lock(this->hwMutex_);

        uint8_t state = this->pwr_[i]->getState();
        this->powerState[i] = state;
        return state;
    }

    bool PinsBoxLightDevice::SetPowerState(int i, int state)
    {
        if (i < 0 || i >= PINSBOX_NUM_POWER_PORTS) {
            return false;
        }

        // Port is read only
        if (this->powerReadOnly[i] == true) {
            return false;
        }

        std::lock_guard<std::mutex> lock(this->hwMutex_);

        // Report the state the port is actually in, a failed switch must not
        // show up as success in the API
        bool ok = this->pwr_[i]->setState(state);
        this->powerState[i] = this->pwr_[i]->getState();
        // Switching clears the over current flag: show it in the status at once
        this->powerOvercurrent[i] = this->pwr_[i]->getOverCurrent() ? 1 : 0;
        return ok;
    }

    bool PinsBoxLightDevice::SetPowerBootState(int i, int state)
    {
        this->powerBootstrap[i] = state;
        SaveSettings();
        return true;
    }

    static bool IsRPI(void)
    {
        std::ifstream file("/proc/device-tree/model");
        if(!file.is_open())
        {
            return false;
        }

        std::string model;
        std::getline(file, model);

        return model.find("Raspberry Pi") != std::string::npos;
    }

    /* Select the hardware config for this unit based on the contents of /etc/pinsLightDevice */
    static const PinsBoxLightHWConfig* SelectHWConfig(void)
    {
        std::ifstream file("/etc/pinsLightDevice");
        if (!file.is_open())
            return &PINSBOX_LIGHT_HW_V1;

        std::string version;
        std::getline(file, version);
        // strip trailing whitespace / CR
        version.erase(version.find_last_not_of(" \t\r\n") + 1);
        file.close();

        PB_DEBUG("Using '%s' hw config", version.c_str());

        if (version == "v1" || version.empty()) return &PINSBOX_LIGHT_HW_V1;
        if (version == "v2")                    return &PINSBOX_LIGHT_HW_V2;

        PB_ERROR("Unknown PinsBoxLight variant '%s', defaulting to v1", version.c_str());
        return &PINSBOX_LIGHT_HW_V1;
    }

    bool ScanPinsBoxLight(int *ids)
    {
        // Check if this is a Raspberry PI
        std::error_code fsError;
        if(!ids || !IsRPI() || !std::filesystem::exists("/etc/pinsLightDevice", fsError))
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        PB_DEBUG("Valid device found!");

        // Check if device already exists, reuse it instead of creating a new one
        if(g_devices.find(0) == g_devices.end())
        {
            PB_DEBUG("Fetching HW config...");
            const PinsBoxLightHWConfig *hwConfig = SelectHWConfig();
            auto device = std::make_shared<PinsBoxLightDevice>(*hwConfig);
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
