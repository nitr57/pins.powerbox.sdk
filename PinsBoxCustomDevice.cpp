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

#include "PinsBoxCustomDevice.h"
#include "PowerBoxLogging.h"
#include "Device.h"
#include "DeviceUtil.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <json/json.h>

#define CUSTOM_CONFIG_PATH   "/etc/pinsCustomDevice"
#define CUSTOM_SETTINGS_PATH "/home/pi/pins/customDeviceSettings.json"

/* Maximum allowed BCM GPIO pin number on Raspberry Pi */
#define CUSTOM_GPIO_MAX 40
/* Maximum PWM carrier frequency accepted from config */
#define CUSTOM_FREQ_MAX 10000u
/* Maximum PWM channel index accepted from config */
#define CUSTOM_PWM_CHANNEL_MAX 7

namespace PowerBox
{
    /* -------------------------------------------------------------------------
     * Config parsing
     * ------------------------------------------------------------------------- */

    /**
     * Validate that a pwm_chip name is safe to use as a sysfs path component.
     * We only allow alphanumeric characters and digits; no slashes, dots, or
     * other metacharacters that could enable path traversal.
     */
    static bool IsSafePWMChipName(const std::string& name)
    {
        if (name.empty()) return false;
        for (char c : name)
        {
            if (!std::isalnum(static_cast<unsigned char>(c))) return false;
        }
        return true;
    }

    static bool IsRPI(void)
    {
        std::ifstream file("/proc/device-tree/model");
        if (!file.is_open()) return false;
        std::string model;
        std::getline(file, model);
        return model.find("Raspberry Pi") != std::string::npos;
    }

    /**
     * Parse /etc/pinsCustomDevice into cfg.
     * Returns false on any parse or validation error.
     */
    static bool ParseCustomConfigImpl(PinsBoxCustomHWConfig& cfg)
    {
        std::ifstream file(CUSTOM_CONFIG_PATH);
        if (!file.is_open())
        {
            PB_ERROR("pinsCustomDevice: cannot open " CUSTOM_CONFIG_PATH);
            return false;
        }

        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;
        if (!Json::parseFromStream(builder, file, &root, &errs))
        {
            PB_ERROR("pinsCustomDevice: JSON parse error: %s", errs.c_str());
            return false;
        }
        file.close();

        cfg.name = root.get("name", "Custom Device").asString();

        /* --- PWM ports ---------------------------------------------------- */
        if (!root.isMember("pwm_ports") || !root["pwm_ports"].isArray())
        {
            PB_ERROR("pinsCustomDevice: missing or invalid 'pwm_ports' array");
            return false;
        }
        const Json::Value& pwmArr = root["pwm_ports"];
        if (pwmArr.size() > PB_MAX_DEW_PORTS)
        {
            PB_ERROR("pinsCustomDevice: too many pwm_ports (max %d)", PB_MAX_DEW_PORTS);
            return false;
        }
        for (const auto& entry : pwmArr)
        {
            CustomPWMPortCfg pc;
            pc.gpio        = static_cast<uint8_t>(entry.get("gpio", 0).asInt());
            pc.freq_hz     = entry.get("frequency_hz", 50).asUInt();
            pc.pwm_chip    = entry.get("pwm_chip", "pwmchip0").asString();
            pc.pwm_channel = entry.get("pwm_channel", 0).asInt();

            if (pc.gpio > CUSTOM_GPIO_MAX)
            {
                PB_ERROR("pinsCustomDevice: pwm gpio %d out of range (max %d)", pc.gpio, CUSTOM_GPIO_MAX);
                return false;
            }
            if (pc.freq_hz < 1 || pc.freq_hz > CUSTOM_FREQ_MAX)
            {
                PB_ERROR("pinsCustomDevice: pwm frequency_hz %u out of range (1-%u)", pc.freq_hz, CUSTOM_FREQ_MAX);
                return false;
            }
            if (!IsSafePWMChipName(pc.pwm_chip))
            {
                PB_ERROR("pinsCustomDevice: pwm_chip name contains invalid characters");
                return false;
            }
            if (pc.pwm_channel < 0 || pc.pwm_channel > CUSTOM_PWM_CHANNEL_MAX)
            {
                PB_ERROR("pinsCustomDevice: pwm_channel %d out of range (0-%d)", pc.pwm_channel, CUSTOM_PWM_CHANNEL_MAX);
                return false;
            }

            cfg.pwm_ports.push_back(pc);
        }

        /* --- Power ports -------------------------------------------------- */
        if (!root.isMember("power_ports") || !root["power_ports"].isArray())
        {
            PB_ERROR("pinsCustomDevice: missing or invalid 'power_ports' array");
            return false;
        }
        const Json::Value& pwrArr = root["power_ports"];
        if (pwrArr.size() > PB_MAX_POWER_PORTS)
        {
            PB_ERROR("pinsCustomDevice: too many power_ports (max %d)", PB_MAX_POWER_PORTS);
            return false;
        }
        for (const auto& entry : pwrArr)
        {
            CustomPowerPortCfg pc;
            pc.gpio = static_cast<uint8_t>(entry.get("gpio", 0).asInt());

            if (pc.gpio > CUSTOM_GPIO_MAX)
            {
                PB_ERROR("pinsCustomDevice: power gpio %d out of range (max %d)", pc.gpio, CUSTOM_GPIO_MAX);
                return false;
            }

            cfg.power_ports.push_back(pc);
        }

        if (cfg.pwm_ports.empty() && cfg.power_ports.empty())
        {
            PB_ERROR("pinsCustomDevice: no ports configured");
            return false;
        }

        PB_DEBUG("pinsCustomDevice: parsed '%s' — %zu power port(s), %zu PWM port(s)",
                 cfg.name.c_str(), cfg.power_ports.size(), cfg.pwm_ports.size());
        return true;
    }

    /* The config is edited by hand and parsed inside PBScan: a value of the wrong
     * type makes jsoncpp throw, which must not escape through the C API */
    static bool ParseCustomConfig(PinsBoxCustomHWConfig& cfg)
    {
        try
        {
            return ParseCustomConfigImpl(cfg);
        }
        catch (const std::exception &e)
        {
            PB_ERROR("pinsCustomDevice: invalid " CUSTOM_CONFIG_PATH " (%s)", e.what());
            return false;
        }
    }

    /* -------------------------------------------------------------------------
     * Constructor / destructor
     * ------------------------------------------------------------------------- */

    PinsBoxCustomDevice::PinsBoxCustomDevice(const PinsBoxCustomHWConfig& hw)
        : hw_(hw), gpio_(nullptr)
    {
        this->statusListenerRunning = false;

        /* Dew-port PWM resolution: 8-bit (power values 0-255), consistent with
         * the full PinsBox. */
        this->dewPwmResolution = 8;

        /* Load persisted boot/power states before initialising hardware */
        this->LoadSettings();

        /* GPIO manager for the plain power ports */
        if (!hw_.power_ports.empty())
        {
            std::vector<uint8_t> gpios;
            gpios.reserve(hw_.power_ports.size());
            for (const auto& p : hw_.power_ports)
                gpios.push_back(p.gpio);

            this->gpio_ = new GPIOManager("/dev/gpiochip0",
                                          nullptr,
                                          -1,
                                          0, nullptr,
                                          static_cast<int>(gpios.size()),
                                          gpios.data());
            this->gpio_->begin();
        }

        /* PWM objects for the PWM ports (exposed as dew ports) */
        for (const auto& p : hw_.pwm_ports)
        {
            PWM* pwm = new PWM(p.pwm_chip.c_str(), p.pwm_channel);
            this->pwm_.push_back(pwm);
            this->pwm_period_ns_.push_back(1000000000u / p.freq_hz);
        }

        /* Mark all power ports as controllable */
        for (int i = 0; i < static_cast<int>(hw_.power_ports.size()); ++i)
            this->powerReadOnly[i] = false;
    }

    PinsBoxCustomDevice::~PinsBoxCustomDevice(void)
    {
        this->StopStatusListener();

        for (PWM* p : this->pwm_)
            delete p;
        this->pwm_.clear();

        if (this->gpio_)
        {
            delete this->gpio_;
            this->gpio_ = nullptr;
        }
    }

    /* -------------------------------------------------------------------------
     * Status listener
     * ------------------------------------------------------------------------- */

    void PinsBoxCustomDevice::StartStatusListener(void)
    {
        this->statusListenerRunning = false;
        if (this->statusListenerThread_.joinable())
            this->statusListenerThread_.join();

        this->statusListenerRunning = true;
        this->sessionStart_ = std::chrono::steady_clock::now();
        this->statusListenerThread_ = std::thread(&PinsBoxCustomDevice::StatusListenerThreadFunc, this);
        PB_DEBUG("PinsBoxCustom StartStatusListener: thread started");
    }

    void PinsBoxCustomDevice::StopStatusListener(void)
    {
        this->statusListenerRunning = false;
        PB_DEBUG("PinsBoxCustom StopStatusListener: stop requested");
        if (this->statusListenerThread_.joinable())
        {
            this->statusListenerThread_.join();
            PB_DEBUG("PinsBoxCustom StopStatusListener: thread joined");
        }
    }

    void PinsBoxCustomDevice::StatusListenerThreadFunc()
    {
        PB_DEBUG("PinsBoxCustom StatusListener: starting");
        while (this->statusListenerRunning)
            std::this_thread::sleep_for(std::chrono::seconds(1));
        PB_DEBUG("PinsBoxCustom StatusListener: exiting");
    }

    /* -------------------------------------------------------------------------
     * Open / Close
     * ------------------------------------------------------------------------- */

    PB_ERROR_TYPE PinsBoxCustomDevice::Open(void)
    {
        PB_DEBUG("PinsBoxCustom Open: initialising '%s'", hw_.name.c_str());

        /* Power ports — apply boot states */
        for (int i = 0; i < static_cast<int>(hw_.power_ports.size()); ++i)
        {
            int state = this->powerBootstrap[i];
            this->gpio_->digitalWrite(hw_.power_ports[i].gpio, static_cast<uint8_t>(state));
            this->powerState[i] = state;
        }

        /* PWM ports — export channels, set period and initial duty cycle */
        for (int i = 0; i < static_cast<int>(hw_.pwm_ports.size()); ++i)
        {
            PWM* p   = this->pwm_[i];
            unsigned int period = this->pwm_period_ns_[i];

            if (!p->setExport())
            {
                PB_ERROR("PinsBoxCustom Open: failed to export PWM port %d (%s ch%d)",
                         i, hw_.pwm_ports[i].pwm_chip.c_str(), hw_.pwm_ports[i].pwm_channel);
                return PB_ERROR_COMMUNICATION;
            }

            p->setPeriod(period);
            p->setDutyCycle(0);
            p->setState(false);

            this->dewState[i] = 0;
            this->dewPWM[i]   = 0;
        }

        this->isOpen = true;
        PB_INFO("[OK] PinsBoxCustom '%s' opened", hw_.name.c_str());
        return PB_SUCCESS;
    }

    void PinsBoxCustomDevice::Close(void)
    {
        /* Drive all PWM ports off */
        for (PWM* p : this->pwm_)
        {
            p->setDutyCycle(0);
            p->setState(false);
        }

        /* Drive all power ports off */
        for (int i = 0; i < static_cast<int>(hw_.power_ports.size()); ++i)
            this->gpio_->digitalWrite(hw_.power_ports[i].gpio, 0);

        this->isOpen = false;
        PB_DEBUG("PinsBoxCustom Close: done");
    }

    /* -------------------------------------------------------------------------
     * Identity / timing
     * ------------------------------------------------------------------------- */

    std::string PinsBoxCustomDevice::GetFullSerial(void)
    {
        std::ifstream file("/proc/device-tree/serial-number");
        if (!file.is_open()) return "";

        std::string serial;
        std::getline(file, serial);
        serial.erase(std::remove_if(serial.begin(), serial.end(),
                                    [](unsigned char c){ return !std::isprint(c); }),
                     serial.end());
        return serial;
    }

    std::string PinsBoxCustomDevice::GetSerial(void)
    {
        std::string s = this->GetFullSerial();
        return s.length() >= 7 ? s.substr(s.length() - 7) : s;
    }

    int PinsBoxCustomDevice::GetUpTime(void)
    {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - this->sessionStart_).count());
    }

    float PinsBoxCustomDevice::GetCoreTemp(void)
    {
        return ReadCoreTemp(-127.f);
    }

    /* -------------------------------------------------------------------------
     * Port counts
     * ------------------------------------------------------------------------- */

    int PinsBoxCustomDevice::GetNumPowerPorts(void) const
    {
        return static_cast<int>(hw_.power_ports.size());
    }

    int PinsBoxCustomDevice::GetNumDewPorts(void) const
    {
        return static_cast<int>(hw_.pwm_ports.size());
    }

    /* -------------------------------------------------------------------------
     * Power port control (simple GPIO on/off)
     * ------------------------------------------------------------------------- */

    int PinsBoxCustomDevice::GetPowerState(int i)
    {
        if (i < 0 || i >= static_cast<int>(hw_.power_ports.size()))
            return 0;
        return this->powerState[i];
    }

    bool PinsBoxCustomDevice::SetPowerState(int i, int state)
    {
        if (i < 0 || i >= static_cast<int>(hw_.power_ports.size()))
            return false;

        this->gpio_->digitalWrite(hw_.power_ports[i].gpio, static_cast<uint8_t>(state != 0 ? 1 : 0));
        this->powerState[i] = state != 0 ? 1 : 0;
        return true;
    }

    bool PinsBoxCustomDevice::SetPowerBootState(int i, int state)
    {
        if (i < 0 || i >= static_cast<int>(hw_.power_ports.size()))
            return false;
        this->powerBootstrap[i] = state;
        SaveSettings();
        return true;
    }

    /* -------------------------------------------------------------------------
     * Dew / PWM port control (Linux sysfs PWM, 8-bit duty cycle)
     * ------------------------------------------------------------------------- */

    int PinsBoxCustomDevice::GetDewState(int i)
    {
        if (i < 0 || i >= static_cast<int>(hw_.pwm_ports.size()))
            return 0;
        return this->dewState[i];
    }

    bool PinsBoxCustomDevice::SetDewState(int i, int state, int power)
    {
        if (i < 0 || i >= static_cast<int>(hw_.pwm_ports.size()))
            return false;

        /* Clamp power to 8-bit range */
        int clamped_power = std::max(0, std::min(255, power));

        PWM* p             = this->pwm_[i];
        unsigned int period = this->pwm_period_ns_[i];

        if (state)
        {
            unsigned int duty = static_cast<unsigned int>(period * clamped_power / 255u);
            p->setDutyCycle(duty);
            p->setState(true);
        }
        else
        {
            p->setDutyCycle(0);
            p->setState(false);
        }

        this->dewState[i] = state != 0 ? 1 : 0;
        this->dewPWM[i]   = clamped_power;
        return true;
    }

    int PinsBoxCustomDevice::GetDewPWMPower(int i)
    {
        if (i < 0 || i >= static_cast<int>(hw_.pwm_ports.size()))
            return 0;
        return this->dewPWM[i];
    }

    /* -------------------------------------------------------------------------
     * Persistent settings
     * ------------------------------------------------------------------------- */

    void PinsBoxCustomDevice::LoadSettings(void)
    {
        Json::Value root;
        SettingsLoad result = LoadSettingsFile(CUSTOM_SETTINGS_PATH, root);
        if (result == SettingsLoad::Missing)
        {
            PB_DEBUG("PinsBoxCustom: settings file not found, using defaults");
            SaveSettings();
            return;
        }
        if (result != SettingsLoad::Ok)
            return;

        /* LoadSettings runs inside PBScan, unexpected value types must not
         * throw across the C API, fall back to defaults instead */
        try
        {
            const Json::Value& ports = root["powerPorts"];
            if (ports.isArray())
            {
                for (int i = 0; i < static_cast<int>(ports.size()) && i < PB_MAX_POWER_PORTS; ++i)
                {
                    int idx = ports[i].get("port", i).asInt();
                    if (idx >= 0 && idx < PB_MAX_POWER_PORTS)
                        this->powerBootstrap[idx] = ports[i].get("bootState", 0).asInt();
                }
            }
        }
        catch (const std::exception &e)
        {
            PB_ERROR("PinsBoxCustom: failed to read settings, using defaults (%s)", e.what());
            std::fill(std::begin(this->powerBootstrap), std::end(this->powerBootstrap), 0);
        }
    }

    void PinsBoxCustomDevice::SaveSettings(void)
    {
        Json::Value root;
        for (int i = 0; i < static_cast<int>(hw_.power_ports.size()); ++i)
        {
            root["powerPorts"][i]["port"]      = i;
            root["powerPorts"][i]["bootState"] = this->powerBootstrap[i];
        }

        if (SaveSettingsFile(CUSTOM_SETTINGS_PATH, root))
            PB_DEBUG("PinsBoxCustom: settings saved");
    }

    /* -------------------------------------------------------------------------
     * Scan
     * ------------------------------------------------------------------------- */

    bool ScanPinsBoxCustom(int *ids)
    {
        std::error_code fsError;
        if (!ids || !IsRPI() || !std::filesystem::exists(CUSTOM_CONFIG_PATH, fsError))
            return false;

        std::lock_guard<std::mutex> lock(g_globalMutex);

        if (g_devices.find(0) == g_devices.end())
        {
            PinsBoxCustomHWConfig cfg;
            if (!ParseCustomConfig(cfg))
                return false;

            auto device = std::make_shared<PinsBoxCustomDevice>(cfg);
            g_devices[0] = device;
            g_devices[0]->StartStatusListener();
        }
        else
        {
            PB_DEBUG("PinsBoxCustom: device at id=0 already exists, reusing");
        }

        ids[0] = 0;
        return true;
    }

} /* namespace PowerBox */
