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

#include "PinsBoxMiniDevice.h"
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

#define PINSBOXMINI_NUM_POWER_PORTS 3
#define PINSBOXMINI_NUM_USB_PORTS   4
#define PINSBOXMINI_NUM_DEW_PORTS   2
#define PINSBOXMINI_NUM_BUCK_PORTS  0
#define PINSBOXMINI_NUM_PWM_PORTS   1

namespace PowerBox
{
    /* -------------------------------------------------------------------------
     * GPIO line map (BCM numbering). These raw lines are bit-banged / driven
     * directly through GPIOManager. The PWM signal lines (GPIO12/13/18 dew &
     * 12V PWM, GPIO16 buzzer) and the DHT22 (GPIO17) are handled through the
     * kernel sysfs PWM / IIO interfaces and are therefore NOT listed here.
     * ------------------------------------------------------------------------- */
    static constexpr unsigned int NUM_PINS = 12;
    static constexpr uint8_t PINS[NUM_PINS] = {
        22, // [0]  12V port #1 switch
        27, // [1]  12V port #2 switch
        20, // [2]  12V port #3 switch
        23, // [3]  PWM 12V port switch
        25, // [4]  dew port #1 switch
        24, // [5]  dew port #2 switch
        21, // [6]  DEN: 12V ports 1 & 2
         2, // [7]  DEN: 12V port 3 & PWM 12V port
         4, // [8]  DEN: dew heaters
        26, // [9]  DSEL
         5, // [10] DS18B20 of dew port #1
         6, // [11] DS18B20 of dew port #2
    };

    static const uint8_t *PWR_PINS     = &PINS[0];   // 12V port #1..#3 switches
    static const uint8_t PWM_PWR_PIN   = PINS[3];    // PWM 12V port switch
    static const uint8_t *DEW_PINS     = &PINS[4];   // dew port #1..#2 switches
    static const uint8_t DEN_12V_12    = PINS[6];
    static const uint8_t DEN_12V_3PWM  = PINS[7];
    static const uint8_t DEN_DEW       = PINS[8];
    static const uint8_t DSEL_PIN      = PINS[9];
    static const uint8_t *DS18_PINS    = &PINS[10];

    /* MCP3202 channels: ch0 = DSEL-multiplexed BTS current sense, ch1 = 12V sense */
    static constexpr uint8_t ADC_CH_ISENSE = 0;
    static constexpr uint8_t ADC_CH_12V    = 1;

    /* 12V supply sense divider (top = 36k, bottom = 4.7k) */
    static constexpr float SUPPLY_R1 = 36e3f;
    static constexpr float SUPPLY_R2 = 4.7e3f;

    /* BTS current-sense scaling (sense current IS = I_load / kILIS) */
    static constexpr float KILIS_BTS7006 = 17700.f; // total input supply
    static constexpr float KILIS_BTS7012 =  4785.f; // 12V & PWM 12V ports
    static constexpr float KILIS_BTS7080 =  1800.f; // dew ports

    /*
     * Hardware quirk: the BTS7006 supply DEN is wired to DSEL (GPIO26). While a
     * "sel 1" port is measured (DSEL driven high) the BTS7006 sense also drives
     * the shared IS line, so those ports read high. Correct each one by
     * subtracting the supply's scaled contribution:
     *   I_port = reported - I_supply * (kILIS_port / kILIS_supply)
     */
    static constexpr float CORR_7012 = KILIS_BTS7012 / KILIS_BTS7006;
    static constexpr float CORR_7080 = KILIS_BTS7080 / KILIS_BTS7006;

    /* -------------------------------------------------------------------------
     * PWM signal mapping (sysfs), mirroring PinsBoxDevice:
     *   - GPIO13 (dew #2) and GPIO18 (PWM 12V port) use the RP1 hardware PWM
     *     block, pwmchip0 channels 1 and 2.
     *   - GPIO12 (dew #1) uses the RP1 PIO via "dtoverlay=pwm-pio,gpio=12",
     *     which exposes its own single-channel pwmchip (pwmchip1, channel 0).
     *   - The buzzer (GPIO16) mirrors PinsBox on pwmchip2 channel 0.
     * ------------------------------------------------------------------------- */
    static constexpr const char* DEW1_PWMCHIP   = "pwmchip1"; // GPIO12 (PIO, pwm-pio overlay)
    static constexpr int         DEW1_PWMCH     = 0;
    static constexpr const char* DEW2_PWMCHIP   = "pwmchip0"; // GPIO13 (HW PWM)
    static constexpr int         DEW2_PWMCH     = 1;
    static constexpr const char* PWM12V_PWMCHIP = "pwmchip0"; // GPIO18 (HW PWM)
    static constexpr int         PWM12V_PWMCH   = 2;
    static constexpr const char* BUZZER_PWMCHIP = "pwmchip2"; // GPIO16
    static constexpr int         BUZZER_PWMCH   = 0;

    PinsBoxMiniDevice::PinsBoxMiniDevice(void)
    {
        this->statusListenerRunning = false;
        this->temperature = DEVICE_DISCONNECTED_C;
        this->humidity = DEVICE_DISCONNECTED_C;
        this->dewPoint = DEVICE_DISCONNECTED_C;
        this->extSensor = false;

        // Initial reset
        this->ResetProperties();

        // Load settings from file at construction time
        this->LoadSettings();

        this->dht_ = new DHT22("device0");

        this->gpio_ = new GPIOManager("/dev/gpiochip0",
                                      nullptr,
                                      -1,
                                      0,
                                      nullptr,
                                      NUM_PINS,
                                      PINS);

        this->gpio_->begin();

        // ADC
        this->adc_ = new MCP3202(*this->gpio_);
        this->adc_->begin(3.3f);

        // Total 12V input current sense (BTS7006, single channel).
        // Hardware quirk: its DEN is wired to DSEL (GPIO26) instead of a
        // dedicated diag line, so it is read by driving DSEL high in isolation
        // (diag_pin = sel_pin = DSEL, port = 1, no switch pin).
        this->supply_ = new BTS7006<MCP3202>(*this->gpio_, *this->adc_);
        this->supply_->begin(1200, DSEL_PIN, 255, DSEL_PIN, 1);
        this->supply_->setMaxCurrent(10.f);
        this->supply_->setChannel(ADC_CH_ISENSE);
        this->supply_->setSampling(100, 50);

        // 12V power ports (3x BTS7012). DSEL selects which port's current sense
        // is routed to the shared ADC channel within each DEN group.
        this->pwr_ = new BTS7012<MCP3202>*[PINSBOXMINI_NUM_POWER_PORTS];
        for(int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; ++i)
        {
            this->pwr_[i] = new BTS7012<MCP3202>(*this->gpio_, *this->adc_);
        }

        this->pwr_[0]->begin(1200, DEN_12V_12,   PWR_PINS[0], DSEL_PIN, 0, this->powerBootstrap[0]);
        this->pwr_[1]->begin(1200, DEN_12V_12,   PWR_PINS[1], DSEL_PIN, 1, this->powerBootstrap[1]);
        this->pwr_[2]->begin(1200, DEN_12V_3PWM, PWR_PINS[2], DSEL_PIN, 0, this->powerBootstrap[2]);

        for(int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; ++i)
        {
            this->pwr_[i]->setMaxCurrent(6.f);
            this->pwr_[i]->setChannel(ADC_CH_ISENSE);
            this->pwr_[i]->setSampling(10, 1);
            this->powerReadOnly[i] = false;
        }

        // USB ports are read-only / non-controllable and always on
        for(int i = 0; i < PINSBOXMINI_NUM_USB_PORTS; ++i)
        {
            this->usbReadOnly[i] = true;
            this->usbState[i] = 1;
        }

        // Dew ports (BTS7080 + DS18B20 probe). Shared DEN (GPIO4), DSEL selects port.
        this->dew_ = new DewPortT<MCP3202>*[PINSBOXMINI_NUM_DEW_PORTS];
        this->dew_[0] = new DewPortT<MCP3202>(*this->gpio_, *this->adc_, DEW1_PWMCHIP, DEW1_PWMCH);
        this->dew_[1] = new DewPortT<MCP3202>(*this->gpio_, *this->adc_, DEW2_PWMCHIP, DEW2_PWMCH);

        this->dew_[0]->begin(DEN_DEW, DEW_PINS[0], DSEL_PIN, DS18_PINS[0], 0, 400, this->dewAuto[0], ADC_CH_ISENSE);
        this->dew_[1]->begin(DEN_DEW, DEW_PINS[1], DSEL_PIN, DS18_PINS[1], 1, 400, this->dewAuto[1], ADC_CH_ISENSE);

        // PWM 12V port — 2nd channel of the BTS7012 shared with 12V port #3
        // (DEN GPIO2, DSEL sel 1). On/off + 8-bit duty cycle, 6A limit.
        this->pwm_ = new PWMPortT<MCP3202, 4785>(*this->gpio_, *this->adc_, PWM12V_PWMCHIP, PWM12V_PWMCH);
        this->pwm_->begin(DEN_12V_3PWM, PWM_PWR_PIN, DSEL_PIN, 1, 30000, ADC_CH_ISENSE, 6.0f);
        this->pwm_->setState(0, 0.f);

        // Buzzer PWM on GPIO16
        this->buzzer_ = new PWM(BUZZER_PWMCHIP, BUZZER_PWMCH);
        this->buzzer_->setExport();
    }

    PinsBoxMiniDevice::~PinsBoxMiniDevice(void)
    {
        // Stop listener if running
        this->StopStatusListener();

        // Clean up DHT22 sensor
        if(this->dht_) {
            delete this->dht_;
            this->dht_ = nullptr;
        }

        // Clean up supply monitoring
        if(this->supply_) {
            delete this->supply_;
            this->supply_ = nullptr;
        }

        // Clean up power ports
        if(this->pwr_) {
            for(int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; ++i) {
                if(this->pwr_[i]) {
                    delete this->pwr_[i];
                    this->pwr_[i] = nullptr;
                }
            }
            delete[] this->pwr_;
            this->pwr_ = nullptr;
        }

        // Clean up dew ports
        if(this->dew_) {
            for(int i = 0; i < PINSBOXMINI_NUM_DEW_PORTS; ++i) {
                if(this->dew_[i]) {
                    delete this->dew_[i];
                    this->dew_[i] = nullptr;
                }
            }
            delete[] this->dew_;
            this->dew_ = nullptr;
        }

        // Clean up PWM port
        if(this->pwm_) {
            delete this->pwm_;
            this->pwm_ = nullptr;
        }

        // Clean up buzzer PWM
        if(this->buzzer_) {
            delete this->buzzer_;
            this->buzzer_ = nullptr;
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

    void PinsBoxMiniDevice::ResetProperties(void)
    {
        for(int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; ++i)
        {
            this->powerBootstrap[i] = 0;
        }

        for(int i = 0; i < PINSBOXMINI_NUM_USB_PORTS; ++i)
        {
            this->usbCurrent[i] = std::numeric_limits<float>::quiet_NaN();
            this->usbVoltage[i] = std::numeric_limits<float>::quiet_NaN();
        }

        for(int i = 0; i < PINSBOXMINI_NUM_DEW_PORTS; ++i)
        {
            this->dewAuto[i] = 1;
            this->dewThreshold[i] = 4.f;
        }

        this->temperatureOffset = 0.f;
        this->humidityOffset = 0.f;
    }

    void PinsBoxMiniDevice::StartStatusListener(void)
    {
        /* Stop any existing listener by setting the flag */
        this->statusListenerRunning = false;

        this->dht22ThreadRunning_ = false;

        /* Wait for old threads to exit if they are still running, assigning
         * to a joinable std::thread would call std::terminate */
        if(this->statusListenerThread_.joinable()) {
            this->statusListenerThread_.join();
        }
        if(this->dht22Thread_.joinable()) {
            this->dht22Thread_.join();
        }

        /* Start new listener thread */
        this->statusListenerRunning = true;
        this->sessionStart_ = std::chrono::steady_clock::now();
        this->statusListenerThread_ = std::thread(&PinsBoxMiniDevice::StatusListenerThreadFunc, this);

        /* Start DHT22 thread separately so it doesn't block the main measurement loop */
        this->dht22ThreadRunning_ = true;
        this->dht22Thread_ = std::thread(&PinsBoxMiniDevice::DHT22ThreadFunc, this);
        PB_DEBUG("StartStatusListener: Listener thread started");
    }

    void PinsBoxMiniDevice::StopStatusListener(void)
    {
        /* Signal listener thread to stop */
        this->statusListenerRunning = false;
        PB_DEBUG("StopStatusListener: Listener stop requested");

        this->dht22ThreadRunning_ = false;

        /* Wait for the listener thread to actually exit */
        if(this->statusListenerThread_.joinable()) {
            this->statusListenerThread_.join();
            PB_DEBUG("StopStatusListener: Listener thread joined");
        }

        if(this->dht22Thread_.joinable()) {
            this->dht22Thread_.join();
            PB_DEBUG("StopStatusListener: DHT22 thread joined");
        }
    }

    /* Background listener thread function for status messages */
    void PinsBoxMiniDevice::StatusListenerThreadFunc()
    {
        PB_DEBUG("StatusListener: starting");

        // Beep on initial listener launch
        this->Beep(20, 400);

        while(this->statusListenerRunning)
        {
            // An exception escaping a std::thread terminates the whole process
            try
            {
                // Fetch MCP3202 data (fast SPI — runs every ~1s)
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

    void PinsBoxMiniDevice::DHT22ThreadFunc()
    {
        PB_DEBUG("DHT22Thread: starting");

        while(this->dht22ThreadRunning_)
        {
            // An exception escaping a std::thread terminates the whole process
            try
            {
                this->GetDHT22Data();
            }
            catch (const std::exception &e)
            {
                PB_ERROR("DHT22Thread: %s", e.what());
            }
            catch (...)
            {
                PB_ERROR("DHT22Thread: unknown exception");
            }

            // Sleep in short steps so StopStatusListener does not wait up to envUpdateRate seconds
            auto wakeUp = std::chrono::steady_clock::now() + std::chrono::seconds(this->envUpdateRate);
            while(this->dht22ThreadRunning_ && std::chrono::steady_clock::now() < wakeUp)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        PB_DEBUG("DHT22Thread: exiting");
    }

    PB_ERROR_TYPE PinsBoxMiniDevice::Open(void)
    {
        PB_DEBUG("PBOpen: Found PI'N'Stars power box mini device");

        // Fetch power ports config
        for(int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; ++i) {
            this->GetPowerState(i);
        }

        // Fetch dew ports config
        this->dewPwmResolution = 8;
        for(int i = 0; i < PINSBOXMINI_NUM_DEW_PORTS; ++i) {
            this->GetDewState(i);
        }

        // Fetch pwm port config
        this->pwmPwmResolution = 8;
        this->GetPWMState();

        this->isOpen = true;
        PB_INFO("[OK] Device opened");
        return PB_SUCCESS;
    }

    void PinsBoxMiniDevice::Close(void)
    {
        this->isOpen = false;
        PB_DEBUG("[OK] Device closed");
    }

    std::string PinsBoxMiniDevice::GetFullSerial(void)
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

    std::string PinsBoxMiniDevice::GetSerial(void)
    {
        return ShortSerial(this->GetFullSerial());
    }

    int PinsBoxMiniDevice::GetUpTime(void)
    {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - this->sessionStart_).count());
    }

    float PinsBoxMiniDevice::GetCoreTemp(void)
    {
        return ReadCoreTemp(DEVICE_DISCONNECTED_C);
    }

    float PinsBoxMiniDevice::GetSupply12AverageA(void)
    {
        int uptimeSeconds = this->GetUpTime();
        if (uptimeSeconds <= 0)
        {
            return 0.0f;
        }

        float uptimeHours = uptimeSeconds / 3600.0f;
        return this->supply12Ah / uptimeHours;
    }

    bool PinsBoxMiniDevice::SetTemperatureOffset(float val)
    {
        this->temperatureOffset = val;
        SaveSettings();
        return true;
    }

    bool PinsBoxMiniDevice::SetExtTemperature(float val)
    {
        if(val > -50.f && val < 80.f)
        {
            this->extTemperature = val;
            this->extTime = std::chrono::steady_clock::now();
            this->extSensor = true;
            return true;
        }
        return false;
    }

    bool PinsBoxMiniDevice::SetHumidityOffset(float val)
    {
        this->humidityOffset = val;
        SaveSettings();
        return true;
    }

    bool PinsBoxMiniDevice::SetExtHumidity(float val)
    {
        if(val > 0.0f && val <= 100.0f)
        {
            this->extHumidity = val;
            this->extTime = std::chrono::steady_clock::now();
            this->extSensor = true;
            return true;
        }
        return false;
    }

    static const char *SETTINGS_FILE = "/home/pi/pins/deviceSettings.json";

    static bool IsValidIndex(int idx, int count)
    {
        return idx >= 0 && idx < count;
    }

    void PinsBoxMiniDevice::LoadSettings(void)
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
            // Load environment settings
            if (root["environment"].isObject()) {
                this->temperatureOffset = root["environment"].get("temperatureOffset", 0.0).asFloat();
                this->humidityOffset = root["environment"].get("humidityOffset", 0.0).asFloat();
            }

            // Load power port bootstrap states
            const Json::Value &powerPorts = root["powerPorts"];
            if (powerPorts.isArray()) {
                for (int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS && i < (int)powerPorts.size(); i++) {
                    int idx = powerPorts[i].get("port", 0).asInt();
                    if (IsValidIndex(idx, PINSBOXMINI_NUM_POWER_PORTS)) {
                        this->powerBootstrap[idx] = powerPorts[i].get("bootState", 0).asInt();
                    }
                }
            }

            // Load Dew port auto mode states
            const Json::Value &dewPorts = root["dewPorts"];
            if (dewPorts.isArray()) {
                for (int i = 0; i < PINSBOXMINI_NUM_DEW_PORTS && i < (int)dewPorts.size(); i++) {
                    int idx = dewPorts[i].get("port", 0).asInt();
                    if (IsValidIndex(idx, PINSBOXMINI_NUM_DEW_PORTS)) {
                        this->dewAuto[idx] = dewPorts[i].get("autoMode", 1).asInt();
                        this->dewThreshold[idx] = dewPorts[i].get("autoThreshold", 4).asFloat();
                    }
                }
            }
        } catch (const std::exception &e) {
            PB_ERROR("LoadSettings: failed to read settings, using defaults (%s)", e.what());
            this->ResetProperties();
        }
    }

    void PinsBoxMiniDevice::SaveSettings(void)
    {
        Json::Value root;

        // Environment settings
        root["environment"]["temperatureOffset"] = this->temperatureOffset;
        root["environment"]["humidityOffset"] = this->humidityOffset;

        // Power port bootstrap states
        for (int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; i++) {
            root["powerPorts"][i]["port"] = i;
            root["powerPorts"][i]["bootState"] = this->powerBootstrap[i];
        }

        // Dew port automode states
        for (int i = 0; i < PINSBOXMINI_NUM_DEW_PORTS; ++i) {
            root["dewPorts"][i]["port"] = i;
            root["dewPorts"][i]["autoMode"] = this->dewAuto[i];
            root["dewPorts"][i]["autoThreshold"] = this->dewThreshold[i];
        }

        if (SaveSettingsFile(SETTINGS_FILE, root)) {
            PB_DEBUG("Settings saved");
        }
    }

    void PinsBoxMiniDevice::GetDHT22Data(void)
    {
        float currentTemperature = this->dht_->getTemperature();
        float currentHumidity = this->dht_->getHumidity();

        if(std::isnan(currentTemperature) ||
           std::isnan(currentHumidity) ||
           currentTemperature == DEVICE_DISCONNECTED_C ||
           currentHumidity == DEVICE_DISCONNECTED_C)
        {
            // External values (e.g. a weather station via the plugin) stay valid for
            // the same grace period as the internal sensors, so a late update does not
            // make the dew point disappear and switch the dew heaters off
            bool extFresh = std::chrono::steady_clock::now() - this->extTime < SENSOR_GRACE_PERIOD;

            if(!extFresh ||
               std::isnan(this->extTemperature) ||
               std::isnan(this->extHumidity) ||
               this->extTemperature == DEVICE_DISCONNECTED_C ||
               this->extHumidity == DEVICE_DISCONNECTED_C)
            {
                this->temperature = DEVICE_DISCONNECTED_C;
                this->humidity = DEVICE_DISCONNECTED_C;
                this->dewPoint = DEVICE_DISCONNECTED_C;
                this->extSensor = false;
                return;
            }

            // If we have externally set values, use them
            currentTemperature = this->extTemperature;
            currentHumidity = this->extHumidity;
            this->extSensor = true;
        }
        else
        {
            // We have valid environment data, disable external sensor
            this->extSensor = false;
        }

        // Apply temperature offset (calibration correction)
        currentTemperature += this->temperatureOffset;
        currentHumidity += this->humidityOffset;

        // Clamp humidity to physically meaningful range to avoid log10(<=0) and NaNs
        if(std::isnan(currentHumidity) || currentHumidity < 0.1f) currentHumidity = 0.1f;
        if(currentHumidity > 100.0f) currentHumidity = 100.0f;

        // Calculate dew point
        float tmp = 0.66077f + 7.5f * currentTemperature / (237.3f + currentTemperature) + (log10(currentHumidity) - 2.f);
        float currentDewPoint = (tmp - 0.66077f) * 237.3f / (0.66077f + 7.5f - tmp);

        this->temperature = currentTemperature;
        this->humidity = currentHumidity;
        this->dewPoint = std::isnan(currentDewPoint) ? DEVICE_DISCONNECTED_C : currentDewPoint;
        this->dewPoint = std::min(this->dewPoint, currentTemperature);
    }

    void PinsBoxMiniDevice::GetMCP3202Data(void)
    {
        // hwMutex_ is taken per measurement block instead of for the whole pass,
        // so an API call waits for at most one block (a dew port is the longest)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);

            // Total input current (BTS7006). Read first and in isolation: driving
            // DSEL high enables only the supply sense (all port DENs are low).
            this->supply_->measureCurrent();
            this->supply12A = this->supply_->getCurrent_mA() * 0.001f;

            // 12V supply sense (ADC channel 1, divider 36k / 4.7k)
            this->supply12V = this->adc_->analogReadAverage(ADC_CH_12V, 20, 0) * (SUPPLY_R1 + SUPPLY_R2) / SUPPLY_R2 * 0.001f;
        }

        // 12V power ports
        for(int i = 0; i < PINSBOXMINI_NUM_POWER_PORTS; ++i)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->pwr_[i]->measureCurrent();
            this->powerCurrent[i] = this->pwr_[i]->getCurrent_mA() * 0.001f;
            this->powerOvercurrent[i] = this->pwr_[i]->getOverCurrent() ? 1 : 0;
            // Track the driver state, it changes on its own on an over current shutdown
            this->powerState[i] = this->pwr_[i]->getState();
        }

        // Dew ports
        for(int i = 0; i < PINSBOXMINI_NUM_DEW_PORTS; ++i)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->dew_[i]->update(this->dewPoint, this->dewThreshold[i]);
            this->dewCurrent[i] = this->dew_[i]->getCurrent_mA() * 0.001f;
            this->dewOvercurrent[i] = this->dew_[i]->getOverCurrent() ? 1 : 0;
            this->dewProbe[i] = std::round(this->dew_[i]->getTemperature() * 100.0f) / 100.0f;
            this->dewPWM[i] = this->dew_[i]->getDutyCycle();
            this->dewState[i] = this->dew_[i]->getState();
        }

        // PWM 12V port
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->pwm_->measureCurrent();
            this->pwmCurrent = this->pwm_->getCurrent_mA() * 0.001f;
            this->pwmOvercurrent = this->pwm_->getOverCurrent() ? 1 : 0;
            this->pwmState = this->pwm_->getState();
        }

        // Correct the "sel 1" ports for the mis-wired BTS7006 sense bleeding onto
        // the shared IS line while DSEL is high: 12V #2 (pwr_[1], BTS7012), the
        // PWM port (BTS7012) and dew #2 (dew_[1], BTS7080).
        this->powerCurrent[1] = std::max(0.f, this->powerCurrent[1] - this->supply12A * CORR_7012);
        this->pwmCurrent      = std::max(0.f, this->pwmCurrent      - this->supply12A * CORR_7012);
        this->dewCurrent[1]   = std::max(0.f, this->dewCurrent[1]   - this->supply12A * CORR_7080);

        // Accumulate energy using actual elapsed time
        auto now = std::chrono::steady_clock::now();
        if(this->energyUpdateInitialized_)
        {
            float timeDeltaHours = std::chrono::duration<float>(now - this->lastEnergyUpdate_).count() / 3600.0f;

            // Ah = Amps * Hours
            this->supply12Ah += this->supply12A * timeDeltaHours;

            // Wh = Volts * Amps * Hours
            this->supply12Wh += this->supply12V * this->supply12A * timeDeltaHours;
        }
        this->lastEnergyUpdate_ = now;
        this->energyUpdateInitialized_ = true;
    }

    int PinsBoxMiniDevice::GetNumPowerPorts(void) const
    {
        return PINSBOXMINI_NUM_POWER_PORTS;
    }

    int PinsBoxMiniDevice::GetNumUSBPorts(void) const
    {
        return PINSBOXMINI_NUM_USB_PORTS;
    }

    int PinsBoxMiniDevice::GetNumDewPorts(void) const
    {
        return PINSBOXMINI_NUM_DEW_PORTS;
    }

    int PinsBoxMiniDevice::GetNumBuckPorts(void) const
    {
        return PINSBOXMINI_NUM_BUCK_PORTS;
    }

    int PinsBoxMiniDevice::GetNumPWMPorts(void) const
    {
        return PINSBOXMINI_NUM_PWM_PORTS;
    }

    int PinsBoxMiniDevice::GetPowerState(int i)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_POWER_PORTS) {
            return 0;
        }

        std::lock_guard<std::mutex> lock(this->hwMutex_);

        uint8_t state = this->pwr_[i]->getState();
        this->powerState[i] = state;
        return state;
    }

    bool PinsBoxMiniDevice::SetPowerState(int i, int state)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_POWER_PORTS) {
            return false;
        }

        std::lock_guard<std::mutex> lock(this->hwMutex_);

        // Report the state the port is actually in, a failed switch must not
        // show up as success in the API
        bool ok = this->pwr_[i]->setState(state);
        this->powerState[i] = this->pwr_[i]->getState();
        return ok;
    }

    bool PinsBoxMiniDevice::SetPowerBootState(int i, int state)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_POWER_PORTS) {
            return false;
        }

        this->powerBootstrap[i] = state;
        SaveSettings();
        return true;
    }

    int PinsBoxMiniDevice::GetDewState(int i)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_DEW_PORTS) {
            return 0;
        }

        std::lock_guard<std::mutex> lock(this->hwMutex_);

        uint8_t state = this->dew_[i]->getState();
        this->dewState[i] = state;
        this->dewPWM[i] = this->dew_[i]->getDutyCycle();
        return state;
    }

    bool PinsBoxMiniDevice::SetDewState(int i, int state, int power)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_DEW_PORTS) {
            return false;
        }

        std::lock_guard<std::mutex> lock(this->hwMutex_);

        // On auto mode, do nothing
        if (dewAuto[i]) {
            return false;
        }

        this->dew_[i]->setState(state, power);

        // Report what the hardware really does
        this->dewState[i] = this->dew_[i]->getState();
        this->dewPWM[i] = this->dew_[i]->getDutyCycle();

        if (state != 0 && this->dewState[i] == 0)
        {
            PB_ERROR("Dew port %d: could not be switched on%s", i, this->dew_[i]->isPwmOk() ? "" : " (PWM not initialized)");
            return false;
        }
        return true;
    }

    bool PinsBoxMiniDevice::SetDewAutoMode(int i, int state)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_DEW_PORTS) {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->dewAuto[i] = state;
            this->dew_[i]->setAutoMode(state);
            if (state && !this->dew_[i]->isPwmOk())
            {
                // No error return: the rest of the dew configuration must still be applied
                PB_ERROR("Dew port %d: auto mode set, but the PWM is not initialized, the heater stays off", i);
            }
        }
        SaveSettings();
        return true;
    }

    bool PinsBoxMiniDevice::SetDewAutoThreshold(int i, float val)
    {
        if (i < 0 || i >= PINSBOXMINI_NUM_DEW_PORTS) {
            return false;
        }

        this->dewThreshold[i] = val;
        SaveSettings();
        return true;
    }

    int PinsBoxMiniDevice::GetPWMState(void)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        uint8_t state = this->pwm_->getState();
        this->pwmState = state;
        this->pwmPWM = this->pwm_->getDutyCycle();
        return state;
    }

    bool PinsBoxMiniDevice::SetPWMState(int state, int power)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        this->pwm_->setState(state, power);

        // Report what the hardware really does (power is not range checked)
        this->pwmState = this->pwm_->getState();
        this->pwmPWM = this->pwm_->getDutyCycle();

        if (state != 0 && this->pwmState == 0)
        {
            PB_ERROR("PWM port: could not be switched on%s", this->pwm_->isPwmOk() ? "" : " (PWM not initialized)");
            return false;
        }
        return true;
    }

    PB_ERROR_TYPE PinsBoxMiniDevice::Beep(int volume, int duration_ms)
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

        // FUET-7525 sweet spot is 2700 Hz
        const unsigned int BUZZER_FREQ_HZ = 2700;
        const unsigned int PERIOD_NS = 1000000000 / BUZZER_FREQ_HZ;

        // Set period if not already set
        if (this->buzzer_->getPeriod() != PERIOD_NS)
        {
            this->buzzer_->setPeriod(PERIOD_NS);
        }

        // Calculate duty cycle based on volume (0-100%)
        unsigned int duty_ns = (PERIOD_NS * volume) / 100;
        this->buzzer_->setDutyCycle(duty_ns);

        // Enable PWM
        this->buzzer_->setState(true);

        // Sleep for the specified duration
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));

        // Disable PWM
        this->buzzer_->setState(false);

        return PB_SUCCESS;
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

    bool ScanPinsBoxMini(int *ids)
    {
        // Check if this is a Raspberry Pi with the mini-device marker present
        std::error_code fsError;
        if(!ids || !IsRPI() || !std::filesystem::exists("/etc/pinsMiniDevice", fsError))
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        PB_DEBUG("Valid device found!");

        // Check if device already exists, reuse it instead of creating a new one
        if(g_devices.find(0) == g_devices.end())
        {
            auto device = std::make_shared<PinsBoxMiniDevice>();
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
