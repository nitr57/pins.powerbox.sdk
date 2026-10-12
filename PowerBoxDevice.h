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

#ifndef POWER_BOX_DEVICE_H
#define POWER_BOX_DEVICE_H

#include "PowerBoxSDK.h"
#include "PowerBoxSerialPort.h"
#include "Device.h"
#include <condition_variable>
#include <atomic>
#include <thread>

/* Handshake retry configuration */
#define HANDSHAKE_MAX_RETRIES 5
#define HANDSHAKE_RETRY_DELAY_MS 200
#define HANDSHAKE_TIMEOUT 1000

#define POWERBOX_NUM_POWER_PORTS 6
#define POWERBOX_NUM_USB_PORTS 6
#define POWERBOX_NUM_DEW_PORTS 2
#define POWERBOX_NUM_BUCK_PORTS 1
#define POWERBOX_NUM_PWM_PORTS 1

namespace PowerBox
{
    class PowerBoxDevice : public Device
    {
        public:
            PowerBoxDevice(std::shared_ptr<SerialPort> port, std::string portName);
            virtual ~PowerBoxDevice(void) {
                statusListenerRunning = false;
                if(statusListenerThread_.joinable()) {
                    statusListenerThread_.join();
                }
            }

            virtual PB_ERROR_TYPE Open(void);
            virtual void Close(void);

            virtual std::string GetSerial(void) { return this->serial; }

            virtual int GetUpTime(void) { return this->upTime; }
            virtual float GetTemperature(void) { return this->temperature; }
            virtual float GetHumidity(void) { return this->humidity; }
            virtual float GetDewPoint(void) { return this->dewPoint; }
            virtual int GetExtSensor(void) { return this->extSensor; }
            virtual int HasWiFi(void) { return 1; }

            virtual float GetSupply12V(void) { return this->supply12V; }
            virtual float GetSupply5V(void) { return this->supply5V; }
            virtual float GetSupply12A(void) { return this->supply12A; }
            virtual float GetSupply12Ah(void) { return this->supply12Ah; }
            virtual float GetSupply12Wh(void) { return this->supply12Wh; }

            virtual float GetTemperatureOffset() { return this->temperatureOffset; }
            virtual bool SetTemperatureOffset(float val);
            virtual bool SetExtTemperature(float val);
            virtual float GetHumidityOffset() { return this->humidityOffset; }
            virtual bool SetHumidityOffset(float val);
            virtual bool SetExtHumidity(float val);
            virtual int GetEnvUpdateRate() { return this->envUpdateRate; }
            virtual bool SetEnvUpdateRate(int val);
            virtual int GetUpdateRate() { return this->updateRate; }
            virtual bool SetUpdateRate(int val);

            virtual int GetNumPowerPorts(void) const { return POWERBOX_NUM_POWER_PORTS; }
            virtual int GetNumUSBPorts(void) const { return POWERBOX_NUM_USB_PORTS; }
            virtual int GetNumDewPorts(void) const { return POWERBOX_NUM_DEW_PORTS; }
            virtual int GetNumBuckPorts(void) const { return POWERBOX_NUM_BUCK_PORTS; }
            virtual int GetNumPWMPorts(void) const { return POWERBOX_NUM_PWM_PORTS; }

            virtual int GetPowerState(int i) { return this->powerState[i]; }
            virtual bool SetPowerState(int i, int state);
            virtual int GetPowerBootState(int i) { return this->powerBootstrap[i]; }
            virtual bool SetPowerBootState(int i, int state);
            virtual bool ResetPowerOvercurrent(int i);

            virtual int GetUSBState(int i) { return this->usbState[i]; }
            virtual bool SetUSBState(int i, int state);
            virtual int GetUSBBootState(int i) { return this->usbBootstrap[i]; }
            virtual bool SetUSBBootState(int i, int state);
            virtual bool ResetUSBOvercurrent(int i);

            virtual int GetDewState(int i) { return this->dewState[i]; }
            virtual bool SetDewState(int i, int state, int power);
            virtual int GetDewAutoMode(int i) { return this->dewAuto[i]; }
            virtual bool SetDewAutoMode(int i, int state);
            virtual float GetDewAutoThreshold(int i) { return this->dewThreshold[i]; }
            virtual bool SetDewAutoThreshold(int i, float val);
            virtual int GetDewPWMPower(int i) { return this->dewPWM[i]; }
            virtual bool ResetDewOvercurrent(int i);

            virtual int GetBuckState() { return this->buckState; }
            virtual bool SetBuckState(int state, float target);
            virtual int GetBuckBootState() { return this->buckBootstrap; }
            virtual bool SetBuckBootState(int state);
            virtual float GetBuckSetVoltage() { return this->buckVset; }
            virtual bool ResetBuckOvercurrent(void);

            virtual int GetPWMState(void) { return this->pwmState; }
            virtual bool SetPWMState(int state, int power);
            virtual bool GetPWMPower(void) { return this->pwmPWM; }
            virtual bool ResetPWMOvercurrent(void);

            virtual PB_ERROR_TYPE ScanWiFi(PB_WIFI_SCAN_RESULT *result);
            virtual PB_ERROR_TYPE GetWiFiConfig(PB_WIFI_CONFIG *config);
            virtual PB_ERROR_TYPE SetWiFiConfig(const PB_WIFI_CONFIG *config);
            virtual PB_ERROR_TYPE GetWiFiStatus(PB_WIFI_STATUS *status);

            virtual int GetFirmwareVersion() { return this->firmwareVersion; }
            virtual std::string GetModelType() { return this->modelType; }
            virtual std::string GetUUID() { return this->uuid; }
            std::string GetPortName() { return this->portName; }

            virtual PB_ERROR_TYPE Restart(void);
            virtual PB_ERROR_TYPE FactoryReset(void);

            virtual void StartStatusListener(void);
            virtual void StopStatusListener(void);

            bool SendCommand(const char *command, int timeoutMs = 3000);
            bool SendAndWaitForReply(const char *command,
                                     std::mutex &configMutex,
                                     std::condition_variable &configCV,
                                     std::atomic<bool> &configPending,
                                     const char *timeoutMsg,
                                     int timeoutMs = 1000);
            bool SendAndWaitForReplyWithRetry(const char *command,
                                              std::mutex &configMutex,
                                              std::condition_variable &configCV,
                                              std::atomic<bool> &configPending,
                                              const char *timeoutMsg,
                                              int timeoutMs = 1000,
                                              int maxRetries = HANDSHAKE_MAX_RETRIES,
                                              int retryDelayMs = HANDSHAKE_RETRY_DELAY_MS);

        private:
            friend PB_ERROR_TYPE ScanPowerBox(int *number, int *ids);
#ifdef __unix__
            friend std::shared_ptr<PowerBoxDevice> ProbePowerBox(const std::string &portName);
#endif

        private:
            void ParseHandshakeMessage(const char *buffer);
            void ParseUptimeMessage(const char *buffer);
            void ParseEnvironmentMessage(const char *buffer);
            void ParseEnvModelMessage(const char *buffer);
            void ParseUpdateRateMessage(const char *buffer);
            void ParsePowerSupplyMessage(const char *buffer);
            void ParsePowerOutputMessage(const char *buffer);
            void ParsePowerOutputConfigMessage(const char *buffer);
            void ParseUSBMessage(const char *buffer);
            void ParseUSBConfigMessage(const char *buffer);
            void ParseDewMessage(const char *buffer);
            void ParseDewConfigMessage(const char *buffer);
            void ParseAdjMessage(const char *buffer);
            void ParseAdjConfigMessage(const char *buffer);
            void ParseWiFiInfoMessage(const char *buffer);
            void ParseWiFiSurveyMessage(const char *buffer);

        protected:
            virtual void StatusListenerThreadFunc();
            /* These synchronization members are protected to allow access from friend functions */
            std::mutex handshakeMutex;
            std::condition_variable handshakeCV;
            std::atomic<bool> handshakePending{false};

        private:
            std::shared_ptr<SerialPort> serialPort;
            std::string portName;
            std::string uuid;
            int firmwareVersion;

            // WiFi scan results
            PB_WIFI_SCAN_RESULT wifiScanResult = {0};
            std::mutex wifiScanMutex;
            std::condition_variable wifiScanCV;
            std::atomic<bool> wifiScanPending{false};

            // WiFi current status
            int wifiMode = 0;                         /* Current WiFi mode (0 = AP, 1 = Client) */
            int wifiChannel = 0;                      /* Current WiFi channel */
            char wifiSSID[PB_SSID_LEN] = {0};         /* Current WiFi SSID */
            char wifiIP[PB_IP_LEN] = {0};             /* Device IP address */
            int wifiRSSI = 0;                         /* WiFi signal strength */
            char wifiHostname[PB_HOSTNAME_LEN] = {0}; /* Device hostname */
            std::mutex wifiInfoMutex;
            std::condition_variable wifiInfoCV;
            std::atomic<bool> wifiInfoPending{false};

            /* Config mutexes etc */
            std::mutex envModelMutex;
            std::mutex updateRateMutex;
            std::mutex pwrConfigMutex;
            std::mutex usbConfigMutex;
            std::mutex dewConfigMutex;
            std::mutex adjConfigMutex;
            std::condition_variable envModelCV;
            std::condition_variable updateRateCV;
            std::condition_variable pwrConfigCV;
            std::condition_variable usbConfigCV;
            std::condition_variable dewConfigCV;
            std::condition_variable adjConfigCV;
            std::atomic<bool> envModelPending{false};
            std::atomic<bool> updateRatePending{false};
            std::atomic<bool> pwrConfigPending{false};
            std::atomic<bool> usbConfigPending{false};
            std::atomic<bool> dewConfigPending{false};
            std::atomic<bool> adjConfigPending{false};
            std::atomic<bool> statusListenerRunning{false};
            std::thread statusListenerThread_;
    };

    PB_ERROR_TYPE ScanPowerBox(int *number, int *ids);
#ifdef __unix__
    std::shared_ptr<PowerBoxDevice> ProbePowerBox(const std::string &portName);
    PB_ERROR_TYPE ScanPowerBoxPort(const char *port, int *id);
    PB_ERROR_TYPE GetPowerBoxPort(int id, char *port);
#endif

} /* namespace PowerBox */

#endif /* POWER_BOX_DEVICE_H */
