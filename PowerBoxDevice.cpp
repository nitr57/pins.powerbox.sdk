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

#include "PowerBoxDevice.h"
#include "PowerBoxSDK.h"
#include "PowerBoxLogging.h"
#include "PowerBoxSerialPort.h"
#include "arduino_base64.hpp"
#include <cstring>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <atomic>
#ifdef __unix__
#include <climits>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <libudev.h>
#elif defined(_WIN32)
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#pragma comment(lib, "setupapi.lib")
#endif

namespace PowerBox
{
    PowerBoxDevice::PowerBoxDevice(std::shared_ptr<SerialPort> port, std::string portName)
    {
        this->serialPort = port;
        this->portName = portName;
        this->statusListenerRunning = false;

        // Power port 0 is read only
        this->powerReadOnly[0] = true;
    }

    PB_ERROR_TYPE PowerBoxDevice::Open(void)
    {
        PB_DEBUG("PBOpen: Found device, portName=%s", this->portName.c_str());

        /* Create a new SerialPort instance and open it */
        if (!this->serialPort)
        {
            PB_DEBUG("PBOpen: Creating new SerialPort instance");
            this->serialPort = std::make_shared<SerialPort>();
        }

        PB_DEBUG("PBOpen: Attempting to open port %s", this->portName.c_str());
        if (!this->serialPort->Open(this->portName.c_str()))
        {
            PB_ERROR("PBOpen: Failed to open port");
            return PB_ERROR_COMMUNICATION;
        }

        PB_DEBUG("PBOpen: Port opened successfully, performing handshake");

        // Send HS to wake up device
        this->SendCommand(":HS#", 200);

        // Start status listener thread
        this->StartStatusListener();

        // Perform handshake with retry mechanism
        if(!this->SendAndWaitForReplyWithRetry(":HS#", this->handshakeMutex, this->handshakeCV,
                                         this->handshakePending, "handshake"))
        {
            PB_ERROR("PBOpen: Handshake failed after retries");
            this->serialPort->Close();
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch ENV config
        if(!this->SendAndWaitForReply(":GES#", this->envModelMutex, this->envModelCV,
                                this->envModelPending, "environment model"))
        {
            PB_ERROR("PBOpen: Failed to fetch environment model");
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch update rate
        if(!this->SendAndWaitForReply(":GUR#", this->updateRateMutex, this->updateRateCV,
                                this->updateRatePending, "update rate"))
        {
            PB_ERROR("PBOpen: Failed to fetch update rate");
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch PWR config
        if(!this->SendAndWaitForReply(":GPS#", this->pwrConfigMutex, this->pwrConfigCV,
                                this->pwrConfigPending, "power port config"))
        {
            PB_ERROR("PBOpen: Failed to fetch power port config");
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch USB config
        if(!this->SendAndWaitForReply(":GUS#", this->usbConfigMutex, this->usbConfigCV, 
                                this->usbConfigPending, "usb port config"))
        {
            PB_ERROR("PBOpen: Failed to fetch usb port config");
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch DEW config
        if(!this->SendAndWaitForReply(":GDS#", this->dewConfigMutex, this->dewConfigCV,
                                this->dewConfigPending, "dew port config"))
        {
            PB_ERROR("PBOpen: Failed to fetch dew port config");
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch ADJ config
        if(!this->SendAndWaitForReply(":GAS#", this->adjConfigMutex, this->adjConfigCV,
                                this->adjConfigPending, "adj port config"))
        {
            PB_ERROR("PBOpen: Failed to fetch adj port config");
            return PB_ERROR_COMMUNICATION;
        }

        // Fetch WiFi info
        if(!this->SendAndWaitForReply(":GWI#", this->wifiInfoMutex, this->wifiInfoCV,
                                this->wifiInfoPending, "WiFi info"))
        {
            PB_ERROR("PBOpen: Failed to fetch WiFi info");
            return PB_ERROR_COMMUNICATION;
        }

        // Start telemetry
        if (!this->SendCommand(":BS#"))
        {
            PB_ERROR("PBOpen: Failed to start telemetry");
            this->StopStatusListener();
            this->serialPort->Close();
            return PB_ERROR_COMMUNICATION;
        }

        this->isOpen = true;
        PB_INFO("[OK] Device opened");
        return PB_SUCCESS;
    }

    void PowerBoxDevice::Close(void)
    {
        // Stop telemetry
        if (this->serialPort && this->serialPort->IsOpen())
        {
            this->SendCommand(":ES#");
        }

        this->StopStatusListener();

        if (this->serialPort)
        {
            this->serialPort->Close();
        }

        PB_INFO("[OK] Device closed");
        this->isOpen = false;
    }

    bool PowerBoxDevice::SendCommand(const char *command, int timeoutMs)
    {
        if(!this->serialPort->IsOpen())
        {
            PB_DEBUG("SendCommand: device=%p, port=%p, isOpen=%d",
                     this, this->serialPort.get(),
                     this->serialPort->IsOpen());
            return false;
        }

        PB_DEBUG("SendCommand: Writing '%s'", command);
        if (!this->serialPort->Write((const unsigned char *)command, strlen(command)))
        {
            PB_DEBUG("SendCommand: Write failed");
            return false;
        }

        return true;
    }

    bool PowerBoxDevice::SendAndWaitForReply(const char *command,
                                             std::mutex &configMutex,
                                             std::condition_variable &configCV,
                                             std::atomic<bool> &configPending,
                                             const char *timeoutMsg,
                                             int timeoutMs)
    {
        {
            std::lock_guard<std::mutex> lock(configMutex);
            configPending = true;
        }

        if (!this->serialPort->Write((const unsigned char *)command, strlen(command)))
        {
            PB_DEBUG("SendAndWaitForReply: Failed to send %s command", command);
            std::lock_guard<std::mutex> lock(configMutex);
            configPending = false;
            return false;
        }

        /* Wait for config to be received with specified timeout */
        {
            std::unique_lock<std::mutex> lock(configMutex);
            configCV.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [&configPending]() { return !configPending; });
            if (configPending)
            {
                PB_DEBUG("SendAndWaitForReply: Timeout waiting for %s (timeout=%dms)", timeoutMsg, timeoutMs);
                configPending = false;
                return false;
            }
        }

        return true;
    }

    bool PowerBoxDevice::SendAndWaitForReplyWithRetry(const char *command,
                                                      std::mutex &configMutex,
                                                      std::condition_variable& configCV,
                                                      std::atomic<bool> &configPending,
                                                      const char *timeoutMsg,
                                                      int timeoutMs,
                                                      int maxRetries,
                                                      int retryDelayMs)
    {
        for (int attempt = 1; attempt <= maxRetries; ++attempt)
        {
            PB_DEBUG("SendAndWaitForReplyWithRetry: Attempt %d/%d for %s (timeout=%dms)", attempt, maxRetries, timeoutMsg, timeoutMs);

            if (this->SendAndWaitForReply(command, configMutex, configCV, configPending, timeoutMsg, timeoutMs))
            {
                PB_DEBUG("SendAndWaitForReplyWithRetry: Success on attempt %d for %s", attempt, timeoutMsg);
                return true;
            }

            if (attempt < maxRetries)
            {
                PB_DEBUG("SendAndWaitForReplyWithRetry: Failed on attempt %d, retrying after %d ms", attempt, retryDelayMs);
                std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs));
            }
        }

        PB_DEBUG("SendAndWaitForReplyWithRetry: All %d attempts failed for %s", maxRetries, timeoutMsg);
        return false;
    }

    void PowerBoxDevice::StartStatusListener(void)
    {
        /* Stop any existing listener by setting the flag */
        this->statusListenerRunning = false;

        /* Wait for old thread to exit if it's still running */
        if(this->statusListenerThread_.joinable()) {
            this->statusListenerThread_.join();
        }

        /* Start new listener thread */
        this->statusListenerRunning = true;
        this->statusListenerThread_ = std::thread(&PowerBoxDevice::StatusListenerThreadFunc, this);
        PB_DEBUG("StartStatusListener: Listener thread started");
    }

    void PowerBoxDevice::StopStatusListener(void)
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
    void PowerBoxDevice::StatusListenerThreadFunc()
    {
        char buffer[256];

        while(this->statusListenerRunning)
        {
            if (!this->serialPort)
            {
                PB_DEBUG("StatusListener: Port unavailable, exiting");
                this->statusListenerRunning = false;
                return;
            }

            if (!this->serialPort->IsOpen())
            {
                PB_DEBUG("StatusListener: Port not open, exiting");
                this->statusListenerRunning = false;
                return;
            }

            if (this->serialPort->Read((unsigned char *)buffer, 256, '#', 5000))
            {
                /* Parse different message types based on prefix */
                if (strstr(buffer, "PINS:") == buffer)
                {
                    /* Handshake message */
                    this->ParseHandshakeMessage(buffer);
                }
                else if (strstr(buffer, "UP:") == buffer)
                {
                    /* Uptime status */
                    this->ParseUptimeMessage(buffer);
                }
                else if (strstr(buffer, "ENVMODEL:") == buffer)
                {
                    /* Environment model */
                    this->ParseEnvModelMessage(buffer);
                }
                else if (strstr(buffer, "UDR:") == buffer)
                {
                    /* Update rate */
                    this->ParseUpdateRateMessage(buffer);
                }
                else if (strstr(buffer, "POUTS:") == buffer)
                {
                    /* Power port config */
                    this->ParsePowerOutputConfigMessage(buffer);
                }
                else if (strstr(buffer, "USBS:") == buffer)
                {
                    /* USB port config */
                    this->ParseUSBConfigMessage(buffer);
                }
                else if (strstr(buffer, "DEWS:") == buffer)
                {
                    /* Dew port config */
                    this->ParseDewConfigMessage(buffer);
                }
                else if (strstr(buffer, "ADJS:") == buffer)
                {
                    /* Adj port config */
                    this->ParseAdjConfigMessage(buffer);
                }
                else if (strstr(buffer, "ENV:") == buffer)
                {
                    /* Environment status */
                    this->ParseEnvironmentMessage(buffer);
                }
                else if (strstr(buffer, "PSUP:") == buffer)
                {
                    /* Power supply status */
                    this->ParsePowerSupplyMessage(buffer);
                }
                else if (strstr(buffer, "POUT:") == buffer)
                {
                    /* Power port status */
                    this->ParsePowerOutputMessage(buffer);
                }
                else if (strstr(buffer, "USB:") == buffer)
                {
                    /* USB port status */
                    this->ParseUSBMessage(buffer);
                }
                else if (strstr(buffer, "DEW:") == buffer)
                {
                    /* Dew port status */
                    this->ParseDewMessage(buffer);
                }
                else if (strstr(buffer, "ADJ:") == buffer)
                {
                    /* Adj port status */
                    this->ParseAdjMessage(buffer);
                }
                else if (strstr(buffer, "WIFI:") == buffer)
                {
                    /* Wifi status */
                    this->ParseWiFiInfoMessage(buffer);
                }
                else if (strstr(buffer, "OC:") == buffer)
                {
                    /* Over current shutdown done by the firmware */
                    this->ParseOvercurrentMessage(buffer);
                }
                else if (strstr(buffer, "WSURV:") == buffer)
                {
                    /* Wifi survey */
                    this->ParseWiFiSurveyMessage(buffer);
                }
            }
        }

        PB_DEBUG("StatusListener: exiting");
    }

    /* The firmware switches a port off on over current and reports it as
     * OC:<type>:<port>#, with the port types from the firmware's def.h. The
     * cached state is only refreshed on request otherwise, so update it here */
    void PowerBoxDevice::ParseOvercurrentMessage(const char *buffer)
    {
        static const int OC_PORT_PWR = 0x01;
        static const int OC_PORT_USB = 0x02;
        static const int OC_PORT_DEW = 0x04;
        static const int OC_PORT_ADJ = 0x08;

        int type, port;
        if (sscanf(buffer, "OC:%d:%d#", &type, &port) != 2 || port < 0)
        {
            return;
        }

        PB_INFO("Over current shutdown reported (type %d, port %d)", type, port);

        switch (type)
        {
            case OC_PORT_PWR:
                if (port < this->GetNumPowerPorts())
                {
                    this->powerState[port] = 0;
                    this->powerOvercurrent[port] = 1;
                }
                break;
            case OC_PORT_USB:
                if (port < this->GetNumUSBPorts())
                {
                    this->usbState[port] = 0;
                    this->usbOvercurrent[port] = 1;
                }
                break;
            case OC_PORT_DEW:
                if (port < this->GetNumDewPorts())
                {
                    this->dewState[port] = 0;
                    this->dewOvercurrent[port] = 1;
                }
                break;
            case OC_PORT_ADJ:
                /* ADJ port 0 is the buck converter, port 1 the PWM port */
                if (port == 0)
                {
                    this->buckState = 0;
                    this->buckOvercurrent = 1;
                }
                else if (port == 1)
                {
                    this->pwmState = 0;
                    this->pwmOvercurrent = 1;
                }
                break;
        }
    }

    void PowerBoxDevice::ParseHandshakeMessage(const char *buffer)
    {
        char model[33];
        char uuid[41];
        char serial[33];
        int firmware;
        if (sscanf(buffer, "PINS:%32[^:]:%40[^:]:%32[^:]:%d#",
                   model, uuid, serial, &firmware) == 4)
        {
            // Store in device
            this->modelType = model;
            this->uuid = uuid;
            this->serial = serial;
            this->firmwareVersion = firmware;
            this->handshakePending = false;
        }

        this->handshakeCV.notify_one();
    }

    void PowerBoxDevice::ParseUptimeMessage(const char *buffer)
    {
        sscanf(buffer, "UP:%d#", &this->upTime);
    }

    void PowerBoxDevice::ParseEnvironmentMessage(const char *buffer)
    {
        sscanf(buffer, "ENV:%f:%f:%f:%d#", &this->temperature, &this->humidity, &this->dewPoint, &this->extSensor);
    }

    void PowerBoxDevice::ParseEnvModelMessage(const char *buffer)
    {
        float tempOffset, humOffset;
        int envUpdate;
        if (sscanf(buffer, "ENVMODEL:%f:%f:%d#", &tempOffset, &humOffset, &envUpdate) == 3)
        {
            // Store in device
            {
                this->temperatureOffset = tempOffset;
                this->humidityOffset = humOffset;
                this->envUpdateRate = envUpdate;
                this->envModelPending = false;
            }

            this->envModelCV.notify_one();
        }
    }

    void PowerBoxDevice::ParseUpdateRateMessage(const char *buffer)
    {
        int rate;
        if (sscanf(buffer, "UDR:%d#", &rate) == 1)
        {
            // Store in device
            {
                this->updateRate = rate;
                this->updateRatePending = false;
            }

            this->updateRateCV.notify_one();
        }
    }

    void PowerBoxDevice::ParsePowerSupplyMessage(const char *buffer)
    {
        int mV12, mA12, mV5;
        float mAh12, mWh12;
        sscanf(buffer, "PSUP:%d:%d:%d:%f:%f#", &mV12, &mA12, &mV5, &mAh12, &mWh12);
        this->supply12V = mV12 / 1000.0f;
        this->supply12A = mA12 / 1000.0f;
        this->supply5V = mV5 / 1000.0f;
        this->supply12Ah = mAh12 / 1000.0f;
        this->supply12Wh = mWh12 / 1000.0f;
    }

    void PowerBoxDevice::ParsePowerOutputMessage(const char *buffer)
    {
        int mA[6];
        sscanf(buffer, "POUT:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
               &mA[0], &this->powerOvercurrent[0],
               &mA[1], &this->powerOvercurrent[1],
               &mA[2], &this->powerOvercurrent[2],
               &mA[3], &this->powerOvercurrent[3],
               &mA[4], &this->powerOvercurrent[4],
               &mA[5], &this->powerOvercurrent[5]);
        for (int i = 0; i < 6; ++i)
        {
            this->powerCurrent[i] = mA[i] / 1000.0f;
        }
    }

    void PowerBoxDevice::ParsePowerOutputConfigMessage(const char *buffer)
    {
        int state[6], boot[6];
        if(sscanf(buffer, "POUTS:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
               &state[0], &boot[0], &state[1], &boot[1], &state[2], &boot[2],
               &state[3], &boot[3], &state[4], &boot[4], &state[5], &boot[5]) == 12)
        {
            // Store in device
            {
                for (int i = 0; i < 6; ++i)
                {
                    this->powerState[i] = state[i];
                    this->powerBootstrap[i] = boot[i];
                }
                this->pwrConfigPending = false;
            }

            this->pwrConfigCV.notify_one();
        }
    }

    void PowerBoxDevice::ParseUSBMessage(const char *buffer)
    {
        int usbV[6];
        int usbA[6];
        sscanf(buffer, "USB:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
                &usbA[0], &usbV[0], &this->usbOvercurrent[0],
                &usbA[1], &usbV[1], &this->usbOvercurrent[1],
                &usbA[2], &usbV[2], &this->usbOvercurrent[2],
                &usbA[3], &usbV[3], &this->usbOvercurrent[3],
                &usbA[4], &usbV[4], &this->usbOvercurrent[4],
                &usbA[5], &usbV[5], &this->usbOvercurrent[5]);
        for (int i = 0; i < 6; ++i)
        {
            this->usbCurrent[i] = usbA[i] / 1000.0f;
            this->usbVoltage[i] = usbV[i] / 1000.0f;
        }
    }

    void PowerBoxDevice::ParseUSBConfigMessage(const char *buffer)
    {
        int state[6], boot[6];
        if(sscanf(buffer, "USBS:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
               &state[0], &boot[0], &state[1], &boot[1], &state[2], &boot[2],
               &state[3], &boot[3], &state[4], &boot[4], &state[5], &boot[5]) == 12)
        {
            // Store in device
            {
                for (int i = 0; i < 6; ++i)
                {
                    this->usbState[i] = state[i];
                    this->usbBootstrap[i] = boot[i];
                }
                this->usbConfigPending = false;
            }

            this->usbConfigCV.notify_one();
        }
    }

    void PowerBoxDevice::ParseDewMessage(const char *buffer)
    {
        int dewA[2];
        sscanf(buffer, "DEW:%d:%f:%d:%d:%d:%d:%f:%d:%d:%d#",
               &dewA[0], &this->dewProbe[0], &this->dewPWM[0], &this->dewState[0], &this->dewOvercurrent[0],
               &dewA[1], &this->dewProbe[1], &this->dewPWM[1], &this->dewState[1], &this->dewOvercurrent[1]);
        for (int i = 0; i < 2; ++i)
        {
            this->dewCurrent[i] = dewA[i] / 1000.0f;
        }
    }

    void PowerBoxDevice::ParseDewConfigMessage(const char *buffer)
    {
        int pwmres, auto0, auto1, state0, state1;
        float thres0, thres1;
        if(sscanf(buffer, "DEWS:%d:%f:%d:%d:%f:%d:%d#",
                  &pwmres,
                  &thres0, &auto0, &state0,
                  &thres1, &auto1, &state1) == 7)
        {
            // Store in device
            {
                std::lock_guard<std::mutex> lock(this->dewConfigMutex);
                this->dewPwmResolution = pwmres;
                this->dewThreshold[0] = thres0;
                this->dewAuto[0] = auto0;
                this->dewState[0] = state0;
                this->dewThreshold[1] = thres1;
                this->dewAuto[1] = auto1;
                this->dewState[1] = state1;
                this->dewConfigPending = false;
            }

            this->dewConfigCV.notify_one();
        }
    }

    void PowerBoxDevice::ParseAdjMessage(const char *buffer)
    {
        int adjA[2], Vadc, Vmin, Vmax, Vset;
        sscanf(buffer, "ADJ:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
               &adjA[0], &Vset, &Vadc, &Vmin, &Vmax, &this->buckOvercurrent,
               &adjA[1], &this->pwmPWM, &this->pwmOvercurrent);
        this->buckCurrent = adjA[0] / 1000.0f;
        this->buckVoltage = Vadc / 1000.0f;
        this->buckVmin = Vmin / 1000.0f;
        this->buckVmax = Vmax / 1000.0f;
        this->buckVset = Vset / 1000.0f;
        this->pwmCurrent = adjA[1] / 1000.0f;
    }

    void PowerBoxDevice::ParseAdjConfigMessage(const char *buffer)
    {
        int bootstate, state, pwmstate, pwmres;
        if (sscanf(buffer, "ADJS:%d:%d:%d:%d#", &bootstate, &state, &pwmstate, &pwmres) == 4)
        {
            // Store in device
            {
                std::lock_guard<std::mutex> lock(this->adjConfigMutex);
                this->buckBootstrap = bootstate;
                this->buckState = state;
                this->pwmState = pwmstate;
                this->pwmPwmResolution = pwmres;
                this->adjConfigPending = false;
            }

            this->adjConfigCV.notify_one();
        }
    }

    void PowerBoxDevice::ParseWiFiInfoMessage(const char *buffer)
    {
        int mode, channel, rssi, hostnameLen;
        char ssid[PB_SSID_LEN] = {0};
        char ip[PB_IP_LEN] = {0};
        char hostname[PB_HOSTNAME_LEN] = {0};

        if (sscanf(buffer, "WIFI:%d:%d:%31[^:]:%15[^:]:%d:%d:", 
                   &mode, &channel, ssid, ip, &rssi, &hostnameLen) == 6)
        {
            // Find the hostname part (after the 7th colon)
            const char *hostStart = buffer;
            int colonCount = 0;
            while (*hostStart && colonCount < 7)
            {
                if (*hostStart == ':')
                {
                    colonCount++;
                }
                hostStart++;
            }

            // Copy hostname
            if (hostnameLen > 0 && hostnameLen < PB_HOSTNAME_LEN)
            {
                strncpy(hostname, hostStart, hostnameLen);
                hostname[hostnameLen] = '\0';
            }

            // Store in device
            {
                std::lock_guard<std::mutex> lock(this->wifiInfoMutex);
                this->wifiMode = mode;
                this->wifiChannel = channel;
                strncpy(this->wifiSSID, ssid, PB_SSID_LEN - 1);
                this->wifiSSID[PB_SSID_LEN - 1] = '\0';
                strncpy(this->wifiIP, ip, PB_IP_LEN - 1);
                this->wifiIP[PB_IP_LEN - 1] = '\0';
                this->wifiRSSI = rssi;
                strncpy(this->wifiHostname, hostname, PB_HOSTNAME_LEN - 1);
                this->wifiHostname[PB_HOSTNAME_LEN - 1] = '\0';
                this->wifiInfoPending = false;
            }
            
            PB_DEBUG("WiFi Info: mode=%d, channel=%d, ssid='%s', ip='%s', rssi=%d, hostname='%s'",
                     mode, channel, ssid, ip, rssi, hostname);
            
            this->wifiInfoCV.notify_one();
        }
    }

    void PowerBoxDevice::ParseWiFiSurveyMessage(const char *buffer)
    {
        int networks;
        if (sscanf(buffer, "WSURV:%d:", &networks) != 1 || networks > PB_MAX_WIFI_NETWORKS)
        {
            return;
        }

        // Find the position after "WSURV:count:"
        char *pos = strchr((char *)buffer + 6, ':'); // Find first ':' after WSURV
        if (!pos)
        {
            return;
        }
        pos++; // Move past the ':'
        
        PB_DEBUG("WiFi Survey: Found %d networks", networks);
        
        // Lock and clear results
        {
            std::lock_guard<std::mutex> lock(this->wifiScanMutex);
            this->wifiScanResult.count = 0;
        }
        
        for (int i = 0; i < networks && i < PB_MAX_WIFI_NETWORKS; i++)
        {
            char ssid[PB_SSID_LEN] = {0};
            int rssi = 0;
            
            // Parse SSID (everything until the next colon)
            int ssidLen = 0;
            while (*pos && *pos != ':' && ssidLen < PB_SSID_LEN - 1)
            {
                ssid[ssidLen++] = *pos++;
            }
            ssid[ssidLen] = '\0';
            
            if (*pos == ':')
            {
                pos++; // Skip the colon
                // Parse RSSI (negative integer)
                sscanf(pos, "%d", &rssi);
                
                // Move to next field
                while (*pos && *pos != ':')
                {
                    pos++;
                }
                if (*pos == ':')
                {
                    pos++;
                }
                
                PB_DEBUG("  Network %d: SSID='%s' RSSI=%d dBm", i + 1, ssid, rssi);
                
                // Store result
                {
                    std::lock_guard<std::mutex> lock(this->wifiScanMutex);
                    strncpy(this->wifiScanResult.networks[i].ssid, ssid, PB_SSID_LEN - 1);
                    this->wifiScanResult.networks[i].ssid[PB_SSID_LEN - 1] = '\0';
                    this->wifiScanResult.networks[i].rssi = rssi;
                    this->wifiScanResult.count = i + 1;
                }
            }
        }
        
        // Signal that scan is complete
        {
            std::lock_guard<std::mutex> lock(this->wifiScanMutex);
            this->wifiScanPending = false;
        }
        this->wifiScanCV.notify_one();
    }    

    PB_ERROR_TYPE PowerBoxDevice::ScanWiFi(PB_WIFI_SCAN_RESULT *result)
    {
        // Clear previous results and mark scan as pending
        {
            std::lock_guard<std::mutex> wifiLock(this->wifiScanMutex);
            this->wifiScanResult.count = 0;
            this->wifiScanPending = true;
        }

        // Ask for wifi survey
        if (this->serialPort && this->serialPort->IsOpen())
        {
            if(!this->SendCommand(":GWS#"))
            {
                PB_ERROR("PBScanWiFi: Failed to start WiFi survey");
                std::lock_guard<std::mutex> wifiLock(this->wifiScanMutex);
                this->wifiScanPending = false;
                return PB_ERROR_COMMUNICATION;
            }
        }
        else
        {
            std::lock_guard<std::mutex> wifiLock(this->wifiScanMutex);
            this->wifiScanPending = false;
            return PB_ERROR_COMMUNICATION;
        }

        // Wait for WiFi scan results (with 10 second timeout)
        {
            std::unique_lock<std::mutex> wifiLock(this->wifiScanMutex);
            const auto timeout = std::chrono::seconds(10);
            bool completed = this->wifiScanCV.wait_for(wifiLock, timeout, [this]() {
                return !this->wifiScanPending;
            });

            if (!completed)
            {
                PB_ERROR("PBScanWiFi: Timeout waiting for WiFi scan results");
                this->wifiScanPending = false;
                return PB_ERROR_TIMEOUT;
            }

            // Copy results to output parameter
            *result = this->wifiScanResult;
        }

        return PB_SUCCESS;
    }

    bool PowerBoxDevice::SetTemperatureOffset(float val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SET%f#", val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->temperatureOffset = val;

        return true;
    }

    bool PowerBoxDevice::SetExtTemperature(float val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SCT%f#", val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->temperature = val;

        return true;
    }

    bool PowerBoxDevice::SetHumidityOffset(float val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SEH%f#", val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->humidityOffset = val;

        return true;
    }

    bool PowerBoxDevice::SetExtHumidity(float val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SCH%f#", val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->humidity = val;

        return true;
    }
    
    bool PowerBoxDevice::SetEnvUpdateRate(int val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SEU%d#", val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->envUpdateRate = val;

        return true;
    }

    bool PowerBoxDevice::SetUpdateRate(int val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SUR%d#", val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->updateRate = val;

        return true;
    }

    bool PowerBoxDevice::SetPowerState(int i, int state)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SPS%d%d#", i, state != 0);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->powerState[i] = state != 0;

        return true;
    }

    bool PowerBoxDevice::SetPowerBootState(int i, int state)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SPB%d%d#", i, state != 0);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->powerBootstrap[i] = state != 0;

        return true;
    }

    bool PowerBoxDevice::ResetPowerOvercurrent(int i)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SPO%d#", i);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        return true;   
    }

    bool PowerBoxDevice::SetUSBState(int i, int state)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SUS%d%d#", i, state != 0);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->usbState[i] = state != 0;

        return true;
    }

    bool PowerBoxDevice::SetUSBBootState(int i, int state)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SUB%d%d#", i, state != 0);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->usbBootstrap[i] = state != 0;

        return true;
    }

    bool PowerBoxDevice::ResetUSBOvercurrent(int i)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SUO%d#", i);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        return true;   
    }

    bool PowerBoxDevice::SetDewState(int i, int state, int power)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SDS%d%d%d#", i, state != 0, power);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->dewState[i] = state != 0;
        this->dewPWM[i] = power;

        return true;
    }

    bool PowerBoxDevice::SetDewAutoMode(int i, int state)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SDB%d%d#", i, state != 0);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->dewAuto[i] = state != 0;

        return true;
    }

    bool PowerBoxDevice::SetDewAutoThreshold(int i, float val)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SDT%d%f#", i, val);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->dewThreshold[i] = val;

        return true;
    }

    bool PowerBoxDevice::ResetDewOvercurrent(int i)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SDO%d#", i);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        return true;
    }

    bool PowerBoxDevice::SetBuckState(int state, float target)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SAS0%d%d#", state != 0, (int)(target * 1000));

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->buckState = state != 0;
        this->buckVset = (int)(target * 1000);

        return true;
    }

    bool PowerBoxDevice::SetBuckBootState(int state)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SAB0%d#", state != 0);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->buckBootstrap = state != 0;

        return true;
    }

    bool PowerBoxDevice::ResetBuckOvercurrent(void)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SAO0#");

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        return true;
    }

    bool PowerBoxDevice::SetPWMState(int state, int power)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SAS1%d%d#", state != 0, power);

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        this->pwmState = state != 0;
        this->pwmPWM = power;

        return true;
    }

    bool PowerBoxDevice::ResetPWMOvercurrent(void)
    {
        // Send command
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":SAO1#");

        if (!this->SendCommand(cmd))
        {
            return false;
        }

        return true;
    }

    PB_ERROR_TYPE PowerBoxDevice::GetWiFiConfig(PB_WIFI_CONFIG *config)
    {
        /* Return cached WiFi info from listener thread */
        {
            std::lock_guard<std::mutex> infoLock(this->wifiInfoMutex);
            config->channel = this->wifiChannel;
            config->mode = this->wifiMode ? PB_WIFI_MODE_CLIENT : PB_WIFI_MODE_AP;
            strncpy(config->ssid, this->wifiSSID, PB_SSID_LEN - 1);
            config->ssid[PB_SSID_LEN - 1] = '\0';
            strncpy(config->hostname, this->wifiHostname, PB_HOSTNAME_LEN - 1);
            config->hostname[PB_HOSTNAME_LEN - 1] = '\0';
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE PowerBoxDevice::SetWiFiConfig(const PB_WIFI_CONFIG *config)
    {
        if (!this->serialPort || !this->serialPort->IsOpen())
        {
            return PB_ERROR_COMMUNICATION;
        }

        // First, query current WiFi info if we need any cached values
        if (!(config->mask & (MASK_WIFI_SSID | MASK_WIFI_PASSWORD | MASK_WIFI_MODE)))
        {
            // No fields to update
            return PB_SUCCESS;
        }

        // Build and send the set WiFi info command
        // Format: :SWI<ssidlen><ssid><passlen><pass><mode>#
        // Always send the full command with cached values for fields not being updated
        
        char cmdBuffer[512];
        int cmdLen = 0;

        // Start command
        cmdLen = snprintf(cmdBuffer, sizeof(cmdBuffer), ":SWI");

        // Add SSID length and SSID
        const char *ssidToUse = (config->mask & MASK_WIFI_SSID) ? config->ssid : this->wifiSSID;
        int ssidLen = strlen(ssidToUse);
        cmdLen += snprintf(cmdBuffer + cmdLen, sizeof(cmdBuffer) - cmdLen, "%d%s", ssidLen, ssidToUse);

        // Add password length and password (AES encrypted and base64 encoded)
        const char *passToUse = (config->mask & MASK_WIFI_PASSWORD) ? config->pass : "";
        
        char encryptedPass[512];
        encryptedPass[0] = '\0';
        size_t encryptedLen = 0;
        
        if (strlen(passToUse) > 0)
        {
            // Encrypt password using device's UUID
            if (base64::EncryptPassword(passToUse, this->uuid.c_str(), encryptedPass))
            {
                encryptedLen = strlen(encryptedPass);
                PB_DEBUG("WiFi Password: plaintext='%s', encrypted='%s', encryptedLen=%zu", passToUse, encryptedPass, encryptedLen);
            }
        }
        
        cmdLen += snprintf(cmdBuffer + cmdLen, sizeof(cmdBuffer) - cmdLen, "%zu%s", encryptedLen, encryptedPass);

        // Add mode
        int modeToUse = (config->mask & MASK_WIFI_MODE) ? config->mode : this->wifiMode;
        cmdLen += snprintf(cmdBuffer + cmdLen, sizeof(cmdBuffer) - cmdLen, "%d", modeToUse);

        // Terminate command
        snprintf(cmdBuffer + cmdLen, sizeof(cmdBuffer) - cmdLen, "#");

        if (!this->SendCommand(cmdBuffer))
        {
            PB_ERROR("PBSetWiFiConfig: Failed to send WiFi config command");
            return PB_ERROR_COMMUNICATION;
        }

        // Wait for device to echo back the updated WiFi info with timeout
        {
            std::lock_guard<std::mutex> infoLock(this->wifiInfoMutex);
            this->wifiInfoPending = true;
        }
        
        std::unique_lock<std::mutex> infoLock(this->wifiInfoMutex);
        if (!this->wifiInfoCV.wait_for(infoLock, std::chrono::seconds(2), 
                                        [this]() { return !this->wifiInfoPending; }))
        {
            PB_DEBUG("PBSetWiFiConfig: Timeout waiting for device WiFi response");
            this->wifiInfoPending = false;
        }

        // Update device cache with the new values that were sent
        if (config->mask & MASK_WIFI_SSID)
        {
            strncpy(this->wifiSSID, config->ssid, PB_SSID_LEN - 1);
            this->wifiSSID[PB_SSID_LEN - 1] = '\0';
        }

        if (config->mask & MASK_WIFI_MODE)
        {
            this->wifiMode = config->mode;
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE PowerBoxDevice::GetWiFiStatus(PB_WIFI_STATUS *status)
    {
        /* Return cached WiFi info from listener thread */
        {
            std::lock_guard<std::mutex> infoLock(this->wifiInfoMutex);
            strncpy(status->IP, this->wifiIP, PB_IP_LEN - 1);
            status->IP[PB_IP_LEN - 1] = '\0';
            status->rssi = this->wifiRSSI;
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE PowerBoxDevice::Restart(void)
    {
        if (!this->serialPort || !this->serialPort->IsOpen())
        {
            return PB_ERROR_INVALID_STATE;
        }

        /* Send factory reset command */
        if (!this->SendCommand(":RD#"))
        {
            return PB_ERROR_COMMUNICATION;
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE PowerBoxDevice::FactoryReset(void)
    {
        if (!this->serialPort || !this->serialPort->IsOpen())
        {
            return PB_ERROR_INVALID_STATE;
        }

        /* Send factory reset command */
        if (!this->SendCommand(":FR#"))
        {
            return PB_ERROR_COMMUNICATION;
        }

        return PB_SUCCESS;
    }

#ifdef __unix__
    /* The device node a port name currently points to. A device is registered
     * under its /dev/serial/by-path/... link where it has one, which after a
     * reconnect may point to a different node - compare ports only through
     * this. Returns the name unchanged if it does not exist (unplugged). */
    static std::string ResolvePort(const std::string &port)
    {
        char resolved[PATH_MAX];
        return realpath(port.c_str(), resolved) ? std::string(resolved) : port;
    }

    /* True if the tty belongs to a CP210x USB-serial chip (10c4:ea60) */
    static bool IsCp210x(struct udev_device *tty)
    {
        struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
            tty, "usb", "usb_device");
        if (!parent)
        {
            return false;
        }

        const char *vid = udev_device_get_sysattr_value(parent, "idVendor");
        const char *pid = udev_device_get_sysattr_value(parent, "idProduct");
        if (!vid || !pid)
        {
            return false;
        }
        PB_DEBUG("Found device with VID:%s PID:%s", vid, pid);
        return strcmp(vid, "10c4") == 0 && strcmp(pid, "ea60") == 0;
    }

    /* The tty's /dev/serial/by-path/... link, which names the USB socket and
     * so survives reconnects, or "" if it has none (or none that fits
     * PB_PORT_LEN). udev lists the links sorted, so the choice is stable. Newer
     * udev versions add a ...-usbv2-... link next to the legacy one; either
     * works. */
    static std::string ByPathLink(struct udev_device *tty)
    {
        struct udev_list_entry *entry;
        udev_list_entry_foreach(entry, udev_device_get_devlinks_list_entry(tty))
        {
            const char *link = udev_list_entry_get_name(entry);
            if (strncmp(link, "/dev/serial/by-path/", 20) == 0 && strlen(link) < PB_PORT_LEN)
            {
                return link;
            }
        }
        return "";
    }

    /* Opens portName, performs the handshake and closes it again. Returns the
     * (not yet opened) device if a PowerBox answered, nullptr otherwise. */
    std::shared_ptr<PowerBoxDevice> ProbePowerBox(const std::string &portName)
    {
        PB_DEBUG("Trying to open device: %s", portName.c_str());

        auto port = std::make_shared<SerialPort>();
        if (!port->Open(portName.c_str()))
        {
            PB_DEBUG("Failed to open port %s", portName.c_str());
            return nullptr;
        }

        PB_DEBUG("Port opened, flushing and sending command...");

        auto tempDevice = std::make_shared<PowerBoxDevice>(port, portName);

        // Send HS to wake up device
        tempDevice->SendCommand(":HS#", 200);

        // Start status listener thread
        tempDevice->StartStatusListener();

        // Perform handshake with retry mechanism
        bool found = tempDevice->SendAndWaitForReplyWithRetry(":HS#", tempDevice->handshakeMutex, tempDevice->handshakeCV,
                                                              tempDevice->handshakePending, "handshake");

        /* Stop listener and close port - a valid device is reopened in PBOpen */
        tempDevice->StopStatusListener();
        port->Close();

        if (!found)
        {
            PB_DEBUG("No response from device after handshake retries");
            return nullptr;
        }

        PB_DEBUG("Valid device found!");
        return tempDevice;
    }
#endif

    PB_ERROR_TYPE ScanPowerBox(int *number, int *ids)
    {
        if (!number || !ids)
        {
            return PB_ERROR_NULL_POINTER;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        // Stop telemetry and listener threads on all currently open devices
        for (auto &pair : g_devices)
        {
            auto device = pair.second;

            if(!device)
            {
                continue;
            }

            if(auto powerBox = std::dynamic_pointer_cast<PowerBoxDevice>(device))
            {
                if (powerBox->IsOpen())
                {
                    powerBox->SendCommand(":ES#");
                    powerBox->StopStatusListener();
                }
            }
        }

        int count = 0;

#ifdef __unix__
        /* Create udev context */
        struct udev *udev = udev_new();
        if (!udev)
        {
            return PB_ERROR_COMMUNICATION;
        }

        /* Create enumeration for tty devices */
        struct udev_enumerate *enumerate = udev_enumerate_new(udev);
        if (!enumerate)
        {
            udev_unref(udev);
            return PB_ERROR_COMMUNICATION;
        }

        /* Filter for tty subsystem */
        udev_enumerate_add_match_subsystem(enumerate, "tty");
        udev_enumerate_scan_devices(enumerate);

        struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry *entry;

        /* Iterate through all tty devices */
        udev_list_entry_foreach(entry, devices)
        {
            if (count >= PB_MAX_NUM)
                break;

            const char *path = udev_list_entry_get_name(entry);
            struct udev_device *device = udev_device_new_from_syspath(udev, path);
            if (!device)
            {
                continue;
            }

            if (!IsCp210x(device))
            {
                udev_device_unref(device);
                continue;
            }

            /* Get the device node (e.g., /dev/ttyUSB0) */
            const char *deviceNode = udev_device_get_devnode(device);
            if (!deviceNode)
            {
                udev_device_unref(device);
                continue;
            }

            /* Register the device under its by-path link if it has one, so
             * PBOpen finds it again after a replug into the same socket */
            std::string link = ByPathLink(device);
            auto tempDevice = ProbePowerBox(link.empty() ? std::string(deviceNode) : link);
            if (tempDevice)
            {
                int id = count;
                g_devices[id] = tempDevice;
                ids[count] = id;
                count++;
            }

            udev_device_unref(device);
        }

        /* Clean up udev resources */
        udev_enumerate_unref(enumerate);
        udev_unref(udev);
#elif defined(_WIN32)
        PB_DEBUG("PBScan: Enumerating serial ports on Windows");

        // Enumerate devices in the Ports class and look for USB devices with matching VID/PID
        HDEVINFO hDevInfo = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);
        if (hDevInfo == INVALID_HANDLE_VALUE)
        {
            PB_DEBUG("SetupDiGetClassDevs failed");
            return PB_SUCCESS;
        }

        for (DWORD idx = 0; count < PB_MAX_NUM; ++idx)
        {
            SP_DEVINFO_DATA devInfo;
            devInfo.cbSize = sizeof(devInfo);
            if (!SetupDiEnumDeviceInfo(hDevInfo, idx, &devInfo))
            {
                DWORD err = GetLastError();
                if (err == ERROR_NO_MORE_ITEMS)
                    break;
                else
                    continue;
            }

            // Try to get the device instance ID (contains VID/PID for USB-serial)
            char instanceId[512] = {0};
            if (!SetupDiGetDeviceInstanceIdA(hDevInfo, &devInfo, instanceId, (DWORD)sizeof(instanceId), NULL))
            {
                // ignore
            }

            // Look for VID_10C4 and PID_EA60 in instance ID (case-insensitive)
            bool isTarget = false;
            if (instanceId[0])
            {
                std::string iid(instanceId);
                for (auto &c : iid) c = (char)toupper((unsigned char)c);
                if (iid.find("VID_10C4") != std::string::npos && iid.find("PID_EA60") != std::string::npos)
                {
                    isTarget = true;
                }
            }

            if (!isTarget)
                continue;

            // Try to extract COM port name. First try PortName from device registry, fallback to FriendlyName
            char portName[128] = {0};

            HKEY hKey = SetupDiOpenDevRegKey(hDevInfo, &devInfo, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            if (hKey != INVALID_HANDLE_VALUE)
            {
                DWORD type = 0;
                DWORD cb = (DWORD)sizeof(portName);
                if (RegQueryValueExA(hKey, "PortName", NULL, &type, (LPBYTE)portName, &cb) == ERROR_SUCCESS)
                {
                    // portName now contains e.g. "COM3"
                }
                RegCloseKey(hKey);
            }

            if (!portName[0])
            {
                // Fallback: get friendly name and parse (e.g., "USB-SERIAL CH340 (COM3)")
                char friendly[256] = {0};
                if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfo, SPDRP_FRIENDLYNAME, NULL, (PBYTE)friendly, (DWORD)sizeof(friendly), NULL))
                {
                    char *p = strstr(friendly, "(COM");
                    if (p)
                    {
                        char *q = strchr(p, ')');
                        if (q && q > p)
                        {
                            size_t len = (size_t)(q - p - 1); // skip '(' and ')'
                            if (len < sizeof(portName))
                            {
                                // p points to "(COM3" so copy from p+1
                                strncpy(portName, p + 1, len);
                                portName[len] = '\0';
                            }
                        }
                    }
                }
            }

            if (!portName[0])
                continue; // can't determine COM port

            PB_DEBUG("Found target device instance=%s port=%s", instanceId, portName);

            // Try to open the port and perform handshake
            std::string deviceNode = std::string(portName); // SerialPort_win handles COM prefix for CreateFile
            auto port = std::make_shared<SerialPort>();
            if (port->Open(deviceNode.c_str()))
            {
                PB_DEBUG("Port opened, flushing and sending command...");

                auto tempDevice = std::make_shared<PowerBoxDevice>(port, deviceNode);

                // Send HS to wake up device
                tempDevice->SendCommand(":HS#", 200);

                // Start status listener thread
                tempDevice->StartStatusListener();

                // Perform handshake with retry mechanism
                if(tempDevice->SendAndWaitForReplyWithRetry(":HS#", tempDevice->handshakeMutex, tempDevice->handshakeCV,
                                                            tempDevice->handshakePending, "handshake"))
                {
                    PB_DEBUG("Valid device found!");

                    /* Stop listener */
                    tempDevice->StopStatusListener();

                    /* Valid device found - close port, will be reopened in PBOpen */
                    port->Close();
                    int id = count;
                    g_devices[id] = tempDevice;
                    ids[count] = id;
                    count++;
                }
                else
                {
                    PB_DEBUG("No response from device after handshake retries");
                    tempDevice->StopStatusListener();
                    port->Close();
                }
            }
            else
            {
                PB_DEBUG("Failed to open port %s", portName);
            }
        }

        SetupDiDestroyDeviceInfoList(hDevInfo);
#endif

        *number += count;
        return PB_SUCCESS;
    }

#ifdef __unix__
    PB_ERROR_TYPE ScanPowerBoxPort(const char *port, int *id)
    {
        if (!port || !id)
        {
            return PB_ERROR_NULL_POINTER;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        /* Compare device nodes: the registered device may have been found on
         * a by-path link or by device node */
        char resolved[PATH_MAX];
        if (!realpath(port, resolved))
        {
            PB_DEBUG("PBScanPort: %s does not exist", port);
            return PB_ERROR_COMMUNICATION;
        }

        for (const auto &pair : g_devices)
        {
            auto powerBox = std::dynamic_pointer_cast<PowerBoxDevice>(pair.second);
            if (powerBox && ResolvePort(powerBox->GetPortName()) == resolved)
            {
                *id = pair.first;
                return PB_SUCCESS;
            }
        }

        /* A by-path link names a USB socket, not a device: if something else
         * is plugged in there now, leave it alone */
        struct udev *udev = udev_new();
        if (!udev)
        {
            return PB_ERROR_COMMUNICATION;
        }
        const char *name = strrchr(resolved, '/');
        struct udev_device *tty = udev_device_new_from_subsystem_sysname(udev, "tty", name ? name + 1 : resolved);
        bool cp210x = tty && IsCp210x(tty);
        if (tty)
        {
            udev_device_unref(tty);
        }
        udev_unref(udev);
        if (!cp210x)
        {
            PB_DEBUG("PBScanPort: %s is not a CP210x port", resolved);
            return PB_ERROR_COMMUNICATION;
        }

        /* Keep the name as given: a by-path link then follows the device to
         * whatever node it gets after a reconnect in the same socket */
        auto device = ProbePowerBox(port);
        if (!device)
        {
            PB_DEBUG("PBScanPort: No PowerBox on %s", resolved);
            return PB_ERROR_COMMUNICATION;
        }

        /* Smallest free id: never replace a device another scan registered */
        int newId = 0;
        while (g_devices.count(newId) > 0)
            newId++;
        g_devices[newId] = device;
        *id = newId;

        PB_INFO("Found PowerBox on %s (id=%d)", port, newId);
        return PB_SUCCESS;
    }

    PB_ERROR_TYPE GetPowerBoxPort(int id, char *port)
    {
        if (!port)
        {
            return PB_ERROR_NULL_POINTER;
        }

        std::string result;
        {
            std::lock_guard<std::mutex> lock(g_globalMutex);
            auto it = g_devices.find(id);
            if (it == g_devices.end())
            {
                return PB_ERROR_INVALID_ID;
            }
            auto powerBox = std::dynamic_pointer_cast<PowerBoxDevice>(it->second);
            if (!powerBox)
            {
                return PB_ERROR_INVALID_STATE;
            }
            result = powerBox->GetPortName();
        }

        /* Found on a by-path link already: that is the name to keep. Otherwise
         * (registered by device node, e.g. PBScanPort("/dev/ttyUSB0")) look the
         * link up. */
        std::string node = ResolvePort(result);
        const char *name = strrchr(node.c_str(), '/');
        struct udev *udev = udev_new();
        if (result.rfind("/dev/serial/by-path/", 0) != 0 && udev && name)
        {
            struct udev_device *tty = udev_device_new_from_subsystem_sysname(udev, "tty", name + 1);
            if (tty)
            {
                std::string link = ByPathLink(tty);
                if (!link.empty())
                {
                    result = link;
                }
                udev_device_unref(tty);
            }
        }
        if (udev)
        {
            udev_unref(udev);
        }

        if (result.size() >= PB_PORT_LEN)
        {
            return PB_ERROR_INVALID_STATE;
        }
        memcpy(port, result.c_str(), result.size() + 1);
        return PB_SUCCESS;
    }
#endif
} /* namespace PowerBox */