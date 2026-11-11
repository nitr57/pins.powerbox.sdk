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

#define SDK_VERSION "1.5.0"

#include "PowerBoxSDK.h"
#include "PowerBoxLogging.h"
#include "PowerBoxDevice.h"
#include "PinsBoxDevice.h"
#include "PinsBoxLightDevice.h"
#include "PinsBoxMiniDevice.h"
#include "PinsBoxCustomDevice.h"
#include "StellaVitaDevice.h"
#include "ASIAirDevice.h"
#include <cstring>
#include <cmath>

using namespace PowerBox;

PBAPI PB_ERROR_TYPE PBScan(int *number, int *ids)
{
    PB_DEBUG("PBScan: Starting device scan");
    if (!number || !ids)
    {
        PB_ERROR("PBScan: Invalid parameters (number or ids is null)");
        return PB_ERROR_NULL_POINTER;
    }
    int pinsBox = ScanPinsBox(ids);
    if (pinsBox < 0)
    {
        // A PinsBox is present but not usable. Do not scan the other variants:
        // the error must not look like "no device", and no other variant may
        // touch the same hardware.
        *number = 0;
        PB_ERROR("PBScan: PinsBox present but not usable (see the messages above)");
        return PB_ERROR_INVALID_STATE;
    }
    *number = pinsBox > 0 ? 1 : 0;
    PB_DEBUG("PBScan: Found %d PinsBox device(s)", *number);
    if (*number > 0)
    {
        PB_DEBUG("PBScan: PinsBox found, skipping PowerBox serial scan");
        return PB_SUCCESS;
    }
    *number = ScanPinsBoxLight(ids) ? 1 : 0;
    PB_DEBUG("PBScan: Found %d PinsBoxLight device(s)", *number);
    if (*number > 0)
    {
        PB_DEBUG("PBScan: PinsBoxLight found, skipping PowerBox serial scan");
        return PB_SUCCESS;
    }
    *number = ScanPinsBoxMini(ids) ? 1 : 0;
    PB_DEBUG("PBScan: Found %d PinsBoxMini device(s)", *number);
    if (*number > 0)
    {
        PB_DEBUG("PBScan: PinsBoxMini found, skipping PowerBox serial scan");
        return PB_SUCCESS;
    }
    *number = ScanPinsBoxCustom(ids) ? 1 : 0;
    PB_DEBUG("PBScan: Found %d PinsBoxCustom device(s)", *number);
    if (*number > 0)
    {
        PB_DEBUG("PBScan: PinsBoxCustom found, skipping PowerBox serial scan");
        return PB_SUCCESS;
    }

    // Always list stella vita device if its a raspi cm4, regardless
    *number = ScanStellaVita(ids) ? 1 : 0;
    PB_DEBUG("PBScan: Found a potential StellaVita device");

    // Always list asiair device if its a raspi, regardless
    *number += ScanASIAir(ids) ? 1 : 0;
    PB_DEBUG("PBScan: Found a potential ASIAir device");

    PB_ERROR_TYPE result = ScanPowerBox(number, ids);
    PB_DEBUG("PBScan: Total devices found: %d", *number);
    return result;
}

#ifdef __unix__
PBAPI PB_ERROR_TYPE PBScanPort(const char *port, int *id)
{
    PB_DEBUG("PBScanPort: Probing %s", port ? port : "(null)");
    return ScanPowerBoxPort(port, id);
}

PBAPI PB_ERROR_TYPE PBGetPort(int id, char *port)
{
    return GetPowerBoxPort(id, port);
}
#endif

PBAPI PB_ERROR_TYPE PBOpen(int id)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);
    PB_DEBUG("PBOpen: Opening device id=%d", id);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBOpen: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    return device->Open();
}

PBAPI PB_ERROR_TYPE PBClose(int id)
{
    PB_DEBUG("PBClose: Closing device id=%d", id);
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBClose: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    device->Close();
    PB_DEBUG("PBClose: Device id=%d closed successfully", id);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetSerial(int id, char *serial)
{
    PB_DEBUG("PBGetSerial: Getting serial for device id=%d", id);
    if (!serial)
    {
        PB_ERROR("PBGetSerial: serial pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetSerial: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    std::string deviceSerial = device->GetSerial();

    strncpy(serial, deviceSerial.c_str(), PB_VERSION_LEN - 1);
    serial[PB_VERSION_LEN - 1] = '\0';
    PB_DEBUG("PBGetSerial: Serial=%s", serial);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBScanWiFi(int id, PB_WIFI_SCAN_RESULT *result)
{
    PB_DEBUG("PBScanWiFi: Scanning WiFi networks for device id=%d", id);
    if (!result)
    {
        PB_ERROR("PBScanWiFi: result pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBScanWiFi: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto state = device->ScanWiFi(result);

    if(state == PB_ERROR_NOT_AVAILABLE)
    {
        result->count = 0;
    }
    else
    {
        PB_DEBUG("PBScanWiFi: WiFi scan completed");
    }

    return state;
}

PBAPI PB_ERROR_TYPE PBGetConfig(int id, PB_DEVICE_CONFIG *config)
{
    PB_DEBUG("PBGetConfig: Getting device config for id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    config->temperatureOffset = device->GetTemperatureOffset();
    config->humidityOffset = device->GetHumidityOffset();
    config->envUpdateRate = device->GetEnvUpdateRate();
    config->updateRate = device->GetUpdateRate();
    PB_DEBUG("PBGetConfig: tempOffset=%.2f, humiOffset=%.2f, envRate=%d, updateRate=%d",
             config->temperatureOffset, config->humidityOffset, config->envUpdateRate, config->updateRate);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBSetConfig(int id, PB_DEVICE_CONFIG *config)
{
    PB_DEBUG("PBSetConfig: Setting device config for id=%d, mask=0x%X", id, config ? config->mask : 0);
    if (!config)
    {
        PB_ERROR("PBSetConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    if (config->mask & MASK_PB_TEMPERATURE_OFFSET)
    {
        if (config->temperatureOffset < -12.5f || config->temperatureOffset > 12.5f)
        {
            PB_ERROR("PBSetConfig: Temperature offset %.2f out of range [-12.5, 12.5]", config->temperatureOffset);
            return PB_ERROR_INVALID_PARAMETER;
        }

        if(!device->SetTemperatureOffset(config->temperatureOffset))
        {
            PB_ERROR("PBSetConfig: Failed to set temperature offset");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetConfig: Temperature offset set to %.2f", config->temperatureOffset);
    }

    if(config->mask & MASK_PB_HUMIDITY_OFFSET)
    {
        if(config->humidityOffset < -12.5f || config->humidityOffset > 12.5f)
        {
            PB_ERROR("PBSetConfig: Humidity offset %.2f out of range [-12.5, 12.5]", config->humidityOffset);
            return PB_ERROR_INVALID_PARAMETER;
        }

        if(!device->SetHumidityOffset(config->humidityOffset))
        {
            PB_ERROR("PBSetConfig: Failed to set humidity offset");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetConfig: Humidity offset set to %.2f", config->humidityOffset);
    }

    if(config->mask & MASK_PB_ENV_UPDATE_RATE)
    {
        if(config->envUpdateRate < 1 || config->envUpdateRate > 60)
        {
            PB_ERROR("PBSetConfig: Environment update rate %d out of range [1, 60]", config->envUpdateRate);
            return PB_ERROR_INVALID_PARAMETER;
        }

        if (!device->SetEnvUpdateRate(config->envUpdateRate))
        {
            PB_ERROR("PBSetConfig: Failed to set environment update rate");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetConfig: Environment update rate set to %d", config->envUpdateRate);
    }

    if(config->mask & MASK_PB_UPDATE_RATE)
    {
        if(config->updateRate < 1 || config->updateRate > 60)
        {
            PB_ERROR("PBSetConfig: Update rate %d out of range [1, 60]", config->updateRate);
            return PB_ERROR_INVALID_PARAMETER;
        }

        if (!device->SetUpdateRate(config->updateRate))
        {
            PB_ERROR("PBSetConfig: Failed to set update rate");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetConfig: Update rate set to %d", config->updateRate);
    }

    if(config->mask & MASK_PB_EXT_TEMPERATURE)
    {
        if(!device->SetExtTemperature(config->temperature))
        {
            PB_ERROR("PBSetConfig: Failed to set external temperature");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetConfig: External temperature set to %.2f", config->temperature);
    }

    if(config->mask & MASK_PB_EXT_HUMIDITY)
    {
        if(!device->SetExtHumidity(config->humidity))
        {
            PB_ERROR("PBSetConfig: Failed to set external humidity");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetConfig: External humidity set to %.2f", config->humidity);
    }

    PB_DEBUG("PBSetConfig: Configuration updated successfully");
    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetPowerPortConfig(int id, PB_POWER_PORT_CONFIG *config)
{
    PB_DEBUG("PBGetPowerPortConfig: Getting power port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetPowerPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetPowerPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto idx = config->index;

    if(idx >= device->GetNumPowerPorts())
    {
        PB_ERROR("PBGetPowerPortConfig: Port index %d out of range [0, %d)", idx, device->GetNumPowerPorts());
        return PB_ERROR_INVALID_PARAMETER;
    }

    config->enabled = device->GetPowerState(idx) != 0;
    config->bootState = device->GetPowerBootState(idx) != 0;
    PB_DEBUG("PBGetPowerPortConfig: Port %d enabled=%d, bootState=%d", idx, config->enabled, config->bootState);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBSetPowerPortConfig(int id, PB_POWER_PORT_CONFIG *config)
{
    PB_DEBUG("PBSetPowerPortConfig: Setting power port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBSetPowerPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetPowerPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto idx = config->index;

    if(idx >= device->GetNumPowerPorts())
    {
        PB_ERROR("PBSetPowerPortConfig: Port %d invalid", idx);
        return PB_ERROR_INVALID_PARAMETER;
    }

    if (config->mask & MASK_PORT_ENABLE)
    {
        if (!device->SetPowerState(config->index, config->enabled))
        {
            PB_ERROR("PBSetPowerPortConfig: Failed to set power state on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetPowerPortConfig: Port %d enabled=%d", config->index, config->enabled);
    }

    if(config->mask & MASK_PORT_BOOT_STATE)
    {
        if (!device->SetPowerBootState(config->index, config->bootState))
        {
            PB_ERROR("PBSetPowerPortConfig: Failed to set boot state on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetPowerPortConfig: Port %d bootState=%d", config->index, config->bootState);
    }

    if(config->mask & MASK_PORT_OVERCURRENT_RESET)
    {
        if (!device->ResetPowerOvercurrent(config->index))
        {
            PB_ERROR("PBSetPowerPortConfig: Failed to reset overcurrent on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetPowerPortConfig: Overcurrent reset on port %d", config->index);
    }

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetUSBPortConfig(int id, PB_USB_PORT_CONFIG *config)
{
    PB_DEBUG("PBGetUSBPortConfig: Getting USB port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetUSBPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetUSBPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto idx = config->index;

    if(idx >= device->GetNumUSBPorts())
    {
        PB_ERROR("PBGetUSBPortConfig: Port index %d out of range [0, %d)", idx, device->GetNumUSBPorts());
        return PB_ERROR_INVALID_PARAMETER;
    }

    config->enabled = device->GetUSBState(idx) != 0;
    config->bootState = device->GetUSBBootState(idx) != 0;
    PB_DEBUG("PBGetUSBPortConfig: Port %d enabled=%d, bootState=%d", idx, config->enabled, config->bootState);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBSetUSBPortConfig(int id, PB_USB_PORT_CONFIG *config)
{
    PB_DEBUG("PBSetUSBPortConfig: Setting USB port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBSetUSBPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetUSBPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto idx = config->index;

    if(idx >= device->GetNumUSBPorts())
    {
        PB_ERROR("PBSetUSBPortConfig: Port index %d out of range [0, %d)", idx, device->GetNumUSBPorts());
        return PB_ERROR_INVALID_PARAMETER;
    }

    if (config->mask & MASK_PORT_ENABLE)
    {
        if (!device->SetUSBState(config->index, config->enabled))
        {
            PB_ERROR("PBSetUSBPortConfig: Failed to set USB state on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetUSBPortConfig: Port %d enabled=%d", config->index, config->enabled);
    }

    if(config->mask & MASK_PORT_BOOT_STATE)
    {
        if (!device->SetUSBBootState(config->index, config->bootState))
        {
            PB_ERROR("PBSetUSBPortConfig: Failed to set boot state on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetUSBPortConfig: Port %d bootState=%d", config->index, config->bootState);
    }

    if(config->mask & MASK_PORT_OVERCURRENT_RESET)
    {
        if (!device->ResetUSBOvercurrent(config->index))
        {
            PB_ERROR("PBSetUSBPortConfig: Failed to reset overcurrent on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetUSBPortConfig: Overcurrent reset on port %d", config->index);
    }

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetDewPortConfig(int id, PB_DEW_PORT_CONFIG *config)
{
    PB_DEBUG("PBGetDewPortConfig: Getting dew port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetDewPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetDewPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto idx = config->index;

    if(idx >= device->GetNumDewPorts())
    {
        PB_ERROR("PBGetDewPortConfig: Port index %d out of range [0, %d)", idx, device->GetNumDewPorts());
        return PB_ERROR_INVALID_PARAMETER;
    }

    config->enabled = device->GetDewState(idx) != 0;
    config->autoMode = device->GetDewAutoMode(idx) != 0;
    config->autoThreshold = device->GetDewAutoThreshold(idx);
    config->power = device->GetDewPWMPower(idx);
    PB_DEBUG("PBGetDewPortConfig: Port %d enabled=%d, autoMode=%d, threshold=%.2f, power=%d",
             idx, config->enabled, config->autoMode, config->autoThreshold, config->power);

    return PB_SUCCESS;
}

/* Highest power value of a dew/PWM port with the given PWM resolution [bits],
 * -1 if the resolution is not valid (port not present or not known yet) */
static int PBMaxPower(int resolution)
{
    if (resolution < 1 || resolution > 16)
    {
        return -1;
    }
    return (1 << resolution) - 1;
}

PBAPI PB_ERROR_TYPE PBSetDewPortConfig(int id, PB_DEW_PORT_CONFIG *config)
{
    PB_DEBUG("PBSetDewPortConfig: Setting dew port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBSetDewPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetDewPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    auto idx = config->index;

    if(idx >= device->GetNumDewPorts())
    {
        PB_ERROR("PBSetDewPortConfig: Port index %d out of range [0, %d)", idx, device->GetNumDewPorts());
        return PB_ERROR_INVALID_PARAMETER;
    }

    // State and power is set with a single call, need to combine
    int newState = (config->mask & MASK_PORT_ENABLE) ? (config->enabled != 0) : device->GetDewState(idx);
    int newPower = (config->mask & MASK_PORT_POWER) ? config->power : device->GetDewPWMPower(idx);
    bool autoMode = false;

    // Check all values before anything is applied. The power is only used to
    // switch on (switching off sets 0), so an invalid power never blocks
    // switching off. Out of range values would otherwise wrap around in the
    // device (e.g. -1 -> full power).
    if ((config->mask & MASK_PORT_POWER) && newState != 0)
    {
        int maxPower = PBMaxPower(device->GetDewPwmResolution());
        if (config->power < 0 || config->power > maxPower)
        {
            PB_ERROR("PBSetDewPortConfig: Power %d out of range [0, %d] on port %d", config->power, maxPower, idx);
            return PB_ERROR_INVALID_PARAMETER;
        }
    }

    // Negated check, so NaN is invalid as well
    if ((config->mask & MASK_PORT_AUTO_DEW_THRESHOLD) &&
        !(config->autoThreshold >= 0.0f && config->autoThreshold <= 25.5f))
    {
        PB_ERROR("PBSetDewPortConfig: Auto threshold %.2f out of range [0, 25.5]", config->autoThreshold);

        // Switching off is never blocked (like the buck port): a request that
        // switches the port off manually is still applied (auto mode off first,
        // if requested), the rest is not
        bool autoAfter = (config->mask & MASK_PORT_AUTO_DEW_MODE) ? (config->autoMode != 0) : (device->GetDewAutoMode(idx) != 0);
        if ((config->mask & MASK_PORT_ENABLE) && config->enabled == 0 && !autoAfter)
        {
            if ((config->mask & MASK_PORT_AUTO_DEW_MODE) && !device->SetDewAutoMode(idx, 0))
            {
                PB_ERROR("PBSetDewPortConfig: Failed to set auto mode on port %d", idx);
                return PB_ERROR_COMMUNICATION;
            }
            if (!device->SetDewState(idx, 0, 0))
            {
                PB_ERROR("PBSetDewPortConfig: Failed to switch off port %d", idx);
                return PB_ERROR_COMMUNICATION;
            }
        }
        return PB_ERROR_INVALID_PARAMETER;
    }

    if (config->mask & MASK_PORT_AUTO_DEW_MODE)
    {
        if (!device->SetDewAutoMode(config->index, config->autoMode))
        {
            PB_ERROR("PBSetDewPortConfig: Failed to set auto mode on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        autoMode = config->autoMode != 0;
        PB_DEBUG("PBSetDewPortConfig: Port %d autoMode=%d", config->index, config->autoMode);
    }

    if ((config->mask & MASK_PORT_ENABLE || config->mask & MASK_PORT_POWER) && !autoMode)
    {
        // When turning off, also set power to 0
        newPower = (newState == 0) ? 0 : newPower;

        if (!device->SetDewState(config->index, newState, newPower))
        {
            PB_ERROR("PBSetDewPortConfig: Failed to set dew state on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetDewPortConfig: Port %d state=%d, power=%d", config->index, newState, newPower);
    }

    if (config->mask & MASK_PORT_AUTO_DEW_THRESHOLD)
    {
        if (!device->SetDewAutoThreshold(config->index, config->autoThreshold))
        {
            PB_ERROR("PBSetDewPortConfig: Failed to set auto threshold on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetDewPortConfig: Port %d autoThreshold=%.2f", config->index, config->autoThreshold);
    }

    if(config->mask & MASK_PORT_OVERCURRENT_RESET)
    {
        if (!device->ResetDewOvercurrent(config->index))
        {
            PB_ERROR("PBSetDewPortConfig: Failed to reset overcurrent on port %d", config->index);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetDewPortConfig: Overcurrent reset on port %d", config->index);
    }

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetBuckPortConfig(int id, PB_BUCK_PORT_CONFIG *config)
{
    PB_DEBUG("PBGetBuckPortConfig: Getting buck port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetBuckPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetBuckPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    config->enabled = device->GetBuckState() != 0;
    config->bootState = device->GetBuckBootState() != 0;
    config->targetVoltage = device->GetBuckSetVoltage();
    PB_DEBUG("PBGetBuckPortConfig: enabled=%d, bootState=%d, targetVoltage=%.2fV",
             config->enabled, config->bootState, config->targetVoltage);

    return PB_SUCCESS;
}

/* Highest valid buck target [V]: the serial PowerBox protocol transfers the
 * target as uint16 in mV (see PowerBoxDevice::SetBuckState, which checks the
 * same limit). The PinsBox converter itself stays below about 11.85 V. */
static const float PB_BUCK_MAX_TARGET_VOLTAGE = 65.535f;

PBAPI PB_ERROR_TYPE PBSetBuckPortConfig(int id, PB_BUCK_PORT_CONFIG *config)
{
    PB_DEBUG("PBSetBuckPortConfig: Setting buck port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBSetBuckPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetBuckPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    // State and power is set with a single call, need to combine
    int newState = (config->mask & MASK_PORT_ENABLE) ? (config->enabled != 0) : (device->GetBuckState() != 0);
    float newTarget = (config->mask & MASK_PORT_VOLTAGE) ? config->targetVoltage : device->GetBuckSetVoltage();

    if (config->mask & MASK_PORT_VOLTAGE)
    {
        // Negated check, so NaN is invalid as well
        if (!(newTarget >= 0.0f && newTarget <= PB_BUCK_MAX_TARGET_VOLTAGE))
        {
            PB_ERROR("PBSetBuckPortConfig: invalid target voltage %f", newTarget);

            if (newState)
            {
                // Switching on with an invalid target: reject, hardware untouched
                return PB_ERROR_INVALID_PARAMETER;
            }

            // Switching off is never blocked: switch off with the stored target,
            // but report that the voltage was not applied
            if (!device->SetBuckState(0, device->GetBuckSetVoltage()))
            {
                PB_ERROR("PBSetBuckPortConfig: Failed to switch off");
                return PB_ERROR_COMMUNICATION;
            }
            return PB_ERROR_INVALID_PARAMETER;
        }

        // 0 V means off, the stored target is kept (same for all device types).
        // Same rounding to mV as the devices use, so e.g. 0.0004 V is 0 V too.
        if (lroundf(newTarget * 1000.0f) == 0)
        {
            newState = 0;
            newTarget = device->GetBuckSetVoltage();
        }
    }

    if (config->mask & MASK_PORT_ENABLE || config->mask & MASK_PORT_VOLTAGE)
    {
        if (!device->SetBuckState(newState, newTarget))
        {
            PB_ERROR("PBSetBuckPortConfig: Failed to set buck state, enabled=%d, voltage=%.2fV", newState, newTarget);
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetBuckPortConfig: state=%d, voltage=%.2fV", newState, newTarget);
    }

    if (config->mask & MASK_PORT_BOOT_STATE)
    {
        if (!device->SetBuckBootState(config->bootState))
        {
            PB_ERROR("PBSetBuckPortConfig: Failed to set boot state");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetBuckPortConfig: bootState=%d", config->bootState);
    }

    if(config->mask & MASK_PORT_OVERCURRENT_RESET)
    {
        if (!device->ResetBuckOvercurrent())
        {
            PB_ERROR("PBSetBuckPortConfig: Failed to reset overcurrent");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetBuckPortConfig: Overcurrent reset");
    }

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetPWMPortConfig(int id, PB_PWM_PORT_CONFIG *config)
{
    PB_DEBUG("PBGetPWMPortConfig: Getting PWM port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetPWMPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetPWMPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    config->enabled = device->GetPWMState() != 0;
    config->power = device->GetPWMPower();
    PB_DEBUG("PBGetPWMPortConfig: enabled=%d, power=%d", config->enabled, config->power);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBSetPWMPortConfig(int id, PB_PWM_PORT_CONFIG *config)
{
    PB_DEBUG("PBSetPWMPortConfig: Setting PWM port config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBSetPWMPortConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetPWMPortConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    // State and power is set with a single call, need to combine
    int newState = (config->mask & MASK_PORT_ENABLE) ? (config->enabled != 0) : device->GetPWMState();
    int newPower = (config->mask & MASK_PORT_POWER) ? config->power : device->GetPWMPower();

    // Checked before anything is applied; only used to switch on (see PBSetDewPortConfig)
    if ((config->mask & MASK_PORT_POWER) && newState != 0)
    {
        if (device->GetNumPWMPorts() < 1)
        {
            PB_ERROR("PBSetPWMPortConfig: device has no PWM port");
            return PB_ERROR_INVALID_PARAMETER;
        }

        int maxPower = PBMaxPower(device->GetPWMPwmResolution());
        if (config->power < 0 || config->power > maxPower)
        {
            PB_ERROR("PBSetPWMPortConfig: Power %d out of range [0, %d]", config->power, maxPower);
            return PB_ERROR_INVALID_PARAMETER;
        }
    }

    if (config->mask & MASK_PORT_ENABLE || config->mask & MASK_PORT_POWER)
    {
        // When turning off, also set power to 0
        newPower = (newState == 0) ? 0 : newPower;

        if (!device->SetPWMState(newState, newPower))
        {
            PB_ERROR("PBSetPWMPortConfig: Failed to set PWM state");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetPWMPortConfig: state=%d, power=%d", newState, newPower);
    }

    if(config->mask & MASK_PORT_OVERCURRENT_RESET)
    {
        if (!device->ResetPWMOvercurrent())
        {
            PB_ERROR("PBSetPWMPortConfig: Failed to reset overcurrent");
            return PB_ERROR_COMMUNICATION;
        }
        PB_DEBUG("PBSetPWMPortConfig: Overcurrent reset");
    }

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetWiFiConfig(int id, PB_WIFI_CONFIG *config)
{
    PB_DEBUG("PBGetWiFiConfig: Getting WiFi config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBGetWiFiConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetWiFiConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    PB_ERROR_TYPE result = device->GetWiFiConfig(config);
    if (result != PB_SUCCESS && result != PB_ERROR_NOT_AVAILABLE)
    {
        PB_ERROR("PBGetWiFiConfig: Failed to get WiFi config, error=%d", result);
    }
    return result;
}

PBAPI PB_ERROR_TYPE PBSetWiFiConfig(int id, PB_WIFI_CONFIG *config)
{
    PB_DEBUG("PBSetWiFiConfig: Setting WiFi config for device id=%d", id);
    if (!config)
    {
        PB_ERROR("PBSetWiFiConfig: config pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBSetWiFiConfig: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    PB_ERROR_TYPE result = device->SetWiFiConfig(config);
    if (result != PB_SUCCESS && result != PB_ERROR_NOT_AVAILABLE)
    {
        PB_ERROR("PBSetWiFiConfig: Failed to set WiFi config, error=%d", result);
    }
    else if(result == PB_SUCCESS)
    {
        PB_DEBUG("PBSetWiFiConfig: WiFi config updated successfully");
    }
    return result;
}

PBAPI PB_ERROR_TYPE PBGetStatus(int id, PB_DEVICE_STATUS *status)
{
    PB_DEBUG("PBGetStatus: Getting device status for id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    status->upTime = device->GetUpTime();
    status->coreTemp = device->GetCoreTemp();
    status->temperature = device->GetTemperature();
    status->humidity = device->GetHumidity();
    status->dewPoint = device->GetDewPoint();
    status->extSensor = device->GetExtSensor();
    status->hasWifi = device->HasWiFi();
    PB_DEBUG("PBGetStatus: upTime=%d, temp=%.2f, humid=%.2f, dew=%.2f, extSensor=%d",
             status->upTime, status->temperature, status->humidity, status->dewPoint, status->extSensor);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetWiFiStatus(int id, PB_WIFI_STATUS *status)
{
    PB_DEBUG("PBGetWiFiStatus: Getting WiFi status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetWiFiStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetWiFiStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    PB_ERROR_TYPE result = device->GetWiFiStatus(status);
    if (result != PB_SUCCESS && result != PB_ERROR_NOT_AVAILABLE)
    {
        PB_ERROR("PBGetWiFiStatus: Failed to get WiFi status, error=%d", result);
    }
    return result;
}

PBAPI PB_ERROR_TYPE PBGetSupplyStatus(int id, PB_SUPPLY_STATUS *status)
{
    PB_DEBUG("PBGetSupplyStatus: Getting supply status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetSupplyStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetSupplyStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;

    status->mainVoltage = device->GetSupply12V();
    status->usbVoltage = device->GetSupply5V();
    status->current = device->GetSupply12A();
    status->averageAmperes = device->GetSupply12AverageA();
    status->ampereHours = device->GetSupply12Ah();
    status->wattHours = device->GetSupply12Wh();
    PB_DEBUG("PBGetSupplyStatus: 12V=%.2fV, 5V=%.2fV, 12A=%.2fA, Ah=%.2f, Wh=%.2f",
             status->mainVoltage, status->usbVoltage, status->current, status->ampereHours, status->wattHours);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBGetPowerPortStatus(int id, PB_POWER_PORT_STATUS *status)
{
    PB_DEBUG("PBGetPowerPortStatus: Getting power port status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetPowerPortStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetPowerPortStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    return device->GetPowerPortStatus(status);
}

PBAPI PB_ERROR_TYPE PBGetUSBPortStatus(int id, PB_USB_PORT_STATUS *status)
{
    PB_DEBUG("PBGetUSBPortStatus: Getting USB port status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetUSBPortStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetUSBPortStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    return device->GetUSBPortStatus(status);
}

PBAPI PB_ERROR_TYPE PBGetDewPortStatus(int id, PB_DEW_PORT_STATUS *status)
{
    PB_DEBUG("PBGetDewPortStatus: Getting dew port status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetDewPortStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetDewPortStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    return device->GetDewPortStatus(status);
}

PBAPI PB_ERROR_TYPE PBGetBuckPortStatus(int id, PB_BUCK_PORT_STATUS *status)
{
    PB_DEBUG("PBGetBuckPortStatus: Getting buck port status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetBuckPortStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetBuckPortStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    return device->GetBuckPortStatus(status);
}

PBAPI PB_ERROR_TYPE PBGetPWMPortStatus(int id, PB_PWM_PORT_STATUS *status)
{
    PB_DEBUG("PBGetPWMPortStatus: Getting PWM port status for device id=%d", id);
    if (!status)
    {
        PB_ERROR("PBGetPWMPortStatus: status pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetPWMPortStatus: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    return device->GetPWMPortStatus(status);
}

PBAPI PB_ERROR_TYPE PBGetVersion(int id, PB_VERSION *version)
{
    PB_DEBUG("PBGetVersion: Getting version info for device id=%d", id);
    if (!version)
    {
        PB_ERROR("PBGetVersion: version pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBGetVersion: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    version->firmware = device->GetFirmwareVersion();

    // Model
    strncpy(version->model, device->GetModelType().c_str(), sizeof(version->model) - 1);
    version->model[sizeof(version->model) - 1] = '\0';

    // UUID
    strncpy(version->uuid, device->GetUUID().c_str(), sizeof(version->uuid) - 1);
    version->uuid[sizeof(version->uuid) - 1] = '\0';

    // Serial
    strncpy(version->serial, device->GetSerial().c_str(), 9);
    version->serial[9] = '\0';

    PB_DEBUG("PBGetVersion: model=%s, serial=%s, firmware=%d", version->model, version->serial, version->firmware);

    return PB_SUCCESS;
}

PBAPI PB_ERROR_TYPE PBBeep(int id, int volume, int duration_ms)
{
    PB_DEBUG("PBBeep: Beeping with device id=%d", id);
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBBeep: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    PB_ERROR_TYPE result = device->Beep(volume, duration_ms);
    if (result == PB_SUCCESS)
    {
        PB_DEBUG("PBBeep: Device beep triggered");
    }
    else
    {
        PB_ERROR("PBBeep: Failed to beep device, error=%d", result);
    }
    return result;
}

PBAPI PB_ERROR_TYPE PBRestart(int id)
{
    PB_DEBUG("PBRestart: Restarting device id=%d", id);
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBRestart: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    PB_ERROR_TYPE result = device->Restart();
    if (result == PB_SUCCESS)
    {
        PB_DEBUG("PBRestart: Device restart initiated");
    }
    else
    {
        PB_ERROR("PBRestart: Failed to restart device, error=%d", result);
    }
    return result;
}

PBAPI PB_ERROR_TYPE PBFactoryReset(int id)
{
    PB_DEBUG("PBFactoryReset: Performing factory reset on device id=%d", id);
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        PB_ERROR("PBFactoryReset: Device id=%d not found", id);
        return PB_ERROR_INVALID_ID;
    }

    auto device = it->second;
    PB_ERROR_TYPE result = device->FactoryReset();
    if (result == PB_SUCCESS)
    {
        PB_DEBUG("PBFactoryReset: Factory reset initiated");
    }
    else
    {
        PB_ERROR("PBFactoryReset: Failed to factory reset device, error=%d", result);
    }
    return result;
}

PBAPI PB_ERROR_TYPE PBGetSDKVersion(char *version)
{
    PB_DEBUG("PBGetSDKVersion: Getting SDK version");
    if (!version)
    {
        PB_ERROR("PBGetSDKVersion: version pointer is null");
        return PB_ERROR_NULL_POINTER;
    }

    strncpy(version, SDK_VERSION, PB_VERSION_LEN - 1);
    version[PB_VERSION_LEN - 1] = '\0';
    PB_DEBUG("PBGetSDKVersion: SDK version=%s", version);
    return PB_SUCCESS;
}
