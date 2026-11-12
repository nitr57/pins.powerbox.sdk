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

#include "PowerBoxSDK.h"
#include "PowerBoxDevice.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <algorithm>
#include <chrono>
#include <thread>

void PrintMenu()
{
    printf("\n=== Power Box Control Menu ===\n");
    printf("1. Get device status\n");
    printf("2. Get device configuration\n");
    printf("3. Set device configuration\n");
    printf("4. Configure power ports\n");
    printf("5. Configure USB ports\n");
    printf("6. Configure dew ports\n");
    printf("7. Configure buck port\n");
    printf("8. Configure PWM port\n");
    printf("9. Open device\n");
    printf("10. Close device\n");
    printf("11. Restart device\n");
    printf("12. Factory reset device\n");
    printf("13. Get supply status\n");
    printf("14. Get port status\n");
    printf("15. Refresh and display all info\n");
    printf("16. Test get/set config\n");
    printf("17. WiFi Survey\n");
    printf("18. WiFi Info\n");
    printf("19. WiFi Config (Get)\n");
    printf("20. WiFi Config (Set)\n");
    printf("21. Buzzer test\n");
    printf("22. Exit\n");
    printf("> ");
}

void DisplayStatus(int deviceId)
{
    PB_DEVICE_STATUS status;
    PB_ERROR_TYPE result = PBGetStatus(deviceId, &status);
    if (result == PB_SUCCESS)
    {
        printf("\n--- Device Status ---\n");
        printf("Up time: %d s\n", status.upTime);
        printf("Temperature: %.2f °C\n", status.temperature);
        printf("Humidity: %.2f %%\n", status.humidity);
        printf("Dew Point: %.2f °C\n", status.dewPoint);
    }
    else
    {
        printf("[FAIL] Failed to get status (Error: %d)\n", result);
    }
} 

void DisplayConfig(int deviceId)
{
    PB_DEVICE_CONFIG config_v;
    PB_ERROR_TYPE result = PBGetConfig(deviceId, &config_v);
    if (result == PB_SUCCESS)
    {
        printf("\n--- Current Configuration ---\n");
        printf("Mask: 0x%X\n", config_v.mask);
        printf("Temperature Offset: %.2f °C\n", config_v.temperatureOffset);
        printf("Humidity Offset: %.2f %%\n", config_v.humidityOffset);
        printf("Update Rate: %d s\n", config_v.envUpdateRate);
    }
    else
    {
        printf("[FAIL] Failed to get config (Error: %d)\n", result);
    }
}

void DisplaySupplyStatus(int deviceId)
{
    PB_SUPPLY_STATUS supply;
    PB_ERROR_TYPE result = PBGetSupplyStatus(deviceId, &supply);
    if (result == PB_SUCCESS)
    {
        printf("\n--- Supply Status ---\n");
        printf("Main Voltage: %.2f V\n", supply.mainVoltage);
        printf("USB Voltage: %.2f V\n", supply.usbVoltage);
        printf("Current: %.2f A\n", supply.current);
        printf("Ampere Hours: %.2f Ah\n", supply.ampereHours);
        printf("Watt Hours: %.2f Wh\n", supply.wattHours);
    }
    else
    {
        printf("[FAIL] Failed to get supply status (Error: %d)\n", result);
    }
}

void DisplayPortStatus(int deviceId)
{
    printf("\n--- Power Port Status ---\n");
    PB_POWER_PORT_STATUS powerPorts;
    PB_ERROR_TYPE result = PBGetPowerPortStatus(deviceId, &powerPorts);
    if (result == PB_SUCCESS)
    {
        for (int i = 0; i < powerPorts.numPorts; ++i)
        {
            PB_POWER_PORT_CONFIG config = {};
            config.index = i;
            PB_ERROR_TYPE cfgResult = PBGetPowerPortConfig(deviceId, &config);
            
            const char *overcurrent = powerPorts.overcurrent[i] ? "[OVERCURRENT]" : "OK";
            const char *readOnly = powerPorts.readOnly[i] ? "[READONLY]" : "";
            if (cfgResult == PB_SUCCESS)
            {
                const char *enabled = config.enabled ? "ENABLED" : "DISABLED";
                const char *bootState = config.bootState ? "ON" : "OFF";
                printf("Power Port %d: %.3f A - State: %s, Boot: %s, %s %s\n", 
                       i + 1, powerPorts.current[i], enabled, bootState, overcurrent, readOnly);
            }
            else
            {
                printf("Power Port %d: %.3f A - %s %s (config unavailable)\n", i + 1, powerPorts.current[i], overcurrent, readOnly);
            }
        }
    }
    else
    {
        printf("[FAIL] Failed to get power port status (Error: %d)\n", result);
    }

    printf("\n--- USB Port Status ---\n");
    PB_USB_PORT_STATUS usbPorts;
    result = PBGetUSBPortStatus(deviceId, &usbPorts);
    if (result == PB_SUCCESS)
    {
        for (int i = 0; i < usbPorts.numPorts; ++i)
        {
            PB_USB_PORT_CONFIG config = {};
            config.index = i;
            PB_ERROR_TYPE cfgResult = PBGetUSBPortConfig(deviceId, &config);
            
            const char *overcurrent = usbPorts.overcurrent[i] ? "[OVERCURRENT]" : "OK";
            const char *readOnly = usbPorts.readOnly[i] ? "[READONLY]" : "";
            if (cfgResult == PB_SUCCESS)
            {
                const char *enabled = config.enabled ? "ENABLED" : "DISABLED";
                const char *bootState = config.bootState ? "ON" : "OFF";
                printf("USB Port %d: %.3f A, %.2f V - State: %s, Boot: %s, %s %s\n", 
                       i + 1, usbPorts.current[i], usbPorts.voltage[i], enabled, bootState, overcurrent, readOnly);
            }
            else
            {
                printf("USB Port %d: %.3f A, %.2f V - %s %s (config unavailable)\n", 
                       i + 1, usbPorts.current[i], usbPorts.voltage[i], overcurrent, readOnly);
            }
        }
    }
    else
    {
        printf("[FAIL] Failed to get USB port status (Error: %d)\n", result);
    }

    printf("\n--- Dew Port Status ---\n");
    PB_DEW_PORT_STATUS dewPorts;
    result = PBGetDewPortStatus(deviceId, &dewPorts);
    if (result == PB_SUCCESS)
    {
        for (int i = 0; i < dewPorts.numPorts; ++i)
        {
            const char *state = dewPorts.state[i] ? "ON" : "OFF";
            const char *overcurrent = dewPorts.overcurrent[i] ? "[OVERCURRENT]" : "OK";
            
            printf("Dew Port %d: %.3f A, Probe Temp: %.2f °C, PWM: %d%%, State: %s - %s\n", 
                   i + 1, dewPorts.current[i], dewPorts.probe[i], dewPorts.pwm[i], state, overcurrent);
        }
    }
    else
    {
        printf("[FAIL] Failed to get dew port status (Error: %d)\n", result);
    }

    printf("\n--- Auxiliary Ports ---\n");
    PB_BUCK_PORT_STATUS buckPort;
    result = PBGetBuckPortStatus(deviceId, &buckPort);
    if (result == PB_SUCCESS)
    {
        const char *buckOvercurrent = buckPort.overcurrent ? "[OVERCURRENT]" : "OK";
        printf("Buck Port: %.3f A, %.2f V (Set: %.2f V, Min: %.2f V, Max: %.2f V) - %s\n", 
               buckPort.current, buckPort.voltage, buckPort.vset, buckPort.vmin, buckPort.vmax, buckOvercurrent);
    }
    else
    {
        printf("[FAIL] Failed to get buck port status (Error: %d)\n", result);
    }

    PB_PWM_PORT_STATUS pwmPort;
    result = PBGetPWMPortStatus(deviceId, &pwmPort);
    if (result == PB_SUCCESS)
    {
        const char *pwmOvercurrent = pwmPort.overcurrent ? "[OVERCURRENT]" : "OK";
        printf("PWM Port: %.3f A, PWM: %d%% - %s\n", pwmPort.current, pwmPort.pwm, pwmOvercurrent);
    }
    else
    {
        printf("[FAIL] Failed to get PWM port status (Error: %d)\n", result);
    }
}

void SetConfigInteractive(int deviceId)
{
    PB_DEVICE_CONFIG curCfg;
    if (PBGetConfig(deviceId, &curCfg) != PB_SUCCESS)
    {
        printf("[FAIL] Could not read current configuration\n");
        return;
    }

    PB_DEVICE_CONFIG updates = {};
    unsigned int mask = 0;
    char input[128];

    printf("\nSet new configuration values (leave empty to keep current)\n");

    printf("Temperature Offset (current: %.4f): ", curCfg.temperatureOffset);
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.temperatureOffset = (float)atof(input);
        mask |= MASK_PB_TEMPERATURE_OFFSET;
    }

    printf("Humidity Offset (current: %.4f): ", curCfg.humidityOffset);
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.humidityOffset = (float)atof(input);
        mask |= MASK_PB_HUMIDITY_OFFSET;
    }

    printf("Environment Update Rate in seconds (current: %d): ", curCfg.envUpdateRate);
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.envUpdateRate = atoi(input);
        mask |= MASK_PB_ENV_UPDATE_RATE;
    }

    printf("Property Update Rate in seconds (current: %d): ", curCfg.updateRate);
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.updateRate = atoi(input);
        mask |= MASK_PB_UPDATE_RATE;
    }

    if (mask == 0)
    {
        printf("No changes provided; nothing to set.\n");
        return;
    }

    updates.mask = mask;
    PB_ERROR_TYPE r = PBSetConfig(deviceId, &updates);
    if (r == PB_SUCCESS)
    {
        printf("Configuration updated successfully.\n");
    }
    else
    {
        printf("Failed to set configuration (Error: %d)\n", r);
    }
}

void DisplayAllInfo(int deviceId)
{
    PB_VERSION version;
    PB_ERROR_TYPE result = PBGetVersion(deviceId, &version);
    if (result == PB_SUCCESS)
    {
        printf("\n=== Device Information ===\n");
        printf("Model: %s\n", version.model);
        printf("Firmware: %u\n", version.firmware);
        printf("UUID: %s\n", version.uuid);
        printf("DeviceID: %s\n", version.serial);
    }
    else
    {
        printf("[FAIL] Failed to get version (Error: %d)\n", result);
        return;
    }

    DisplayStatus(deviceId);
    DisplayConfig(deviceId);
    DisplaySupplyStatus(deviceId);
    DisplayPortStatus(deviceId);
}

void TestGetSetConfig(int deviceId)
{
    printf("\n=== Testing Get/Set Configuration ===\n");

    /* Get current configuration */
    printf("\nGetting current device configuration...\n");
    PB_DEVICE_CONFIG config;
    PB_ERROR_TYPE result = PBGetConfig(deviceId, &config);
    
    if (result != PB_SUCCESS)
    {
        printf("[FAIL] Failed to get configuration (Error: %d)\n", result);
        return;
    }

    printf("[OK] Current Configuration:\n");
    printf("  Temperature Offset: %.2f °C\n", config.temperatureOffset);
    printf("  Humidity Offset: %.2f %%\n", config.humidityOffset);
    printf("  Environment Update Rate: %d s\n", config.envUpdateRate);
    printf("  Property Update Rate: %d s\n", config.updateRate);

    /* Test setting temperature offset */
    printf("\nTesting set temperature offset...\n");
    PB_DEVICE_CONFIG updateCfg = {};
    updateCfg.mask = MASK_PB_TEMPERATURE_OFFSET;
    updateCfg.temperatureOffset = 0.5f;
    
    result = PBSetConfig(deviceId, &updateCfg);
    if (result == PB_SUCCESS)
    {
        printf("[OK] Temperature offset updated\n");
        
        /* Verify by getting config again */
        PB_DEVICE_CONFIG verifyConfig;
        result = PBGetConfig(deviceId, &verifyConfig);
        if (result == PB_SUCCESS)
        {
            printf("  Verified temperature offset: %.2f °C\n", verifyConfig.temperatureOffset);
        }
    }
    else
    {
        printf("[FAIL] Failed to set temperature offset (Error: %d)\n", result);
    }

    /* Test setting humidity offset */
    printf("\nTesting set humidity offset...\n");
    updateCfg = {};
    updateCfg.mask = MASK_PB_HUMIDITY_OFFSET;
    updateCfg.humidityOffset = 1.0f;
    
    result = PBSetConfig(deviceId, &updateCfg);
    if (result == PB_SUCCESS)
    {
        printf("[OK] Humidity offset updated\n");
        
        /* Verify by getting config again */
        PB_DEVICE_CONFIG verifyConfig;
        result = PBGetConfig(deviceId, &verifyConfig);
        if (result == PB_SUCCESS)
        {
            printf("  Verified humidity offset: %.2f %%\n", verifyConfig.humidityOffset);
        }
    }
    else
    {
        printf("[FAIL] Failed to set humidity offset (Error: %d)\n", result);
    }

    /* Test setting update rates */
    printf("\nTesting set update rates...\n");
    updateCfg = {};
    updateCfg.mask = MASK_PB_ENV_UPDATE_RATE | MASK_PB_UPDATE_RATE;
    updateCfg.envUpdateRate = 5;
    updateCfg.updateRate = 2;
    
    result = PBSetConfig(deviceId, &updateCfg);
    if (result == PB_SUCCESS)
    {
        printf("[OK] Update rates changed\n");
        
        /* Verify by getting config again */
        PB_DEVICE_CONFIG verifyConfig;
        result = PBGetConfig(deviceId, &verifyConfig);
        if (result == PB_SUCCESS)
        {
            printf("  Verified environment update rate: %d s\n", verifyConfig.envUpdateRate);
            printf("  Verified property update rate: %d s\n", verifyConfig.updateRate);
        }
    }
    else
    {
        printf("[FAIL] Failed to set update rates (Error: %d)\n", result);
    }

    /* Restore original config */
    printf("\nRestoring original configuration...\n");
    updateCfg = config;
    updateCfg.mask = MASK_PB_ALL;
    
    result = PBSetConfig(deviceId, &updateCfg);
    if (result == PB_SUCCESS)
    {
        printf("[OK] Configuration restored\n");
    }
    else
    {
        printf("[FAIL] Failed to restore configuration (Error: %d)\n", result);
    }

    printf("\n=== Config Test Complete ===\n");
}

void ConfigurePowerPorts(int deviceId)
{
    PB_POWER_PORT_STATUS ports;
    auto result = PBGetPowerPortStatus(deviceId, &ports);
    if (result == PB_SUCCESS)
    {
        printf("\n=== Power Ports Configuration ===\n");
        printf("You have %d power ports (1-%d)\n\n", ports.numPorts, ports.numPorts);

        for (int port = 0; port < ports.numPorts; port++)
        {
            PB_POWER_PORT_CONFIG config = {};
            config.index = port;
        
            PB_ERROR_TYPE result = PBGetPowerPortConfig(deviceId, &config);
            if (result != PB_SUCCESS)
            {
                printf("[FAIL] Failed to get config for power port %d (Error: %d)\n", port + 1, result);
                continue;
            }

            printf("--- Power Port %d ---\n", port + 1);
            printf("Current Enabled: %s\n", config.enabled ? "YES" : "NO");
            printf("Current Boot State: %s\n\n", config.bootState ? "ON" : "OFF");

            char input[128];
            unsigned int mask = 0;
            PB_POWER_PORT_CONFIG updates = {};
            updates.index = port;

            printf("Enable port? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.enabled = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_ENABLE;
            }

            printf("Boot state (1=ON, 0=OFF, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.bootState = atoi(input);
                mask |= MASK_PORT_BOOT_STATE;
            }

            printf("Reset overcurrent? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.overcurrentReset = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_OVERCURRENT_RESET;
            }

            if (mask > 0)
            {
                updates.mask = mask;
                result = PBSetPowerPortConfig(deviceId, &updates);
                if (result == PB_SUCCESS)
                {
                    printf("[OK] Power port %d updated\n\n", port + 1);
                }
                else
                {
                    printf("[FAIL] Failed to update power port %d (Error: %d)\n\n", port + 1, result);
                }
            }
            else
            {
                printf("No changes for power port %d\n\n", port + 1);
            }
        }
    }
}

void ConfigureUSBPorts(int deviceId)
{
    PB_USB_PORT_STATUS ports;
    auto result = PBGetUSBPortStatus(deviceId, &ports);
    if (result == PB_SUCCESS)
    {
        printf("\n=== USB Ports Configuration ===\n");
        printf("You have %d USB ports (1-%d)\n\n", ports.numPorts, ports.numPorts);

        for (int port = 0; port < ports.numPorts; port++)
        {
            PB_USB_PORT_CONFIG config = {};
            config.index = port;
            
            PB_ERROR_TYPE result = PBGetUSBPortConfig(deviceId, &config);
            if (result != PB_SUCCESS)
            {
                printf("[FAIL] Failed to get config for USB port %d (Error: %d)\n", port + 1, result);
                continue;
            }

            printf("--- USB Port %d ---\n", port + 1);
            printf("Current Enabled: %s\n", config.enabled ? "YES" : "NO");
            printf("Current Boot State: %s\n\n", config.bootState ? "ON" : "OFF");

            char input[128];
            unsigned int mask = 0;
            PB_USB_PORT_CONFIG updates = {};
            updates.index = port;

            printf("Enable port? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.enabled = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_ENABLE;
            }

            printf("Boot state (1=ON, 0=OFF, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.bootState = atoi(input);
                mask |= MASK_PORT_BOOT_STATE;
            }

            printf("Reset overcurrent? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.overcurrentReset = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_OVERCURRENT_RESET;
            }

            if (mask > 0)
            {
                updates.mask = mask;
                result = PBSetUSBPortConfig(deviceId, &updates);
                if (result == PB_SUCCESS)
                {
                    printf("[OK] USB port %d updated\n\n", port + 1);
                }
                else
                {
                    printf("[FAIL] Failed to update USB port %d (Error: %d)\n\n", port + 1, result);
                }
            }
            else
            {
                printf("No changes for USB port %d\n\n", port + 1);
            }
        }
    }
}

void ConfigureDewPorts(int deviceId)
{
    PB_DEW_PORT_STATUS ports;
    auto result = PBGetDewPortStatus(deviceId, &ports);
    if (result == PB_SUCCESS)
    {
        printf("\n=== Dew Ports Configuration ===\n");
        printf("You have %d dew ports (1-%d)\n\n", ports.numPorts, ports.numPorts);

        /* Get PWM resolution from status */
        PB_DEW_PORT_STATUS dewStatus;
        PB_ERROR_TYPE statusResult = PBGetDewPortStatus(deviceId, &dewStatus);
        int maxPWMValue = 255; /* Default, will be updated if status is available */
        
        if (statusResult == PB_SUCCESS && dewStatus.pwmResolution > 0)
        {
            maxPWMValue = (1 << dewStatus.pwmResolution) - 1; /* 2^resolution - 1 */
            printf("PWM Resolution: %d bits (max value: %d)\n\n", dewStatus.pwmResolution, maxPWMValue);
        }
        else
        {
            printf("Warning: Could not determine PWM resolution, using default max value: %d\n\n", maxPWMValue);
        }

        for (int port = 0; port < ports.numPorts; port++)
        {
            PB_DEW_PORT_CONFIG config = {};
            config.index = port;
            
            PB_ERROR_TYPE result = PBGetDewPortConfig(deviceId, &config);
            if (result != PB_SUCCESS)
            {
                printf("[FAIL] Failed to get config for dew port %d (Error: %d)\n", port + 1, result);
                continue;
            }

            printf("--- Dew Port %d ---\n", port + 1);
            printf("Current Enabled: %s\n", config.enabled ? "YES" : "NO");
            printf("Current Auto Mode: %s\n", config.autoMode ? "YES" : "NO");
            printf("Current Auto Threshold: %.2f °C\n", config.autoThreshold);
            printf("Current PWM Value: %d (0-%d)\n\n", config.power, maxPWMValue);

            char input[128];
            unsigned int mask = 0;
            PB_DEW_PORT_CONFIG updates = {};
            updates.index = port;

            printf("Enable port? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.enabled = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_ENABLE;
            }

            printf("Auto mode? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.autoMode = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_AUTO_DEW_MODE;
            }

            printf("Auto threshold in °C (leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.autoThreshold = (float)atof(input);
                mask |= MASK_PORT_AUTO_DEW_THRESHOLD;
            }

            printf("PWM value 0-%d (leave empty to skip): ", maxPWMValue);
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.power = atoi(input);
                mask |= MASK_PORT_POWER;
            }

            printf("Reset overcurrent? (y/n, leave empty to skip): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
            {
                updates.overcurrentReset = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
                mask |= MASK_PORT_OVERCURRENT_RESET;
            }

            if (mask > 0)
            {
                updates.mask = mask;
                result = PBSetDewPortConfig(deviceId, &updates);
                if (result == PB_SUCCESS)
                {
                    printf("[OK] Dew port %d updated\n\n", port + 1);
                }
                else
                {
                    printf("[FAIL] Failed to update dew port %d (Error: %d)\n\n", port + 1, result);
                }
            }
            else
            {
                printf("No changes for dew port %d\n\n", port + 1);
            }
        }
    }
}

void ConfigureBuckPort(int deviceId)
{
    printf("\n=== Buck Port Configuration ===\n");

    PB_BUCK_PORT_CONFIG config = {};
    PB_ERROR_TYPE result = PBGetBuckPortConfig(deviceId, &config);
    
    if (result != PB_SUCCESS)
    {
        printf("[FAIL] Failed to get buck port configuration (Error: %d)\n", result);
        return;
    }

    printf("Current Enabled: %s\n", config.enabled ? "YES" : "NO");
    printf("Current Boot State: %s\n", config.bootState ? "ON" : "OFF");
    printf("Target Voltage: %.2f V\n\n", config.targetVoltage);

    char input[128];
    unsigned int mask = 0;
    PB_BUCK_PORT_CONFIG updates = {};

    printf("Enable port? (y/n, leave empty to skip): ");
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.enabled = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
        mask |= MASK_PORT_ENABLE;
    }

    printf("Boot state (1=ON, 0=OFF, leave empty to skip): ");
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.bootState = atoi(input);
        mask |= MASK_PORT_BOOT_STATE;
    }

    printf("Output voltage in V (leave empty to skip): ");
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.targetVoltage = (float)atof(input);
        mask |= MASK_PORT_VOLTAGE;
    }

    printf("Reset overcurrent? (y/n, leave empty to skip): ");
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.overcurrentReset = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
        mask |= MASK_PORT_OVERCURRENT_RESET;
    }

    if (mask > 0)
    {
        updates.mask = mask;
        result = PBSetBuckPortConfig(deviceId, &updates);
        if (result == PB_SUCCESS)
        {
            printf("[OK] Buck port configuration updated\n");
        }
        else
        {
            printf("[FAIL] Failed to update buck port (Error: %d)\n", result);
        }
    }
    else
    {
        printf("No changes for buck port\n");
    }
}

void ConfigurePWMPort(int deviceId)
{
    printf("\n=== PWM Port Configuration ===\n");

    /* Get PWM resolution from status */
    PB_PWM_PORT_STATUS pwmStatus;
    PB_ERROR_TYPE statusResult = PBGetPWMPortStatus(deviceId, &pwmStatus);
    int maxPWMValue = 255; /* Default, will be updated if status is available */
    
    if (statusResult == PB_SUCCESS && pwmStatus.pwmResolution > 0)
    {
        maxPWMValue = (1 << pwmStatus.pwmResolution) - 1; /* 2^resolution - 1 */
        printf("PWM Resolution: %d bits (max value: %d)\n\n", pwmStatus.pwmResolution, maxPWMValue);
    }
    else
    {
        printf("Warning: Could not determine PWM resolution, using default max value: %d\n\n", maxPWMValue);
    }

    PB_PWM_PORT_CONFIG config = {};
    PB_ERROR_TYPE result = PBGetPWMPortConfig(deviceId, &config);
    
    if (result != PB_SUCCESS)
    {
        printf("[FAIL] Failed to get PWM port configuration (Error: %d)\n", result);
        return;
    }

    printf("Current Enabled: %s\n", config.enabled ? "YES" : "NO");
    printf("Current PWM Value: %d (0-%d)\n\n", config.power, maxPWMValue);

    char input[128];
    unsigned int mask = 0;
    PB_PWM_PORT_CONFIG updates = {};

    printf("Enable port? (y/n, leave empty to skip): ");
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.enabled = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
        mask |= MASK_PORT_ENABLE;
    }

    printf("PWM value 0-%d (leave empty to skip): ", maxPWMValue);
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.power = atoi(input);
        mask |= MASK_PORT_POWER;
    }

    printf("Reset overcurrent? (y/n, leave empty to skip): ");
    fflush(stdout);
    if (fgets(input, sizeof(input), stdin) && input[0] != '\n')
    {
        updates.overcurrentReset = (input[0] == 'y' || input[0] == 'Y') ? 1 : 0;
        mask |= MASK_PORT_OVERCURRENT_RESET;
    }

    if (mask > 0)
    {
        updates.mask = mask;
        result = PBSetPWMPortConfig(deviceId, &updates);
        if (result == PB_SUCCESS)
        {
            printf("[OK] PWM port configuration updated\n");
        }
        else
        {
            printf("[FAIL] Failed to update PWM port (Error: %d)\n", result);
        }
    }
    else
    {
        printf("No changes for PWM port\n");
    }
}

int main(int argc, char *argv[])
{
    printf("=== Power Box Interactive Control ===\n\n");

    /* Get SDK version */
    char sdkVersion[32];
    PB_ERROR_TYPE result = PBGetSDKVersion(sdkVersion);
    if (result == PB_SUCCESS)
    {
        printf("SDK Version: %s\n\n", sdkVersion);
    }

    /* Scan for devices */
    printf("Scanning for Power Box devices...\n");
    int deviceCount = 32;
    int deviceIds[32] = {0};

    result = PBScan(&deviceCount, deviceIds);
    if (result != PB_SUCCESS)
    {
        printf("[FAIL] Scan failed (Error: %d)\n", result);
        return 1;
    }

    printf("[OK] Found %d device(s)\n\n", deviceCount);

    if (deviceCount == 0)
    {
        printf("No Power Box devices found. Please connect a device.\n");
        return 0;
    }

    /* List found devices */
    printf("Available devices:\n");
    for (int i = 0; i < deviceCount; i++)
    {
        printf("  [%d] Device ID: %d\n", i, deviceIds[i]);
    }
    printf("\n");

    /* Select device */
    int deviceId = deviceIds[0];
    if (deviceCount > 1)
    {
        printf("Enter device index to use (0-%d) [default: 0]: ", deviceCount - 1);
        char input[10];
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            int idx = atoi(input);
            if (idx >= 0 && idx < deviceCount)
            {
                deviceId = deviceIds[idx];
            }
        }
    }

    printf("Using device: %d\n\n", deviceId);

    /* Start with device closed; user may open via menu option 9 */
    bool deviceOpened = false;
    printf("Device is currently closed. Use menu option 9 to open it.\n\n");

    /* Interactive menu */
    char input[256];
    bool running = true;

    while (running)
    {
        PrintMenu();
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL)
        {
            break;
        }

        int choice = atoi(input);

        switch (choice)
        {
        case 1:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                DisplayStatus(deviceId);
            }
            break;

        case 2:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                DisplayConfig(deviceId);
            }
            break;

        case 3:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                SetConfigInteractive(deviceId);
            }
            break;

        case 4:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                ConfigurePowerPorts(deviceId);
            }
            break;

        case 5:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                ConfigureUSBPorts(deviceId);
            }
            break;

        case 6:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                ConfigureDewPorts(deviceId);
            }
            break;

        case 7:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                ConfigureBuckPort(deviceId);
            }
            break;

        case 8:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                ConfigurePWMPort(deviceId);
            }
            break;

        case 9:
            if (deviceOpened)
            {
                printf("Device already open.\n");
            }
            else
            {
                printf("Opening device...\n");
                result = PBOpen(deviceId);
                if (result == PB_SUCCESS)
                {
                    deviceOpened = true;
                    printf("[OK] Device opened\n");
                }
                else
                {
                    printf("[FAIL] Failed to open device (Error: %d)\n", result);
                }
            }
            break;

        case 10:
            if (!deviceOpened)
            {
                printf("Device already closed.\n");
            }
            else
            {
                printf("Closing device...\n");
                result = PBClose(deviceId);
                if (result == PB_SUCCESS)
                {
                    deviceOpened = false;
                    printf("[OK] Device closed\n");
                }
                else
                {
                    printf("[FAIL] Failed to close device (Error: %d)\n", result);
                }
            }
            break;

        case 11:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("Restarting device...\n");
                result = PBRestart(deviceId);
                if (result == PB_SUCCESS)
                {
                    printf("[OK] Restart command sent\n");
                }
                else
                {
                    printf("[FAIL] Failed to restart device (Error: %d)\n", result);
                }
            }
            break;

        case 12:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("Factory reset will restore device to defaults. Are you sure? (y/N): ");
                fflush(stdout);
                char conf[8];
                if (fgets(conf, sizeof(conf), stdin) && (conf[0] == 'y' || conf[0] == 'Y'))
                {
                    printf("Performing factory reset...\n");
                    result = PBFactoryReset(deviceId);
                    if (result == PB_SUCCESS)
                    {
                        printf("[OK] Factory reset command sent\n");
                    }
                    else
                    {
                        printf("[FAIL] Failed to factory reset device (Error: %d)\n", result);
                    }
                }
                else
                {
                    printf("Factory reset cancelled.\n");
                }
            }
            break;

        case 13:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                DisplaySupplyStatus(deviceId);
            }
            break;

        case 14:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                DisplayPortStatus(deviceId);
            }
            break;

        case 15:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                DisplayAllInfo(deviceId);
            }
            break;

        case 16:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                TestGetSetConfig(deviceId);
            }
            break;

        case 17:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("Scanning WiFi networks...\n");
                PB_WIFI_SCAN_RESULT result;
                PB_ERROR_TYPE error = PBScanWiFi(deviceId, &result);
                if (error == PB_SUCCESS)
                {
                    printf("\n--- WiFi Networks Found: %d ---\n", result.count);
                    for (int i = 0; i < result.count; i++)
                    {
                        printf("  %d. SSID: %-32s RSSI: %d dBm\n", i + 1, 
                               result.networks[i].ssid, result.networks[i].rssi);
                    }
                }
                else
                {
                    printf("[FAIL] Failed to scan WiFi networks (Error: %d)\n", error);
                }
            }
            break;

        case 18:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("Getting WiFi info...\n");
                PB_WIFI_STATUS wifiStatus;
                PB_ERROR_TYPE error = PBGetWiFiStatus(deviceId, &wifiStatus);
                if (error == PB_SUCCESS)
                {
                    printf("\n--- WiFi Status ---\n");
                    printf("IP Address: %s\n", wifiStatus.IP);
                    printf("Signal Strength: %d dBm\n", wifiStatus.rssi);
                }
                else
                {
                    printf("[FAIL] Failed to get WiFi status (Error: %d)\n", error);
                }
            }
            break;

        case 19:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("Getting WiFi config...\n");
                PB_WIFI_CONFIG wifiConfig;
                PB_ERROR_TYPE error = PBGetWiFiConfig(deviceId, &wifiConfig);
                if (error == PB_SUCCESS)
                {
                    const char *modeStr = (wifiConfig.mode == PB_WIFI_MODE_AP) ? "AP" :
                                         (wifiConfig.mode == PB_WIFI_MODE_CLIENT) ? "Client" : "Off";
                    printf("\n--- WiFi Configuration ---\n");
                    printf("Mode: %s\n", modeStr);
                    printf("Channel: %d\n", wifiConfig.channel);
                    printf("SSID: %s\n", wifiConfig.ssid);
                    printf("Hostname: %s\n", wifiConfig.hostname);
                }
                else
                {
                    printf("[FAIL] Failed to get WiFi config (Error: %d)\n", error);
                }
            }
            break;

        case 20:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("Setting WiFi configuration...\n");
                
                PB_WIFI_CONFIG wifiConfig;
                memset(&wifiConfig, 0, sizeof(wifiConfig));
                
                printf("Enter SSID (leave empty to skip): ");
                char ssidInput[PB_SSID_LEN];
                if (fgets(ssidInput, sizeof(ssidInput), stdin) != NULL)
                {
                    // Remove newline
                    size_t len = strlen(ssidInput);
                    if (len > 0 && ssidInput[len-1] == '\n')
                        ssidInput[len-1] = '\0';
                    
                    if (strlen(ssidInput) > 0)
                    {
                        strncpy(wifiConfig.ssid, ssidInput, PB_SSID_LEN - 1);
                        wifiConfig.ssid[PB_SSID_LEN - 1] = '\0';
                        wifiConfig.mask |= MASK_WIFI_SSID;
                    }
                }
                
                printf("Enter password (leave empty to skip): ");
                char passInput[PB_PASSWORD_LEN];
                if (fgets(passInput, sizeof(passInput), stdin) != NULL)
                {
                    // Remove newline
                    size_t len = strlen(passInput);
                    if (len > 0 && passInput[len-1] == '\n')
                        passInput[len-1] = '\0';
                    
                    if (strlen(passInput) > 0)
                    {
                        strncpy(wifiConfig.pass, passInput, PB_PASSWORD_LEN - 1);
                        wifiConfig.pass[PB_PASSWORD_LEN - 1] = '\0';
                        wifiConfig.mask |= MASK_WIFI_PASSWORD;
                    }
                }
                
                printf("Enter WiFi mode (0=AP, 1=Client, leave empty to skip): ");
                char modeInput[16];
                if (fgets(modeInput, sizeof(modeInput), stdin) != NULL)
                {
                    // Remove newline
                    size_t len = strlen(modeInput);
                    if (len > 0 && modeInput[len-1] == '\n')
                        modeInput[len-1] = '\0';
                    
                    if (strlen(modeInput) > 0)
                    {
                        int mode = atoi(modeInput);
                        if (mode == 0 || mode == 1)
                        {
                            wifiConfig.mode = (PB_WIFI_MODE)mode;
                            wifiConfig.mask |= MASK_WIFI_MODE;
                        }
                        else
                        {
                            printf("[WARN] Invalid mode, skipping.\n");
                        }
                    }
                }
                
                if (wifiConfig.mask == 0)
                {
                    printf("[WARN] No configuration to set.\n");
                }
                else
                {
                    PB_ERROR_TYPE error = PBSetWiFiConfig(deviceId, &wifiConfig);
                    if (error == PB_SUCCESS)
                    {
                        printf("[OK] WiFi configuration set successfully.\n");
                    }
                    else
                    {
                        printf("[FAIL] Failed to set WiFi config (Error: %d)\n", error);
                    }
                }
            }
            break;

        case 21:
            if (!deviceOpened)
            {
                printf("Device not opened. Use option 9 to open it first.\n");
            }
            else
            {
                printf("\n=== Buzzer Test ===\n");
                
                printf("Enter volume (0-100): ");
                int volume;
                if (scanf("%d", &volume) != 1)
                {
                    printf("[FAIL] Invalid input\n");
                    // Clear input buffer
                    int c;
                    while ((c = getchar()) != '\n' && c != EOF);
                    break;
                }
                
                printf("Enter duration in milliseconds: ");
                int duration;
                if (scanf("%d", &duration) != 1)
                {
                    printf("[FAIL] Invalid input\n");
                    // Clear input buffer
                    int c;
                    while ((c = getchar()) != '\n' && c != EOF);
                    break;
                }
                
                // Clear input buffer
                int c;
                while ((c = getchar()) != '\n' && c != EOF);
                
                printf("Testing buzzer with volume=%d%%, duration=%dms...\n", volume, duration);
                PB_ERROR_TYPE error = PBBeep(deviceId, volume, duration);
                if (error == PB_SUCCESS)
                {
                    // PBBeep returns at once: wait for the tone (at most 10 s)
                    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(std::max(duration, 0), 10000)));
                    printf("[OK] Buzzer test completed successfully\n");
                }
                else
                {
                    printf("[FAIL] Buzzer test failed (Error: %d)\n", error);
                }
            }
            break;

        case 22:
            running = false;
            break;

        default:
            printf("[FAIL] Unknown command\n");
            break;
        }
    }

    /* Close device if still open */
    if (deviceOpened)
    {
        printf("\nClosing device...\n");
        result = PBClose(deviceId);
        if (result == PB_SUCCESS)
        {
            printf("[OK] Device closed\n");
        }
        else
        {
            printf("[FAIL] Failed to close device (Error: %d)\n", result);
        }
    } else {
        printf("\nDevice was already closed.\n");
    }

    printf("\n=== Test Complete ===\n");
    return 0;
}
