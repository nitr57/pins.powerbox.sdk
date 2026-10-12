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

#ifndef POWER_BOX_SDK_H
#define POWER_BOX_SDK_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _WINDOWS
#define PBAPI __declspec(dllexport)
#else
#define PBAPI
#endif

#define PB_MAX_NUM 32               /* Maximum device numbers supported by this SDK */
#define PB_MAX_WIFI_NETWORKS 32     /* Maximum WiFi networks in scan results */
#define PB_NAME_LEN 32              /* Buffer length for name strings */
#define PB_VERSION_LEN 32           /* Buffer length for version strings */
#define PB_UUID_LEN 37              /* Buffer length for UUID */
#define PB_PORT_LEN 256             /* Buffer length for port names */
#define PB_IP_LEN 16                /* Buffer length for IP address */
#define PB_SSID_LEN 32              /* Maximum SSID length */
#define PB_HOSTNAME_LEN 32          /* Maximum hostname length */
#define PB_PASSWORD_LEN 64          /* Maximum WiFi password length */

#define PB_MAX_POWER_PORTS 8
#define PB_MAX_USB_PORTS 8
#define PB_MAX_DEW_PORTS 2

    typedef enum _PB_ERROR_TYPE
    {
        PB_SUCCESS = 0,             /* Success */
        PB_ERROR_INVALID_ID,        /* Device ID is invalid */
        PB_ERROR_INVALID_PARAMETER, /* One or more parameters are invalid */
        PB_ERROR_INVALID_STATE,     /* Device is not in correct state for specific API call */
        PB_ERROR_COMMUNICATION,     /* Data communication error such as device has been removed from USB port */
        PB_ERROR_NULL_POINTER,      /* Caller passes null-pointer parameter which is not expected */
        PB_ERROR_TIMEOUT,           /* Function call timed out */
        PB_ERROR_NOT_AVAILABLE,     /* Function not available on given device */
    } PB_ERROR_TYPE;

    typedef enum _PB_WIFI_MODE
    {
        PB_WIFI_MODE_AP = 0,
        PB_WIFI_MODE_CLIENT = 1,
        PB_WIFI_MODE_OFF = 2
    } PB_WIFI_MODE;

/*
 * Used by PBxxxSetConfig() to indicate which field wants to be set
 */
#define MASK_PB_TEMPERATURE_OFFSET 0x01 /* Temperature offset */
#define MASK_PB_HUMIDITY_OFFSET    0x02 /* Humidity offset */
#define MASK_PB_ENV_UPDATE_RATE    0x04 /* Environment update rate */
#define MASK_PB_UPDATE_RATE        0x08 /* Property update rate */
#define MASK_PB_EXT_TEMPERATURE    0x10 /* External temperature */
#define MASK_PB_EXT_HUMIDITY       0x20 /* External humidity */
#define MASK_PB_ALL                0x3F /* All valid masks combined */

    typedef struct _PB_VERSION
    {
        unsigned int firmware;          /* Device firmware version */
        char model[PB_NAME_LEN];        /* Model type (e.g., "Lite", "Mini") */
        char uuid[PB_UUID_LEN];         /* Unique identifier */
        char serial[PB_VERSION_LEN];    /* Device ID */
    } PB_VERSION;

    typedef struct _PB_DEVICE_CONFIG
    {
        unsigned int mask;              /* Used by PBSetConfig() to indicate which field wants to be set */
        float temperatureOffset;        /* Temperature offset [-12.5, +12.5] [°C] */
        float humidityOffset;           /* Temperature offset [-12.5, +12.5] [%] */
        int envUpdateRate;              /* Sensor refresh rate [s], default: 3s */
        int updateRate;                 /* Property refresh rate [s], default: 1s */
        float temperature;              /* Set temperature externally */
        float humidity;                 /* Set humidity externally */
    } PB_DEVICE_CONFIG;

    typedef struct _PB_DEVICE_STATUS
    {
        int upTime;                 /* Up time [s] */
        float coreTemp;             /* Core temperature [°C] */
        float temperature;          /* Ambient temperature [°C] */
        float humidity;             /* Humidity [%] */
        float dewPoint;             /* Dew Point [°C] */
        int extSensor;              /* External sensor connected flag */
        int hasWifi;                /* On-board WiFi */
    } PB_DEVICE_STATUS;

    typedef struct _PB_SUPPLY_STATUS
    {
        float mainVoltage;                           /* Main supply voltage [V] */
        float usbVoltage;                            /* USB supply voltage [V] */
        float current;                               /* Total current draw [A] */
        float averageAmperes;                        /* Average current draw [A] */
        float ampereHours;                           /* Ampere hours [Ah] */
        float wattHours;                             /* Watt hours [Wh] */
    } PB_SUPPLY_STATUS;

    typedef struct _PB_POWER_PORT_STATUS
    {
        int numPorts;                               /* Number of ports */
        float current[PB_MAX_POWER_PORTS];          /* Current draw [A] */
        int overcurrent[PB_MAX_POWER_PORTS];        /* Overcurrent flag */
        int readOnly[PB_MAX_POWER_PORTS];           /* Indicates if the port can be modified */
    } PB_POWER_PORT_STATUS;

    typedef struct _PB_USB_PORT_STATUS
    {
        int numPorts;                               /* Number of ports */
        float current[PB_MAX_USB_PORTS];            /* Current draw [A] */
        float voltage[PB_MAX_USB_PORTS];            /* Output voltage [V] */
        int overcurrent[PB_MAX_USB_PORTS];          /* Overcurrent flag */
        int readOnly[PB_MAX_POWER_PORTS];           /* Indicates if the port can be modified */
    } PB_USB_PORT_STATUS;

    typedef struct _PB_DEW_PORT_STATUS
    {
        int numPorts;                               /* Number of ports */
        int pwmResolution;                          /* PWM resolution (bit depth) */
        float current[PB_MAX_DEW_PORTS];            /* Current draw [A] */
        int overcurrent[PB_MAX_DEW_PORTS];          /* Overcurrent flag */
        float probe[PB_MAX_DEW_PORTS];              /* Probe temperature [°C] */
        int pwm[PB_MAX_DEW_PORTS];                  /* PWM value [%] */
        int state[PB_MAX_DEW_PORTS];                /* Port state (ON/OFF) */
    } PB_DEW_PORT_STATUS;

    typedef struct _PB_BUCK_PORT_STATUS
    {
        int numPorts;                                /* Number of ports */
        float current;                               /* Current draw [A] */
        float voltage;                               /* Output voltage [V] */
        int overcurrent;                             /* Overcurrent flag */
        float vset;                                  /* Set voltage [V] */
        float vmin;                                  /* Minimum voltage [V] */
        float vmax;                                  /* Maximum voltage [V] */
    } PB_BUCK_PORT_STATUS;

    typedef struct _PB_PWM_PORT_STATUS
    {
        int numPorts;                                /* Number of ports */
        int pwmResolution;                           /* PWM resolution (bit depth) */
        float current;                               /* Current draw [A] */
        int overcurrent;                             /* Overcurrent flag */
        int pwm;                                     /* PWM value [%] */
    } PB_PWM_PORT_STATUS;

    typedef struct _PB_WIFI_STATUS
    {
        char IP[PB_IP_LEN];
        int rssi;
    } PB_WIFI_STATUS;

    typedef struct _PB_WIFI_NETWORK
    {
        char ssid[PB_SSID_LEN];
        int rssi;
        int channel;
    } PB_WIFI_NETWORK;

    typedef struct _PB_WIFI_SCAN_RESULT
    {
        int count;
        PB_WIFI_NETWORK networks[PB_MAX_WIFI_NETWORKS];
    } PB_WIFI_SCAN_RESULT;

/*
 * Mask definitions for port configs - reused across different port types
 * Each structure only uses the masks that apply to its fields
 */
#define MASK_PORT_ENABLE                0x01    /* Enable/disable port */
#define MASK_PORT_BOOT_STATE            0x02    /* Port state at boot */
#define MASK_PORT_AUTO_DEW_MODE         0x04    /* Auto dew mode (dew only) */
#define MASK_PORT_AUTO_DEW_THRESHOLD    0x08    /* Auto dew threshold (dew only) */
#define MASK_PORT_POWER                 0x10    /* Power percentage (dew, pwm) */
#define MASK_PORT_VOLTAGE               0x20    /* Voltage (buck only) */
#define MASK_PORT_OVERCURRENT_RESET     0x40    /* Overcurrent reset */
#define MASK_PORT_ALL                   0x7F    /* All valid masks combined */

    typedef struct _PB_POWER_PORT_CONFIG
    {
        unsigned int mask;      /* Which fields to update */
        unsigned int index;     /* Which port to update */
        int enabled;            /* Enable/disable each power port */
        int bootState;          /* Power port state on boot */
        int overcurrentReset;   /* Reset overcurrent flag */
    } PB_POWER_PORT_CONFIG;

    typedef struct _PB_USB_PORT_CONFIG
    {
        unsigned int mask;          /* Which fields to update */
        unsigned int index;         /* Which port to update */
        int enabled;                /* Enable/disable the USB port */
        int bootState;              /* USB port state on boot */
        int overcurrentReset;   /* Reset overcurrent flag */
    } PB_USB_PORT_CONFIG;

    typedef struct _PB_DEW_PORT_CONFIG
    {
        unsigned int mask;          /* Which fields to update */
        unsigned int index;         /* Which port to update */
        int enabled;                /* Enable/disable the dew heater */
        int autoMode;               /* Auto mode (automatic regulation) */
        float autoThreshold;        /* Auto dew threshold gap [°C] (dew point - probe temperature) */
        int power;                  /* Dew heater PWM value (0 to 2^pwmResolution-1, see PB_DEW_PORT_STATUS) */
        int overcurrentReset;   /* Reset overcurrent flag */
    } PB_DEW_PORT_CONFIG;

    typedef struct _PB_BUCK_PORT_CONFIG
    {
        unsigned int mask;          /* Which fields to update */
        unsigned int index;         /* Which port to update */
        float targetVoltage;        /* Target voltage [V] */
        int enabled;                /* Enable/disable the buck converter */
        int bootState;              /* Buck converter state on boot */
        int overcurrentReset;   /* Reset overcurrent flag */
    } PB_BUCK_PORT_CONFIG;

    typedef struct _PB_PWM_PORT_CONFIG
    {
        unsigned int mask;          /* Which fields to update */
        unsigned int index;         /* Which port to update */
        int enabled;                /* Enable/disable the PWM port */
        int power;                  /* PWM value (0 to 2^pwmResolution-1, see PB_PWM_PORT_STATUS) */
        int overcurrentReset;   /* Reset overcurrent flag */
    } PB_PWM_PORT_CONFIG;

#define MASK_WIFI_MODE                  0x01
#define MASK_WIFI_SSID                  0x02
#define MASK_WIFI_PASSWORD              0x04
#define MASK_WIFI_CHANNEL               0x08
#define MASK_WIFI_HOSTNAME              0x10
#define MASK_WIFI_ALL                   0x1F

    typedef struct _PB_WIFI_CONFIG
    {
        unsigned int mask;
        PB_WIFI_MODE mode;
        char ssid[PB_SSID_LEN];
        char pass[PB_PASSWORD_LEN];
        char hostname[PB_HOSTNAME_LEN];
        int channel;
    } PB_WIFI_CONFIG;

    /* Device scanning and management */
    PBAPI PB_ERROR_TYPE PBScan(int *number, int *ids);
#ifdef __unix__
    /* Probes only the given port (as returned by PBGetPort, or a device node
     * such as /dev/ttyUSB0) for a USB PowerBox instead of all CP210x ports, so
     * no other device is touched. *id receives the device's id; a device
     * already registered on that port keeps its id and is not probed again.
     * Returns PB_ERROR_COMMUNICATION if no PowerBox answers there - fall back
     * to PBScan then. */
    PBAPI PB_ERROR_TYPE PBScanPort(const char *port, int *id);
    /* port must have room for PB_PORT_LEN bytes. Receives a USB PowerBox's
     * /dev/serial/by-path/... link, which stays the same across reconnects and
     * reboots as long as it is plugged into the same USB socket, or its device
     * node if there is no such link. Devices are registered under this link, so
     * PBClose + PBOpen after a replug into the same socket reconnect even if
     * the device comes back as a different /dev/ttyUSBn. Returns
     * PB_ERROR_INVALID_STATE for devices without a serial port (PinsBox,
     * StellaVita, ASIAir). */
    PBAPI PB_ERROR_TYPE PBGetPort(int id, char *port);
#endif
    PBAPI PB_ERROR_TYPE PBOpen(int id);
    PBAPI PB_ERROR_TYPE PBClose(int id);
    PBAPI PB_ERROR_TYPE PBGetSerial(int id, char *serial);
    PBAPI PB_ERROR_TYPE PBScanWiFi(int id, PB_WIFI_SCAN_RESULT *result);

    /* Configuration */
    PBAPI PB_ERROR_TYPE PBGetConfig(int id, PB_DEVICE_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetConfig(int id, PB_DEVICE_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetPowerPortConfig(int id, PB_POWER_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetPowerPortConfig(int id, PB_POWER_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetUSBPortConfig(int id, PB_USB_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetUSBPortConfig(int id, PB_USB_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetDewPortConfig(int id, PB_DEW_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetDewPortConfig(int id, PB_DEW_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetBuckPortConfig(int id, PB_BUCK_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetBuckPortConfig(int id, PB_BUCK_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetPWMPortConfig(int id, PB_PWM_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetPWMPortConfig(int id, PB_PWM_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetWiFiConfig(int id, PB_WIFI_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBSetWiFiConfig(int id, PB_WIFI_CONFIG *config);

    /* Status and information */
    PBAPI PB_ERROR_TYPE PBGetStatus(int id, PB_DEVICE_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetWiFiStatus(int id, PB_WIFI_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetSupplyStatus(int id, PB_SUPPLY_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetPowerPortStatus(int id, PB_POWER_PORT_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetUSBPortStatus(int id, PB_USB_PORT_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetDewPortStatus(int id, PB_DEW_PORT_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetBuckPortStatus(int id, PB_BUCK_PORT_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetPWMPortStatus(int id, PB_PWM_PORT_STATUS *status);
    PBAPI PB_ERROR_TYPE PBGetVersion(int id, PB_VERSION *version);

    /* Utility */
    PBAPI PB_ERROR_TYPE PBBeep(int id, int volume, int duration_ms);
    PBAPI PB_ERROR_TYPE PBRestart(int id);
    PBAPI PB_ERROR_TYPE PBFactoryReset(int id);
    PBAPI PB_ERROR_TYPE PBGetSDKVersion(char *version);

#ifdef __cplusplus
}
#endif

#endif /* POWER_BOX_SDK_H */
