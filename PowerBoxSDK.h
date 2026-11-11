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
        int state[PB_MAX_POWER_PORTS];              /* Port state (ON/OFF), also reflects over current shutdowns */
    } PB_POWER_PORT_STATUS;

    typedef struct _PB_USB_PORT_STATUS
    {
        int numPorts;                               /* Number of ports */
        float current[PB_MAX_USB_PORTS];            /* Current draw [A] */
        float voltage[PB_MAX_USB_PORTS];            /* Output voltage [V] */
        int overcurrent[PB_MAX_USB_PORTS];          /* Overcurrent flag */
        int readOnly[PB_MAX_POWER_PORTS];           /* Indicates if the port can be modified */
        int state[PB_MAX_USB_PORTS];                /* Port state (ON/OFF), also reflects over current shutdowns */
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
        int state;                                   /* Port state (ON/OFF), also reflects over current shutdowns */
    } PB_BUCK_PORT_STATUS;

    typedef struct _PB_PWM_PORT_STATUS
    {
        int numPorts;                                /* Number of ports */
        int pwmResolution;                           /* PWM resolution (bit depth) */
        float current;                               /* Current draw [A] */
        int overcurrent;                             /* Overcurrent flag */
        int pwm;                                     /* PWM value [%] */
        int state;                                   /* Port state (ON/OFF), also reflects over current shutdowns */
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

    /* PB_BOOT_STATES (PinsBox): the boot states are applied at the first SDK
     * start after a Linux boot, and for the ports of the I/O expander also after
     * it lost its state (e.g. a brownout). Later SDK starts in the same boot (e.g.
     * NINA restarted) keep the current port states, including dew and PWM duty
     * and the voltage the buck converter runs with (if the running configuration
     * is plausible, otherwise the port starts off). Dew and PWM ports have no
     * boot state, they start off. Outputs never pulse at an SDK start.
     * Over a reboot without power loss the ports of the I/O expander (12 V
     * ports, dew and PWM enable) keep their state until the first SDK start
     * applies the boot states. The CM5 lines (USB, buck enable) are reset by the
     * reboot and held low by the external pull-downs (EN_BUCK is GPIO4, whose
     * internal pull-up is on by default: the external pull-down has to override
     * it, not yet verified on hardware) until the first SDK start. */

    /* PB_EXIT (PinsBox): dew regulation, software over-current protection and
     * the status listener run in the process that loaded the SDK, from the first
     * PBScan on and independent of PBOpen/PBClose. The SDK changes no output when
     * the process ends (normal exit, SIGTERM, SIGKILL, crash): ports, USB and the
     * buck keep their state (CM5 lines: pinctrl-rp1 keeps released outputs by
     * default, not yet verified on hardware), dew heaters and the PWM port keep
     * their last duty - unregulated (a dew heater in auto mode stays at its last
     * duty, up to 100 %) and without software over-current protection (only the
     * switches' own protection). The next SDK start in the same Linux boot
     * takes the states over (see PB_BOOT_STATES). */

    typedef struct _PB_POWER_PORT_CONFIG
    {
        unsigned int mask;      /* Which fields to update */
        unsigned int index;     /* Which port to update */
        int enabled;            /* Enable/disable each power port */
        int bootState;          /* Power port state on boot (PinsBox: see PB_BOOT_STATES) */
        int overcurrentReset;   /* Reset overcurrent flag */
    } PB_POWER_PORT_CONFIG;

    typedef struct _PB_USB_PORT_CONFIG
    {
        unsigned int mask;          /* Which fields to update */
        unsigned int index;         /* Which port to update */
        int enabled;                /* Enable/disable the USB port */
        int bootState;              /* USB port state on boot (PinsBox: see PB_BOOT_STATES) */
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
        float targetVoltage;        /* Target voltage [V]; with MASK_PORT_VOLTAGE 0 = switch off (see PBSetBuckPortConfig) */
        int enabled;                /* Enable/disable the buck converter */
        int bootState;              /* Buck converter state on boot (PinsBox: see
                                       PB_BOOT_STATES). PinsBox: set to 0
                                       automatically if the stored target is invalid at
                                       startup (the target is reset to the minimum and
                                       saved), but not when a running converter is taken
                                       over: then the voltage it runs with becomes the
                                       target and is saved, bootState is kept. Stored
                                       targets between about 11.85 V and 12.03 V
                                       (accepted by older versions) are limited to about
                                       11.85 V and saved at startup. If switching on at
                                       startup fails for another reason (target above the
                                       supply dependent maximum, DAC not reachable or not
                                       settable, switch error), the buck stays off
                                       (bootState and the target are kept, the next boot
                                       tries again). */
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
    /* Returns PB_ERROR_INVALID_STATE with *number = 0 if a PinsBox is present but
     * not usable (e.g. used by another process, its GPIO / I/O expander could not
     * be initialized, or id 0 is used by another device; the reason is in the SDK
     * log). No other device type is scanned then. A later PBScan tries again (if
     * id 0 is used by another device: not before the process ends). */
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
    /* The mask fields are applied in this order: auto mode, enable/power (only
     * if auto mode is not switched on in the same call), threshold, over-current
     * reset. On an error, the fields before are applied and the remaining ones
     * are skipped: read the configuration again.
     * PinsBox / PinsBox Mini: PB_ERROR_COMMUNICATION if the port could not be
     * switched on (e.g. its PWM is not set up) or off, if enable/power is set
     * while the port is in auto mode, and for MASK_PORT_OVERCURRENT_RESET (not
     * supported). */
    PBAPI PB_ERROR_TYPE PBSetDewPortConfig(int id, PB_DEW_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetBuckPortConfig(int id, PB_BUCK_PORT_CONFIG *config);
    /* MASK_PORT_ENABLE and MASK_PORT_VOLTAGE are applied in one call
     * (PinsBox: DAC first, then switch on; serial PowerBox: see below).
     * Without MASK_PORT_ENABLE the current (reported) on/off state is kept
     * (except for 0 V, see below). Without MASK_PORT_VOLTAGE the stored target
     * is used (see PBGetBuckPortConfig).
     *
     * All devices:
     * - With MASK_PORT_VOLTAGE, a valid targetVoltage that rounds to 0 mV
     *   (0 <= targetVoltage < 0.0005) switches off (enabled is ignored); the
     *   stored target is used for that request.
     * - With MASK_PORT_VOLTAGE, a targetVoltage that is NaN, < 0 or
     *   > 65.535 V (65535 mV, the limit of the serial protocol) is invalid.
     *   If the port would be on (enabled != 0, or no MASK_PORT_ENABLE and
     *   currently (reported) on), the request is rejected with
     *   PB_ERROR_INVALID_PARAMETER without touching the hardware. Otherwise
     *   the port is switched off anyway (with the stored target), then
     *   PB_ERROR_INVALID_PARAMETER is returned (PB_ERROR_COMMUNICATION if
     *   switching off itself failed).
     * - Devices without buck port (status numPorts 0): no effect. PB_SUCCESS,
     *   except an invalid targetVoltage (PB_ERROR_INVALID_PARAMETER, see above)
     *   and MASK_PORT_OVERCURRENT_RESET (PB_ERROR_COMMUNICATION).
     *
     * PinsBox:
     * - The converter is only switched on once the DAC verifiably holds the
     *   value for the target (the output voltage itself is not measured).
     * - 0 V keeps the last target.
     * - Switching off is never blocked by the target. When switching off, a
     *   new target is stored if it is within the converter's range (status
     *   vmin .. about 11.85 V, independent of the supply); otherwise it is not
     *   taken over and an error is returned (the port is off anyway).
     * - Requests that switch on or keep the port on additionally require the
     *   target to be within the supply dependent maximum (status vmax);
     *   rejected targets leave the port as it is. They are also rejected while
     *   a failed over-current shutdown is pending (status: on and overcurrent);
     *   in that pending case switching off is still possible and, if it
     *   succeeds, clears the over-current flag.
     * - If the DAC cannot be set or the switch does not turn on, the port is
     *   switched off, even if it was on before. If that switch-off fails too,
     *   the port stays on; the DAC is parked if reachable (lowest output
     *   voltage), otherwise the output voltage is undefined.
     * - If a requested switch-off fails, the port stays on, the DAC is parked
     *   if reachable (lowest output voltage, about 0.9 V) and the target is not
     *   taken over.
     * - MASK_PORT_OVERCURRENT_RESET is not supported (PB_ERROR_COMMUNICATION).
     *   An over-current flag after a successful shutdown is cleared only by a
     *   successful switch-on, not by switching off.
     * - Range errors (a valid target other than 0 V outside vmin .. about
     *   11.85 V, or above vmax when switching on or keeping the port on) and
     *   the cases above are currently reported as PB_ERROR_COMMUNICATION;
     *   invalid values (see All devices) as PB_ERROR_INVALID_PARAMETER.
     *
     * Serial PowerBox:
     * - The command is sent with the target in mV (rounded); the firmware
     *   checks the range (vmin .. supply dependent vmax). PB_SUCCESS only means
     *   the command was sent; a rejection by the firmware is not reported.
     * - On a rejection (also of a switch-off request) the firmware switches
     *   the port off, even if it was on, parks the DAC (lowest output) and sets
     *   its target to vmin.
     * - When switching on, the firmware enables the output before it checks
     *   the target and sets the DAC: the port briefly runs at the voltage last
     *   set on the DAC (after a rejection the lowest output), and a rejected
     *   target then switches it off again.
     * - After PB_SUCCESS the reported target is the one sent until the
     *   next ADJ status line replaces it with the firmware's target (vmin after
     *   a rejection). The reported on/off state stays as requested until it is
     *   set again, an over-current message arrives or the device is
     *   reconnected; a request without MASK_PORT_ENABLE uses that reported
     *   state and can switch the port on again after a rejection.
     * - "Stored target" is the last one sent or received, whichever is newer
     *   (1.0 V after PBSearch until the first ADJ status line, which arrives up
     *   to one update interval (1-60 s) after PBOpen). A request without
     *   MASK_PORT_VOLTAGE, or with 0 V, sends that stored target to the
     *   firmware, so before the first ADJ status line it sets the firmware's
     *   target to 1.0 V, even when switching off.
     * - After PBOpen, until the first ADJ status line, vmax is reported as
     *   12.0 V, a placeholder (the firmware's power-on value); the firmware's
     *   actual limit is 95 % of the measured supply, at most about 12.03 V (so
     *   below 12.0 V unless the supply is above about 12.6 V).
     * - After a firmware start the supply is only measured while telemetry is
     *   streamed. From about 3 s after the firmware start until then (and up to
     *   about 3 s after the first measurement) the firmware's vmax is 0 and it
     *   rejects every target, so the first ADJ status line(s) after PBOpen may
     *   report vmax 0 (up to about one update interval plus 3 s).
     *
     * On an error, parts of the mask may already be applied and the remaining
     * fields are skipped: read the configuration again. */
    PBAPI PB_ERROR_TYPE PBSetBuckPortConfig(int id, PB_BUCK_PORT_CONFIG *config);
    PBAPI PB_ERROR_TYPE PBGetPWMPortConfig(int id, PB_PWM_PORT_CONFIG *config);
    /* Enable and power are applied in one step; switching off also sets the
     * power to 0. Power alone while the port is off is not applied (the port
     * stays off, PB_SUCCESS).
     * PinsBox / PinsBox Mini: enabled with power 0 is "on" with no output (the
     * PWM channel runs only with a power above 0), so a later power change takes
     * effect. PB_ERROR_COMMUNICATION if the port could not be switched on (e.g.
     * its PWM is not set up) or off, and for MASK_PORT_OVERCURRENT_RESET (not
     * supported); the remaining mask fields are skipped then, read the
     * configuration again. */
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
