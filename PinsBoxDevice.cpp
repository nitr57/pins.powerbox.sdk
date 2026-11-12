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

#include "PinsBoxDevice.h"
#include "PowerBoxLogging.h"
#include "MCP320X.h"
#include "Device.h"
#include "DeviceUtil.h"
#include <filesystem>
#include <mutex>
#include <fstream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>
#include <cctype>
#include <json/json.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define PINSBOX_NUM_POWER_PORTS 8
#define PINSBOX_NUM_USB_PORTS 8
#define PINSBOX_NUM_DEW_PORTS 2
#define PINSBOX_NUM_BUCK_PORTS 1
#define PINSBOX_NUM_PWM_PORTS 1

namespace PowerBox
{
    static constexpr unsigned int NUM_PINS = 29;
    static constexpr uint8_t PINS[NUM_PINS] = {
        22, // USB_EN1
        27, // USB_EN2
        20, // USB_EN3
        23, // USB_EN4
        25, // USB_EN5
        24, // USB_EN6
        26, // USB_EN7
        21, // USB_EN8
         4, // EN_BUCK
        55, // port 2: P17
        54, // port 3: P16
        51, // port 4: P13
        50, // port 5: P12
        48, // port 6: P10
        41, // port 7: P01
        40, // port 8: P00
        43, // dew 1: P03
        45, // dew 2: P05
        47, // pwm: P07
        53, // DEN12V_12
        52, // DEN12V_34
        49, // DEN12V_56
        42, // DEN12V_78
        46, // DEN12V_IN
        44, // DEN12_DEW
         3, // DEN12_ADJ
         6, // TEMP1_DEW
         5, // TEMP2_DEW
         2, // DSEL
    };

    static const uint8_t *PWR_PINS = &PINS[9];
    static const uint8_t *USB_PINS = &PINS[0];
    static const uint8_t *DEW_PINS = &PINS[16];
    static const uint8_t BUCK_PIN = PINS[8];
    static const uint8_t PWM_PIN = PINS[18];
    static const uint8_t DSEL_PIN = PINS[28];
    static const uint8_t INDEN_PIN = PINS[23];
    static const uint8_t *PWRDEN_PINS = &PINS[19];
    static const uint8_t DEWDEN_PIN = PINS[24];
    static const uint8_t ADJDEN_PIN = PINS[25];
    static const uint8_t *DS18_PINS = &PINS[26];

    /* Boot states are applied once per Linux boot. The marker holds the boot id
     * of the boot in which they were applied; /run/lock is a tmpfs (cleared at
     * every boot), the boot id protects against a /run/lock that is not. */
    static const char *BOOT_MARKER = "/run/lock/pinsbox.boot";

    static std::string ReadBootId(void)
    {
        std::ifstream ifs("/proc/sys/kernel/random/boot_id");
        std::string id;
        if(!(ifs >> id))
        {
            PB_ERROR("PinsBox: boot id not readable, boot states are applied");
            return "";
        }
        return id;
    }

    static bool IsSameBoot(const std::string &bootId)
    {
        if(bootId.empty())
        {
            return false;
        }

        std::ifstream ifs(BOOT_MARKER);
        std::string id;
        return (ifs >> id) && id == bootId;
    }

    static void WriteBootMarker(const std::string &bootId)
    {
        // Atomic (temp file + rename), so a reader never sees an empty marker;
        // readable for every user (root and a normal user may both start the SDK)
        // The temp file is created exclusively (never follows or reuses an
        // existing path in the world writable directory); a leftover is removed once
        std::string tmp = std::string(BOOT_MARKER) + "." + std::to_string(getpid());
        const int flags = O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC;
        int fd = open(tmp.c_str(), flags, 0644);
        if(fd < 0 && errno == EEXIST)
        {
            unlink(tmp.c_str());
            fd = open(tmp.c_str(), flags, 0644);
        }
        if(fd < 0)
        {
            int err = errno;
            PB_ERROR("PinsBox: boot marker %s could not be written (%s), boot states will be applied again at the next start", BOOT_MARKER, strerror(err));
            return;
        }

        std::string data = bootId + "\n";
        int err = 0;
        ssize_t written = write(fd, data.c_str(), data.size());
        bool ok = written == (ssize_t)data.size();
        if(written < 0)
        {
            err = errno;  // a short write leaves err 0 ("short write")
        }
        fchmod(fd, 0644);
        close(fd);

        if(ok && rename(tmp.c_str(), BOOT_MARKER) != 0)
        {
            ok = false;
            err = errno;
        }

        if(!ok)
        {
            PB_ERROR("PinsBox: boot marker %s could not be written (%s), boot states will be applied again at the next start", BOOT_MARKER, err ? strerror(err) : "short write");
            unlink(tmp.c_str());
        }
    }

    /* ------------------------------------------------------------------------
     * config.txt boot GPIO check (APO.Mate.5 / CM5 PinsBox)
     * The firmware applies "gpio=<pins>=<options>" before Linux starts. A switch
     * line driven high there switches ports / the buck converter / the buzzer on
     * until the SDK starts. Grammar (Raspberry Pi docs, config.txt "gpio"):
     *   pins: N, A-B or a comma list of both; options: ip, op, a0-a9, dh, dl,
     *   pu, pd, pn, np; lines are applied in order (later lines override earlier
     *   ones, only the attributes they name); conditional filters apply;
     *   "include <file>" inserts another file of the boot partition.
     * Conservative: a model filter for another board disables the following
     * lines, every non-model filter (EDID, HDMI, gpio, serial, tryboot ...) is
     * treated as true. Read errors are silent (debug only).
     * ------------------------------------------------------------------------ */
    static const char *BOOT_CONFIG_DIR = "/boot/firmware";
    static const char *BOOT_CONFIG_FILE = "config.txt";
    static const char *BOOT_GPIO_RECOMMENDED = "gpio=2,3,4,12,13,16,18,20-27=op,dl";

    // DSEL, DEN12_ADJ, EN_BUCK, dew gates 12/13, buzzer 16, PWM gate 18, USB_EN1-8
    static constexpr uint64_t BOOT_LOW_PINS =
        (1ull << 2) | (1ull << 3) | (1ull << 4) | (1ull << 12) | (1ull << 13) |
        (1ull << 16) | (1ull << 18) | (0xFFull << 20);

    struct BootGpioState
    {
        uint64_t output = 0;   // last direction attribute was "op"
        uint64_t high = 0;     // last level attribute was "dh"
        bool active = true;    // current conditional filter state
        bool fileRead = false; // main file read
        int files = 0;         // files opened (includes can fan out)
    };

    static std::string TrimCopy(const std::string &s)
    {
        const char *ws = " \t\r\n";
        size_t b = s.find_first_not_of(ws);
        if(b == std::string::npos)
        {
            return "";
        }
        size_t e = s.find_last_not_of(ws);
        return s.substr(b, e - b + 1);
    }

    static std::string LowerNoSpace(const std::string &s)
    {
        std::string out;
        for(char c : s)
        {
            if(c != ' ' && c != '\t' && c != '\r' && c != '\n')
            {
                out += (char)std::tolower((unsigned char)c);
            }
        }
        return out;
    }

    static bool ParseBootPinNumber(const std::string &s, unsigned &pin)
    {
        if(s.empty() || s.size() > 2 || !std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; }))
        {
            return false;
        }
        pin = (unsigned)std::stoul(s);
        return pin < 64;
    }

    // "3", "18-27", "18-27,3,4" -> mask; false (line ignored) if malformed
    static bool ParseBootPinList(const std::string &list, uint64_t &mask)
    {
        mask = 0;
        size_t start = 0;
        while(start <= list.size())
        {
            size_t comma = list.find(',', start);
            std::string item = list.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
            size_t dash = item.find('-');
            unsigned a, b;
            if(dash == std::string::npos)
            {
                if(!ParseBootPinNumber(item, a))
                {
                    return false;
                }
                b = a;
            }
            else if(!ParseBootPinNumber(item.substr(0, dash), a) || !ParseBootPinNumber(item.substr(dash + 1), b) || b < a)
            {
                return false;
            }
            for(unsigned p = a; p <= b; ++p)
            {
                mask |= 1ull << p;
            }
            if(comma == std::string::npos)
            {
                break;
            }
            start = comma + 1;
        }
        return true;
    }

    // One "[...]" filter. The SDK only runs on a CM5 (ScanPinsBox).
    static void ApplyBootFilter(const std::string &filter, BootGpioState &st)
    {
        static const char *OTHER_MODELS[] = {
            "pi0", "pi0w", "pi02", "pi1", "pi2", "pi3", "pi3+", "pi4", "pi400",
            "pi500", "pi500+", "cm1", "cm3", "cm3+", "cm4", "cm4s", "none" };

        if(filter == "all" || filter == "cm5" || filter == "pi5")
        {
            st.active = true;   // [all] resets every filter; cm5/pi5 match this board
            return;
        }
        for(const char *m : OTHER_MODELS)
        {
            if(filter == m)
            {
                st.active = false;
                return;
            }
        }
        // Other filter types (EDID, HDMI, gpioN, serial, tryboot, ...): unknown
        // here, combined with the model filter (AND) -> active state unchanged
    }

    static constexpr off_t BOOT_CONFIG_MAX_SIZE = 1024 * 1024;

    static void ParseBootConfigFile(const std::string &dir, const std::string &path, int depth, BootGpioState &st)
    {
        // Depth limits the nesting, this the total (many include lines per file)
        if(++st.files > 16)
        {
            return;
        }

        // Only regular files of a sane size (no FIFO / device that could block
        // or grow without limit: this runs in the device setup under the SDK lock)
        struct stat sb;
        if(stat(path.c_str(), &sb) != 0 || !S_ISREG(sb.st_mode) || sb.st_size > BOOT_CONFIG_MAX_SIZE)
        {
            PB_DEBUG("PinsBox: %s not read (missing, not a regular file or too large)", path.c_str());
            return;
        }
        std::ifstream ifs(path);
        if(!ifs.is_open())
        {
            PB_DEBUG("PinsBox: %s not readable", path.c_str());
            return;
        }
        if(depth == 0)
        {
            st.fileRead = true;
        }

        std::string raw;
        int lines = 0;
        while(std::getline(ifs, raw) && ++lines <= 10000)
        {
            // Comments (also trailing ones: '#' is never part of a gpio line), CR, blanks
            std::string line = TrimCopy(raw.substr(0, raw.find('#')));
            if(line.empty())
            {
                continue;
            }

            if(line[0] == '[')
            {
                size_t close = line.find(']');
                if(close != std::string::npos)
                {
                    ApplyBootFilter(LowerNoSpace(line.substr(1, close - 1)), st);
                }
                continue;
            }

            if(!st.active)
            {
                continue;
            }

            // include <file> (relative to the boot partition)
            // (stays on the boot partition: a leading '/' is its root, ".." is not followed)
            std::string key = LowerNoSpace(line.substr(0, 8));
            if(key == "include" && line.size() > 8 && (line[7] == ' ' || line[7] == '\t'))
            {
                std::string inc = TrimCopy(line.substr(8));
                while(!inc.empty() && inc[0] == '/')
                {
                    inc.erase(0, 1);
                }
                if(!inc.empty() && depth < 4 && inc.find("..") == std::string::npos)
                {
                    ParseBootConfigFile(dir, dir + "/" + inc, depth + 1, st);
                }
                continue;
            }

            std::string compact = LowerNoSpace(line);
            if(compact.compare(0, 5, "gpio=") != 0)
            {
                continue;
            }
            std::string value = compact.substr(5);
            size_t eq = value.find('=');
            uint64_t pins;
            if(eq == std::string::npos || !ParseBootPinList(value.substr(0, eq), pins))
            {
                continue;
            }

            // Attributes in order, each one changes only its own property
            std::string opts = value.substr(eq + 1);
            size_t start = 0;
            while(start <= opts.size())
            {
                size_t comma = opts.find(',', start);
                std::string o = opts.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
                if(o == "op")
                {
                    st.output |= pins;
                }
                else if(o == "ip" || (o.size() == 2 && o[0] == 'a' && o[1] >= '0' && o[1] <= '9'))
                {
                    st.output &= ~pins;
                }
                else if(o == "dh")
                {
                    st.high |= pins;
                }
                else if(o == "dl")
                {
                    st.high &= ~pins;
                }
                if(comma == std::string::npos)
                {
                    break;
                }
                start = comma + 1;
            }
        }
    }

    // "3,4,12,13,16,18,20-27"
    static std::string FormatPinMask(uint64_t mask)
    {
        std::string out;
        for(unsigned p = 0; p < 64; ++p)
        {
            if(!(mask & (1ull << p)))
            {
                continue;
            }
            unsigned e = p;
            while(e + 1 < 64 && (mask & (1ull << (e + 1))))
            {
                ++e;
            }
            if(!out.empty())
            {
                out += ",";
            }
            out += std::to_string(p);
            if(e >= p + 2)
            {
                out += "-" + std::to_string(e);   // ranges from 3 pins on
                p = e;
            }
        }
        return out;
    }

    /* Once per process (the config is persistent: each N.I.N.A. session log
     * shows it) */
    static void CheckBootGpioConfig(void)
    {
        static std::atomic<bool> done{false};
        if(done.exchange(true))
        {
            return;
        }

        BootGpioState st;
        ParseBootConfigFile(BOOT_CONFIG_DIR, std::string(BOOT_CONFIG_DIR) + "/" + BOOT_CONFIG_FILE, 0, st);
        const uint64_t cfgHigh = st.output & st.high & BOOT_LOW_PINS;

        if(cfgHigh)
        {
            PB_ERROR("PinsBox: %s/%s drives GPIO %s high at boot (ports, buck converter or buzzer are switched on before the SDK starts); "
                     "recommended: %s (replace the old gpio= line, then reboot)",
                     BOOT_CONFIG_DIR, BOOT_CONFIG_FILE, FormatPinMask(cfgHigh).c_str(), BOOT_GPIO_RECOMMENDED);
        }
        else if(!st.fileRead)
        {
            PB_DEBUG("PinsBox: boot GPIO check skipped (config.txt not readable)");
        }
    }

    /* Lock file of the PinsBox hardware (world writable tmpfs) */
    static const char *LOCK_FILE = "/run/lock/pinsbox.lock";

    ProcessLock::~ProcessLock(void)
    {
        if(this->fd_ >= 0)
        {
            close(this->fd_);
        }
    }

    ProcessLock& ProcessLock::operator=(ProcessLock&& other) noexcept
    {
        if(this != &other)
        {
            if(this->fd_ >= 0)
            {
                close(this->fd_);
            }
            this->fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    ProcessLock::Result ProcessLock::acquire(const char* path)
    {
        // A few attempts only for the case the file is replaced while locking
        for(int attempt = 0; attempt < 3; ++attempt)
        {
            // Read only is enough for flock and avoids protected_regular issues
            // with a file created by another user. O_NONBLOCK: a FIFO planted in
            // the world writable directory must not block the scan.
            const int flags = O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK;
            int fd = open(path, flags);
            if(fd < 0 && errno == ENOENT)
            {
                fd = open(path, flags | O_CREAT | O_EXCL, 0644);
                if(fd >= 0)
                {
                    fchmod(fd, 0644);
                }
                else if(errno == EEXIST)
                {
                    continue;  // created by someone else in the meantime
                }
            }
            if(fd < 0)
            {
                PB_ERROR("PinsBox: lock file %s not usable (%s), continuing without process lock", path, strerror(errno));
                return Unavailable;
            }

            struct stat st;
            if(fstat(fd, &st) != 0 || !S_ISREG(st.st_mode))
            {
                close(fd);
                PB_ERROR("PinsBox: lock file %s is not a regular file, continuing without process lock", path);
                return Unavailable;
            }

            if(flock(fd, LOCK_EX | LOCK_NB) != 0)
            {
                int err = errno;
                close(fd);
                if(err == EWOULDBLOCK)
                {
                    return Busy;
                }
                PB_ERROR("PinsBox: lock file %s could not be locked (%s), continuing without process lock", path, strerror(err));
                return Unavailable;
            }

            // The file may have been removed and recreated while we locked it:
            // then a lock on the old inode protects nothing, try again
            struct stat cur;
            if(stat(path, &cur) == 0 && cur.st_dev == st.st_dev && cur.st_ino == st.st_ino)
            {
                if(this->fd_ >= 0)
                {
                    close(this->fd_);
                }
                this->fd_ = fd;
                return Acquired;
            }
            close(fd);
        }

        PB_ERROR("PinsBox: lock file %s keeps changing, continuing without process lock", path);
        return Unavailable;
    }

    PinsBoxDevice::PinsBoxDevice(void)
    {
        this->statusListenerRunning = false;
        this->temperature = DEVICE_DISCONNECTED_C;
        this->humidity = DEVICE_DISCONNECTED_C;
        this->dewPoint = DEVICE_DISCONNECTED_C;
        this->extSensor = false;

        // Initial reset (defaults until the settings are loaded after the GPIO
        // check: a failed setup must not create a settings file)
        this->ResetProperties();

        this->dht_ = new DHT22("device0");

        this->gpio_ = new GPIOManager("/dev/gpiochip0",
                                      "/dev/i2c-10",
                                      0x20,
                                      0,
                                      nullptr,
                                      NUM_PINS,
                                      PINS);

        // First SDK start in this Linux boot: apply the boot states. Later starts
        // (e.g. NINA restarted) take the current port states over.
        const std::string bootId = ReadBootId();
        const bool sameBoot = IsSameBoot(bootId);
        if(sameBoot)
        {
            PB_INFO("PinsBox: started again in the same boot, current port states are taken over");
        }

        // Switch pins (PINS[0..18]: USB_EN1-8, EN_BUCK, power ports, dew, PWM)
        // keep an already configured output level, the 1-wire pins idle high,
        // all other lines (sense enables, DSEL) start low.
        const bool gpioOk = this->gpio_->begin(sameBoot, PINS, 19, DS18_PINS, 2);

        // config.txt boot levels of the switch lines (once per process; also when
        // the GPIO setup failed, a config driving the lines may be the reason)
        CheckBootGpioConfig();

        if(!gpioOk)
        {
            // GPIO lines busy (another process, an overlay) or not readable, or the
            // I/O expander not usable: set nothing else up. In particular no DAC,
            // EEPROM or sysfs PWM access, which would change running outputs.
            PB_ERROR("PinsBox: GPIO initialization failed, the device is not used and no port output is switched");
            return;
        }

        // Load settings from file (creates it with defaults if missing)
        this->LoadSettings();

        // Taken over level, or the boot state
        auto initState = [this](uint8_t pin, int bootState) {
            int kept = this->gpio_->keptValue(pin);
            return (uint8_t)(kept >= 0 ? kept : bootState);
        };

        // ADC
        this->adc_ = new MCP3204(*this->gpio_);
        this->adc_->begin(3.3f);

        // Supply
        this->supply_ = new BTS7006<MCP3204>(*this->gpio_, *this->adc_);
        this->supply_->setName("Supply");
        this->supply_->begin(1200, INDEN_PIN, 255, DSEL_PIN, 0);
        this->supply_->setMaxCurrent(12.f);
        this->supply_->setChannel(2);
        this->supply_->setSampling(100, 50);

        // PWR ports
        this->pwr_ = new BTS7012<MCP3204>*[PINSBOX_NUM_POWER_PORTS];
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            this->pwr_[i] = new BTS7012<MCP3204>(*this->gpio_, *this->adc_);
            this->pwr_[i]->setName("12V port " + std::to_string(i));
        }

        // Port 0 is readonly
        this->powerReadOnly[0] = true;

        // Initialize PWR ports
        this->pwr_[0]->begin(1200, PWRDEN_PINS[0], 255, DSEL_PIN, 0);
        this->powerState[0] = 1;  // port 0 has no switch and is always on
        this->pwr_[1]->begin(1200, PWRDEN_PINS[0], PWR_PINS[0], DSEL_PIN, 1, initState(PWR_PINS[0], this->powerBootstrap[1]));
        this->pwr_[2]->begin(1200, PWRDEN_PINS[1], PWR_PINS[1], DSEL_PIN, 0, initState(PWR_PINS[1], this->powerBootstrap[2]));
        this->pwr_[3]->begin(1200, PWRDEN_PINS[1], PWR_PINS[2], DSEL_PIN, 1, initState(PWR_PINS[2], this->powerBootstrap[3]));
        this->pwr_[4]->begin(1200, PWRDEN_PINS[2], PWR_PINS[3], DSEL_PIN, 0, initState(PWR_PINS[3], this->powerBootstrap[4]));
        this->pwr_[5]->begin(1200, PWRDEN_PINS[2], PWR_PINS[4], DSEL_PIN, 1, initState(PWR_PINS[4], this->powerBootstrap[5]));
        this->pwr_[6]->begin(1200, PWRDEN_PINS[3], PWR_PINS[5], DSEL_PIN, 0, initState(PWR_PINS[5], this->powerBootstrap[6]));
        this->pwr_[7]->begin(1200, PWRDEN_PINS[3], PWR_PINS[6], DSEL_PIN, 1, initState(PWR_PINS[6], this->powerBootstrap[7]));

        // PWR sense sample rate and max current
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            this->pwr_[i]->setMaxCurrent(6.f);
            this->pwr_[i]->setChannel(2);
            this->pwr_[i]->setSampling(10, 1);
        }

        // USB ports
        this->usb_ = new USB*[PINSBOX_NUM_USB_PORTS];
        for(int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i) {
            this->usb_[i] = new USB(*this->gpio_, "/dev/i2c-10");
        }

        // Initialize USB ports
        // Ports 5/6 and 7/8 use swapped enable pins and INA219 addresses; the boot
        // state always belongs to the API index (usb_[k] <- usbBootstrap[k])
        this->usb_[0]->begin(USB_PINS[0], 0x40, 2.6f, initState(USB_PINS[0], this->usbBootstrap[0]), 0);
        this->usb_[1]->begin(USB_PINS[1], 0x41, 2.6f, initState(USB_PINS[1], this->usbBootstrap[1]), 1);
        this->usb_[2]->begin(USB_PINS[2], 0x42, 2.0f, initState(USB_PINS[2], this->usbBootstrap[2]), 2);
        this->usb_[3]->begin(USB_PINS[3], 0x43, 2.0f, initState(USB_PINS[3], this->usbBootstrap[3]), 3);
        this->usb_[5]->begin(USB_PINS[4], 0x44, 1.2f, initState(USB_PINS[4], this->usbBootstrap[5]), 5);
        this->usb_[4]->begin(USB_PINS[5], 0x45, 1.2f, initState(USB_PINS[5], this->usbBootstrap[4]), 4);
        this->usb_[6]->begin(USB_PINS[7], 0x47, 1.2f, initState(USB_PINS[7], this->usbBootstrap[6]), 6);
        this->usb_[7]->begin(USB_PINS[6], 0x46, 1.2f, initState(USB_PINS[6], this->usbBootstrap[7]), 7);

        // Dew ports
        this->dew_ = new DewPort*[PINSBOX_NUM_DEW_PORTS];
        this->dew_[0] = new DewPort(*this->gpio_, *this->adc_, "pwmchip0", 1);
        this->dew_[1] = new DewPort(*this->gpio_, *this->adc_, "pwmchip1", 0);

        // Initialize Dew ports
        // Dew ports have no boot state: off at the first start, taken over later
        this->dew_[0]->begin(DEWDEN_PIN, DEW_PINS[1], DSEL_PIN, DS18_PINS[0], 1, 400, this->dewAuto[0], 2, this->gpio_->keptValue(DEW_PINS[1]), 0);
        this->dew_[1]->begin(DEWDEN_PIN, DEW_PINS[0], DSEL_PIN, DS18_PINS[1], 0, 400, this->dewAuto[1], 2, this->gpio_->keptValue(DEW_PINS[0]), 1);

        // Initialize adjustable ports
        this->buck_ = new BuckPort(*this->gpio_, *this->adc_, "/dev/i2c-10", 0x60);
        const float loadedVset = this->buckVset;
        this->buck_->begin(ADJDEN_PIN, BUCK_PIN, DSEL_PIN, 0, this->buckVset, this->buckBootstrap, this->gpio_->keptValue(BUCK_PIN));

        // Report what the port really uses right away, not only after the
        // first listener cycle (an invalid stored target may have been replaced)
        this->buckVset = this->buck_->getTargetVoltage();
        this->buckState = this->buck_->getState();
        this->buckVmin = this->buck_->getMinVoltage();
        this->buckVmax = this->buck_->getMaxVoltage();
        this->buckOvervoltage = this->buck_->getOverVoltage() ? 1 : 0;

        // An invalid stored target kept the buck off. Make that permanent and
        // visible (boot state off), otherwise the next boot would switch on
        // with the replacement target as soon as anything else saves settings.
        if (this->buck_->getBootTargetInvalid())
        {
            if (this->buckBootstrap)
            {
                PB_ERROR("Buck: boot state set to off because of the invalid stored target");
                this->buckBootstrap = 0;
            }
            SaveSettings();
        }
        // The target in use differs from the stored one (an old target limited to
        // the highest target, or a running converter taken over with a different
        // DAC value): store the target in use (compared in mV, no float compare)
        else if (!(loadedVset >= 0.f && loadedVset <= 100.f) ||
                 lroundf(loadedVset * 1000.f) != lroundf(this->buckVset * 1000.f))
        {
            SaveSettings();
        }

        // PWM port has no boot state: off at the first start, taken over later
        this->pwm_ = new PWMPort(*this->gpio_, *this->adc_);
        this->pwm_->begin(ADJDEN_PIN, PWM_PIN, DSEL_PIN, 1, 30000, 2, 3.0f, this->gpio_->keptValue(PWM_PIN));
        if(!this->pwm_->isTakenOver())
        {
            this->pwm_->setState(0, 0.f);
        }
        this->pwmState = this->pwm_->getState();
        this->pwmPWM = this->pwm_->getDutyCycle();

        // Initialize buzzer PWM on GPIO16 (pwmchip2, channel 0)
        this->buzzer_ = new PWM("pwmchip2", 0);
        this->buzzer_->setExport();

        // Boot states are written: later starts in this boot take the states over
        if(!sameBoot && !bootId.empty())
        {
            WriteBootMarker(bootId);
        }

        this->initOk_ = true;
    }

    PinsBoxDevice::~PinsBoxDevice(void)
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
            for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i) {
                if(this->pwr_[i]) {
                    delete this->pwr_[i];
                    this->pwr_[i] = nullptr;
                }
            }
            delete[] this->pwr_;
            this->pwr_ = nullptr;
        }

        // Clean up USB ports
        if(this->usb_) {
            for(int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i) {
                if(this->usb_[i]) {
                    delete this->usb_[i];
                    this->usb_[i] = nullptr;
                }
            }
            delete[] this->usb_;
            this->usb_ = nullptr;
        }

        // Clean up dew ports
        if(this->dew_) {
            for(int i = 0; i < PINSBOX_NUM_DEW_PORTS; ++i) {
                if(this->dew_[i]) {
                    delete this->dew_[i];
                    this->dew_[i] = nullptr;
                }
            }
            delete[] this->dew_;
            this->dew_ = nullptr;
        }

        // Clean up adjustable ports
        if(this->buck_) {
            delete this->buck_;
            this->buck_ = nullptr;
        }

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

    void PinsBoxDevice::ResetProperties(void)
    {
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            this->powerBootstrap[i] = 0;
        }

        for(int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i)
        {
            this->usbBootstrap[i] = 0;
        }

        for(int i = 0; i < PINSBOX_NUM_DEW_PORTS; ++i)
        {
            this->dewAuto[i] = 1;
            this->dewThreshold[i] = 4.f;
        }

        this->buckBootstrap = 0;
        this->buckVset = 1.f;

        this->temperatureOffset = 0.f;
        this->humidityOffset = 0.f;
    }

    void PinsBoxDevice::StartStatusListener(void)
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
        this->statusListenerThread_ = std::thread(&PinsBoxDevice::StatusListenerThreadFunc, this);

        /* Start DHT22 thread separately so it doesn't block the main measurement loop */
        this->dht22ThreadRunning_ = true;
        this->dht22Thread_ = std::thread(&PinsBoxDevice::DHT22ThreadFunc, this);
        PB_DEBUG("StartStatusListener: Listener thread started");
    }

    void PinsBoxDevice::StopStatusListener(void)
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
    void PinsBoxDevice::StatusListenerThreadFunc()
    {
        PB_DEBUG("StatusListener: starting");

        // Beep on initial listener launch
        this->Beep(20, 400);

        while(this->statusListenerRunning)
        {
            // An exception escaping a std::thread terminates the whole process
            try
            {
                // Fetch MCP3208 data (fast SPI/I2C — runs every ~1s)
                this->GetMCP3208Data();
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

    void PinsBoxDevice::DHT22ThreadFunc()
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

    PB_ERROR_TYPE PinsBoxDevice::Open(void)
    {
        PB_DEBUG("PBOpen: Found PI'N'Stars power box device");

        // Fetch power ports config
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i) {
            this->GetPowerState(i);
        }

        // Fetch usb ports config
        for(int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i) {
            this->GetUSBState(i);
        }

        // Fetch dew ports config
        this->dewPwmResolution = 8;
        for(int i = 0; i < PINSBOX_NUM_DEW_PORTS; ++i) {
            this->GetDewState(i);
        }

        // Fetch buck port config
        this->GetBuckState();

        // Fetch pwm port config
        this->pwmPwmResolution = 8;
        this->GetPWMState();

        this->isOpen = true;
        PB_INFO("[OK] Device opened");
        return PB_SUCCESS;
    }

    void PinsBoxDevice::Close(void)
    {
        this->isOpen = false;
        PB_DEBUG("[OK] Device closed");
    }

    std::string PinsBoxDevice::GetFullSerial(void)
    {
        std::ifstream file("/proc/device-tree/serial-number");
        if(!file.is_open())
        {
            return "";
        }

        std::string serial;
        std::getline(file, serial);

        return serial;
    }

    std::string PinsBoxDevice::GetSerial(void)
    {
        return ShortSerial(this->GetFullSerial());
    }

    int PinsBoxDevice::GetUpTime(void)
    {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - this->sessionStart_).count());
    }

    float PinsBoxDevice::GetCoreTemp(void)
    {
        return ReadCoreTemp(DEVICE_DISCONNECTED_C);
    }

    float PinsBoxDevice::GetSupply12AverageA(void)
    {
        int uptimeSeconds = this->GetUpTime();
        if (uptimeSeconds <= 0)
        {
            return 0.0f;
        }
        
        float uptimeHours = uptimeSeconds / 3600.0f;
        return this->supply12Ah / uptimeHours;
    }

    bool PinsBoxDevice::SetTemperatureOffset(float val)
    {
        this->temperatureOffset = val;
        SaveSettings();
        return true;
    }

    bool PinsBoxDevice::SetExtTemperature(float val)
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

    bool PinsBoxDevice::SetHumidityOffset(float val)
    {
        this->humidityOffset = val;
        SaveSettings();
        return true;
    }

    bool PinsBoxDevice::SetExtHumidity(float val)
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

    void PinsBoxDevice::LoadSettings(void)
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
                this->powerBootstrap[0] = true;
                for (int i = 1; i < PINSBOX_NUM_POWER_PORTS && i < (int)powerPorts.size(); i++) {
                    int idx = powerPorts[i].get("port", 0).asInt();
                    if (IsValidIndex(idx, PINSBOX_NUM_POWER_PORTS)) {
                        this->powerBootstrap[idx] = powerPorts[i].get("bootState", 0).asInt();
                    }
                }
            }

            // Load USB port bootstrap states
            const Json::Value &usbPorts = root["usbPorts"];
            if (usbPorts.isArray()) {
                for (int i = 0; i < PINSBOX_NUM_USB_PORTS && i < (int)usbPorts.size(); i++) {
                    int idx = usbPorts[i].get("port", 0).asInt();
                    if (IsValidIndex(idx, PINSBOX_NUM_USB_PORTS)) {
                        this->usbBootstrap[idx] = usbPorts[i].get("bootState", 0).asInt();
                    }
                }
            }

            // Load Dew port auto mode states
            const Json::Value &dewPorts = root["dewPorts"];
            if (dewPorts.isArray()) {
                for (int i = 0; i < PINSBOX_NUM_DEW_PORTS && i < (int)dewPorts.size(); i++) {
                    int idx = dewPorts[i].get("port", 0).asInt();
                    if (IsValidIndex(idx, PINSBOX_NUM_DEW_PORTS)) {
                        this->dewAuto[idx] = dewPorts[i].get("autoMode", 1).asInt();
                        this->dewThreshold[idx] = dewPorts[i].get("autoThreshold", 4).asFloat();
                    }
                }
            }

            // Load Buck port bootstrap states
            const Json::Value &buckPorts = root["buckPorts"];
            if (buckPorts.isArray() && buckPorts.size() > 0) {
                this->buckBootstrap = buckPorts[0].get("bootState", 0).asInt();
                this->buckVset = buckPorts[0].get("targetVoltage", 1).asFloat();
            }
        } catch (const std::exception &e) {
            PB_ERROR("LoadSettings: failed to read settings, using defaults (%s)", e.what());
            this->ResetProperties();
        }
    }

    void PinsBoxDevice::SaveSettings(void)
    {
        try {
            Json::Value root;

            // Environment settings
            root["environment"]["temperatureOffset"] = this->temperatureOffset;
            root["environment"]["humidityOffset"] = this->humidityOffset;

            // Power port bootstrap states
            root["powerPorts"][0]["port"] = 0;
            root["powerPorts"][0]["bootState"] = 1;
            for (int i = 1; i < PINSBOX_NUM_POWER_PORTS; i++) {
                root["powerPorts"][i]["port"] = i;
                root["powerPorts"][i]["bootState"] = this->powerBootstrap[i];
            }

            // USB port bootstrap states
            for (int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i) {
                root["usbPorts"][i]["port"] = i;
                root["usbPorts"][i]["bootState"] = this->usbBootstrap[i];
            }

            // Dew port automode states
            for (int i = 0; i < PINSBOX_NUM_DEW_PORTS; ++i) {
                root["dewPorts"][i]["port"] = i;
                root["dewPorts"][i]["autoMode"] = this->dewAuto[i];
                root["dewPorts"][i]["autoThreshold"] = this->dewThreshold[i];
            }

            // Buck port bootstrap states
            root["buckPorts"][0]["port"] = 0;
            root["buckPorts"][0]["bootState"] = this->buckBootstrap;
            root["buckPorts"][0]["targetVoltage"] = this->buckVset;

            if (SaveSettingsFile(SETTINGS_FILE, root)) {
                PB_DEBUG("Settings saved");
            }
        } catch (const std::exception &e) {
            PB_ERROR("SaveSettings: %s", e.what());
        }
    }

    void PinsBoxDevice::GetDHT22Data(void)
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

    void PinsBoxDevice::GetMCP3208Data(void)
    {
        // hwMutex_ is taken per measurement block instead of for the whole pass,
        // so an API call waits for at most one block (a dew port is the longest)

        // Supply
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->supply_->measureCurrent();
            this->supply12A = this->supply_->getCurrent_mA() * 0.001f;
            this->supply12V = this->adc_->analogReadAverage(1, 20, 0) * (36e3 + 4.7e3) / 4.7e3 * 0.001f;
            this->supply5V = this->adc_->analogReadAverage(0, 20, 0) * (35.7e3 + 10e3) / 10e3 * 0.001f;
        }

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

        // PWR ports
        for(int i = 0; i < PINSBOX_NUM_POWER_PORTS; ++i)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->pwr_[i]->measureCurrent();
            this->powerCurrent[i] = this->pwr_[i]->getCurrent_mA() * 0.001f;
            this->powerOvercurrent[i] = this->pwr_[i]->getOverCurrent() ? 1 : 0;
            // Track the driver state, it changes on its own on an over current shutdown
            this->powerState[i] = i == 0 ? 1 : this->pwr_[i]->getState();
        }
        
        // I2C results of this pass: from USB::BUS_TIMEOUT_LIMIT timeouts on, the
        // remaining ports skip the bus (a hung bus would block the hardware lock)
        USB::BusPass i2cPass;
        for(int i = 0; i < PINSBOX_NUM_USB_PORTS; ++i)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->usb_[i]->measure(i2cPass);

            // NaN: port on, but no valid measurement (I2C error)
            bool valid = this->usb_[i]->isValid();
            this->usbCurrent[i] = valid ? this->usb_[i]->getCurrent_mA() * 0.001f : std::numeric_limits<float>::quiet_NaN();
            this->usbVoltage[i] = valid ? this->usb_[i]->getVoltage_mV() * 0.001f : std::numeric_limits<float>::quiet_NaN();
            this->usbOvercurrent[i] = this->usb_[i]->getOverCurrent() ? 1 : 0;
            this->usbState[i] = this->usb_[i]->getState();
        }

        if(i2cPass.timeouts >= USB::BUS_TIMEOUT_LIMIT)
        {
            if(this->usbBusError_.fail())
            {
                PB_ERROR("PinsBox: I2C bus times out, USB current measurement skipped (%u passes)", this->usbBusError_.failures());
            }
        }
        else if(i2cPass.timeouts == 0 && i2cPass.answered && this->usbBusError_.succeed())
        {
            PB_INFO("PinsBox: I2C bus answers again, USB current measurement works");
        }

        // Dew ports
        for(int i = 0; i < PINSBOX_NUM_DEW_PORTS; ++i)
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->dew_[i]->update(this->dewPoint, this->dewThreshold[i]);
            this->dewCurrent[i] = this->dew_[i]->getCurrent_mA() * 0.001f;
            this->dewOvercurrent[i] = this->dew_[i]->getOverCurrent() ? 1 : 0;
            this->dewProbe[i] = std::round(this->dew_[i]->getTemperature() * 100.0f) / 100.0f;
            this->dewPWM[i] = this->dew_[i]->getDutyCycle();
            this->dewState[i] = this->dew_[i]->getState();
        }

        // Adjustable ports
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->buck_->setSupplyVoltage(this->supply12V);
            this->buck_->measureCurrent();
            this->buckVmin = this->buck_->getMinVoltage();
            this->buckVmax = this->buck_->getMaxVoltage();
            this->buckVset = this->buck_->getTargetVoltage();
            this->buckVoltage = this->buck_->getVoltage();
            this->buckCurrent = this->buck_->getCurrent_mA() * 0.001f;
            this->buckOvercurrent = this->buck_->getOverCurrent() ? 1 : 0;
            this->buckOvervoltage = this->buck_->getOverVoltage() ? 1 : 0;
            this->buckState = this->buck_->getState();
        }

        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->pwm_->measureCurrent();
            this->pwmCurrent = this->pwm_->getCurrent_mA() * 0.001f;
            this->pwmOvercurrent = this->pwm_->getOverCurrent() ? 1 : 0;
            this->pwmState = this->pwm_->getState();
            this->pwmPWM = this->pwm_->getDutyCycle();
        }
    }

    int PinsBoxDevice::GetNumPowerPorts(void) const
    {
        return PINSBOX_NUM_POWER_PORTS;
    }

    int PinsBoxDevice::GetNumUSBPorts(void) const
    {
        return PINSBOX_NUM_USB_PORTS;
    }

    int PinsBoxDevice::GetNumDewPorts(void) const
    {
        return PINSBOX_NUM_DEW_PORTS;
    }

    int PinsBoxDevice::GetNumBuckPorts(void) const
    {
        return PINSBOX_NUM_BUCK_PORTS;
    }

    int PinsBoxDevice::GetNumPWMPorts(void) const
    {
        return PINSBOX_NUM_PWM_PORTS;
    }

    int PinsBoxDevice::GetPowerState(int i)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        // Port 0 is always on
        if (i == 0) {
            return 1;
        }

        if (i < 1 || i >= PINSBOX_NUM_POWER_PORTS) {
            return 0;
        }

        uint8_t state = this->pwr_[i]->getState();
        this->powerState[i] = state;
        return state;
    }

    bool PinsBoxDevice::SetPowerState(int i, int state)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        // Port 0 is always on
        if (i == 0) {
            return false;
        }

        if (i < 1 || i >= PINSBOX_NUM_POWER_PORTS) {
            return false;
        }

        // Report the state the port is actually in, a failed switch must not
        // show up as success in the API
        bool ok = this->pwr_[i]->setState(state);
        this->powerState[i] = this->pwr_[i]->getState();
        // Switching clears the over current flag: show it in the status at once
        this->powerOvercurrent[i] = this->pwr_[i]->getOverCurrent() ? 1 : 0;
        return ok;
    }

    bool PinsBoxDevice::SetPowerBootState(int i, int state)
    {
        this->powerBootstrap[i] = state;
        SaveSettings();
        return true;
    }

    int PinsBoxDevice::GetUSBState(int i)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        if (i < 0 || i >= PINSBOX_NUM_USB_PORTS) {
            return 0;
        }

        uint8_t state = this->usb_[i]->getState();
        this->usbState[i] = state;
        return state;
    }

    bool PinsBoxDevice::SetUSBState(int i, int state)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        if (i < 0 || i >= PINSBOX_NUM_USB_PORTS) {
            return false;
        }

        // Report the state the port is actually in, a failed switch must not
        // show up as success in the API
        bool ok = state ? this->usb_[i]->enable() : this->usb_[i]->disable();
        this->usbState[i] = this->usb_[i]->getState();
        // Switching clears the over current flag: show it in the status at once
        this->usbOvercurrent[i] = this->usb_[i]->getOverCurrent() ? 1 : 0;
        return ok;
    }

    bool PinsBoxDevice::SetUSBBootState(int i, int state)
    {
        this->usbBootstrap[i] = state;
        SaveSettings();
        return true;
    }

    int PinsBoxDevice::GetDewState(int i)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        if (i < 0 || i >= PINSBOX_NUM_DEW_PORTS) {
            return 0;
        }

        uint8_t state = this->dew_[i]->getState();
        this->dewState[i] = state;
        this->dewPWM[i] = this->dew_[i]->getDutyCycle();
        return state;
    }

    bool PinsBoxDevice::SetDewState(int i, int state, int power)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        if (i < 0 || i >= PINSBOX_NUM_DEW_PORTS) {
            return false;
        }

        // On auto mode, do nothing
        if (dewAuto[i]) {
            return false;
        }

        this->dew_[i]->setState(state, power);

        // Report what the hardware really does; switching clears the over
        // current flag: show it in the status at once
        this->dewState[i] = this->dew_[i]->getState();
        this->dewPWM[i] = this->dew_[i]->getDutyCycle();
        this->dewOvercurrent[i] = this->dew_[i]->getOverCurrent() ? 1 : 0;

        if (state != 0 && this->dewState[i] == 0)
        {
            PB_ERROR("Dew port %d: could not be switched on%s", i, this->dew_[i]->isPwmOk() ? "" : " (PWM not initialized)");
            return false;
        }
        if (state != 0 && this->dewOvercurrent[i] != 0)
        {
            // A failed over current shutdown is pending: the request was rejected
            // (normally logged by the port), the heater stays at duty 0
            return false;
        }
        if (state == 0 && this->dewState[i] != 0)
        {
            // The heater keeps running: the caller must know
            PB_ERROR("Dew port %d: could not be switched off", i);
            return false;
        }
        return true;
    }

    bool PinsBoxDevice::SetDewAutoMode(int i, int state)
    {
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            this->dewAuto[i] = state;
            this->dew_[i]->setAutoMode(state);
            // Auto mode on clears the over current flag: show it at once
            this->dewOvercurrent[i] = this->dew_[i]->getOverCurrent() ? 1 : 0;
            this->dewState[i] = this->dew_[i]->getState();
            if (state && !this->dew_[i]->isPwmOk())
            {
                // No error return: the rest of the dew configuration must still be applied
                PB_ERROR("Dew port %d: auto mode set, but the PWM is not initialized, the heater stays off until a retry of the PWM setup succeeds", i);
            }
        }
        SaveSettings();
        return true;
    }

    bool PinsBoxDevice::SetDewAutoThreshold(int i, float val)
    {
        this->dewThreshold[i] = val;
        SaveSettings();
        return true;
    }

    int PinsBoxDevice::GetBuckState(void)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        uint8_t state = this->buck_->getState();
        this->buckState = state;
        this->buckVmin = this->buck_->getMinVoltage();
        this->buckVmax = this->buck_->getMaxVoltage();
        return state;
    }

    bool PinsBoxDevice::SetBuckState(int state, float target)
    {
        bool ok;
        {
            std::lock_guard<std::mutex> lock(this->hwMutex_);
            ok = this->buck_->setState(state != 0, target);
            this->buckState = this->buck_->getState();
            // Switching clears the over current flag: show it at once
            this->buckOvercurrent = this->buck_->getOverCurrent() ? 1 : 0;
            // Report and persist the target the port really uses, a rejected
            // target must not be applied again at the next boot
            this->buckVset = this->buck_->getTargetVoltage();
            this->buckOvervoltage = this->buck_->getOverVoltage() ? 1 : 0;
        }
        SaveSettings();
        return ok;
    }

    bool PinsBoxDevice::SetBuckBootState(int state)
    {
        this->buckBootstrap = state;
        SaveSettings();
        return true;
    }

    int PinsBoxDevice::GetPWMState(void)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        uint8_t state = this->pwm_->getState();
        this->pwmState = state;
        this->pwmPWM = this->pwm_->getDutyCycle();
        return state;
    }

    bool PinsBoxDevice::SetPWMState(int state, int power)
    {
        std::lock_guard<std::mutex> lock(this->hwMutex_);
        this->pwm_->setState(state, power);

        // Report what the hardware really does (power is not range checked);
        // switching clears the over current flag: show it at once
        this->pwmState = this->pwm_->getState();
        this->pwmPWM = this->pwm_->getDutyCycle();
        this->pwmOvercurrent = this->pwm_->getOverCurrent() ? 1 : 0;

        if (state != 0 && this->pwmState == 0)
        {
            PB_ERROR("PWM port: could not be switched on%s", this->pwm_->isPwmOk() ? "" : " (PWM not initialized)");
            return false;
        }
        if (state != 0 && this->pwmOvercurrent != 0)
        {
            // A failed over current shutdown is pending: the request was rejected
            // (normally logged by the port), the output stays at duty 0
            return false;
        }
        if (state == 0 && this->pwmState != 0)
        {
            // The port keeps running: the caller must know
            PB_ERROR("PWM port: could not be switched off");
            return false;
        }
        return true;
    }









    PB_ERROR_TYPE PinsBoxDevice::Beep(int volume, int duration_ms)
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






    static bool IsCM5(void)
    {
        std::ifstream file("/proc/device-tree/model");
        if(!file.is_open())
        {
            return false;
        }

        std::string model;
        std::getline(file, model);

        return model.find("Compute Module 5") != std::string::npos;
    }

    int ScanPinsBox(int *ids)
    {
        // Check if this is a Raspberry PI CM5
        std::error_code fsError;
        if(!ids || !IsCM5() || !std::filesystem::exists("/etc/pinsDevice", fsError))
        {
            return 0;
        }

        std::lock_guard<std::mutex> lock(g_globalMutex);

        PB_DEBUG("Valid device found!");

        // Check if device already exists, reuse it instead of creating a new one.
        // The process lock is only taken here: a second flock on a new file
        // descriptor would fail against our own lock.
        if(g_devices.find(0) == g_devices.end())
        {
            ProcessLock processLock;
            if(processLock.acquire(LOCK_FILE) == ProcessLock::Busy)
            {
                PB_ERROR("PinsBox: used by another process, not opened here");
                return -1;
            }

            // Lock held from reading the boot marker to writing it. Declared
            // after the lock: if setup fails, the device (GPIO, I/O expander) is
            // released before the lock.
            auto device = std::make_shared<PinsBoxDevice>();
            if(!device->IsInitialized())
            {
                return -1;
            }

            device->AdoptLock(std::move(processLock));
            g_devices[0] = device;

            // Start status listener thread
            g_devices[0]->StartStatusListener();
        }
        else
        {
            // Id 0 may hold another device type (e.g. a serial PowerBox registered
            // by PBScanPort after a failed PinsBox scan). Do not report it as the
            // PinsBox. g_devices is never cleared, so this lasts until the process
            // ends (PBScanPort should not use id 0 on a PinsBox - not done yet).
            if(!std::dynamic_pointer_cast<PinsBoxDevice>(g_devices[0]))
            {
                PB_ERROR("PinsBox: id 0 is used by another device, the PinsBox is not opened");
                return -1;
            }
            PB_DEBUG("Device at id=0 already exists, reusing it");
        }
        
        ids[0] = 0;

        return 1;
    }
} /* namespace PowerBox */
