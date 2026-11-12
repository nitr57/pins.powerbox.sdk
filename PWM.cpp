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

#include "PWM.h"
#include <thread>
#include <chrono>
#include <string>
#include <fstream>
#include <cstdlib>
#include <sys/stat.h>
#include <unistd.h>
#include <atomic>
#include "PowerBoxLogging.h"

/* Separate, so a missing chip (e.g. PIO overlay) does not stop healthy chips from
 * waiting for udev. Each channel waits at most 2 s for its export and 2 s for
 * udev; after the first export timeout and the first udev timeout in the process
 * no further channel waits for that (scan under the global mutex) */
static std::atomic<bool> g_exportTimedOut{false};
static std::atomic<bool> g_udevTimedOut{false};
static const std::chrono::milliseconds EXPORT_TIMEOUT{2000};
static const std::chrono::milliseconds EXPORT_POLL{50};

PWM::PWM(const char* chip, int ch)
{
    this->chip_ = "/sys/class/pwm/" + std::string(chip);
    this->channel_ = ch;
    this->exported_ = false;
    this->exportErrorLogged_ = false;
}

PWM::~PWM(void)
{
}

bool PWM::exists(void) const
{
    struct stat st;
    return stat((this->chip_ + "/pwm" + std::to_string(this->channel_)).c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool PWM::writable_(void) const
{
    // udev changes group and mode of all attributes, not in a fixed order
    const std::string ch = this->chip_ + "/pwm" + std::to_string(this->channel_);
    return access((ch + "/period").c_str(), W_OK) == 0 &&
           access((ch + "/duty_cycle").c_str(), W_OK) == 0 &&
           access((ch + "/enable").c_str(), W_OK) == 0;
}

bool PWM::setExport(bool wait)
{
    if(this->exported_)
    {
        return true;
    }

    const std::string name = this->chip_ + "/pwm" + std::to_string(this->channel_);

    // Already exported (e.g. by an earlier process): keep it as it is, no wait
    if(this->exists())
    {
        this->exported_ = true;
        if(!this->writable_())
        {
            PB_ERROR("PWM %s is not writable (udev rule for /sys/class/pwm, group gpio?)", name.c_str());
        }
        return true;
    }

    // The chip or the permission for export may not be there yet at boot
    const bool waitExport = wait && !g_exportTimedOut;
    const auto exportDeadline = std::chrono::steady_clock::now() + (waitExport ? EXPORT_TIMEOUT : std::chrono::milliseconds(0));
    for(;;)
    {
        if(this->write_(this->chip_ + "/export", std::to_string(this->channel_)) || this->exists())
        {
            break;
        }
        if(std::chrono::steady_clock::now() >= exportDeadline)
        {
            if(waitExport)
            {
                g_exportTimedOut = true;
            }
            // Retries keep failing the same way: report it once as error
            if(!this->exportErrorLogged_)
            {
                PB_ERROR("PWM %s could not be exported (PWM overlay loaded? permission for %s/export?)", name.c_str(), this->chip_.c_str());
                this->exportErrorLogged_ = true;
            }
            else
            {
                PB_DEBUG("PWM %s could not be exported", name.c_str());
            }
            return false;
        }
        std::this_thread::sleep_for(EXPORT_POLL);
    }

    this->exported_ = true;

    // udev makes the attributes group writable shortly after the export: own
    // deadline from the export on, so a late chip still gets its udev time
    const bool waitUdev = wait && !g_udevTimedOut;
    const auto udevDeadline = std::chrono::steady_clock::now() + (waitUdev ? EXPORT_TIMEOUT : std::chrono::milliseconds(0));
    while(!this->writable_())
    {
        if(std::chrono::steady_clock::now() >= udevDeadline)
        {
            if(waitUdev)
            {
                g_udevTimedOut = true;
            }
            if(wait)
            {
                PB_ERROR("PWM %s not writable after the export (udev rule for /sys/class/pwm, group gpio?)", name.c_str());
            }
            else
            {
                // A retry without waiting: udev is usually just not done yet
                PB_DEBUG("PWM %s not writable yet after the export", name.c_str());
            }
            break;
        }
        std::this_thread::sleep_for(EXPORT_POLL);
    }

    return true;
}

bool PWM::setPeriod(unsigned int period_ns)
{
    return this->write_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/period", std::to_string(period_ns));
}

bool PWM::setDutyCycle(unsigned int duty_ns)
{
    return this->write_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/duty_cycle", std::to_string(duty_ns));
}

bool PWM::setState(bool state)
{
    return this->write_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/enable", state ? "1" : "0");
}

unsigned int PWM::getPeriod(void) const
{
    std::string val = this->read_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/period");
    // strtoul instead of std::stoul, this runs in device threads and must not throw
    return val.empty() ? 0 : (unsigned int)std::strtoul(val.c_str(), nullptr, 10);
}

bool PWM::readPeriod(unsigned int& period_ns) const
{
    std::string val = this->read_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/period");
    if(val.empty())
    {
        return false;
    }
    period_ns = (unsigned int)std::strtoul(val.c_str(), nullptr, 10);
    return true;
}

unsigned int PWM::getDutyCycle(void) const
{
    std::string val = this->read_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/duty_cycle");
    // strtoul instead of std::stoul, this runs in device threads and must not throw
    return val.empty() ? 0 : (unsigned int)std::strtoul(val.c_str(), nullptr, 10);
}

int PWM::getState(void) const
{
    std::string val = this->read_(this->chip_ + "/pwm" + std::to_string(this->channel_) + "/enable");
    return val == "1";
}

std::string PWM::read_(const std::string& path) const
{
    std::ifstream ifs(path);
    if(!ifs)
    {
        return "";
    }

    std::string val;
    ifs >> val;

    return val;
}

bool PWM::write_(const std::string& path, const std::string& val)
{
    std::ofstream ofs(path);
    if(!ofs)
    {
        return false;
    }

    ofs << val;

    // sysfs reports rejected values (EINVAL, EBUSY) from write(2), which the
    // stream only issues when it is flushed: close explicitly and check
    ofs.close();
    return !ofs.fail();
}