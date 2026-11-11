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

PWM::PWM(const char* chip, int ch)
{
    this->chip_ = "/sys/class/pwm/" + std::string(chip);
    this->channel_ = ch;
    this->exported_ = false;
}

PWM::~PWM(void)
{
}

bool PWM::setExport(void)
{
    if(this->exported_)
    {
        return true;
    }

    // Already exported (e.g. by an earlier process): keep it as it is
    struct stat st;
    if(stat((this->chip_ + "/pwm" + std::to_string(this->channel_)).c_str(), &st) == 0 && S_ISDIR(st.st_mode))
    {
        this->exported_ = true;
        return true;
    }

    this->exported_ = this->write_(this->chip_ + "/export", std::to_string(this->channel_));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return this->exported_;
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