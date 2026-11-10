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

#include "DHT22.h"
#include "DS18B20.h"
#include <fstream>
#include <cstdlib>
#include <cmath>

namespace PowerBox
{
    DHT22::DHT22(const char* dev)
    {
        this->path_ = "/sys/bus/iio/devices/iio:" + std::string(dev);
    }

    DHT22::~DHT22(void)
    {
    }

    float DHT22::getTemperature(void) const
    {
        return this->hold_(this->read_(this->path_ + "/in_temp_input"), this->temperature_);
    }

    float DHT22::getHumidity(void) const
    {
        return this->hold_(this->read_(this->path_ + "/in_humidityrelative_input"), this->humidity_);
    }

    float DHT22::hold_(const std::string& val, Reading& last) const
    {
        auto now = std::chrono::steady_clock::now();

        if(!val.empty())
        {
            // strtof instead of std::stof, a garbled read must not throw and kill the DHT22 thread
            char *end = nullptr;
            float raw = std::strtof(val.c_str(), &end);
            if(end != val.c_str() && std::isfinite(raw))
            {
                last.value = raw * 0.001f;
                last.valid = true;
                last.time = now;

                return last.value;
            }
        }

        // The kernel dht11 driver fails reads regularly, keep the last good value for a while
        if(last.valid && now - last.time < SENSOR_GRACE_PERIOD)
        {
            return last.value;
        }

        return DEVICE_DISCONNECTED_C;
    }

    std::string DHT22::read_(const std::string& path) const
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
} /* namespace PowerBox */