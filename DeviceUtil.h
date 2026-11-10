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

#ifndef DEVICE_UTIL_H
#define DEVICE_UTIL_H

#include "PowerBoxLogging.h"
#include <json/json.h>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <fcntl.h>
#include <unistd.h>

/* Helpers shared by the Raspberry Pi based devices. None of them throws: they run
 * inside SDK API calls and device threads, where an escaping C++ exception
 * terminates the host process (NINA) */

namespace PowerBox
{
    enum class SettingsLoad
    {
        Missing,    /* file does not exist yet */
        Invalid,    /* file exists but is not a valid JSON object */
        Ok
    };

    /* Parses a JSON settings file into root, which must be a JSON object */
    inline SettingsLoad LoadSettingsFile(const char *path, Json::Value &root)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            return SettingsLoad::Missing;
        }

        try
        {
            Json::CharReaderBuilder builder;
            std::string errs;
            if (!Json::parseFromStream(builder, file, &root, &errs) || !root.isObject())
            {
                PB_ERROR("Invalid settings file %s, using defaults (%s)", path, errs.c_str());
                return SettingsLoad::Invalid;
            }
        }
        catch (const std::exception &e)
        {
            PB_ERROR("Cannot read settings file %s, using defaults (%s)", path, e.what());
            return SettingsLoad::Invalid;
        }

        return SettingsLoad::Ok;
    }

    /* Writes root to a temp file, syncs it and renames it over path, so a power
     * loss during the write cannot leave a truncated settings file behind */
    inline bool SaveSettingsFile(const char *path, const Json::Value &root)
    {
        std::string content;
        try
        {
            Json::StreamWriterBuilder builder;
            content = Json::writeString(builder, root);
        }
        catch (const std::exception &e)
        {
            PB_ERROR("Cannot serialize settings for %s (%s)", path, e.what());
            return false;
        }

        std::string tmpPath = std::string(path) + ".tmp";
        int fd = ::open(tmpPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0)
        {
            PB_ERROR("Cannot open %s", tmpPath.c_str());
            return false;
        }

        bool ok = ::write(fd, content.data(), content.size()) == (ssize_t)content.size();
        ok = (::fsync(fd) == 0) && ok;
        ok = (::close(fd) == 0) && ok;

        if (!ok || std::rename(tmpPath.c_str(), path) != 0)
        {
            PB_ERROR("Failed to write settings file %s", path);
            std::remove(tmpPath.c_str());
            return false;
        }

        return true;
    }

    /* SoC temperature in degrees, or fallback if it cannot be read */
    inline float ReadCoreTemp(float fallback)
    {
        std::ifstream ifs("/sys/class/thermal/thermal_zone0/temp");
        std::string line;
        if (!ifs || !std::getline(ifs, line))
        {
            return fallback;
        }

        /* The file holds millidegrees; strtol instead of std::stoi so an empty or
         * garbled read cannot throw */
        char *end = nullptr;
        long milli = std::strtol(line.c_str(), &end, 10);
        if (end == line.c_str())
        {
            return fallback;
        }

        return milli / 1000.f;
    }

    /* Last 7 characters of the serial, or the whole serial if it is shorter */
    inline std::string ShortSerial(const std::string &serial)
    {
        return serial.length() >= 7 ? serial.substr(serial.length() - 7) : serial;
    }

} /* namespace PowerBox */

#endif /* DEVICE_UTIL_H */
