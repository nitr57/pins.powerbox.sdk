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

#include "PowerBoxLogging.h"
#include <cstdio>
#include <cstdarg>
#include <ctime>

namespace PowerBox
{
	/* ============================================================================
	 * LOGGING IMPLEMENTATION
	 * ============================================================================ */

	const char *PBGetTimestamp()
	{
		static char timestamp[20];
		time_t now = time(nullptr);
		struct tm *timeinfo = localtime(&now);
		strftime(timestamp, sizeof(timestamp), "%H:%M:%S", timeinfo);
		return timestamp;
	}

	/* Formats the whole line into one buffer and writes it with one call, so
	 * lines of different threads do not interleave (longer lines are cut) */
	static void PBLogLine(const char *level, const char *fmt, va_list args)
	{
		char line[1024];
		size_t len = 0;
		int n;

		if (PowerBox::PB_TIMESTAMP_ENABLED)
		{
			char timestamp[20] = "";
			time_t now = time(nullptr);
			struct tm timeinfo;
#ifdef _WIN32
			if (localtime_s(&timeinfo, &now) == 0)
#else
			if (localtime_r(&now, &timeinfo) != nullptr)
#endif
			{
				strftime(timestamp, sizeof(timestamp), "%H:%M:%S", &timeinfo);
			}
			n = snprintf(line, sizeof(line), "[%s] [%s] ", timestamp, level);
		}
		else
		{
			n = snprintf(line, sizeof(line), "[%s] ", level);
		}
		if (n > 0)
		{
			len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
		}

		// Keep room for the newline
		n = vsnprintf(line + len, sizeof(line) - len - 1, fmt, args);
		if (n > 0)
		{
			len += (size_t)n < sizeof(line) - len - 1 ? (size_t)n : sizeof(line) - len - 2;
		}
		line[len++] = '\n';
		line[len] = '\0';

		fputs(line, stderr);
	}

	void PBLogDebug(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		PBLogLine("PB_DEBUG", fmt, args);
		va_end(args);
	}

	void PBLogInfo(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		PBLogLine("PB_INFO", fmt, args);
		va_end(args);
	}

	void PBLogError(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		PBLogLine("PB_ERROR", fmt, args);
		va_end(args);
	}

} /* namespace PowerBox */
