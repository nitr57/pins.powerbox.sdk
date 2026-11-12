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
#include "PowerBoxSDK.h"
#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <atomic>
#include <chrono>
#include <thread>

namespace PowerBox
{
	/* ============================================================================
	 * LOGGING IMPLEMENTATION
	 * ============================================================================ */

	/* Callback of the application (PBSetLogCallback), constant-initialized so it
	 * also works during static initialization and destruction. g_logInFlight
	 * counts running calls: PBSetLogCallback(NULL) waits for them. */
	static std::atomic<PB_LOG_CALLBACK> g_logCallback{nullptr};
	static std::atomic<int> g_logInFlight{0};
	static std::atomic<int> g_logLevel{PB_LOG_INFO};

	/* Set while this thread is inside the callback: no recursion, no self-wait */
	static thread_local bool t_inLogCallback = false;

	/* Formats the whole line into one buffer and writes it with one call, so
	 * lines of different threads do not interleave (longer lines are cut);
	 * then passes the message part to the callback */
	static void PBLogLine(int level, const char *tag, const char *fmt, va_list args)
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
			n = snprintf(line, sizeof(line), "[%s] [%s] ", timestamp, tag);
		}
		else
		{
			n = snprintf(line, sizeof(line), "[%s] ", tag);
		}
		if (n > 0)
		{
			len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
		}
		const size_t prefixLen = len;

		// Keep room for the newline
		n = vsnprintf(line + len, sizeof(line) - len - 1, fmt, args);
		if (n > 0)
		{
			len += (size_t)n < sizeof(line) - len - 1 ? (size_t)n : sizeof(line) - len - 2;
		}
		line[len++] = '\n';
		line[len] = '\0';

		// stderr (journal) first: the line is there even if the callback hangs
		fputs(line, stderr);

		if (t_inLogCallback || level > g_logLevel.load())
		{
			return;
		}

		// Counts this call while it may use the callback; also undone if the
		// callback throws (the in-callback flag as well)
		struct InFlight
		{
			InFlight() { g_logInFlight.fetch_add(1); }
			~InFlight() { t_inLogCallback = false; g_logInFlight.fetch_sub(1); }
		} inFlight;

		PB_LOG_CALLBACK callback = g_logCallback.load();
		if (callback != nullptr)
		{
			line[len - 1] = '\0';  // without the newline
			t_inLogCallback = true;
			callback(level, line + prefixLen);
		}
	}

	void PBLogDebug(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		PBLogLine(PB_LOG_DEBUG, "PB_DEBUG", fmt, args);
		va_end(args);
	}

	void PBLogInfo(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		PBLogLine(PB_LOG_INFO, "PB_INFO", fmt, args);
		va_end(args);
	}

	void PBLogError(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		PBLogLine(PB_LOG_ERROR, "PB_ERROR", fmt, args);
		va_end(args);
	}

} /* namespace PowerBox */

PBAPI void PBSetLogCallback(PB_LOG_CALLBACK callback)
{
	using namespace PowerBox;
	g_logCallback.store(callback);

	// Called from inside the callback: do not wait for ourselves
	if (t_inLogCallback)
	{
		return;
	}

	// Wait for calls that may still use the previous callback (at most 1 s):
	// a few yields first (calls are short), then sleep instead of spinning
	auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
	for (int spin = 0; g_logInFlight.load() > 0 && std::chrono::steady_clock::now() < deadline; ++spin)
	{
		if (spin < 100)
		{
			std::this_thread::yield();
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

PBAPI void PBSetLogLevel(int level)
{
	if (level < PB_LOG_ERROR)
	{
		level = PB_LOG_ERROR;
	}
	if (level > PB_LOG_DEBUG)
	{
		level = PB_LOG_DEBUG;
	}
	PowerBox::g_logLevel.store(level);
}
