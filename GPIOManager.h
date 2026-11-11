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

#ifndef GPIO_MANAGER_H
#define GPIO_MANAGER_H

#include "TCA953X.h"
#ifdef HAVE_LIBGPIOD
#include <gpiod.h>
#endif
#include <cstdint>

namespace PowerBox
{
    class GPIOManager
    {
        public:
            GPIOManager(const char* gpiochip, const char* i2c, uint8_t addr, int ngpio_in, const uint8_t* gpio_in, int ngpio_out, const uint8_t* gpio_out);
            ~GPIOManager(void);

            void clear(void);

            /* Original start: all CM5 lines as output HIGH, TCA pins switched to
             * output without touching the output latch. Used by the variants that
             * have not been reworked yet. */
            bool begin(void);

            /* Glitch free start (CM5 PinsBox):
             * - An output that is already configured (by an earlier process, the
             *   firmware or a TCA that kept its state over a reboot) keeps its level
             *   if it is one of keepPins; no pulse on any start.
             * - All other lines start at their idle level: HIGH for idleHigh, else
             *   LOW. TCA output latches are written before the pins become outputs.
             * - keepState: report taken over levels via keptValue() (otherwise the
             *   caller applies its boot states).
             * Order: the CM5 request comes first, a second process fails there
             * (EBUSY) before it touches the TCA. */
            bool begin(bool keepState, const uint8_t* keepPins, int nKeepPins, const uint8_t* idleHigh, int nIdleHigh);

            /* Level taken over by begin(keepState = true, ...): -1 = not taken over */
            int keptValue(uint8_t pin) const;

            uint8_t digitalWrite(uint8_t pin, uint8_t data) const;
            uint8_t digitalRead(uint8_t pin) const;

        private:
            bool init_;

            // libgpiod (real types when available, placeholders otherwise)
#ifdef HAVE_LIBGPIOD
            bool InitGpiod_(void);
            bool InitGpiodKeep_(bool keepState, const uint8_t* keepPins, int nKeepPins, const uint8_t* idleHigh, int nIdleHigh);
            bool InitTcaKeep_(bool keepState, const uint8_t* keepPins, int nKeepPins);

            const char* gpiochip_;
            gpiod_request_config* req_;
            gpiod_line_request* request_;
            gpiod_line_settings* settings_;
            gpiod_line_config* line_;
            gpiod_chip* chip_;
#else
            // Stubbed-out helpers so the SDK builds on non-RPi hosts
            bool InitGpiod_(void) { return false; }

            const char* gpiochip_;
            void* req_;
            void* request_;
            void* settings_;
            void* line_;
            void* chip_;
#endif

            // TCA953X
            TCA9535* tca_;

            // Levels taken over by the glitch free begin (bit n = pin n)
            uint64_t cm5KeptMask_;
            uint64_t cm5KeptVal_;
            uint16_t tcaKeptMask_;
            uint16_t tcaKeptVal_;

            // List of gpios
            int cm5_ngpio_;
            unsigned int* cm5_gpio_;

            int tca_ngpio_;
            unsigned int* tca_gpio_;
    };

#ifndef HAVE_LIBGPIOD
    /* Inline stub implementations for builds without libgpiod */
    inline GPIOManager::GPIOManager(const char* gpiochip, const char* i2c, uint8_t addr, int ngpio_in, const uint8_t* gpio_in, int ngpio_out, const uint8_t* gpio_out)
        : init_(false), gpiochip_(gpiochip), req_(nullptr), request_(nullptr), settings_(nullptr), line_(nullptr), chip_(nullptr), tca_(nullptr), cm5KeptMask_(0), cm5KeptVal_(0), tcaKeptMask_(0), tcaKeptVal_(0), cm5_ngpio_(0), cm5_gpio_(nullptr), tca_ngpio_(0), tca_gpio_(nullptr)
    {
        (void)ngpio_in; (void)gpio_in; (void)ngpio_out; (void)gpio_out; (void)addr;
        if (i2c)
            this->tca_ = new TCA9535(i2c, addr);
    }

    inline GPIOManager::~GPIOManager(void)
    {
        if (this->tca_) { delete this->tca_; this->tca_ = nullptr; }
        if (this->cm5_gpio_) { delete[] this->cm5_gpio_; this->cm5_gpio_ = nullptr; }
        if (this->tca_gpio_) { delete[] this->tca_gpio_; this->tca_gpio_ = nullptr; }
    }

    inline void GPIOManager::clear(void) { }
    inline bool GPIOManager::begin(void) { this->init_ = true; if (this->tca_) this->tca_->begin(); return true; }
    inline bool GPIOManager::begin(bool, const uint8_t*, int, const uint8_t*, int) { return this->begin(); }
    inline int GPIOManager::keptValue(uint8_t /*pin*/) const { return -1; }
    inline uint8_t GPIOManager::digitalWrite(uint8_t /*pin*/, uint8_t /*data*/) const { return 1; }
    inline uint8_t GPIOManager::digitalRead(uint8_t /*pin*/) const { return 0; }
#endif

} /* namespace PowerBox */

#endif /* GPIO_MANAGER_H */