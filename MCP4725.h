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

#ifndef MCP4725_H
#define MCP4725_H

#include <stdint.h>

namespace PowerBox
{
    class MCP4725
    {
        public:
            MCP4725(const char* dev, uint8_t address, float vref);
            ~MCP4725(void);

            bool begin(void);

            /* Writes the DAC register only (no EEPROM wear) and verifies it by
             * reading it back. Returns false if the DAC does not hold the
             * requested value afterwards. */
            bool setVoltage(float v);

            /* Programs the EEPROM, i.e. the value the DAC loads at power-on.
             * The EEPROM is only written if it holds a different value.
             * Note: this also sets the DAC output to that value. */
            bool setPowerOnVoltage(float v);

            /* Voltage the DAC currently outputs, as last read back from it */
            float getVoltage(void) const;

        private:
            bool toValue_(float v, uint16_t* val) const;
            bool setValue_(uint16_t val);

            bool readState_(uint8_t* status, uint16_t* dac, uint16_t* eeprom, uint8_t* eepromPd);
            bool waitReady_(void);
            bool writeRegister_(uint8_t cmd, uint16_t val);

            // I2C
            int i2c_fd_;
            const char* dev_;
            uint8_t address_;

            float vref_;
            uint16_t current_val_;
    };
} /* namespace PowerBox */

#endif // MCP4725_H
