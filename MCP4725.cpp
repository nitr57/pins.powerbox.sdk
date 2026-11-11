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

#include "MCP4725.h"
#include "common.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

#define MCP4725_MAX 4095

/* Command byte: C2 C1 C0 x x PD1 PD0 x, power down bits always 00 (normal) */
#define MCP4725_CMD_WRITE_DAC           0x40
#define MCP4725_CMD_WRITE_DAC_EEPROM    0x60

/* Status byte of a read: bit 7 is RDY/BSY, 1 = EEPROM write finished,
 * bits 2:1 are the power down bits of the DAC register (00 = normal) */
#define MCP4725_STATUS_READY            0x80
#define MCP4725_STATUS_PD_MASK          0x06

/* First EEPROM byte of a read: x PD1 PD0 x D11..D8 */
#define MCP4725_EEPROM_PD_MASK          0x60

/* An EEPROM write takes 25 ms typ., 50 ms max. */
#define MCP4725_EEPROM_TIMEOUT_MS       100
#define MCP4725_WRITE_RETRIES           3

namespace PowerBox
{
    MCP4725::MCP4725(const char* dev, uint8_t address, float vref)
    {
        this->dev_ = dev;
        this->address_ = address;
        this->i2c_fd_ = -1;
        this->vref_ = vref;
        this->current_val_ = 0;
    }

    MCP4725::~MCP4725(void)
    {
        if (this->i2c_fd_ >= 0) {
            close(this->i2c_fd_);
            this->i2c_fd_ = -1;
        }
    }

    bool MCP4725::begin(void)
    {
        // Open I2C device
        this->i2c_fd_ = open(this->dev_, O_RDWR);
        if (this->i2c_fd_ < 0)
        {
            PB_ERROR("MCP4725: failed to open %s", this->dev_);
            return false;
        }

        // Set I2C slave address
        if (ioctl(this->i2c_fd_, I2C_SLAVE, this->address_) < 0) {
            PB_ERROR("MCP4725: failed to set I2C address 0x%02x", this->address_);
            close(this->i2c_fd_);
            this->i2c_fd_ = -1;
            return false;
        }

        // Find out what the DAC outputs right now (with retries: a running
        // converter is switched off if this fails)
        uint16_t dac;
        bool pd;
        if (!this->readDac(dac, pd))
        {
            PB_ERROR("MCP4725: no response at 0x%02x", this->address_);
            close(this->i2c_fd_);
            this->i2c_fd_ = -1;
            return false;
        }

        this->current_val_ = dac;

        return true;
    }

    bool MCP4725::setVoltage(float v)
    {
        uint16_t val;
        if (!this->toValue_(v, &val))
        {
            PB_ERROR("MCP4725: voltage %.3f V is out of range (0 - %.3f V)", v, this->vref_);
            return false;
        }

        return this->setValue_(val);
    }

    bool MCP4725::setPowerOnVoltage(float v)
    {
        uint16_t val;
        if (!this->toValue_(v, &val))
        {
            PB_ERROR("MCP4725: power-on voltage %.3f V is out of range (0 - %.3f V)", v, this->vref_);
            return false;
        }

        if (this->i2c_fd_ < 0)
        {
            return false;
        }

        // Only write the EEPROM if needed, it has limited write cycles.
        // Power down bits other than 00 would switch the output off at power-on
        // (pulled to GND), which gives the highest converter output voltage.
        uint16_t dac, eeprom;
        uint8_t eepromPd;
        if (!this->readState_(nullptr, &dac, &eeprom, &eepromPd))
        {
            return false;
        }

        if (eeprom == val && eepromPd == 0)
        {
            return true;
        }

        PB_INFO("MCP4725: changing power-on value from %u (PD %u) to %u", eeprom, eepromPd, val);

        if (!this->waitReady_() || !this->writeRegister_(MCP4725_CMD_WRITE_DAC_EEPROM, val) || !this->waitReady_())
        {
            PB_ERROR("MCP4725: failed to write EEPROM");
        }

        // The write above also changed the DAC register, keep track of it
        if (!this->readState_(nullptr, &dac, &eeprom, &eepromPd))
        {
            return false;
        }

        this->current_val_ = dac;

        return eeprom == val && eepromPd == 0;
    }

    bool MCP4725::readDac(uint16_t& code, bool& poweredDown)
    {
        if (this->i2c_fd_ < 0)
        {
            return false;
        }

        for (int attempt = 0; attempt < MCP4725_WRITE_RETRIES; attempt++)
        {
            uint8_t status;
            if (this->readState_(&status, &code, nullptr, nullptr))
            {
                poweredDown = (status & MCP4725_STATUS_PD_MASK) != 0;
                return true;
            }

            if (attempt + 1 < MCP4725_WRITE_RETRIES)
            {
                usleep(2000);
            }
        }

        return false;
    }

    float MCP4725::getVoltage(void) const
    {
        return this->current_val_ * this->vref_ / MCP4725_MAX;
    }

    bool MCP4725::toValue_(float v, uint16_t* val) const
    {
        // Negated check, so NaN is rejected as well
        if (!(v >= 0.f && v <= this->vref_))
        {
            return false;
        }

        long raw = lroundf((v * MCP4725_MAX) / this->vref_);
        if (raw < 0 || raw > MCP4725_MAX)
        {
            return false;
        }

        *val = (uint16_t)raw;
        return true;
    }

    bool MCP4725::setValue_(uint16_t val)
    {
        if (this->i2c_fd_ < 0)
        {
            return false;
        }

        // Always write and verify. Skipping the write based on a cached value
        // is not safe, the DAC may have been changed or reset in the meantime.
        for (int attempt = 0; attempt < MCP4725_WRITE_RETRIES; attempt++)
        {
            uint8_t status;
            uint16_t dac;
            if (this->writeRegister_(MCP4725_CMD_WRITE_DAC, val) &&
                this->readState_(&status, &dac, nullptr, nullptr))
            {
                this->current_val_ = dac;

                // The value only counts if the output is not powered down
                if (dac == val && (status & MCP4725_STATUS_PD_MASK) == 0)
                {
                    return true;
                }
            }

            // The chip ignores writes while an EEPROM write is in progress
            if (attempt + 1 < MCP4725_WRITE_RETRIES)
            {
                usleep(10000);
            }
        }

        PB_ERROR("MCP4725: DAC does not accept value %u (holds %u)", val, this->current_val_);
        return false;
    }

    bool MCP4725::readState_(uint8_t* status, uint16_t* dac, uint16_t* eeprom, uint8_t* eepromPd)
    {
        // Read: status, DAC register (2 bytes), EEPROM (2 bytes)
        uint8_t buffer[5];

        if (read(this->i2c_fd_, buffer, sizeof(buffer)) != (ssize_t)sizeof(buffer))
        {
            return false;
        }

        if (status)
        {
            *status = buffer[0];
        }

        if (dac)
        {
            *dac = ((uint16_t)buffer[1] << 4) | (buffer[2] >> 4);
        }

        if (eeprom)
        {
            *eeprom = ((uint16_t)(buffer[3] & 0x0F) << 8) | buffer[4];
        }

        if (eepromPd)
        {
            *eepromPd = (buffer[3] & MCP4725_EEPROM_PD_MASK) >> 5;
        }

        return true;
    }

    bool MCP4725::waitReady_(void)
    {
        for (int waited = 0; ; waited += 5)
        {
            uint8_t status;
            if (this->readState_(&status, nullptr, nullptr, nullptr) && (status & MCP4725_STATUS_READY))
            {
                return true;
            }

            if (waited >= MCP4725_EEPROM_TIMEOUT_MS)
            {
                return false;
            }

            usleep(5000);
        }
    }

    bool MCP4725::writeRegister_(uint8_t cmd, uint16_t val)
    {
        uint8_t msb = (val >> 4) & 0xFF;
        uint8_t lsb = (val & 0x0F) << 4;

        uint8_t buffer[3] = { cmd, msb, lsb };

        return write(this->i2c_fd_, buffer, 3) == 3;
    }
} /* namespace PowerBox */
