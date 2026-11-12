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


#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "INA219.h"
#include "I2CUtil.h"
#include "PowerBoxLogging.h"

#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_CALIBRATION   0x05

/* BRNG 0 (16 V), PG 11 (/8, +-320 mV), BADC 0011 (12 bit), SADC 1111 (128
 * samples, 68.1 ms), MODE 111 (shunt and bus, continuous). Power-on default
 * is 0x399F (32 V, /8, 12 bit, continuous). */
#define INA219_CONFIG_VALUE      0x19FF
#define INA219_CONFIG_DEFAULT    0x399F
#define INA219_CONFIG_PG_MASK    0x1800

/* Current LSB 100 uA at 0.1 ohm: 0.04096 / (100e-6 * 0.1) = 4096, so the
 * current register equals the shunt register (not used for the measurement) */
#define INA219_CALIBRATION_VALUE 4096

/* Transfer errors (no answer) happen now and then: retry, but not on a timeout */
static const int INA219_ATTEMPTS = 3;
static const useconds_t INA219_RETRY_DELAY_US = 500;

namespace PowerBox
{
    INA219::INA219(const char* dev, uint8_t address)
    {
        this->dev_ = dev;
        this->address_ = address;
        this->i2c_fd_ = -1;
        this->configured_ = false;
    }

    INA219::~INA219(void)
    {
        if (this->i2c_fd_ >= 0) {
            close(this->i2c_fd_);
            this->i2c_fd_ = -1;
        }
    }

    INA219::Result INA219::begin(void)
    {
        this->configured_ = false;

        Result res = OK;
        const char* what = "could not be configured";
        if (this->i2c_fd_ < 0)
        {
            int fd = open(this->dev_, O_RDWR);
            if (fd < 0)
            {
                res = ERROR;
                what = "failed to open the I2C device";
            }
            // Set I2C slave address (fails if a kernel driver uses the address)
            else if (ioctl(fd, I2C_SLAVE, this->address_) < 0)
            {
                close(fd);
                res = ERROR;
                what = "failed to set the I2C address";
            }
            else
            {
                SetI2CTimeout(fd, "INA219");
                this->i2c_fd_ = fd;
            }
        }

        if (res == OK)
        {
            res = this->configure_();
        }

        if (res != OK)
        {
            if (this->setupError_.fail())
            {
                PB_ERROR("INA219 0x%02x: %s, no current measurement (retried, %u failures)", this->address_, what, this->setupError_.failures());
            }
            return res;
        }

        // Report the recovery only after a logged failure
        if (this->setupError_.succeed())
        {
            PB_INFO("INA219 0x%02x: set up again, current measurement works", this->address_);
        }
        this->configured_ = true;
        return OK;
    }

    INA219::Result INA219::configure_(void)
    {
        Result res = this->writeRegister_(INA219_REG_CONFIG, INA219_CONFIG_VALUE);
        if (res == OK)
        {
            res = this->writeRegister_(INA219_REG_CALIBRATION, INA219_CALIBRATION_VALUE);
        }

        uint16_t config = 0;
        if (res == OK)
        {
            res = this->readRegister_(INA219_REG_CONFIG, config);
        }
        if (res == OK && config != INA219_CONFIG_VALUE)
        {
            res = ERROR;
        }
        return res;
    }

    INA219::Result INA219::checkConfig(bool& rewritten, bool& otherRange)
    {
        rewritten = false;
        otherRange = false;

        uint16_t config = 0;
        Result res = this->readRegister_(INA219_REG_CONFIG, config);
        if (res != OK)
        {
            return res;
        }

        if (config == INA219_CONFIG_VALUE)
        {
            return OK;
        }

        // Reset (e.g. brown-out): the default is also +-320 mV, so the shunt value
        // stays valid, but the averaging is lost. Any other value points to a bus
        // fault (e.g. SDA held low reads 0x0000). Logged rate limited, with the
        // number of occurrences since the start
        rewritten = true;
        otherRange = (config & INA219_CONFIG_PG_MASK) != (INA219_CONFIG_VALUE & INA219_CONFIG_PG_MASK);
        res = this->configure_();
        if (res == OK)
        {
            if (config == INA219_CONFIG_DEFAULT)
            {
                if (this->resetLog_.fail())
                {
                    PB_INFO("INA219 0x%02x: chip reset (configuration 0x%04x), configuration written again (%u times)", this->address_, config, this->resetLog_.failures());
                }
            }
            else if (this->unexpectedLog_.fail())
            {
                PB_ERROR("INA219 0x%02x: unexpected configuration 0x%04x (bus fault?), written again (%u times)", this->address_, config, this->unexpectedLog_.failures());
            }
            return OK;
        }

        // The chip state is unknown (a bus fault may read 0 A): no measurement
        // until begin() configures it again
        this->configured_ = false;
        if (this->setupError_.fail())
        {
            PB_ERROR("INA219 0x%02x: configuration 0x%04x found and could not be written again, no current measurement (retried, %u failures)", this->address_, config, this->setupError_.failures());
        }
        return res;
    }

    INA219::Result INA219::readShunt(int16_t& raw)
    {
        uint16_t reg;
        Result res = this->readRegister_(INA219_REG_SHUNT_VOLTAGE, reg);
        if (res == OK)
        {
            // Two's complement, 10 uV per LSB, not shifted; clamps at +-32000 (/8)
            raw = (int16_t)reg;
        }
        return res;
    }

    INA219::Result INA219::readBus_mV(uint16_t& mV)
    {
        uint16_t reg;
        Result res = this->readRegister_(INA219_REG_BUS_VOLTAGE, reg);
        if (res == OK)
        {
            // Bits 15..3, 4 mV per LSB
            mV = (uint16_t)((reg >> 3) * 4);
        }
        return res;
    }

    INA219::Result INA219::writeRegister_(uint8_t reg, uint16_t val)
    {
        if (this->i2c_fd_ < 0)
        {
            return ERROR;
        }

        uint8_t buffer[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xff) };
        struct i2c_msg msg = { this->address_, 0, 3, buffer };
        struct i2c_rdwr_ioctl_data data = { &msg, 1 };

        for (int attempt = 0; attempt < INA219_ATTEMPTS; ++attempt)
        {
            int ret = ioctl(this->i2c_fd_, I2C_RDWR, &data);
            if (ret == 1)
            {
                if (this->ioError_.succeed())
                {
                    PB_INFO("INA219 0x%02x: I2C access works again", this->address_);
                }
                return OK;
            }

            if (ret < 0 && errno == ETIMEDOUT)
            {
                if (this->ioError_.fail())
                {
                    PB_ERROR("INA219 0x%02x: I2C timeout writing register %u (%u failures)", this->address_, reg, this->ioError_.failures());
                }
                return TIMEOUT;
            }

            if (attempt + 1 < INA219_ATTEMPTS)
            {
                usleep(INA219_RETRY_DELAY_US);
            }
        }

        if (this->ioError_.fail())
        {
            PB_ERROR("INA219 0x%02x: failed to write register %u (%u failures)", this->address_, reg, this->ioError_.failures());
        }
        return ERROR;
    }

    INA219::Result INA219::readRegister_(uint8_t reg, uint16_t& val)
    {
        if (this->i2c_fd_ < 0)
        {
            return ERROR;
        }

        // Register pointer and read in one transfer (repeated start)
        uint8_t data[2] = { 0, 0 };
        struct i2c_msg msgs[2] = {
            { this->address_, 0, 1, &reg },
            { this->address_, I2C_M_RD, 2, data }
        };
        struct i2c_rdwr_ioctl_data xfer = { msgs, 2 };

        for (int attempt = 0; attempt < INA219_ATTEMPTS; ++attempt)
        {
            int ret = ioctl(this->i2c_fd_, I2C_RDWR, &xfer);
            if (ret == 2)
            {
                if (this->ioError_.succeed())
                {
                    PB_INFO("INA219 0x%02x: I2C access works again", this->address_);
                }
                val = (uint16_t)((data[0] << 8) | data[1]);
                return OK;
            }

            if (ret < 0 && errno == ETIMEDOUT)
            {
                if (this->ioError_.fail())
                {
                    PB_ERROR("INA219 0x%02x: I2C timeout reading register %u (%u failures)", this->address_, reg, this->ioError_.failures());
                }
                return TIMEOUT;
            }

            if (attempt + 1 < INA219_ATTEMPTS)
            {
                usleep(INA219_RETRY_DELAY_US);
            }
        }

        if (this->ioError_.fail())
        {
            PB_ERROR("INA219 0x%02x: failed to read register %u (%u failures)", this->address_, reg, this->ioError_.failures());
        }
        return ERROR;
    }
} /* namespace PowerBox */
