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

#include "TCA953X.h"
#include "common.h"
#include "PowerBoxLogging.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* Transient I2C errors happen now and then, retry before giving up */
static const int I2C_ATTEMPTS = 3;
static const useconds_t I2C_RETRY_DELAY_US = 500;

template <uint8_t PINS>
TCA953X_<PINS>::TCA953X_(const char* dev, uint8_t address)
{
    this->init_ = false;
    this->dev_ = dev;
    this->address_ = address;
    this->i2c_fd_ = -1;
    this->lastInput_[0] = 0;
    this->lastInput_[1] = 0;
}

template <uint8_t PINS>
TCA953X_<PINS>::~TCA953X_(void)
{
    if (this->i2c_fd_ >= 0) {
        close(this->i2c_fd_);
        this->i2c_fd_ = -1;
    }
    this->init_ = false;
}

template <uint8_t PINS>
uint8_t TCA953X_<PINS>::begin(void)
{
    if(this->init_)
    {
        return true;
    }

    // Open I2C device
    this->i2c_fd_ = open(this->dev_, O_RDWR);
    if (this->i2c_fd_ < 0)
    {
        PB_DEBUG("TCA953X failed to open %s", this->dev_);
        return false;
    }

    PB_DEBUG("TCA953X opened %s, fd=%d", this->dev_, this->i2c_fd_);

    // Set I2C slave address
    if (ioctl(this->i2c_fd_, I2C_SLAVE, this->address_) < 0) {
        PB_DEBUG("TCA953X failed to set I2C slave address 0x%02x", this->address_);
        ::close(this->i2c_fd_);
        this->i2c_fd_ = -1;
        return false;
    }

    PB_DEBUG("TCA953X configured to address 0x%02x", this->address_);
    this->init_ = true;
    return true;
}

template <uint8_t PINS>
uint8_t TCA953X_<PINS>::pinMode(uint8_t pin, uint8_t mode)
{
    if(pin >= PINS)
    {
        return false;
    }

    if((mode != INPUT) && (mode != OUTPUT))
    {
        return false;
    }

    uint8_t config = PINS > 8 ? (pin < 8 ? 0x06 : 0x07) : 0x03;
    uint8_t mask   = 1 << (pin < 8 ? pin : pin - 8);
    uint8_t val;

    // Never write back after a failed read: val would be 0 and turn every
    // other pin of this register into an output
    if(!this->readRegister_(config, val))
    {
        return false;
    }

    if(mode == INPUT)
    {
        val |= mask;
    }

    if(mode == OUTPUT)
    {
        val &= ~mask;
    }

    return this->writeRegister_(config, val);
}

template <uint8_t PINS>
uint8_t TCA953X_<PINS>::digitalWrite(uint8_t pin, uint8_t data)
{
    if(pin >= PINS)
    {
        return false;
    }

    uint8_t config = PINS > 8 ? (pin < 8 ? 0x02 : 0x03) : 0x01;
    uint8_t mask   = 1 << (pin < 8 ? pin : pin - 8);
    uint8_t val;

    // Never write back after a failed read: val would be 0 and switch off
    // every other output of this register
    if(!this->readRegister_(config, val))
    {
        return false;
    }

    if(data)
    {
        val |= mask;
    }
    else
    {
        val &= ~mask;
    }

    return this->writeRegister_(config, val);
}

template <uint8_t PINS>
uint8_t TCA953X_<PINS>::digitalRead(uint8_t pin)
{
    if(pin >= PINS)
    {
        return false;
    }

    uint8_t config = PINS > 8 ? (pin < 8 ? 0x00 : 0x01) : 0x00;
    uint8_t mask   = 1 << (pin < 8 ? pin : pin - 8);
    uint8_t val;

    // On a failed read report the last known state instead of "all low"
    if(this->readRegister_(config, val))
    {
        this->lastInput_[config] = val;
    }
    else
    {
        val = this->lastInput_[config];
    }

    if(val & mask)
    {
        return 1;
    }

    return 0;
}

template <uint8_t PINS>
bool TCA953X_<PINS>::writeRegister_(uint8_t reg, uint8_t val)
{
    if (this->i2c_fd_ < 0) {
        PB_DEBUG("TCA953X writeRegister: i2c_fd_ is invalid (%d)", this->i2c_fd_);
        return false;
    }

    uint8_t buffer[2] = { reg, val };

    for (int attempt = 0; attempt < I2C_ATTEMPTS; ++attempt) {
        if (write(this->i2c_fd_, buffer, 2) == 2) {
            if (this->ioError_.succeed()) {
                PB_INFO("TCA953X: I2C access works again");
            }
            return true;
        }
        if (attempt + 1 < I2C_ATTEMPTS) {
            usleep(I2C_RETRY_DELAY_US);
        }
    }

    if (this->ioError_.fail()) {
        PB_ERROR("TCA953X writeRegister: failed to write register 0x%02x (%u failures)", reg, this->ioError_.failures());
    }
    return false;
}

template <uint8_t PINS>
bool TCA953X_<PINS>::readRegister_(uint8_t reg, uint8_t &data)
{
    if (this->i2c_fd_ < 0) {
        PB_DEBUG("TCA953X readRegister: i2c_fd_ is invalid (%d)", this->i2c_fd_);
        return false;
    }

    for (int attempt = 0; attempt < I2C_ATTEMPTS; ++attempt) {
        // Write register address, then read 1 byte
        if (write(this->i2c_fd_, &reg, 1) == 1 && read(this->i2c_fd_, &data, 1) == 1) {
            if (this->ioError_.succeed()) {
                PB_INFO("TCA953X: I2C access works again");
            }
            return true;
        }
        if (attempt + 1 < I2C_ATTEMPTS) {
            usleep(I2C_RETRY_DELAY_US);
        }
    }

    if (this->ioError_.fail()) {
        PB_ERROR("TCA953X readRegister: failed to read register 0x%02x (%u failures)", reg, this->ioError_.failures());
    }
    return false;
}

template class TCA953X_<8>;
template class TCA953X_<16>;
