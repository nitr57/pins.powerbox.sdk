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

#ifndef SPI_H
#define SPI_H

#include "ErrorThrottle.h"
#include "PowerBoxLogging.h"
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

namespace PowerBox
{
    class SPIBus {
        private:
            int fd;
            uint8_t mode = 0;
            uint8_t bits = 8;
            uint32_t speed = 1000000; // 1MHz
            bool connected_ = false;
            // Rate limits the log of failing transfers (each sample is one)
            ErrorThrottle transfer_error_;

        public:
            SPIBus(const char* device) {
                fd = open(device, O_RDWR);
                if (fd < 0) {
                    PB_ERROR("SPI: failed to open %s (dtparam=spi=on in config.txt?)", device);
                    fd = -1;
                    connected_ = false;
                } else {
                    connected_ = true;
                }
            }

            ~SPIBus() { 
                if (fd >= 0) {
                    close(fd);
                }
            }

            bool isConnected() const {
                return connected_;
            }

            void begin(uint32_t speed, uint8_t bits, uint8_t mode)
            {
                if (!connected_) {
                    PB_ERROR("SPI: bus not open, not configured");
                    return;
                }

                this->speed = speed;
                this->bits = bits;
                this->mode = mode;

                // Set SPI mode, bits per word, and max speed
                if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
                    PB_ERROR("SPI: failed to set the mode");
                }
                if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
                    PB_ERROR("SPI: failed to set the bits per word");
                }
                if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
                    PB_ERROR("SPI: failed to set the speed");
                }
            }

            void transfer(const std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
                if (!connected_) {
                    if (transfer_error_.fail()) {
                        PB_ERROR("SPI: transfer on a bus that is not open, reads 0 (%u times)", transfer_error_.failures());
                    }
                    rx.resize(tx.size(), 0);
                    return;
                }

                if (rx.size() < tx.size()) {
                    rx.resize(tx.size());
                }

                struct spi_ioc_transfer tr = {};
                tr.tx_buf = (unsigned long)tx.data();
                tr.rx_buf = (unsigned long)rx.data();
                tr.len = tx.size();
                tr.delay_usecs = 0;
                tr.speed_hz = speed;
                tr.bits_per_word = bits;

                if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
                    if (transfer_error_.fail()) {
                        PB_ERROR("SPI: transfer failed (%u times)", transfer_error_.failures());
                    }
                }
                else if (transfer_error_.succeed()) {
                    PB_INFO("SPI: transfers work again");
                }
            }
    };
} /* namespace PowerBox */

#endif /* SPI_H */