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
#include <cstring>
#include "Device.h"

namespace PowerBox
{
    PB_ERROR_TYPE Device::GetPowerPortStatus(PB_POWER_PORT_STATUS *status) const
    {
        // Ports beyond numPorts are not filled below, do not hand out garbage for them
        memset(status, 0, sizeof(*status));

        status->numPorts = this->GetNumPowerPorts();

        for (int i = 0; i < this->GetNumPowerPorts(); ++i)
        {
            status->current[i] = this->powerCurrent[i];
            status->overcurrent[i] = this->powerOvercurrent[i];
            status->readOnly[i] = this->powerReadOnly[i];
            status->state[i] = this->powerState[i];
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE Device::GetUSBPortStatus(PB_USB_PORT_STATUS *status) const
    {
        // Ports beyond numPorts are not filled below, do not hand out garbage for them
        memset(status, 0, sizeof(*status));

        status->numPorts = this->GetNumUSBPorts();

        for (int i = 0; i < this->GetNumUSBPorts(); ++i)
        {
            status->current[i] = this->usbCurrent[i];
            status->voltage[i] = this->usbVoltage[i];
            status->overcurrent[i] = this->usbOvercurrent[i];
            status->readOnly[i] = this->usbReadOnly[i];
            status->state[i] = this->usbState[i];
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE Device::GetDewPortStatus(PB_DEW_PORT_STATUS *status) const
    {
        // Ports beyond numPorts are not filled below, do not hand out garbage for them
        memset(status, 0, sizeof(*status));

        status->numPorts = this->GetNumDewPorts();
        status->pwmResolution = this->dewPwmResolution;

        for (int i = 0; i < this->GetNumDewPorts(); ++i)
        {
            status->current[i] = this->dewCurrent[i];
            status->overcurrent[i] = this->dewOvercurrent[i];
            status->probe[i] = this->dewProbe[i];
            status->pwm[i] = this->dewPWM[i];
            status->state[i] = this->dewState[i];
        }

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE Device::GetBuckPortStatus(PB_BUCK_PORT_STATUS *status) const
    {
        // Ports beyond numPorts are not filled below, do not hand out garbage for them
        memset(status, 0, sizeof(*status));

        status->numPorts = this->GetNumBuckPorts();
        status->current = this->buckCurrent;
        status->voltage = this->buckVoltage;
        status->overcurrent = this->buckOvercurrent;
        status->vset = this->buckVset;
        status->vmin = this->buckVmin;
        status->vmax = this->buckVmax;
        status->state = this->buckState;

        return PB_SUCCESS;
    }

    PB_ERROR_TYPE Device::GetPWMPortStatus(PB_PWM_PORT_STATUS *status) const
    {
        // Ports beyond numPorts are not filled below, do not hand out garbage for them
        memset(status, 0, sizeof(*status));

        status->numPorts = this->GetNumPWMPorts();
        status->pwmResolution = this->pwmPwmResolution;
        status->current = this->pwmCurrent;
        status->overcurrent = this->pwmOvercurrent;
        status->pwm = this->pwmPWM;
        status->state = this->pwmState;

        return PB_SUCCESS;
    }

    std::map<int, std::shared_ptr<Device>> g_devices;
    std::mutex g_globalMutex;

} /* namespace PowerBox */
