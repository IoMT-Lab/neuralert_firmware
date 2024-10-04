/**
 ****************************************************************************************
 *
 * @file user_gpio_handle.c
 *
 * @brief Config GPIO interrupt
 *
 *
 * Modified from Renesas Electronics SDK example code with the same name
 *
 * Copyright (c) 2024, Vanderbilt University
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************
 */


#include "sdk_type.h"

#include "system_start.h"
#include "da16x_system.h"
#include "gpio.h"


int set_gpio_interrupt(void)
{
    UINT32 ioctldata[3] = { 0, };
    HANDLE hgpio = NULL;

#ifdef __SUPPORT_WPS_BTN__
    UINT16 io = (UINT16)((1 << wps_btn) | (1 << factory_rst_btn));
#else
    UINT16 io = (1 << factory_rst_btn);
#endif /* __SUPPORT_WPS_BTN__ */

    hgpio = GPIO_CREATE(GPIO_UNIT_A);

    if (hgpio == NULL) {
        PRINTF("[%s] GPIO handle error !!!\n", __func__);
        return FALSE;
    }

    GPIO_INIT(hgpio);
    GPIO_IOCTL(hgpio, GPIO_SET_INPUT, &io);

    GPIO_IOCTL(hgpio, GPIO_GET_INTR_MODE, &ioctldata[0]);

    ioctldata[0] |= (UINT32)io;         // 1 edge, 0 level
    ioctldata[1] &= ~(UINT32)io;        // 1 high, 0 low
    GPIO_IOCTL(hgpio, GPIO_SET_INTR_MODE, &ioctldata[0]);

#ifdef __SUPPORT_WPS_BTN__
    ioctldata[0] = (UINT32)(0x01 << wps_btn);
    ioctldata[1] = (UINT32)gpio_callback;
    ioctldata[2] = (UINT32)wps_btn;
    GPIO_IOCTL(hgpio, GPIO_SET_CALLACK, ioctldata);
#endif /* __SUPPORT_WPS_BTN__ */

    ioctldata[0] = (UINT32)(0x01 << factory_rst_btn);
    ioctldata[1] = (UINT32)gpio_callback;
    ioctldata[2] = (UINT32)factory_rst_btn;
    GPIO_IOCTL(hgpio, GPIO_SET_CALLACK, ioctldata);

    GPIO_IOCTL(hgpio, GPIO_SET_INTR_ENABLE, &io);

    return TRUE;
}

/* EOF */
