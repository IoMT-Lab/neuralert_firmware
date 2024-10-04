/**
 ****************************************************************************************
 *
 * @file user_def.h
 *
 * @brief Define for User system definition variables for SDK
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

#ifndef __USER_DEF_H__
#define __USER_DEF_H__

#include "da16x_types.h"
#include "sdk_type.h"

#define INTERRUPT_LOW              0
#define INTERRUPT_HIGH             1

#define BUTTON1_GPIO               3                /* GPIO 3 */
#define BUTTON2_GPIO               6                /* GPIO 6 */

#define FACTORY_BUTTON             BUTTON1_GPIO
#define WPS_BUTTON                 BUTTON2_GPIO

#define CHECK_TIME_FACTORY_BTN     2                /* 2 sec. */
#define CHECK_TIME_WPS_BTN         1                /* 1 sec. */

#if defined (__ATCMD_IF_SPI__)
  #error "Config error: If __ATCMD_IF_SPI__ is defined, assign unused GPIO for LEDs"
#endif // __ATCMD_IF_SPI__

  #define BUTTON1_LED              12                /* GPIO 2 */
  #define BUTTON2_LED              5                 /* GPIO 3 */

  #define FACTORY_LED              BUTTON1_LED
  #define WPS_LED                  BUTTON2_LED

  #define BLINK_FAST_TIME          10               /* 100ms */
  #define BLINK_MID_TIME           30               /* 300ms */
  #define BLINK_SLOW_TIME          50               /* 500ms */

  #define LED_OFF                  0
  #define LED_ON                   1
  #define LED_BLINK                2

#endif /* __USER_DEF_H__ */

/* EOF */
