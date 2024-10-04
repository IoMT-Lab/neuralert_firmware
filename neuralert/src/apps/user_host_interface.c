/**
 ****************************************************************************************
 *
 * @file user_host_interface.c
 *
 * @brief User host interfaces
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

#include "command_host.h"
#include "sdk_type.h"
#include "da16x_system.h"
#include "da16x_types.h"
#include "common_uart.h"
#include "target.h"

/**************************************************************************
 * External Variables
 **************************************************************************/

extern unsigned int    hostif;

/**************************************************************************
 * Internal Functions
 **************************************************************************/

void user_host_interface_init(void);

/**************************************************************************
 * External Functions
 **************************************************************************/

extern void host_spi_slave_init(void);
extern void host_i2c_slave_init(void);
extern void host_callback_handler(UINT32 address);


void user_host_interface_init(void)
{
    if (hostif == HOSTIF_SPI) {
        host_spi_slave_init();
    } else if (hostif == HOSTIF_UART) {

    } else if (hostif == HOSTIF_I2C) {
        host_i2c_slave_init();
    } else if (hostif == HOSTIF_SDIO) {

        SDIO_SLAVE_INIT();

        PRINTF(">>> Initialize SDIO Slave ...\n");

        SDIO_SLAVE_CALLBACK_REGISTER(host_callback_handler);
        DA16X_DIOCFG->EXT_INTB_CTRL =  0xa;

        // drive strength
    }
}

/* EOF */
