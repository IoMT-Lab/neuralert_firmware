/**
 ****************************************************************************************
 *
 * @file user_main.c
 *
 * @brief MAIN starting entry point
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

#include "driver.h"
#include "da16x_system.h"
#include "common_def.h"
#include "sys_specific.h"
#include "adc.h" //JW: This was added by Fred -- may not be needed.
#include "common.h" //JW: This was added to support retmmem
#include "user_main.h"

#ifdef __SUPPORT_NAT__
#include "common_config.h"
#include "common_def.h"
#endif /* __SUPPORT_NAT__ */


extern void user_time64_msec_since_poweron(__time64_t *cur_msec); //JW: ADDED FOR NEURALERT


//JW: This function added for Neuralert
void run_i2c()
{
	//UINT32 status;

	// Device Address for AT24C512
	UINT32 addr = 0xa0;

	// I2C Working Clock [KHz]
	UINT32 i2c_clock = 400;
//	UINT32 i2c_clock = 100;

	//APRINTF_Y("I2C Init ...\r\n");

	DA16X_CLOCK_SCGATE->Off_DAPB_I2CM = 0;
	DA16X_CLOCK_SCGATE->Off_DAPB_APBS = 0;

	// Create Handle for I2C Device
	I2C = DRV_I2C_CREATE(i2c_0);

	// Initialization I2C Device
	DRV_I2C_INIT(I2C);

	// Set Address for Atmel eeprom AT24C512
	DRV_I2C_IOCTL(I2C, I2C_SET_CHIPADDR, &addr);

	// Set I2C Working Clock. Unit = KHz
	DRV_I2C_IOCTL(I2C, I2C_SET_CLOCK, &i2c_clock);

}


/*
 *********************************************************************************
 * @brief    Configure system Pin-Mux
 * @return   None
 *********************************************************************************
 */
int config_pin_mux(void)
{

    int status;
	unsigned int type;

	/* Neuralert pin-configuration */   //NJ

//	_da16x_io_pinmux(PIN_AMUX, AMUX_GPIO);
	_da16x_io_pinmux(PIN_BMUX, BMUX_GPIO);

	/* pinmux config for I2C  - GPIOA[5:4] */
	_da16x_io_pinmux(PIN_CMUX, CMUX_I2Cm);
	/* pinmux config for SPI Host  - GPIOA[9:6] */
	_da16x_io_pinmux(PIN_DMUX, DMUX_SPIm);
	_da16x_io_pinmux(PIN_EMUX, EMUX_SPIm);
	_da16x_io_pinmux(PIN_FMUX, FMUX_GPIO);
	_da16x_io_pinmux(PIN_HMUX, HMUX_JTAG);
	/* pinmux config for LED GPIO  - GPIOC[8:6] */
	_da16x_io_pinmux(PIN_UMUX, UMUX_GPIO);


	// LEDs are on GPIOC 6, 7, & 8 (R, G, B)
	uint32_t pin;
	uint16_t write_data_red;
	uint16_t write_data_green;
	uint16_t write_data_blue;

	gpioc = GPIO_CREATE(GPIO_UNIT_C);
	GPIO_INIT(gpioc);

	pin = GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8;
	GPIO_IOCTL(gpioc, GPIO_SET_OUTPUT, &pin);

	write_data_red = GPIO_PIN6;
	write_data_green = GPIO_PIN7;
	write_data_blue = GPIO_PIN8;

	// Turn them off
	GPIO_WRITE(gpioc, GPIO_PIN6, &write_data_red, sizeof(uint16_t));
	GPIO_WRITE(gpioc, GPIO_PIN7, &write_data_green, sizeof(uint16_t));
	GPIO_WRITE(gpioc, GPIO_PIN8, &write_data_blue, sizeof(uint16_t));

	/*
	 * Battery voltage measurement is on GPIOA
	 * A0 is the ADC input
	 * A10 is the measurement circuit enable
	 * GPIOA port init
	 */
	gpioa = GPIO_CREATE(GPIO_UNIT_A);
	GPIO_INIT(gpioa);
	/*
	 * GPIOA[10] output high
	 */
	pin = GPIO_PIN10;
	GPIO_IOCTL(gpioa, GPIO_SET_OUTPUT, &pin);

	//  Set GPIO_0 (ADC_CH0), GPIO_1(ADC_CH1)
	_da16x_io_pinmux(PIN_AMUX, AMUX_AD12);

	// Create Handle
	hadc = DRV_ADC_CREATE(DA16200_ADC_DEVICE_ID);

	// Initialization
	DRV_ADC_INIT(hadc, FC9050_ADC_NO_TIMESTAMP);

    // Start. Set Sampling Frequency. 12B ADC Set to 200KHz
    // Clock = 1MHZ / (value + 1)
    // Ex) If Value = 4, Clock = 1MHz / (4+1) = 200KHz
    status = DRV_ADC_START(hadc, FC9050_ADC_DIVIDER_12, 0);
    PRINTF("config_pin_mux: ADC start: %d\n",status);

    // Set ADC_0 to 12Bit ADC
    status = DRV_ADC_ENABLE_CHANNEL(hadc, DA16200_ADC_CH_0, FC9050_ADC_SEL_ADC_12, 0);
    PRINTF("config_pin_mux: ADC enable: %d\n",status);

    //Set Data type offset binary, 0 : 2's complement ,  1 : offset binary
    type = 0;
    DRV_ADC_IOCTL(hadc, ADC_SET_DATA_MODE,&type);


	/* Need to configure by customer */   //NJ - location for DA16200 system initialization of SPI and I2C lines
	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_A, GPIO_PIN1 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 );
//	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_A, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN10);
//	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_C, GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN7 );

	run_i2c();
    return TRUE;
}


/**
 ******************************************************************************
 * @brief      System entry point
 * @input[in]  init_state - initalize result of pTIME and RamLib
 * @return     None
 ******************************************************************************
 */
int user_main(char init_state)
{
	// Brief delay needed to prevent garbage UART output while
	// the pins are being configured
	vTaskDelay(10);

    int	status = 0;
	__time64_t my_wakeup_msec;
	unsigned long long time_old;

    /*
     * 1. Restore saved GPIO PINs
     * 2. RTC PAD connection
     */
    __GPIO_RETAIN_HIGH_RECOVERY();

#if defined ( __REMOVE_32KHZ_CRYSTAL__ )
    /* Initialize Alternative RTC counter */
    ALT_RTC_ENABLE();
#endif // __REMOVE_32KHZ_CRYSTAL__

    time_old = RTC_GET_COUNTER();

	// Save time of interrupt for use as timestamp in user application
//	da16x_time64_msec(NULL, &my_wakeup_msec);
//	Printf("\nuser_main: wakeup msec: %u\n\n", my_wakeup_msec);
//	user_rtc_wakeup_time = time_old;	// publish interrupt time to user
//	user_rtc_wakeup_time_msec = my_wakeup_msec;

	// Get relative time since power on from the RTC time counter register
	user_time64_msec_since_poweron(&user_raw_rtc_wakeup_time_msec);

    /* Entry point for customer main */
    if (init_state == pdTRUE) {
        system_start();
    } else {
        PRINTF("\nFailed to initialize the RamLib or pTIM !!!\n");
    }

    return status;
}

/* EOF */
