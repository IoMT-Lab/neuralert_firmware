/**
 ****************************************************************************************
 *
 * @file user_main.c
 *
 * @brief MAIN starting entry point
 *
 * Copyright (c) 2016-2020 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */

#include "sdk_type.h"

#include "driver.h"
#include "da16x_system.h"
#include "common_def.h"
#include "sys_specific.h"
#include "adc.h"
#include "common.h"

#ifdef __SUPPORT_NAT__
#include "common_config.h"
#include "common_def.h"
#endif /* __SUPPORT_NAT__ */



/*
 * extern symbols : Not for customer
 */
//extern void		version_display(int argc, char *argv[]);
//extern int		get_boot_mode(void);
//extern int		get_run_mode(void);
extern void		system_start(void);
extern INT32	__GPIO_RETAIN_HIGH_RECOVERY();

extern void user_time64_msec_since_poweron(__time64_t *cur_msec);

#if defined ( __REMOVE_32KHZ_CRYSTAL__ )
extern int		ALT_RTC_ENABLE(void);
#endif	// __REMOVE_32KHZ_CRYSTAL__
#if defined ( __SUPPORT_NAT__ )
extern void		set_nat_flag_enable(void);
extern int		chk_dpm_wakeup(void);
#endif // __SUPPORT_NAT__

//extern long long user_rtc_wakeup_time;    		// RTC clock when wakeup happened
//extern __time64_t user_rtc_wakeup_time_msec;  	// da16xx time when wakeup happened
extern __time64_t user_raw_rtc_wakeup_time_msec;  // relative msec time when wakeup happened
//extern int user_raw_rtc_wakeup_time_source;			// where this time came from
/*
 *********************************************************************************
 * @brief	Configure system Pin-Mux
 * @return	None
 *********************************************************************************
 */

void run_i2c()
{
	UINT32 status;

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

int config_pin_mux(void)
{

    int status;
	unsigned int type;

#if defined ( __BLE_COMBO_REF__ )

#if defined (__ATCMD_IF_UART1__) || defined (__SUPPORT_UART1__)
	_da16x_io_pinmux(PIN_AMUX, AMUX_UART1d);		// TXD: GPIO[0], RXD: GPIO[1]
#endif
	_da16x_io_pinmux(PIN_CMUX, CMUX_GPIO);		// RTS: GPIO[4], CTS: GPIO[5]
#if defined(__ATCMD_IF_SPI__)
	_da16x_io_pinmux(PIN_BMUX, BMUX_SPIs);		/* sSPI_CLK,sSPI_CSB */
	_da16x_io_pinmux(PIN_EMUX, EMUX_SPIs);		/* sSPI_MOSI,sSPI_MISO */

	_da16x_io_pinmux(PIN_DMUX, DMUX_GPIO);		// For GPIO 6,7
#else
	_da16x_io_pinmux(PIN_BMUX, BMUX_GPIO);

	_da16x_io_pinmux(PIN_DMUX, DMUX_GPIO);		// For GPIO 6,7
	_da16x_io_pinmux(PIN_EMUX, EMUX_GPIO);
#endif /* __ATCMD_IF_SPI__ */
	_da16x_io_pinmux(PIN_FMUX, FMUX_GPIO);
	_da16x_io_pinmux(PIN_HMUX, HMUX_JTAG);
#if defined (__ATCMD_IF_UART2__) || defined (__SUPPORT_UART2__)
	#if defined(__ATCMD_IF_SPI__)
	#error "Please, check a features for AT command interface..."
	#endif
	_da16x_io_pinmux(PIN_UMUX, UMUX_UART2GPIO);   // UART2 for AT commands or user uart2
#endif
#ifdef __CFG_ENABLE_BLE_HW_RESET__
	_da16x_io_pinmux(PIN_UMUX, UMUX_GPIO);		// For GPIO 8
#endif

	DA16X_DIOCFG->GPIOA_PE_PS |= (0x10);//Pull Enable jason200908
	DA16X_DIOCFG->GPIOA_PE_PS |= (0x10 << 16); //Pull Up Enable

#if !defined(__ATCMD_IF_SPI__)
	/* Need to configure by customer */
	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_A, GPIO_PIN6 | GPIO_PIN7);
#endif

#if defined(__ATCMD_IF_SPI__) || defined (__ATCMD_IF_SDIO__)
	// Set GPIOC6 as Interrupt pin
	static HANDLE gpio;
	gpio = GPIO_CREATE(GPIO_UNIT_C);
	GPIO_INIT(gpio);

	PRINTF("[WS]>>> Initialize interrupt: %x\n", GPIO_ALT_FUNC_GPIO6);
	GPIO_SET_ALT_FUNC(gpio, GPIO_ALT_FUNC_EXT_INTR,
						(GPIO_ALT_GPIO_NUM_TYPE)(GPIO_ALT_FUNC_GPIO6));
	GPIO_CLOSE(gpio);
#endif
#else	//( __BLE_COMBO_REF__ )
	/* DA16200 default pin-configuration */

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


	/* PIN remapping for UART, SPI and SDIO */	
	#if defined (__ATCMD_IF_UART1__) || defined (__SUPPORT_UART1__)
		_da16x_io_pinmux(PIN_CMUX, CMUX_UART1d);
	#elif defined(__ATCMD_IF_SPI__)
		_da16x_io_pinmux(PIN_BMUX, BMUX_SPIs);
		_da16x_io_pinmux(PIN_EMUX, EMUX_SPIs);
	#elif defined(__ATCMD_IF_SDIO__)
		_da16x_io_pinmux(PIN_CMUX, CMUX_SDs);
		_da16x_io_pinmux(PIN_DMUX, DMUX_SDs);
		_da16x_io_pinmux(PIN_EMUX, EMUX_SDs);
	#endif /* PIN remapping for UART, SPI and SDIO */

#if defined (__ATCMD_IF_UART2__) || defined (__SUPPORT_UART2__)
	_da16x_io_pinmux(PIN_UMUX, UMUX_UART2GPIO);   // UART2 for AT commands or user uart2
#endif
#if defined(__ATCMD_IF_SPI__)|| defined (__ATCMD_IF_SDIO__)
	// Set GPIOC6 as Interrupt pin
	static HANDLE gpio;
	gpio = GPIO_CREATE(GPIO_UNIT_C);
	GPIO_INIT(gpio);

	PRINTF("[WS]>>> Initialize interrupt: %x\n", GPIO_ALT_FUNC_GPIO6);
	GPIO_SET_ALT_FUNC(gpio, GPIO_ALT_FUNC_EXT_INTR,
						(GPIO_ALT_GPIO_NUM_TYPE)(GPIO_ALT_FUNC_GPIO6));
	GPIO_CLOSE(gpio);
#endif

	/* Need to configure by customer */   //NJ - location for DA16200 system initialization of SPI and I2C lines
	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_A, GPIO_PIN1 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 );
//	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_A, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN10);
//	SAVE_PULLUP_PINS_INFO(GPIO_UNIT_C, GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN7 );

	run_i2c();

#endif /* __BLE_COMBO_REF__ */

	return TRUE;
}


/**
 ******************************************************************************
 * @brief		System entry point
 * @input[in]	init_state	initialize result of pTIME and RamLib
 * @return		None
 ******************************************************************************
 */
int user_main(char init_state)
{
	int	status = 0;
	__time64_t my_wakeup_msec;
	unsigned long long time_old;
//	__time64_t my_uptime;

	/*
	 * 1. Restore saved GPIO PINs
	 * 2. RTC PAD connection
	 */
	__GPIO_RETAIN_HIGH_RECOVERY();

//	PRINTF("\n**Neuralert: In user_main()\n"); // FRSDEBUG


#if defined ( __REMOVE_32KHZ_CRYSTAL__ )
	/* Initialize Alternative RTC counter */
	ALT_RTC_ENABLE();
#endif	// __REMOVE_32KHZ_CRYSTAL__

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
