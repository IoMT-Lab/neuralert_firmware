/**
 ****************************************************************************************
 *
 * @file system_start.c
 *
 * @brief DA16200 Generic feature - User application entry point
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

#include "user_main.h"
#include "common_def.h"
#include "da16x_crypto.h"
#include "net_bsd_sockets.h"
#include "system_start.h"
#include "user_dpm_api.h"
#include "sys_specific.h"
#include "util_api.h"
#include "limits.h"
#include "da16x_sys_watchdog.h"
#include "da16x_sys_watchdog_ids.h"

#ifdef __SET_BOR_CIRCUIT__
#include "ramsymbols.h"
#include "rtc.h"
#endif // __SET_BOR_CIRCUIT__

#if defined (CFG_USE_RETMEM_WITHOUT_DPM)
#include "user_retmem.h"
#endif

/*
 * Config customer's console baud-rate.
 */
void config_customer_console_baudrate(void)
{
#define BAUDRATE_1    9600
#define BAUDRATE_2    19200
#define BAUDRATE_3    38400
#define BAUDRATE_4    57600
#define BAUDRATE_5    115200
#define BAUDRATE_6    230400
#define BAUDRATE_7    380400
#define BAUDRATE_8    460800
#define BAUDRATE_9    921600

    /* 9600 bps ~ 921600 bps */
    customer_console_baudrate = UART0_BAUDRATE;
}

#if defined ( __SET_WAKEUP_HW_RESOURCE__)


extern UCHAR	runtime_cal_flag;		// system flag that enables runtime timing analysis

extern int user_factory_reset_btn_onetouch(void);

/*****************************************************************************/
/*  The extenal_wakeup_pin can be used for interrupt pin.                    */
/*  DA16200 6*6 have two external_wakeup_pin And                             */
/*  DA16200 8*8 have four external_wakeup_pin.                               */
/*****************************************************************************/
static void rtc_ext_cb(void *data)
{
    unsigned long long time_old;
    UINT32 ioctl = 0;

    DA16X_UNUSED_ARG(data);

    time_old = RTC_GET_COUNTER();

    /* Waits until the RTC register is updated*/
    while (RTC_GET_COUNTER() < (time_old + 10)) {
        ;
    }

    RTC_IOCTL(RTC_GET_GPIO_WAKEUP_CONTROL_REG, &ioctl);

    if (ioctl & RTC_WAKEUP_STATUS) {
        Printf(">>> rtc1 wakeup interrupt ...\r\n");
#if defined ( __SUPPORT_NOTIFY_RTC_WAKEUP__ )
        rtc_wakeup_intr_chk_flag = pdTRUE;
#endif // __SUPPORT_NOTIFY_RTC_WAKEUP__
    }


    if (ioctl & RTC_WAKEUP2_STATUS) {
        Printf(">>> rtc2 wakeup interrupt ...\r\n");
#if defined ( __SUPPORT_NOTIFY_RTC_WAKEUP__ )
        rtc_wakeup_intr_chk_flag = pdTRUE;
#endif // __SUPPORT_NOTIFY_RTC_WAKEUP__
    }

    if (ioctl & RTC_WAKEUP3_STATUS) {
        Printf(">>> rtc3 wakeup interrupt ...\r\n");
    }

    if (ioctl & RTC_WAKEUP4_STATUS) {
        Printf(">>> rtc4 wakeup interrupt ...\r\n");
    }

    /* clear wakeup source */
    time_old = RTC_GET_COUNTER();
    RTC_CLEAR_EXT_SIGNAL();

    while (RTC_GET_COUNTER() < (time_old + 1)) {
        ;
    }

    time_old = RTC_GET_COUNTER();
    ioctl = 0;
    RTC_IOCTL(RTC_SET_WAKEUP_SOURCE_REG, &ioctl);

    while (RTC_GET_COUNTER() < (time_old + 1)) {
        ;
    }
}

static void rtc_ext_intr(void)
{
    //INTR_CNTXT_SAVE(); //JW: not sure why this isn't used. (See peripheral_sample_wakeup project)
    INTR_CNTXT_CALL(rtc_ext_cb);
    //INTR_CNTXT_RESTORE(); //JW: not sure why this isn't used.
}

void config_ext_wakeup_resource(void)
{
    UINT32 intr_src, ioctldata;

    RTC_IOCTL(RTC_GET_RTC_CONTROL_REG, &intr_src);
    intr_src &= ~(WAKEUP_INTERRUPT_ENABLE(1));  // Set Wakeup interrupt enable bit (active high)
	intr_src &= ~(WAKEUP_POLARITY(1));          // Set polarity to rising edge (active high)
	RTC_IOCTL(RTC_SET_RTC_CONTROL_REG, &intr_src);


    _sys_nvic_write(RTC_ExtWkInt_IRQn, (void *)rtc_ext_intr, 0x7);
}
#endif // __SET_WAKEUP_HW_RESOURCE__

#if defined ( __SET_BOR_CIRCUIT__ )
RAMLIB_SYMBOL_TYPE *ramsymbols = (RAMLIB_SYMBOL_TYPE *) DA16X_RAMSYM_BASE;

#define MEM_LONG_WRITE(addr, data)       *((volatile UINT *) addr) = data
#define MEM_LONG_OR_WRITE32(addr, data)  *((volatile UINT *) addr) = *((volatile UINT *) addr) | (UINT)data
#define MEM_LONG_AND_WRITE32(addr, data) *((volatile UINT *) addr) = *((volatile UINT *) addr) & (UINT)data

/* it never return */
ATTRIBUTE_RAM_FUNC void flash_pin_stuck(void)
{
    int cnt = 0;
    HANDLE wdog;

    vPortEnterCritical();

    // Disable WDOG
    wdog = WDOG_CREATE(WDOG_UNIT_0) ;
    WDOG_IOCTL(wdog , WDOG_SET_DISABLE, NULL );

    // XFC off
    MEM_LONG_WRITE(0x500B0000, 0x00);
    // QSPI CS to high
    MEM_LONG_WRITE(0x500B012C, 0x000FFFFF);
    // QSPI clk to low
    MEM_LONG_WRITE(0x500B0128, 0x00000003);
    // QSPI data to low
    MEM_LONG_WRITE(0x500B0124, 0x0000FFFF);

    MEM_LONG_AND_WRITE32(0x5009104c, ~(0xf0));
    MEM_LONG_OR_WRITE32(0x5009104c, 0x70);

    // infinity loop
    while (1) {
        if ( (*((volatile UINT * )0x5009104c) & 0x00004000 ) == 0) {
            cnt++;
        } else {
            cnt = 0;
        }

        if (cnt > 0x200000) {    // POR 0x1000000  ~7s @120Mhz
            MEM_LONG_AND_WRITE32(0x5009104c, ~(0xf0));
            MEM_LONG_OR_WRITE32(0x5009104c, br_ctrl << 4);
            MEM_LONG_WRITE(0x5009101C, 3);

            while(1) {
                ;
            }
        }
    }

    vPortExitCritical();
}

static void rtc_brown_cb(void)
{
    Printf("BR\r\n" );
    // It can make the next bootup as POR
    RTC_CLEAR_RETENTION_FLAG();
    da16x_environ_lock(TRUE);

    // stuck the flash pins
    vTaskSuspendAll();
    flash_pin_stuck();
}

static void config_bor_circuit(void)
{
    unsigned long bor_circuit = 0;

    RTC_IOCTL(RTC_GET_BOR_CIRCUIT, &bor_circuit);

    // BR CTRL
    bor_circuit = (unsigned long)((bor_circuit & 0xffffff0f) | (unsigned long)(br_ctrl << 4));
    RTC_IOCTL(RTC_SET_BOR_CIRCUIT, &bor_circuit);

    RTC_ENABLE_BROWN_BLACK(rtc_brown_cb, NULL);
    RTC_IOCTL(RTC_GET_BOR_CIRCUIT, &bor_circuit);

    if (bor_circuit & 0x00002000) {
        // brown out call brown out cb
        Printf(">>> Brown-out \r\n" );
        rtc_brown_cb();
    }
}
#endif // __SET_BOR_CIRCUIT__

static void config_user_wu_hw_resource(void)
{
#if defined ( __SET_WAKEUP_HW_RESOURCE__ )
    /* Config HW Wakeup resources */
    config_ext_wakeup_resource();
#endif // __SET_WAKEUP_HW_RESOURCE__

#if defined ( __SET_BOR_CIRCUIT__ )
    config_bor_circuit();
#endif  // __SET_BOR_CIRCUIT__
}

/*
 * Format of dpm abnormal wakeup interval
 *
 *  value allowed for wakeup interval
 *                 1 : mininum
 *          0x1FFFFF : maximum, 24 days
 *        0xDEADBEAF : no wakeup
 *       wrong value : < 1 or > 0x1FFFFF, set to 3600 sec (default)
 *
 *  Example:
 *     unsigned long long _user_defined_wakeup_interval[];
 *     {
 *         ULLONG_MAX,            // Initial value : ULLONG_MAX
 *         10, 10, 10, 10, 10,
 *         60,
 *         3600,
 *         3600,
 *         3600 * 24
 *     }
 */
unsigned long long _user_defined_wakeup_interval[DPM_MON_RETRY_CNT] = {
    ULLONG_MAX,            // Initial value : ULLONG_MAX
    60,                    // 1st Wakeup
    60,                    // 2nd Wakeup    : 0xdeadbeaf for no-wakeup
    60,                    // 3rd Wakeup    : 0xdeadbeaf for no-wakeup
    60 * 30,               // 4th Wakeup    : 0xdeadbeaf for no-wakeup
    60 * 30,               // 5th Wakeup    : 0xdeadbeaf for no-wakeup
    60 * 30,               // 6th Wakeup    : 0xdeadbeaf for no-wakeup
    60 * 60,               // 7th Wakeup    : 0xdeadbeaf for no-wakeup
    60 * 60,               // 8th Wakeup    : 0xdeadbeaf for no-wakeup
    0xDEADBEAF             // 9th Wakeup    : 0xdeadbeaf for no-wakeup
};

static void set_dpm_abnorm_user_wakeup_interval(void)
{
#if defined ( __USER_DPM_ABNORM_WU_INTERVAL__ )
	extern unsigned long long *dpm_abnorm_user_wakeup_interval;

    dpm_abnorm_user_wakeup_interval = (unsigned long long *)_user_defined_wakeup_interval;
#endif // __USER_DPM_ABNORM_WU_INTERVAL__
}

/* Customer defined operation if needed ... */
int chk_factory_rst_button_press_status(void)
{
#if defined ( __SUPPORT_FACTORY_RESET_BTN__ )

#if defined ( __SUPPORT_WIFI_CONCURRENT__ )
    // Register call-back function for one-touch press factory-reset button.
    if (button1_one_touch_cb == NULL) {
       button1_one_touch_cb = factory_reset_btn_onetouch;
    }
#endif // __SUPPORT_WIFI_CONCURRENT__

#if defined ( __SUPPORT_EVK_LED__ )
    if (check_factory_button(factory_rst_btn, factory_rst_led, factory_rst_btn_chk_time) == 1)
#else
    if (check_factory_button(factory_rst_btn, factory_rst_btn_chk_time) == 1)
#endif // __SUPPORT_EVK_LED__
    {
        factory_reset_default(1);
        return FALSE;
    }

#endif // __SUPPORT_FACTORY_RESET_BTN__

    return TRUE;
}

void set_customer_softap_config(void)
{
#if defined ( __SUPPORT_FACTORY_RST_APMODE__ ) || defined ( __SUPPORT_FACTORY_RST_CONCURR_MODE__ )
    /* Set to user costomer's configuration */
    ap_config_param->customer_cfg_flag = MODE_DISABLE;    // MODE_ENABLE, MODE_DISABLE


    /*
     * Wi-Fi configuration
     */

    /* SSID prefix */
    sprintf(ap_config_param->ssid_name, "%s", "DA16200");

    /* Default open mode : AP_OPEN_MODE, AP_SECURITY_MODE */
    ap_config_param->auth_type = AP_OPEN_MODE;

    if (ap_config_param->auth_type == AP_SECURITY_MODE) {
        sprintf(ap_config_param->psk, "%s", "N12345678");
    }

    /* Country Code : Default country US */
    sprintf(ap_config_param->country_code, "%s", DFLT_AP_COUNTRY_CODE);

    /*
     * Network IP address configuration
     */
    ap_config_param->customer_ip_address = IPADDR_DEFAULT;

    if (ap_config_param->customer_ip_address == IPADDR_CUSTOMER) {
        sprintf(ap_config_param->ip_addr, "%s", "192.168.1.1");
        sprintf(ap_config_param->subnet_mask, "%s", "255.255.255.0");
        sprintf(ap_config_param->default_gw, "%s", "192.168.1.1");
        sprintf(ap_config_param->dns_ip_addr, "%s", "8.8.8.8");
    }

    /*
     * DHCP Server configuration
     */
    ap_config_param->customer_dhcpd_flag = DHCPD_DEFAULT;

    if (ap_config_param->customer_dhcpd_flag == DHCPD_CUSTOMER) {
        ap_config_param->dhcpd_lease_time = 3600;

        sprintf(ap_config_param->dhcpd_start_ip, "%s", "192.168.1.101");
        sprintf(ap_config_param->dhcpd_end_ip, "%s", "192.168.1.108");
        sprintf(ap_config_param->dhcpd_dns_ip_addr, "%s", "8.8.8.8");
    }
#endif // __SUPPORT_FACTORY_RST_APMODE__ || __SUPPORT_FACTORY_RST_CONCURR_MODE__
}

static void button1_one_touch_cb_fn(void)
{

    /* User application function if needed ... */
#if defined ( __SUPPORT_WIFI_CONCURRENT__ )
    button1_one_touch_cb = factory_reset_btn_onetouch;
#else
    button1_one_touch_cb = NULL;
    //button1_one_touch_cb = user_factory_reset_btn_onetouch;
#endif // __SUPPORT_WIFI_CONCURRENT__
}

static void config_gpio_button(void)
{
#if defined ( __SUPPORT_EVK_LED__ )
    /* Set factory-reset button configuration
     *    : button-gpio / led-gpio / time_seconds */
    config_factory_reset_button(FACTORY_BUTTON, FACTORY_LED, CHECK_TIME_FACTORY_BTN);

    /* Set wps button configuration
     *    : button-gpio / led-gpio / time_seconds */
    config_wps_button(WPS_BUTTON, WPS_LED, CHECK_TIME_WPS_BTN);
#else
    /* Set factory-reset button configuration
     *    : button-gpio / time_seconds */
    config_factory_reset_button(FACTORY_BUTTON, CHECK_TIME_FACTORY_BTN);

    /* Set wps button configuration
     *    : button-gpio / time_seconds */
    config_wps_button(WPS_BUTTON, CHECK_TIME_WPS_BTN);
#endif // __SUPPORT_EVK_LED__

    /* Register call-back function for factory-reset button short-press */
    button1_one_touch_cb_fn();

    /* Set gpio interrupt for WPS and factory-reset button */
    set_gpio_interrupt();

#if defined ( __TRIGGER_DPM_MCU_WAKEUP__ )
    /* Retain GPIO for DPM sleep */
    if (dpm_mode_is_enabled() == TRUE &&  dpm_mode_is_wakeup() == FALSE) {
        _da16x_io_pinmux(PIN_FMUX, FMUX_GPIO);
        _GPIO_RETAIN_LOW(GPIO_UNIT_A, GPIO_PIN11);
    }
#endif  // __TRIGGER_MCU_WAKEUP__


	// Configure the LED GPIO pins to be held during sleep2
	// so LEDs don't flash on while sleeping
	_da16x_io_pinmux(PIN_UMUX, UMUX_GPIO);
    //PRINTF("###GPIO_UNIT_C GPIO 6,7,8 retain-high\n");
    _GPIO_RETAIN_HIGH(GPIO_UNIT_C, GPIO_PIN6);
    _GPIO_RETAIN_HIGH(GPIO_UNIT_C, GPIO_PIN7);
    _GPIO_RETAIN_HIGH(GPIO_UNIT_C, GPIO_PIN8);
}

void trigger_mcu_wakeup_gpio(void)
{
#if defined ( __TRIGGER_DPM_MCU_WAKEUP__ )
    HANDLE gpio;
    UINT32 pin;
    UINT16 write_data;

    if (dpm_mode_is_enabled() == TRUE &&  dpm_mode_is_wakeup() == TRUE) {
        _da16x_io_pinmux(PIN_FMUX, FMUX_GPIO);

        gpio = GPIO_CREATE(GPIO_UNIT_A);
        GPIO_INIT(gpio);

        pin = GPIO_PIN11;
        GPIO_IOCTL(gpio, GPIO_SET_OUTPUT, &pin);

        PRINTF("Waking up MCU ...\n");
        write_data = GPIO_PIN11;
        GPIO_WRITE(gpio, GPIO_PIN11, &write_data, sizeof(UINT16));

        write_data = 0;
        GPIO_WRITE(gpio, GPIO_PIN11, &write_data, sizeof(UINT16));

        _GPIO_RETAIN_LOW(GPIO_UNIT_A, GPIO_PIN11);
    }
#endif // __TRIGGER_DPM_MCU_WAKEUP__
}

void regist_user_apps_to_DPM_manager(void)
{
#if defined ( __SUPPORT_DPM_MANAGER__ )

    /*
     * Need to add user operation to use DPM manager...
     */

#endif // __SUPPORT_DPM_MANAGER__
}

static const mbedtls_net_primitive_type da16x_crypto_net_apis = {
    &mbedtls_net_bsd_init,
    &mbedtls_net_bsd_connect,
    &mbedtls_net_bsd_bind,
    &mbedtls_net_bsd_accept,
    &mbedtls_net_bsd_set_block,
    &mbedtls_net_bsd_set_nonblock,
    &mbedtls_net_bsd_usleep,
    &mbedtls_net_bsd_recv,
    &mbedtls_net_bsd_send,
    &mbedtls_net_bsd_recv_timeout,
    &mbedtls_net_bsd_free
};

static int regist_crypto_net_socket_apis(void)
{
    int ret = (int)mbedtls_platform_bind(&da16x_crypto_net_apis);

    if (ret != 2) {
        PRINTF("Failed to register crypto net apis(%d)\r\n", ret);
        return -1;
    }

    return 0;
}

#if defined ( __SUPPORT_USER_DPM_RCV_READY_TO__ )
static void set_user_dpm_data_rcv_ready_timeout(void)
{
    /* For dpm_app_register() in user application
     *   Tunning value : 1 ~ 32768 ( 10msec ~ 327680 msec )
     */
    dpm_app_register_timeout = 20;              // 200 msec, unit is 10msec

    /* For dpm_app_data_rcv_ready_set() in user application
     *   Tunning value : 1 ~ 32768 ( 10msec ~ 327680 msec )
     */
    dpm_app_data_rcv_ready_set_timeout = 50;    // 500 msec, unit is 10msec
}
#endif // __SUPPORT_USER_DPM_RCV_READY_TO__


#ifdef CFG_USE_SYSTEM_CONTROL
#define USER_RTM_SYS_CTRL_TAG			"sys_ctrl"
#define SYSTEM_CONTROL_WLAN_DEFAULT		(1)	// 0: Off, 1: On

struct system_control_t {
	uint8_t wlan_enabled;
};
static struct system_control_t *system_control = NULL;

static uint8_t system_control_init(void)
{
#ifdef CFG_USE_RETMEM_WITHOUT_DPM
	/* Create the user retention memory and Initialize. */
	unsigned int result = user_retmmem_get(USER_RTM_SYS_CTRL_TAG,
			(UCHAR **)&system_control);
	if (result == 0) {
		// created a memory for the user data
		result = user_retmmem_allocate(USER_RTM_SYS_CTRL_TAG,
				(void**)&system_control, sizeof(struct system_control_t));
		if (result == 0) {
			memset(system_control, 0, sizeof(struct system_control_t));
			system_control->wlan_enabled = SYSTEM_CONTROL_WLAN_DEFAULT;
		} else {
			PRINTF("%s: Failed to allocate retention memory\n", __func__);
		}
	}

	return (system_control != NULL);
#else
	return pdTRUE;
#endif
}

static uint8_t system_control_wlan_enabled(void)
{
	if (system_control_init() && system_control) {
		return system_control->wlan_enabled;
	} else {
		return pdTRUE;
	}
}

void system_control_wlan_enable(uint8_t onoff)
{
	if (system_control) {
		system_control->wlan_enabled = onoff;
	} else {
		PRINTF("%s: System control is not available!\n", __func__);
	}
}
#endif // CFG_USE_SYSTEM_CONTROL


void system_start(void)
{
#if defined ( __SUPPORT_USER_DPM_RCV_READY_TO__ )
    /* Set user DPM register timeout value */
    set_user_dpm_data_rcv_ready_timeout();
#endif // __SUPPORT_USER_DPM_RCV_READY_TO__

    /* Config HW wakeup resource */
    config_user_wu_hw_resource();

    /* Create gpio handler event */
    gpio_handle_create_event();

    /* Set configuration for H/W button */
    config_gpio_button();

    /* Set paramters for system running */
    set_sys_config();

#if defined (CFG_USE_RETMEM_WITHOUT_DPM)
	user_retmem_init();
#endif

    /* Register Wi-Fi connect/disconnect status notify call-back functions */
    regist_wifi_notify_cb();

    /* Initialize WLAN interface */
#ifdef CFG_USE_SYSTEM_CONTROL
	wlaninit(system_control_wlan_enabled());
#else
	wlaninit();
#endif

	if (runtime_cal_flag == pdTRUE) {
		printf_with_run_time(" return from wlaninit");
	}

#if defined ( __SUPPORT_BTCOEX__ )
    /* Initialize Wi-Fi & BT/BLE Coexistance for Wi-Fi & BLE Combo module */
    initialize_bt_coex();
#endif  // __SUPPORT_BTCOEX__

    /* Setup WPS button */
    if (check_wps_button(wps_btn,
#if defined ( __SUPPORT_EVK_LED__ )
                         wps_led,
#endif // __SUPPORT_EVK_LED__
                         wps_btn_chk_time) == pdTRUE) {
        wps_setup(NULL);
    }

    /* Start GPIO polling thread */
    start_gpio_thread();

    set_dpm_abnorm_user_wakeup_interval();

    /* Register mbedtls net apis */
    regist_crypto_net_socket_apis();

#if defined ( __SUPPORT_DPM_MANAGER__ )
    /* Register User DPM application before start system application */
    regist_user_apps_to_DPM_manager();
#endif // __SUPPORT_DPM_MANAGER__

    /* Copy NVRAM from #0 to #1 if POR and both NVRAM0 and NVRAM1 are different. */
    if ((dpm_mode_get_wakeup_source() & WAKEUP_SOURCE_POR) && compare_NVRAM() == 1) {
        copy_nvram_flash(SFLASH_NVRAM_BACKUP, SFLASH_NVRAM_ADDR, SFLASH_NVRAM_BACKUP_SZ);
    }

    da16x_sys_watchdog_notify(da16x_sys_wdog_id_get_system_launcher());

    /* Start system applications for DA16XXX */
    start_sys_apps();

    /*
     * Entry point of user's applications
     *     : defined in user_apps_table.c
     */
    /* Start system applications for DA16XXX */
    start_user_apps();
}

/* EOF */
