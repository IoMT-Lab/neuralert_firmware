/**
 ****************************************************************************************
 *
 * @file user_main.h
 *
 * @brief User Main entry point definition
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
 */

#include "da16x_network_common.h"
#include "common_def.h"
#include "da16x_system.h"


/*
 * External global variables
 */
extern int    factory_rst_btn;
extern int    factory_rst_led;
extern int    factory_rst_btn_chk_time;

extern int    wps_btn;
extern int    wps_led;
extern int    wps_btn_chk_time;

#if defined ( __SUPPORT_NOTIFY_RTC_WAKEUP__ )
extern int rtc_wakeup_intr_chk_flag;
#endif // __SUPPORT_NOTIFY_RTC_WAKEUP__



/*
 * For Customer's configuration
 *
 * #define MAX_SSID_PREFIX_LEN        64
 * #define MAX_PASSKEY_LEN            64
 * #define MAX_IP_ADDR_LEN            16
 *
 * #define AP_OPEN_MODE               0
 * #define AP_SECURITY_MODE           1
 *
 * #define IPADDR_DEFAULT             0
 * #define IPADDR_CUSTOMER            1
 *
 * #define DHCPD_DEFAULT              0
 * #define DHCPD_CUSTOMER             1
 *
 *
 *    typedef struct _softap_config {
 *        int    customer_cfg_flag;    // MODE_ENABLE, MODE_DISABLE
 *
 *        char    ssid_name[MAX_SSID_LEN];
 *        char    psk[MAX_PASSKEY_LEN];
 *        char    auth_type;        // AP_OPEN_MODE, AP_SECURITY_MODE
 *
 *        // AD  AE  AF  AI  AL  AM  AR  AS  AT  AU  AW  AZ  BA  BB  BD  BE  BF  BG  BH  BL
 *        // BM  BN  BO  BR  BS  BT  BY  BZ  CA  CF  CH  CI  CL  CN  CO  CR  CU  CX  CY  CZ
 *        // DE  DK  DM  DO  DZ  EC  EE  EG  ES  ET  EU  FI  FM  FR  GA  GB  GD  GE  GF  GH
 *        // GL  GP  GR  GT  GU  GY  HK  HN  HR  HT  HU  ID  IE  IL  IN  IR  IS  IT  JM  JO
 *        // JP  KE  KH  KN  KP  KR  KW  KY  KZ  LB  LC  LI  LK  LS  LT  LU  LV  MA  MC  MD
 *        // ME  MF  MH  MK  MN  MO  MP  MQ  MR  MT  MU  MV  MW  MX  MY  NG  NI  NL  NO  NP
 *        // NZ  OM  PA  PE  PF  PG  PH  PK  PL  PM  PR  PT  PW  PY  QA  RE  RO  RS  RU  RW
 *        // SA  SE  SG  SI  SK  SN  SR  SV  SY  TC  TD  TG  TH  TN  TR  TT  TW  TZ  UA  UG
 *        // UK  US  UY  UZ  VA  VC  VE  VI  VN  VU  WF  WS  YE  YT  ZA  ZW
 *        char    country_code[4];
 *
 *         int    customer_ip_address;    // IPADDR_DEFAULT, IPADDR_CUSTOMER
 *        char    ip_addr[MAX_IP_ADDR_LEN];
 *        char    subnet_mask[MAX_IP_ADDR_LEN];
 *        char    default_gw[MAX_IP_ADDR_LEN];
 *        char    dns_ip_addr[MAX_IP_ADDR_LEN];
 *
 *        int    customer_dhcpd_flag;    // DHCPD_DEFAULT, DHCPD_CUSTOMER
 *        int    dhcpd_lease_time;
 *        char    dhcpd_start_ip[MAX_IP_ADDR_LEN];
 *        char    dhcpd_end_ip[MAX_IP_ADDR_LEN];
 *        char    dhcpd_dns_ip_addr[MAX_IP_ADDR_LEN];
 *
 *    } softap_config_t;
 */
extern softap_config_t *ap_config_param;
extern int customer_console_baudrate;


/*
 * External global functions
 */
extern UINT _da16x_io_pinmux(UINT mux, UINT config);
extern UINT factory_reset_default(int reboot_flag);
extern void version_display(int argc, char *argv[]);
extern int  get_boot_mode(void);
extern int  get_run_mode(void);
extern void system_start(void);
extern void regist_wifi_notify_cb(void);
extern UINT wps_setup(char *macaddr);
extern void start_gpio_thread(void);
extern int  set_gpio_interrupt(void);

extern int  (*button1_one_touch_cb)(void);
extern INT32 __GPIO_RETAIN_HIGH_RECOVERY();

#if defined ( __REMOVE_32KHZ_CRYSTAL__ )
extern int  ALT_RTC_ENABLE(void);
#endif    // __REMOVE_32KHZ_CRYSTAL__

#if defined ( __SUPPORT_NAT__ )
extern void set_nat_flag_enable(void);
extern int  chk_dpm_wakeup(void);
#endif // __SUPPORT_NAT__


/*
 ****************************************************************************************
 * @brief      Set paramters for system running
 * @param[in]  None
 * @return     None
 ****************************************************************************************
 */
extern void set_sys_config(void);

/*
 ****************************************************************************************
 * @brief      Set paramters for system running. Initialize WLAN interface
 * @param[in]  None
 * @return     None
 ****************************************************************************************
 */
#ifdef CFG_USE_SYSTEM_CONTROL
extern UINT	wlaninit(uint8_t);
#else
extern UINT	wlaninit(void);
#endif

/*
 ****************************************************************************************
 * @brief      Config Pin-mux to control Wi-Fi & BT co-existence
 * @return     None
 ****************************************************************************************
 */
extern void initialize_bt_coex(void);

/*
 ****************************************************************************************
 * @brief      Set Factory-Reset button configuration
 * @param[in]  reset_gpio  GPIO pin number for Factory-Reset button
 * @param[in]  led_gpio    GPIO pin number for Factory-Reset LED
 * @param[in]  seconds     Checking time for button press ( seconds )
 * @return     None
 ****************************************************************************************
 */
extern void    config_factory_reset_button(int reset_gpio, int led_gpio, int seconds);

/*
 ****************************************************************************************
 * @brief      Set Wi-Fi WPS button configuration
 * @param[in]  reset_gpio  GPIO pin number for Factory-Reset button
 * @param[in]  led_gpio    GPIO pin number for Factory-Reset LED
 * @param[in]  seconds     Checking time for button press ( seconds )
 * @return     None
 ****************************************************************************************
 */
extern void config_wps_button(int wps_gpio, int led_gpio, int seconds);

/*
 ****************************************************************************************
 * @brief      Check button press status for Factory-Reset button
 * @param[in]  reset_gpio  GPIO pin number for Factory-Reset button
 * @param[in]  led_gpio    GPIO pin number for Factory-Reset LED
 * @param[in]  seconds     Checking time for button press ( seconds )
 * @return     None
 ****************************************************************************************
 */
extern UINT    check_factory_button(int btn_gpio_num, int led_gpio_num, int check_time);

/*
 ****************************************************************************************
 * @brief      Check button press status for Wi-Fi WPS button
 * @param[in]  wps_gpio    GPIO pin number for Wi-FI WPS button
 * @param[in]  led_gpio    GPIO pin number for Wi-FI WPS LED
 * @param[in]  seconds     Checking time for button press ( seconds )
 * @return     None
 ****************************************************************************************
 */
extern UINT    check_wps_button(int btn_gpio_num, int led_gpio_num, int check_time);

/*
 ****************************************************************************************
 * @brief      Start system applications to run DA16200 ( Registered in sys_apps_tablep[] )
 * @return     None
 ****************************************************************************************
 */
extern void    start_sys_apps(void);

/*
 ****************************************************************************************
 * @brief      Start system applications to run DA16200 ( Registered in user_apps_tablep[] )
 * @return     None
 ****************************************************************************************
 */
extern void    start_user_apps(void);

/*
 ****************************************************************************************
 * @brief        
 * @input[in]
 * @return     None
 ****************************************************************************************
 */
extern void gpio_callback(void *param);

/**
 *********************************************************************************
 * @brief      Configurate pin-mux for DA16200
 * @param      None
 * @return     Returns always TRUE
 *********************************************************************************
 */
int config_pin_mux(void);

/**
 *********************************************************************************
 * @brief      Set external-wakeup configuration for Sleep-Mode
 * @param      None
 * @return     None
 *********************************************************************************
 */
void config_ext_wakeup_resource(void);

/**
 *********************************************************************************
 * @brief      Register one-touch press call-back function for Factory-Reset button
 * @return     None
 *********************************************************************************
 */
void set_customer_softap_config(void);

/**
 *********************************************************************************
 * @brief      Register user application to handle DPM opeartion on DPM manager
 * @return     None
 *********************************************************************************
 */
void regist_user_apps_to_DPM_manager(void);

/**
 *********************************************************************************
 * @brief      Customer main application entry point
 * @input[in]  init_state - Initialize flag RamLib and PTIM : Normal is pdTRUE
 * @return     Return always TRUE
 *********************************************************************************
 */
int user_main(char init_state);

/**
 *********************************************************************************
 * @brief      Set gpio interrupt configuration for Factory_Reset and WPS button
 * @return     None
 *********************************************************************************
 */
int set_gpio_interrupt(void);

/* EOF */
