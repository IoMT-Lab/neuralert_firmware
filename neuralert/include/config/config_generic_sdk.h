/**
 ****************************************************************************************
 *
 * @file config_generic_sdk.h
 *
 * @brief Configuration for Generic-SDK
 *
 * Copyright (c) 2016-2022 Renesas Electronics. All rights reserved.
 *
 * This software ("Software") is owned by Renesas Electronics.
 *
 * By using this Software you agree that Renesas Electronics retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Renesas Electronics is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Renesas Electronics products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * RENESAS ELECTRONICS BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */

#ifndef __CONFIG_GENERIC_SDK_H__
#define __CONFIG_GENERIC_SDK_H__

///
/// Features for Generic SDK
///

// SDK Name
// Show the kind of SDK displayed by the "sdk" command in the console.
//
//   - SDK Version     : V3.X.Y.0 GEN
//
#undef  SDK_NAME
    #define SDK_NAME    "GEN"

// SDK Version number
// Define the SDK version number by the "sdk" command in the console.
//
//   - SDK Version     : V3.X.Y.0 GEN
//
//    #define SDK_MAJOR       3               // SDK Package Major number
//    #define SDK_MINOR       X               // SDK Package Minor number
//    #define SDK_REVISION    Y               // SDK Package Revision number
//    #define SDK_ENG_VER     0               // SDK Engineering release number
//
#include "sdk_ver.h"

///////////////////////////////////////////////////////////////////////
//
// MAIN-features for Generic-SDK
//
///////////////////////////////////////////////////////////////////////

//-----------------------
// HW Features
//-----------------------

    //
    // Enable/Disable WPS button on DA16X00 EVK : Default is GPIOA6
    //
    // When enable this feature, user might be able to change the WPS_BUTTON in user_def.h based on H/W.
    // And it is defined below function :
    //    static void config_gpio_button(void) in ~/apps/da16200/get_started/src/user_main/system_start.c
    //
    #undef __SUPPORT_WPS_BTN__

    //
    // Enable/Disable Factory-Reset button on DA16X00 EVK : Default is GPIOA7
    //
    // When enable this feature, user might be able to change the FACTORY_BUTTON in user_def.h based on H/W.
    // And it is defined below function :
    //    static void config_gpio_button(void) in ~/apps/da16200/get_started/src/user_main/system_start.c
    //
    #define __SUPPORT_FACTORY_RESET_BTN__

    //
    // Enable/Disable BOR(Brown & Black out interrupt) Circuit
    //
    //  When enable this feature,
    //   user can change call-back function
    //    : static void rtc_brown_cb(void) in ~/apps/da16200/get_started/src/user_main/system_start.c
    //
    #define __SET_BOR_CIRCUIT__

    //
    // Enable/Disable RF antenna share for Wi-Fi & BT co-existence
    //
    #undef  __SUPPORT_BTCOEX__

    //
    // Antenna switching diversity features
    //  Related functions are located in ~/core/system/src/common/main/asd.c
    //
    #undef  __SUPPORT_ASD__

    //
    // UART1 interface connect to MCU
    //  This interface (UART1) initialize when AT-CMD module start...
    //
    //  When enable this feature,
    //   user can change the default values for UART1
    //    : static void atcmd_UART_init(void) in ~/apps/da16200/get_started/src/apps/user_interface.c
    //     - DEFAULT_UART_BAUD : 115,200bps
    //     - UART_BITPERCHAR   : 8bits
    //     - UART_PARITY       : No parity
    //     - UART_STOPBIT      : 1 stopbit
    //     - UART_FLOW_CTRL    : No flow-control
    //
    //   : Or can change UART1 configuration values thought ATB command.
    //     See AT-CMD user manual.
    #undef  __SUPPORT_UART1__

    //
    // UART2 interface
    //  Same as __SUPPORT_UART1__
    #undef  __SUPPORT_UART2__

    //
    // Enable/Disable External wake-up resources
    //  External wakeup by external interrupt when DPM sleep state or SLEEP-2  
    //
    //  When enable this feature,
    //   user can change RTC call-back function
    //    : static void rtc_ext_cb(void *data) in ~/apps/da16200/get_started/src/user_main/system_start.c
    #define  __SET_WAKEUP_HW_RESOURCE__


//-----------------------
// DPM Features
//-----------------------

    //
    // Enable/Disable DPM abnormal operation
    //  Default is enabled DPM abnormal operation
    //
    //  When enable this feature, Sleep/Wakeup in SLEEP 3 (DPM) mode works normally.
    //  However, the DA16200 cannot enter the sleep mode by itself
    //    even if abnormal conditions such as communication failure
    //    or other Wi-Fi operation and network operation are confirmed.
    //  In this case, Customer-SW have to control for about this one.
    //  If not, battery consumption will increase the same as when DPM is not used.
    //
    #define  __DISABLE_DPM_ABNORM__

    //
    // Enable/Disable DPM Abnormal wakeup User-timer
    //  This values are used to define DPM abnormal wakeup interval time.
    //
    //  Default values are defined in ~/library/libdpm.a
    //   : const unsigned long long dpm_monitor_retry_interval[DPM_MON_RETRY_CNT] = {
    //         ULLONG_MAX,    // Initial value : ULLONG_MAX
    //         60,            // 1st Wakeup
    //         60,            // 2nd Wakeup    : 0xdeadbeaf for no-wakeup
    //         60,            // 3rd Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 30,       // 4th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 30,       // 5th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 30,       // 6th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 60,       // 7th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 60,       // 8th Wakeup    : 0xdeadbeaf for no-wakeup
    //         0xDEADBEAF     // 9th Wakeup    : 0xdeadbeaf for no-wakeup
    //     };
    //
    //  When enable this feature,
    //   user can set these values to any value they want
    //   in ~/apps/da16200/get_started/src/user_main/system_start.c :
    //
    //   : unsigned long long _user_defined_wakeup_interval[DPM_MON_RETRY_CNT] = {
    //         ULLONG_MAX,    // Initial value : ULLONG_MAX
    //         60,            // 1st Wakeup
    //         60,            // 2nd Wakeup    : 0xdeadbeaf for no-wakeup
    //         60,            // 3rd Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 30,       // 4th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 30,       // 5th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 30,       // 6th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 60,       // 7th Wakeup    : 0xdeadbeaf for no-wakeup
    //         60 * 60,       // 8th Wakeup    : 0xdeadbeaf for no-wakeup
    //         0xDEADBEAF     // 9th Wakeup    : 0xdeadbeaf for no-wakeup
    //     };
    #undef __USER_DPM_ABNORM_WU_INTERVAL__

    //
    // DPM pTIM Unicast max receiving timeout value
    //  Default value is defined in ~/core/system/include/common/da16x_dpm_system.h
    //
    //  When enable this feature,
    //   user can change as they wants in ~/apps/da16200/get_started/src/apps/user_system_feature.c
    //   : unsigned char   dpm_tim_uc_max_twait = 3;
    //
    #undef  __SUPPORT_USER_TIM_UC_WAKEUP_TIME__

    //
    // Default TIM wakeup time
    //  This value defines the interval pTIM wakeup time to receive Wi-Fi packet.
    //  
    //  When enable this feature,
    //   user can change this value in ~/core/system/include/common/common_def.h
    //    : #define USER_DPM_TIM_WAKEUP_COUNT
    //   And this value is applied when DPM setting is performed again after SDK is rebuilt.
    //
    #undef __SUPPORT_DPM_DTIM_DFLT__

    //
    // DPM : Unicast packet recevie ready timeout before user network application starting.
    //
    //  When wakeup from DPM sleep mode by unicast packet receiving,
    //  DA16200 start again from power-on mode,
    //  so Wi-Fi MAC needs to wait to toss to network application starting.
    //
    //  Default values are defined in ~/library/libumac.a :
    //   // Value to wait until TASK is ready to receive // (0 ~ unlimit)
    //   short dpm_app_data_rcv_ready_set_timeout = 50;  // 500 msec
    //
    //   // Value to wait until PORT is registered       // (5 ~ 20)
    //   short dpm_app_register_timeout = 20;            // 50~200  msec
    //
    //  When enable this feature,
    //   user can change these values
    //   : static void set_user_dpm_data_rcv_ready_timeout(void)
    //     in ~/apps/da16200/get_started/src/user_main/system_start.c
    //
    #undef  __SUPPORT_USER_DPM_RCV_READY_TO__

    //
    // DPM Sleep Manager
    //
    #undef __SUPPORT_DPM_MANAGER__

    //
    // Watchdog Service
    //
    #define __SUPPORT_SYS_WATCHDOG__


//-----------------------
// System Features
//-----------------------

    //
    // Release/Debug build option on IDE build-configuration
    //
    //  If DISABLE_CONSOLE is defined, that is "Release-mode", 
    //  Message won't be printed-out on the console and input is also impossible.
    //  If DISABLE_CONSOLE is not defined, that is "Debug-mode", 
    //  Message will be printed-out on the console and input is possible.
    //
    //  If user wants to break to boot with "Release-mode" image with DA16200 EVK,
    //  follows below steps.
    //    1) Power-Off
    //    2) Connect teminal utility to console
    //    3) Press <ESC> key and turn-on the power switch
    //      : Then, the DA16200 EVK goes to [MROM] Mask-ROM booter mode.
    //
    //  Conversely, in "Debugging-mode", input/output is possible on the console.
    //
#if defined ( RELEASE_BUILD )
    // Disable Console(UART0) input & output
    #define __DISABLE_CONSOLE__       // Release-mode
#else
    // Enable Console(UART0) input & output
    #undef  __DISABLE_CONSOLE__       // Debug-mode
#endif // ... Build option ...

    //
    // Support Console enable/disable using by HW control.
    //  This feature contols UART0 HW register.
    //  It is very similar function with "__DISABLE_CONSOLE__"
    //   however this feature controls console-off phisically.
    //
    //  User can check these feature in ~/core/system/src/common/console.c
    //   - int console_on(void)
    //   - int console_off(void)
    //
    #undef  __CONSOLE_CONTROL__

    //
    // Console Security : Support console password
    // 
    //  When enable this feature, User needs to input default password that is "da16200".
    //  And password input timeout is 120 seconds. After password is typed, the logs will be printed.
    //  User can change password using by console command "chg_pw"
    //  and new password is saved in NVRAM as "PASSWORD" value.
    //
	#undef  __SUPPORT_CONSOLE_PWD__


    //
    // Fast Connection on SLEEP mode 2
    //
    // Enable/Disable Fast-connection feature.
    //  This function is internal one and user just can select enable/disable when SDK rebuild.
    //
    //  And AT-CMD mode, two AT-CMDs are supported to handle by MCU.
    //    : AT+GETFASTCONN
    //    : AT+SETFASTCONN
    //
    #undef  __SUPPORT_FASTCONN__

    //
    // Version check when boot time
    //
    // This feature supports to check RTOS image validation with header version information.
    //   For secure boot and secured SLIB : EncUE-BOOT and SSLIB.
    //   For non-secure boot and secured : SLIB, NSLIB
    //
    //   And check mismatch state of vendor-ID,
    //     : FRTOS_GEN01-01-4576-000000.img
    //             ^^^^^
    //   and check mismatch state of major-number,
    //     : FRTOS_GEN01-01-4576-000000.img
    //                   ^^
    //   In case of mismatch version value, DA16200 will not be start boot-procedure.
    //
    #define __SUPPORT_BOOT_FW_VER_CHK__

    //
    // Support cipher suites of TLS session by H/W engine.
    //
    // When enable this feature, only HW cipher-suites of CC312 HW enbine are used to communicate for TLS.
    // This operation set in each TLS application to start TLS operation.
    //
    //   #if defined(__SUPPORT_TLS_HW_CIPHER_SUITES__)
    //      preset = MBEDTLS_SSL_PRESET_DA16X_ONLY_HW;
    //   #endif // __SUPPORT_TLS_HW_CIPHER_SUITES__
    #undef  __SUPPORT_TLS_HW_CIPHER_SUITES__

    //
    // Support console commands for LMAC test.
    //
    //  Command is defined in ~/core/system/src/common/command/command_lmac.c
    //   LMAC         : lmac
    //   -------      : --------------------------------
    //   per          : lmac per logging
    //   tx           : lmac tx command
    //
    #undef __ENABLE_LMAC_CMD__

    //
    // Support console commands for LMAC.TX test.
    //
    //  All commands are defined in ~/core/system/src/common/command/command_lmac_tx.c
    //
    //   LMAC_TX      : lmac_tx
    //   -------      : ----------------------------
    //   init         : init (type addr) - lmac tx init
    //   start        : lmac ops start
    //   stop         : lmac ops stop
    //   ch           : channel
    //   state        : state
    //   dpm_mib      : MIB RCV for DPM Test
    //   power2       : set power2 level
    //   txp          : tx power measurement
    //   txpwr        : tx power measurement for fixed power
    //   txp1m        : tx power measurement for 11b 1Mbps
    //   txp6m        : tx power measurement for 11g 6Mbps
    //   txpm0        : tx power measurement for 11n MCS0
    //   vec          : lmac test vector test [vec][num][rate][power][length][cont][ch]
    //   status       : dma status
    //   pt_rate      : set policy table rate [0:4] value
    //   mac_clock    : set mac clock 20MHz(20) or 40MHz(40)
    //   rftx         : rftx
    //   rfcwtest     : rfcwtest
    //   rfcwstop     : rfcwtest stop
    //   rfcwpll      : rfcwpll set/reset
    //   mib          : mib
    //   cal_req      : RF CAL
    //   dpd          : RF DPD
    //   mac_recover  : MAC reset
    //   cont_tx      : CONT. TX
    //   dbg          : debug options
    //   radar        : radar isr
    //
    #undef __ENABLE_LMAC_TX_CMD__

    //
    // Support console commands for RF test.
    //
    //  All commands are defined in ~/core/system/src/common/command/command_rf.c
    //
    //   RF           : RF
    //   -------      : --------------------------------
    //   dpd          : dpd [0/1]
    //   ird          : tuner read cmd
    //   iwr          : tuner write cmd
    //   iverify      : tuner write cmd
    //
    #undef __ENABLE_RF_CMD__


//-----------------------
// Wi-Fi Features
//-----------------------

    //
    // Support Wi-Fi WPA Enterprise feature.
    //
    //  This feature is to support WPA3-Enterprise. And it provides EAP TLS, EAP TTLS, PEAPv0, PEAPv1.
    //  User can check WPA3-Enterprise sub-feature is enabled by default.
    //
    //  Note) If "__SUPPORT_ATCMD__" is enabled, this feature is default enabled also.
    //        Refer to AT-CMD user's guide document for how to use it.
    //  Note) This feature is internal feature in ~/library/libsupplicant.a
    //
    #define  __SUPPORT_WPA_ENTERPRISE__

    //
    // Support Wi-Fi WPA3-Personal features.
    //
    //  This feature is to support WPA3-Personal. And it provides Simultaneous Authentication of Equals(SAE), 
    //  Opportunistic Wireless Encryption (OWE).
    //  User can check WPA3-Enterprise sub-feature is enabled by default.
    //
    //  Note #1) See Wi-Fi specification to understand WPA3 SAE and OWE operation.
    //  Note #2) If "__SUPPORT_ATCMD__" is enabled, this feature is default enabled also.
    //  Note #3) This feature is internal feature in ~/library/libsupplicant.a
    //
    //
    #undef  __SUPPORT_WPA3_PERSONAL__

    //
    // Support user call-back functions to notify Wi-Fi connection status.
    //
    //  This features notify the Wi-Fi connection status through registered user call-back function.
    //    - Wi-Fi connection status
    //    - Wi-Fi disconnection status
    //    - Wi-Fi connection failed status
    //
    //  Users can change these call-back function to whatever operation they want.
    //   These call-back functions are located in ~/apps/da16200/get_started/src/apps/user_apps.c
    //    - static void user_wifi_conn(void *arg)
    //    - static void user_wifi_conn_fail(void *arg)
    //    - static void user_wifi_disconn(void *arg)
    //
    //  These functions are registered through the "void regist_wifi_notify_cb(void)"
    //   in ~/apps/da16200/get_started/src/apps/user_apps.c
    //   And when start the DA16200, "regist_wifi_notifi_cb()" is called in system_start().
    //
    //  In case of enabled "AT-CMD feature", the DA16200 notifies the connection status to MCU.
    //   User can check this function "atcmd_asynchony_event()" in ~/core/system/src/at_cmd/atcmd.c
    //   These notification are reported throught internal events operation.
    //
    //     - STA connect OK
    //        : +WFJAP:1,TEST_SSID,IP_ADDR
    //     - STA connect Fail
    //        : +WFJAP:0,TIMEOUT
    //        : +WFJAP:0,WRONGPWD
    //        : +WFJAP:0,ACCESSLIMIT
    //        : +WFJAP:0,OTHER,reason_code
    //     - STA disconnect
    //        : +WFDAP:0,AUTH_NOT_VALID
    //        : +WFDAP:0,DEAUTH
    //        : +WFDAP:0,INACTIVITY
    //        : +WFDAP:0,APBUSY
    //        : +WFDAP:0,OTHER,reason_code
    //
    #undef __SUPPORT_WIFI_CONN_CB__

    //
    // Support Wi-Fi Concurrent-mode.
    //
    //  Wi-Fi Concurrent mode means two interface will be enabled.
    //  One (wlan0) is STA and the other (wlan1) is Soft-AP.
    //
    //  In easy-setup, three types of Wi-Fi operation mode will be provided.
    //    SYSMODE(WLAN MODE) ?
    //      1. Station
    //      2. Soft-AP
    //      3. Station & SOFT-AP                   <= Wi-Fi Concurrent-mode
    //    MODE ?  [1/2/3/Quit] (Default Station) :
    //
    #undef __SUPPORT_WIFI_CONCURRENT__


//-----------------------
// AT-CMD Features
//-----------------------

    //
    // Enable/Disable AT-CMD module
    //
    // When enable this feature, more detail features are support below sub-features.
    // User can check all AT-CMDs in ~/core/system/src/at_cmd/atcmd.c
    //
    #undef  __SUPPORT_ATCMD__


//-----------------------
// Network Features
//-----------------------

    //
    // Support DHCP Server on the DA16200 Soft-AP mode.
    //
    #undef __SUPPORT_DHCP_SVR__

    //
    // Support OTA-Update through Wi-Fi connection
    //  This function operates with HTTP Client protocol.
    //
    //  User can check the OTA-update source code in ~/core/system/src/ota ,
    //   however this operation is oriented internal operation.
    //
    //  Note) If user want to change HTTP client operation for OTA-update,
    //        please contact to user support team of Wi-Fi.
    //  Note) Refer to ATCMD UM, MQTT Programmer guide, or MQTT sample code.
    //       - UM-WI-048_DA16200_DA16600_FreeRTOS_OTA_Update_User_Manual_RevXvY.pdf
    //
    #undef __SUPPORT_OTA__

    //
    // Support MQTT Client feature.
    //
    //  This feature enables MQTT Client to communicate with MQTT Broker (MQTT v3.1 or v3.1.1)
    //
    //  Simply, user can reference using by MQTT sample code.
    //    ~/apps/common/examples/Network/MQTT_Client
    //
    //  Configure and run MQTT Client using AT Commands (AT+NWMQ...) or MQTT APIs //  (mqtt_client.h).
    //
    //  Note) Refer to ATCMD UM, MQTT Programmer guide, or MQTT sample code.
    //       - UM-WI-003_DA16200_DA16600_AT-Command_Rev_XvY.pdf
    //       - UM-WI-010_DA16200_DA16600_MQTT_Programmer_Guide_RevXvY.pdf
    //
    #define __SUPPORT_MQTT__

    //
    // Support SNTP Client feature
    //
    //  This feature enable SNTP client to get current time from the defined Internet time server.
    //
    #define __SUPPORT_SNTP_CLIENT__

    //
    // Support Websocket Client
    //
    //  This feature enable Websocket client to communicate with peer Websocket server.
    //
    #undef __SUPPORT_WEBSOCKET_CLIENT__

    //
    // Support User-format to send HTTP-Client request ( header + body )
    //
    //  When a user requests an http-client using AT-CMD,
    //   the header format already defined in http_client.c is not used, but the header is written in free form.
    //   If there is data to be sent, such as PUT or POST, it can be configured in the form of header + data.
    //
    //  This function is used by MCU through AT-CMDs.
    //    : AT+NWHTC=<URL>,message,'XXXXX'
    //
    //  Note) Refer to AT-CMD user's guide document for how to use it.
    //
    #undef __SUPPORT_HTTP_CLIENT_USER_MSG__

    //
    // Support Zero-Config(mDNS, DNS-SD, and xmDNS) service
    //
    // This Zero-Config functions support three type of protocols.
    //   - mDNS (Multicast DNS)
    //   - xmDNS (Extended multicast DNS)
    //   - DNS-SD (DNS Service Discovery)
    #undef  __SUPPORT_ZERO_CONFIG__

    //
    // Support "nslookup" network utility
    //
    // This function is used to get IP-address from doman-name.
    //  Support console-command under "net" command layer.
    //
    // Default disabled.
    //
    #undef  __SUPPORT_NSLOOKUP__

    //
    // Support Auto-start HTTP server application when system starting.
    //
    //  User can check this task in ~/core/system/src/common/main/sys_apps.c
    //
    //  static const app_task_info_t sys_apps_table[] = {
    //   ...
    //  #if defined ( __HTTP_SVR_AUTO_START__ )
    //    { APP_HTTP_SVR, auto_run_http_svr, 256, (OS_TASK_PRIORITY_USER + 6), TRUE, FALSE, HTTP_SVR_PORT, RUN_AP_MODE },
    //  #endif // __HTTP_SVR_AUTO_START__
    //
    //  Note) If "__SUPPORT_ATCMD__" is enabled, this feature is default enabled also.
    //        Refer to AT-CMD user's guide document for how to use it.
    //
    #undef  __HTTP_SVR_AUTO_START__

    //
    // Support Auto-start HTTPs server application when system starting.
    //
    //  User can check this task in ~/core/system/src/common/main/sys_apps.c
    //
    //  static const app_task_info_t sys_apps_table[] = {
    //   ...
    //  #if defined ( __HTTP_SVR_AUTO_START__ )
    //    { APP_HTTPS_SVR, auto_run_https_svr, 256, (OS_TASK_PRIORITY_USER + 6), TRUE, FALSE, HTTP_SVR_PORT, RUN_AP_MODE },
    //  #endif // __HTTP_SVR_AUTO_START__
    //
    //  If user wants to run on STA mode of the DA16200,
    //   after change "running mode" above sourc code from "RUN_AP_MODE" to "RUN_STA_MODE" or "RUN_BASIC_MODE"
    //   and rebuild SDK.
    //
    //  Note) If "__SUPPORT_ATCMD__" is enabled, this feature is default enabled also.
    //        Refer to AT-CMD user's guide document for how to use it.
    //
    #undef  __HTTPS_SVR_AUTO_START__

    //
    // Support CLI command on Console for HTTP-Server operation
    //  This feature supports to check the operation status with console command.
    //
    //  NET              : Network
    //  -------          : --------------------------------
    //  http-server      : http-server -I [wlan0|wlan1] [start|stop]
    //  https-server     : https-serer -I [wlan0|wlan1] [start|stop]
    //
    //  These commands operate with own IP-address and 80 port number as default.
    //
    #undef  __SUPPORT_HTTP_SERVER_FOR_CLI__

    //
    // Support CLI command on Console for HTTP-Client operation
    //  This feature supports to check the operation status with console command.
    //
    //  NET              : Network
    //  -------          : --------------------------------
    //  http-client      : http-client help
    //
    //  [/DA16200/NET] # http-client
    //
    //    Usage: HTTP Client
    //    Name
    //          http-client - HTTP Client
    //    SYNOPSIS
    //        http-client [OPTION]...URL
    //    DESCRIPTION
    //        Request client's method to URL
    //        -i [wlan0|wlan1]
    //                Set interface of HTTP Client
    //        -status
    //                Display status of HTTP Client
    //        -help
    //                Display help
    //        -head
    //                Request HEAD method to URI
    //        -get
    //                Request GET method to URI
    //        -post RESOURCE
    //                Request POST method to URI with RESOURCE
    //        -put RESOURCE
    //                Request PUT method to URI with RESOURCE
    //        -incoming Size
    //                Set incoming buffer size of TLS Contents
    //        -outgoing Size
    //                Set outgoing buffer size of TLS Contents
    //        -sni <Server Name Indicator>
    //                Set SNI for TLS extension
    //        -alpn <ALPN Protocols>
    //                Set ALPN for TLS extension
    //
    #undef  __SUPPORT_HTTP_CLIENT_FOR_CLI__

    //
    // Support DNS 2nd cache internally.
    //
    // This function supports semi-cache operation to reduce DNS query time.
    //  The address found as a new domain name is stored in the memory,
    //  and when a new DNS Query is performed, the address is first found in the cache area.
    //  This caching IP addresses are also save in NVRAM to reload when booting time.
    //
    //  Stores up to 25 domain-names and IP address.
    //  The maximum length of domain name is 128 Bytes.
    //
    //  Note) If this feature is enabled,
    //        do not use 4KB from SFLASH_USER_AREA_START area for other purpose.
    //
    #undef  __DNS_2ND_CACHE_SUPPORT__

    //
    // Support User DHCP-client hostname function.
    //
    //  This feature support to set user DHCP hostname rather than default value (DA16200-XXYY).
    //  When enable this feature, console command is enabled.
    //
    //  Note #1) This feature complies with RFCs(952, 1123).
    //   : RFCs(952, 1123) mandate that a hostname's labels may contain only
    //     the ASCII letters 'a' through 'z' (case-insensitive),
    //     the digits '0' through '9', and the hyphen.
    //     Hostname labels cannot begin or end with a hyphen.
    //     No other symbols, punctuation characters, or blank spaces are permitted.
    //
    //  Note #2) If "__SUPPORT_ATCMD__" is enabled, this feature is default enabled also.
    //
    //        - AT+NWDHCHN
    //        - AT+NWDHCHNDEL
    //
    //        Refer to AT-CMD user's guide document for how to use it.
    //
    #undef  __USER_DHCP_HOSTNAME__

    //
    // Support iPerf tool for network performance measurement.
    //
    //  This test operation can operate using by console commands.
    //
    //  Note) iPerf 2.X version is compatible but don't compatible with iPerf V3.X on peer device.
    //  Note) Refer to "DA1620_DA16600_FreeRTOS Getting Started Guide" document for more detailed information
    //       - UM-WI-056_DA16200_DA16600_FreeRTOS_Getting_Started_Guide_Rev_XvY.pdf
    //
    #undef __SUPPORT_IPERF__


//-------------------
// User Features
//-------------------

    //
    // Config default Wi-Fi running mode after factory_reset_default().
    //
    // STA mode, Soft-AP mode and Concurrent-mode (STA + Soft-AP) are provided in DA16200 SDK.
    // If all modes does not defined for customer decision,
    //    customer can make their own sequence after reset operation.
    //  
    //    ; ~/apps/da16200/get_started/src/apps/user_system_feature.c
    //
    //    void factory_reset_user_define(void)
    //    {
    //        // User define mode
    //    }
    //    
    //    UINT factory_reset_default(int reboot_flag)
    //    {
    //        PRINTF("\nFactory Reset...\n");
    //    
    //    #if defined( __SUPPORT_FACTORY_RST_APMODE__ )
    //      ... ...
    //    #elif defined( __SUPPORT_FACTORY_RST_STAMODE__ )
    //      ... ...
    //    #elif defined( __SUPPORT_FACTORY_RST_CONCURR_MODE__ )
    //      ... ...
    //    #else
    //      factory_reset_user_define();
    //    #endif // __SUPPORT_FACTORY_RST_APMODE__
    //
    #if defined ( __SUPPORT_WIFI_CONCURRENT__ )
        #define __SUPPORT_FACTORY_RST_CONCURR_MODE__        // Factory reset Concurrent-Mode
        #define __SUPPORT_REMOVE_MAC_NAME__                 // Remove MAC of tail for connection mobile APP

        #undef  __SUPPORT_FACTORY_RST_APMODE__              // Factory reset AP-Mode
        #undef  __SUPPORT_FACTORY_RST_STAMODE__             // Factory reset STA-Mode
    #else
        #define __SUPPORT_FACTORY_RST_APMODE__              // Factory reset AP-Mode
        #undef  __SUPPORT_FACTORY_RST_STAMODE__             // Factory reset STA-Mode
    #endif // __SUPPORT_WIFI_CONCURRENT__

    //
    // User can set maximum size of http-client's tx buffer. (Default 4KB, Available 2KB~8KB)
    //
    //   HTTPC_REQ_DATA_MAX_SIZE affects the size of TX_PAYLOAD_MAX_SIZE and USER_ATCMD_BUF defined in atcmd.h.
    //   TX_PAYLOAD_MAX_SIZE and USER_ATCMD_BUF must always be greater than HTTPC_REQ_DATA_MAX_SIZE.
    //
    //     #define TX_PAYLOAD_MAX_SIZE              HTTPC_REQ_DATA_MAX_SIZE
    //     #define USER_ATCMD_BUF                   (TX_PAYLOAD_MAX_SIZE + 32)
    //
    #define HTTPC_REQ_DATA_MAX_SIZE		       (1024 * 4)
    #if (HTTPC_REQ_DATA_MAX_SIZE < (1024 * 2))
      #error "Supporting buffer size error : Too small !!!"
    #elif (HTTPC_REQ_DATA_MAX_SIZE > (1024 * 8))
      #error "Supporting buffer size error : Too large !!!"
    #endif

    //
    // User application for "Provisioning" operation
    //   which is matched with DA16200 Mobile-APP.
    //
    #undef __SUPPORT_WIFI_PROVISIONING__


///////////////////////////////////////////////////////////////////////
//
// SUB-features for Generic-SDK
//
///////////////////////////////////////////////////////////////////////


    #if defined ( __SUPPORT_MQTT__ )
        #define __MQTT_SELF_RECONNECT__
        #define __MQTT_CONN_STATUS__
        #define __MQTT_TLS_OPTIONAL_CONFIG__

        #undef __MQTT_CLEAN_SESSION_MODE_SUPPORT__
            #if defined (__MQTT_CLEAN_SESSION_MODE_SUPPORT__)
                // Max payload length of a preserved message
                #define MQTT_MSG_TBL_PRESVD_MAX_PLAYLOAD_LEN    100

                // Max number of preserved messages
                #define MQTT_MSG_TBL_PRESVD_MAX_MSG_CNT         10

                #undef  __MQTT_EMUL_CMD__
            #endif // __MQTT_CLEAN_SESSION_MODE_SUPPORT__

        #undef __MQTT_DPM_WAIT_TLS_FULL_DECRYPT__

    #endif // __SUPPORT_MQTT__

    #if defined ( __SUPPORT_WPA_ENTERPRISE__ )
        #define __SUPPORT_WPA3_ENTERPRISE__

        #if defined __SUPPORT_WPA3_ENTERPRISE__
            #undef  __SUPPORT_WPA3_ENTERPRISE_192B__        // Unsupported on DA16200
        #endif // __SUPPORT_WPA3_ENTERPRISE__
    #endif /* __SUPPORT_WPA_ENTERPRISE__ */


///////////////////////////////////////////////////////////////////////
//
// !!! Notice !!!
//    Do not remove this header-file.
//    And do not change the location of this file ...
//

#include "sys_common_features.h"

///////////////////////////////////////////////////////////////////////

#define CFG_USE_RETMEM_WITHOUT_DPM
#define CFG_USE_SYSTEM_CONTROL

///////////////////////////////////////////////////////////////////////

#endif    // GENERIC_SDK  -------------------------------------

/* EOF */
