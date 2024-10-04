/**
 ****************************************************************************************
 *
 * @file sys_common_features.h
 *
 * @brief Define for common system features
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

#ifndef __SYS_COMMON_FEATURES_H__
#define __SYS_COMMON_FEATURES_H__

#include "project_config.h"

// Wi-Fi configuration features
#include "wifi_common_features.h"

// Console Prompt
#define PROMPT        "/"CHIPSET_NAME

///////////////////////////////////////////////////////////////////////
//
// Main features for Generic-SDK
//
///////////////////////////////////////////////////////////////////////

// Non-mounted 32KHz Crystal
#undef  __REMOVE_32KHZ_CRYSTAL__

// SIGMA Throughput Test for Wi-Fi Certification
#undef  __SUPPORT_SIGMA_TEST__

// IPv6
#undef  __SUPPORT_IPV6__

// DPM command
#define __SUPPORT_DPM_CMD__

// Unused debugging system commands
#undef  __SUPPORT_DBG_SYS_CMD__

// RF Test AT-CMD
#undef  __SUPPORT_CMD_AT__

// UART1 interrupt
#define __ENABLE_UART1_CLOCK__

// GPIO interrupt
#define __ENABLE_GPIO_CLOCK__

// Reboot by WDT
#define __AUTO_REBOOT_WDT__

// Reboot by exception
#define __AUTO_REBOOT_EXCEPTION__

// Add redundancy heap
#define __USING_REDUNDANCY_HEAP__

// Reduce DPM Wakeup -> Sleep
#undef  __OPTIMIZE_DPM_SLP_TIME__

// DPM Auto ARP Enable in TIM, it should be enabled
#define __DPM_AUTO_ARP_ENABLE__

#define __SUPPORT_DPM_DHCP_CLIENT__

// For debugging
#undef  __DA16X_DPM_MON_CLIENT__

// DPM Abnormal Monitor commands
#undef  __CHK_DPM_ABNORM_STS__

#undef  __DPM_FINAL_ABNORMAL__

// With timp
#undef  __CHECK_BADCONDITION_WAKEUP__

// Wi-Fi re-connect stops when wifi conn fail by wrong key
#undef  __WIFI_CONN_RETRY_STOP_AT_WK_CONN_FAIL__

// Support RTC Timer running in DPM Abnormal mode
#undef  __SUPPORT_DPM_ABN_RTC_TIMER_WU__

// DPM Dynamic Period Setting by TIM BUFP
#define __SUPPORT_DPM_DYNAMIC_PERIOD_SET__

// Response to continued fault
#define __CHECK_CONTINUOUS_FAULT__

// DPM TCP Ack for TIM
#define __SUPPORT_DPM_TIM_TCP_ACK__

// Mandatory feature for v3.0
#define __TIM_LOAD_FROM_IMAGE__

// !!! Caution !!!
// Built-in Secure NVRAM feature
// Do not change this feature to UNDEF
#define __SUPPORT_SECURE_NVRAM__        0    // 0: Non-secure, 1: HUK, 2: KCP, 3: KPICV

// Unsupport some commands
#define __REDUCE_CODE_SIZE__

// Log message for Code run-time
#undef  __RUNTIME_CALCULATION__


///////////////////////////////////////////////////////////////////////
//
// Dependent features for Generic-SDK
//
///////////////////////////////////////////////////////////////////////

#if defined ( __FOR_FCCSP_SDK__ )
    #if defined ( __FCCSP_LOW_POWER__ )
        #undef  SDK_NAME
        #define SDK_NAME    "CSP-LP"        // CSP Type Low-Power mode
    #else
        #undef  SDK_NAME
        #define SDK_NAME    "CSP-NP"        // CSP Type Normal-Power mode
    #endif    // __FCCSP_LOW_POWER__
#endif // __FOR_FCCSP_SDK__

#if defined ( __SUPPORT_WIFI_CONCURRENT__ )
    #if defined ( XIP_CACHE_BOOT )          // RTOS
        #undef  __SUPPORT_DPM_MANAGER__     // Disable DPM-Manager because of memory shortage
    #endif    // XIP_CACHE_BOOT

#else
    //
    // Option feature to boot with "STA+Soft-AP" mode on Non-concurrent RTOS
    //
    //  !!! Not supported on SDK3.2.5 !!!
    //
    #undef  __IGNORE_SYS_RUN_MODE__
#endif // __SUPPORT_WIFI_CONCURRENT__

///////////////////////////////////////////////////////////////////////
//
// DPM Sub-features ...
//
///////////////////////////////////////////////////////////////////////

// DHCP Client features on DPM mode
#if defined ( __SUPPORT_DPM_DHCP_CLIENT__ )
    #define __SUPPORT_DHCP_RETRANS_TIMEOUT_4SEC__    // retry timeout 4sec
    #define __SUPPORT_DHCP_CLIENT_RENEW_5_OF_6__     // DHCP Lease Time 5/6
    #define __RECONNECT_NON_ACTION_DHCP_CLIENT__
#endif // __SUPPORT_DPM_DHCP_CLIENT__

#if defined (__WIFI_CONN_RETRY_STOP_AT_WK_CONN_FAIL__)
    // optional: dpm abnormal sleep2 w/o rtc-timer when wifi conn fail by wrong key
    #undef  __DPM_ABN_SLEEP1_BY_WK_CONN_FAIL__
#endif // __WIFI_CONN_RETRY_STOP_AT_WK_CONN_FAIL__

#if defined (__TIM_LOAD_FROM_IMAGE__)
    #undef  __PRINT_TIM_LOAD_FROM_IMAGE__
#endif // __TIM_LOAD_FROM_IMAGE__


///////////////////////////////////////////////////////////////////////
//
// Wi-Fi Certification features : Support SIGMA test environment
//
///////////////////////////////////////////////////////////////////////
#if defined ( __SUPPORT_SIGMA_TEST__ )
    #define __SUPPORT_SIGMA_THROUGHPUT__
    #define __SUPPORT_SIGMA_UAPSD__

    #undef  __SUPPORT_WPS_BTN__
    #define __SUPPORT_UART1__
    #undef  __SUPPORT_ATCMD__
    #define __SUPPORT_IEEE80211W__          // IEEE 802.11W(PMF)
#endif // __SUPPORT_SIGMA_TEST__

#if defined ( __ENABLE_UMAC_CMD__ )
    #define __ENABLE_LMAC_TX_CMD__
#endif // __ENABLE_UMAC_CMD__

#if defined ( __TIM_LOAD_FROM_IMAGE__ )
    #undef  __USING_REDUNDANCY_HEAP__       // Conflict with RLIB memory
#endif // __TIM_LOAD_FROM_IMAGE__


#endif    // __SYS_COMMON_FEATURES_H__

/* EOF */
