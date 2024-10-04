/**
 ****************************************************************************************
 *
 * @file user_apps.c
 *
 * @brief Config table to start user applications
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

#include "da16x_system.h"
#include "application.h"
#include "common_def.h"

//#include "sample_defs.h" //JW Added this -- probably could combine
// by adding the following two lines, I think we can now remove sample_defs.h from our app
#define USER_READ_DATA						"USER_READ"
#define USER_TRANSMIT_DATA					"USER_TX"

/******************************************************************************
 * External global variables
 ******************************************************************************/
#if defined ( __SUPPORT_WIFI_CONN_CB__ ) && !defined ( __ENABLE_SAMPLE_APP__ )

extern EventGroupHandle_t  evt_grp_wifi_conn_notify;

/* Station mode */
extern short wifi_conn_fail_reason;
extern short wifi_disconn_reason;

/* AP mode */
extern short ap_wifi_conn_fail_reason;
extern short ap_wifi_disconn_reason;
#endif  // __SUPPORT_WIFI_CONN_CB__ && !( __ENABLE_SAMPLE_APP__ )

/******************************************************************************
 * Local static functions
 ******************************************************************************/
#if defined ( __SUPPORT_WIFI_CONN_CB__ ) && !defined ( __ENABLE_SAMPLE_APP__ )
static void user_wifi_conn(void *arg);
static void user_wifi_conn_fail(void *arg);
static void user_wifi_disconn(void *arg);
#endif  // __SUPPORT_WIFI_CONN_CB__ && !( __ENABLE_SAMPLE_APP__ )

extern void	tcp_client_sleep2_sample(void *param);
extern void user_start_MQTT_client();
extern void user_terminate_transmit();

/******************************************************************************
 * Local variables
 ******************************************************************************/


/**********************************************************
 * Customer's application thread table
 **********************************************************/

const app_task_info_t    user_apps_table[] = {
/* name, func, stack_size, priority, net_chk_flag, dpm_flag, port_no, run_sys_mode */

/*
 * !!! Caution !!!
 *
 *     User applications should not affect the operation of the Sample code.
 *
 *     Do not remove "__ENABLE_SAMPLE_APP__" feature in this table.
 */

/*  Task Name,      Funtion,                Stack Size,      Task Priority, Network, DPM,  Net Port,  Run Sys_Mode  */
#if defined ( __SUPPORT_WIFI_CONN_CB__ )
  { WIFI_CONN,          user_wifi_conn,             256,    (OS_TASK_PRIORITY_USER+1),      FALSE, FALSE, UNDEF_PORT, RUN_ALL_MODE    },
  { WIFI_CONN_FAIL,     user_wifi_conn_fail,        256,    (OS_TASK_PRIORITY_USER+1),      FALSE, FALSE, UNDEF_PORT, RUN_ALL_MODE    },
  { WIFI_DISCONN,       user_wifi_disconn,          256,    (OS_TASK_PRIORITY_USER+1),      FALSE, FALSE, UNDEF_PORT, RUN_ALL_MODE    },
  { USER_READ_DATA,	    tcp_client_sleep2_sample,   3072,   (OS_TASK_PRIORITY_USER + 3),    FALSE, FALSE, UNDEF_PORT, RUN_ALL_MODE    },
#endif  // __SUPPORT_WIFI_CONN_CB__
    { NULL,    NULL,    0, 0, FALSE, FALSE, UNDEF_PORT, 0    }
};


/*============================================================
 *
 * Customer's applications ...
 *
 *============================================================*/

#if defined ( __SUPPORT_WIFI_CONN_CB__ ) && !defined ( __ENABLE_SAMPLE_APP__ )

///////////////////////////////////////////////////////////////////////////////
////  Customer call-back function to notify WI-Fi connection status
///////////////////////////////////////////////////////////////////////////////

static void user_wifi_conn(void *arg)
{
    DA16X_UNUSED_ARG(arg);

    EventBits_t wifi_conn_ev_bits;

    while (TRUE) {
       wifi_conn_ev_bits = xEventGroupWaitBits(
                                       evt_grp_wifi_conn_notify,
                                       (WIFI_CONN_SUCC_STA | WIFI_CONN_SUCC_SOFTAP),
                                       pdTRUE,
                                       pdFALSE,
                                       WIFI_CONN_NOTI_WAIT_TICK);

       if (wifi_conn_ev_bits & WIFI_CONN_SUCC_STA) {
            xEventGroupClearBits(evt_grp_wifi_conn_notify, WIFI_CONN_SUCC_STA);

            PRINTF("\n### User Call-back : Success to connect Wi-Fi ...\n");

            user_start_MQTT_client();
            
       } else if (wifi_conn_ev_bits & WIFI_CONN_SUCC_SOFTAP) {
            xEventGroupClearBits(evt_grp_wifi_conn_notify, WIFI_CONN_SUCC_SOFTAP);

            PRINTF("\n### User Call-back : Success to connect Wi-Fi ...\n");
       } else {
           // Not proper event
       }
    }

    vTaskDelete(NULL);
}

static void user_wifi_conn_fail(void *arg)
{
    DA16X_UNUSED_ARG(arg);

    EventBits_t wifi_conn_ev_bits;

    while (TRUE) {
       wifi_conn_ev_bits = xEventGroupWaitBits(
                                       evt_grp_wifi_conn_notify,
                                       (WIFI_CONN_FAIL_STA | WIFI_CONN_FAIL_SOFTAP),
                                       pdTRUE,
                                       pdFALSE,
                                       WIFI_CONN_NOTI_WAIT_TICK);

       if (wifi_conn_ev_bits & WIFI_CONN_FAIL_STA) {
            xEventGroupClearBits(evt_grp_wifi_conn_notify, WIFI_CONN_FAIL_STA);

            PRINTF("\n### User Call-back : Failed to connect Wi-Fi ( reason_code = %d ) ...\n", wifi_conn_fail_reason);
       } else if (wifi_conn_ev_bits & WIFI_CONN_FAIL_SOFTAP) {
            xEventGroupClearBits(evt_grp_wifi_conn_notify, WIFI_CONN_FAIL_SOFTAP);

            PRINTF("\n### User Call-back : Failed to connect Wi-Fi ( reason_code = %d ) ...\n", ap_wifi_conn_fail_reason);
       } else {
           // Not proper event
       }
    }

    vTaskDelete(NULL);
}

static void user_wifi_disconn(void *arg)
{
    DA16X_UNUSED_ARG(arg);

    EventBits_t wifi_conn_ev_bits;

    while (TRUE) {
       wifi_conn_ev_bits = xEventGroupWaitBits(
                                       evt_grp_wifi_conn_notify,
                                       (WIFI_DISCONN_STA | WIFI_DISCONN_SOFTAP),
                                       pdTRUE,
                                       pdFALSE,
                                       WIFI_CONN_NOTI_WAIT_TICK);

       if (wifi_conn_ev_bits == WIFI_CONN_FAIL_STA) {
            xEventGroupClearBits(evt_grp_wifi_conn_notify, WIFI_DISCONN_STA);

            PRINTF("\n### User Call-back : Wi-Fi disconnected ( reason_code = %d ) ...\n", wifi_disconn_reason);
       } else if (wifi_conn_ev_bits == WIFI_CONN_FAIL_SOFTAP) {
            xEventGroupClearBits(evt_grp_wifi_conn_notify, WIFI_DISCONN_SOFTAP);

            PRINTF("\n### User Call-back : Disassociated because STA has left ( reason_code = %d ) ...\n", ap_wifi_disconn_reason);
       } else {
           // Not proper event
       }
    }

    vTaskDelete(NULL);
}

#endif // __SUPPORT_WIFI_CONN_CB__ && !( __ENABLE_SAMPLE_APP__ )

/* EOF */
