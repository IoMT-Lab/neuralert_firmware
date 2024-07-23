/**
 ****************************************************************************************
 *
 * @file da16x_wifi_monitor.c
 *
 * @brief Wi-Fi status monitor Module
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


#include "da16x_wifi_monitor.h"

#include "supp_def.h"        // For feature __LIGHT_SUPPLICANT__
#include "asd.h"
#include "da16x_sys_watchdog.h"
#include "da16x_sys_watchdog_ids.h"

// Extern functions
#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
extern EventGroupHandle_t wifi_monitor_event_group;
#endif /*CONFIG_MONITOR_THREAD_EVENT_CHANGE*/

extern QueueHandle_t wifi_monitor_event_q;

// Internal functions
static void wifi_monitor_thread_entry(void *params);

// Variables
static TaskHandle_t wifi_monitor_thread;

int start_wifi_monitor(void)
{

    wifi_monitor_thread = xTaskGetHandle("wifi_ev_mon");

    if (wifi_monitor_thread != NULL && eTaskGetState(wifi_monitor_thread) != eDeleted) {
        PRINTF("wifi_ev_mon is already running.\n");
        return -1;
    }

    xTaskCreate(wifi_monitor_thread_entry,
                "wifi_ev_mon",
                WIFI_MONITOR_STACK_SIZE,
                (void *)NULL,
                WIFI_MONITOR_THREAD_PRIORITY,
                &wifi_monitor_thread);
    return 0;
}

#ifdef __SUPPORT_SIGMA_TEST__
extern HANDLE    uart1;
#endif  /* __SUPPORT_SIGMA_TEST__ */

static void wifi_monitor_thread_entry(void *params)
{
    DA16X_UNUSED_ARG(params);

    int sys_wdog_id = DA16X_SYS_WDOG_ID_DEF_ID;
    UINT    status = 0;
#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
    ULONG    events;
    ULONG    set_flags;
#endif    //!CONFIG_MONITOR_THREAD_EVENT_CHANGE

#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
    wifi_monitor_msg_buf_t *wifi_monitor_ev_q_buf = pvPortMalloc(sizeof( wifi_monitor_msg_buf_t));
#else
    wifi_monitor_msg_buf_t *wifi_monitor_ev_q_buf = NULL;
#endif /*CONFIG_MONITOR_THREAD_EVENT_CHANGE*/

    extern int UART_WRITE(void *handler, void *p_data, UINT32 p_dlen);

    sys_wdog_id = da16x_sys_watchdog_register(pdFALSE);
    if (sys_wdog_id  >= 0) {
        da16x_sys_wdog_id_set_wifi_ev_mon(sys_wdog_id);
    }

#ifdef __SUPPORT_ASD__
    asd_init();
#endif /* __SUPPORT_ASD__ */

#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
    /* Create event group for wifi monitor events. */
    wifi_monitor_event_group = xEventGroupCreate();
#endif // CONFIG_MONITOR_THREAD_EVENT_CHANGE

    /* Create message queue for wifi monitor events. */
    /* fc80211 driver -> Supplicant CMD Buffer MSG Queue */
    wifi_monitor_event_q = xQueueCreate(WIFI_MONITOR_QUEUE_SIZE, 
#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
                                        sizeof(wifi_monitor_msg_buf_t) /*64 bytes*/
#else
                                        sizeof(wifi_monitor_msg_buf_t *)     /*4 bytes*/
#endif // CONFIG_MONITOR_THREAD_EVENT_CHANGE
                                        );

    if (wifi_monitor_event_q == NULL) {
        PRINTF("[%s] Failed to create queue \"wifi_monitor_ev_q\" (0x02%x)\n",
               __func__, status);

        da16x_sys_watchdog_unregister(da16x_sys_wdog_id_get_wifi_ev_mon());
        da16x_sys_wdog_id_set_wifi_ev_mon(DA16X_SYS_WDOG_ID_DEF_ID);

        return;
    }
#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
    set_flags = WIFI_EVENT_SEND_MSG_TO_APP | WIFI_EVENT_SEND_MSG_TO_UART;
#endif    //CONFIG_MONITOR_THREAD_EVENT_CHANGE

    /* Loop to process Wifi Events from DA16X_Supplicant */
    while (pdTRUE) {
        da16x_sys_watchdog_notify(da16x_sys_wdog_id_get_wifi_ev_mon());

        da16x_sys_watchdog_suspend(da16x_sys_wdog_id_get_wifi_ev_mon());

#ifndef CONFIG_MONITOR_THREAD_EVENT_CHANGE
        events = xEventGroupWaitBits(wifi_monitor_event_group,
                                     set_flags,
                                     pdTRUE,
                                     pdFALSE,
                                     portMAX_DELAY);

        da16x_sys_watchdog_notify_and_resume(da16x_sys_wdog_id_get_wifi_ev_mon());

        if ((events & set_flags) == pdFALSE) {
            /* If an error occurs, simply continue the loop.  */
            continue;
        }

        /* TCP session processing */
        if (events & WIFI_EVENT_SEND_MSG_TO_APP) {
            status = xQueueReceive(wifi_monitor_event_q, wifi_monitor_ev_q_buf, portNO_DELAY);

            if (status == pdTRUE) {
                /* Run event handler */
                switch (wifi_monitor_ev_q_buf->cmd) {
                    case WIFI_CMD_SEND_DHCP_RECONFIG:
                        if (wifi_monitor_ev_q_buf->data[0] == '3') {
                            reconfig_net(INF_NET_STATION); /* STATION */
                        }
#if !defined (__LIGHT_SUPPLICANT__)
                        else if (wifi_monitor_ev_q_buf->data[0] == '0') {
                            reconfig_net(P2P_NET_GO); /* P2P_GO */
                        } else if (wifi_monitor_ev_q_buf->data[0] == '1') {
                            reconfig_net(P2P_NET_CLIENT); /* P2P_CLIENT */
                        } else if (wifi_monitor_ev_q_buf->data[0] == '2') {
                            reconfig_net(P2P_NET_DEVICE); /* P2P_DEVICE */
                        }
#endif // !(__LIGHT_SUPPLICANT__)
#ifdef __SUPPORT_MESH__
                        else if (wifi_monitor_ev_q_buf->data[0] == '4') {
                            reconfig_net(MESH_POINT); /* MESH_POINT */
                        }
#endif /* __SUPPORT_MESH__ */
                        break;

                    default:
                        //PRINTF("Unsupport Command: %s\n",
                        //    wifi_monitor_ev_q_buf->data);
                        break;
                    }
                } else if (status == pdFALSE) {
                    set_flags = WIFI_EVENT_SEND_MSG_TO_APP;
                    events = xEventGroupWaitBits(wifi_monitor_event_group,
                                                 set_flags,
                                                 pdTRUE,
                                                 pdFALSE,
                                                 portNO_DELAY);

                    if ((events & set_flags) == pdFALSE) {
                        PRINTF("<%s> get event_flags error (status=%d)>>>\n", __func__, events);
                    }
                }
            }

#ifdef __SUPPORT_SIGMA_TEST__
        /* Login processing */
        if (events & WIFI_EVENT_SEND_MSG_TO_UART) {
            status = xQueueReceive(wifi_monitor_event_q, wifi_monitor_ev_q_buf, portNO_DELAY);

            if (status == pdTRUE) {
                /* Run event handler */
                switch (wifi_monitor_ev_q_buf->cmd) {
#if !defined (__LIGHT_SUPPLICANT__)
                case WIFI_CMD_P2P_READ_AP_STR:
                case WIFI_CMD_P2P_READ_PIN:
                case WIFI_CMD_P2P_READ_MAIN_STR:
                case WIFI_CMD_P2P_READ_GID_STR:
                    UART_WRITE(uart1, (void *)wifi_monitor_ev_q_buf->data, wifi_monitor_ev_q_buf->data_len);
                    break;
#endif // !(__LIGHT_SUPPLICANT__)

                default:
                    PRINTF("Unsupport UART CMD: %s\n",
                           wifi_monitor_ev_q_buf->data_len);
                }
            } else if (status == ER_QUEUE_EMPTY) {
                set_flags = WIFI_EVENT_SEND_MSG_TO_UART;

                events = xEventGroupWaitBits(wifi_monitor_event_group,
                                             set_flags,
                                             pdTRUE,
                                             pdFALSE,
                                             portNO_DELAY);

                if (events & set_flags == pdFALSE) {
                    PRINTF("<%s> Failed to get event_flags (0x02%x)\n", __func__, status);
                }
            }
        }
#endif    /* __SUPPORT_SIGMA_TEST__ */

#else // CONFIG_MONITOR_THREAD_EVENT_CHANGE

        status = xQueueReceive(wifi_monitor_event_q, &wifi_monitor_ev_q_buf, portMAX_DELAY);

        da16x_sys_watchdog_notify_and_resume(da16x_sys_wdog_id_get_wifi_ev_mon());

        if (status == pdTRUE) {

            switch (wifi_monitor_ev_q_buf->cmd) {
            case WIFI_CMD_SEND_DHCP_RECONFIG:
                if (wifi_monitor_ev_q_buf->data[0] == '3') {
                    reconfig_net(INF_NET_STATION); /* STATION */
                }
#ifdef    CONFIG_P2P
                else if (wifi_monitor_ev_q_buf->data[0] == '0') {
                    reconfig_net(P2P_NET_GO); /* P2P_GO */
                } else if (wifi_monitor_ev_q_buf->data[0] == '1') {
                    reconfig_net(P2P_NET_CLIENT); /* P2P_CLIENT */
                } else if (wifi_monitor_ev_q_buf->data[0] == '2') {
                    reconfig_net(P2P_NET_DEVICE); /* P2P_DEVICE */
                }
#endif    /* CONFIG_P2P */
#ifdef __SUPPORT_MESH__
                else if (wifi_monitor_ev_q_buf->data[0] == '4') {
                    reconfig_net(MESH_POINT); /* MESH_POINT */
                }
#endif /* __SUPPORT_MESH__ */

                break;
#ifdef __SUPPORT_SIGMA_TEST__
#ifdef    CONFIG_P2P
            case WIFI_CMD_P2P_READ_AP_STR:
            case WIFI_CMD_P2P_READ_PIN:
            case WIFI_CMD_P2P_READ_MAIN_STR:
            case WIFI_CMD_P2P_READ_GID_STR:
                UART_WRITE(uart1, (void *)wifi_monitor_ev_q_buf->data, wifi_monitor_ev_q_buf->data_len);
                break;
#endif    /* CONFIG_P2P */
#endif    /* __SUPPORT_SIGMA_TEST__ */

            default:
                //PRINTF("Unsupport Command: %s\n",
                //    wifi_monitor_ev_q_buf->data);
                break;
            }

            if (wifi_monitor_ev_q_buf != NULL) {
                vPortFree(wifi_monitor_ev_q_buf);
                wifi_monitor_ev_q_buf = NULL;
            }
        }

#endif // CONFIG_MONITOR_THREAD_EVENT_CHANGE
    }
}

/* EOF */
