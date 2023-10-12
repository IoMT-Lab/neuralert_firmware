/**
 ****************************************************************************************
 *
 * @file app_atcommand_pal.c
 *
 * @brief adaptation layer between platform and at-command driver for Apps at command
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
#if defined (__SUPPORT_AWS_IOT__) || defined(__SUPPORT_AZURE_IOT__)
#include "sdk_type.h"
#include "nvedit.h"
#include "environ.h"
#include "sys_feature.h"
#include "app_dpm_thread.h"
#include "app_common_util.h"
#if defined(__SUPPORT_ATCMD__)
#include "app_atcommand.h"
#endif  /* __SUPPORT_ATCMD__ */
#include "app_atcommand_pal.h"
#include "app_provision.h"
#include "command_net.h"
#include "da16x_network_common.h"
#include "app_dpm_interface.h"
#include "app_thing_manager.h"
#if defined(__SUPPORT_AWS_IOT__)
#include "app_aws_user_conf.h"
#include "shadow.h"
#endif  /* __SUPPORT_AWS_IOT__ */
#if defined(__SUPPORT_AZURE_IOT__)
#include "../../../azure/src/app_azure_iot/core/deps/parson/parson.h"
#endif  /* __SUPPORT_AZURE_IOT__ */

#ifndef APP_UNUSED_ARG
#define APP_UNUSED_ARG(x)                   (void)x
#endif /* DA16X_UNUSED_ARG */

#if defined(__SUPPORT_ATCMD__)
#define Cmd_Idle                            0xFF
#define MAX_RESPONSE_WAIT_COUNT             600

#if defined (__SUPPORT_AWS_IOT__)
#define DEFAULT_SHADOW_NAME                 SHADOW_NAME_CLASSIC
#define SHADOW_NAME_LENGTH    ( ( uint16_t ) ( sizeof( DEFAULT_SHADOW_NAME ) - 1 ) )
#define MAX_PUBLISH_PAYLOAD_LEN             512
#define MAX_SUBSCRIBE_TOPIC_LEN             256
#endif  /* __SUPPORT_AWS_IOT__ */
#if defined (__SUPPORT_AZURE_IOT__)
#define RESP_STR_SIZE                       32
#endif  /* __SUPPORT_AZURE_IOT__ */

struct atcmd_nvparams atnvParam;
static int8_t userCommand = Cmd_Idle;
int8_t set_atnvParam = 0;
int8_t mcu_wakeup_port = -1;
uint16_t mcu_wakeup_pin = 0;
int8_t set_own = 0;
static char atBuf[256] = { 0, };

extern jsonStruct_t registeredShadow[MAX_SHADOW_COUNT];
extern int32_t gShadowCount;

#if defined (__SUPPORT_AWS_IOT__)
static char payload_buf[MAX_PUBLISH_PAYLOAD_LEN] = { 0, };
static char topic_buf[MAX_SUBSCRIBE_TOPIC_LEN] = { 0, };
#endif  /* __SUPPORT_AWS_IOT__ */
#if defined (__SUPPORT_AZURE_IOT__)
struct atcmd_twin_params atcmd_twins;
char resp_string[RESP_STR_SIZE];
#endif  /* __SUPPORT_AZURE_IOT__ */
#endif  /* __SUPPORT_ATCMD__ */
#if defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AWS_IOT__)
static void pal_app_making_shadow_payload(char *payload, uint32_t lpayload);
#endif  /* __SUPPORT_ATCMD__ && __SUPPORT_AWS_IOT__*/
void wait_MCU_resp(uint8_t owner)
{
#if defined(__SUPPORT_ATCMD__)
    set_own = owner;
#else
    DA16X_UNUSED_ARG(owner);
#endif  /* __SUPPORT_ATCMD__ */
}

uint8_t get_MCU_resp(void)
{
#if defined(__SUPPORT_ATCMD__)
    return set_own;
#else
    return 0;
#endif  /* __SUPPORT_ATCMD__ */
}

void pal_app_get_def_local_port(uint32_t *port)
{
#if defined(__SUPPORT_ATCMD__)
    uint32_t my_local_port = 0;

    if (read_nvram_int(APP_NVRAM_CONFIG_LPORT, (int*)&my_local_port)) {
#if defined(__SUPPORT_AWS_IOT__)
        *port = my_local_port = AWS_MQTT_PORT;
#elif defined(__SUPPORT_AZURE_IOT__)
        *port = my_local_port = AZURE_MQTT_PORT;
#endif  /* __SUPPORT_AWS_IOT__ */
        if (write_nvram_int(APP_NVRAM_CONFIG_LPORT, my_local_port))
            APRINTF_E("write local port nvram: failed\n");
    } else {
        *port = my_local_port;
    }
#else
#if defined(__SUPPORT_AWS_IOT__)
    *port = AWS_MQTT_PORT;
#elif defined(__SUPPORT_AZURE_IOT__)
    *port = AZURE_MQTT_PORT;
#else
    DA16X_UNUSED_ARG(port);
#endif  /* __SUPPORT_AWS_IOT__ */
#endif  /* __SUPPORT_ATCMD__ */
}

char* pal_app_get_cert_ca(char *cert)
{
#if defined(__SUPPORT_ATCMD__)
    pal_app_get_atcmd_parameter();
    return atnvParam.certCA;
#else
    return cert;
#endif  /* __SUPPORT_ATCMD__ */
}

uint8_t pal_app_get_atcmd_parameter(void)
{
#if defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AWS_IOT__)
    char *nvp;
    uint32_t port;
    uint32_t fpstatus = getUseFPstatus();

    if (set_atnvParam)
        return 1;
    nvp = read_nvram_string(AWS_NVRAM_CONFIG_BROKER_URL);
    if (nvp != NULL) {
        APRINTF_I("pal_app_get_atcmd_parameter BROKER(%s) : (%d)\n", nvp, strlen(nvp));
        atnvParam.broker = APP_MALLOC(strlen(nvp) + 1);
        if (atnvParam.broker == NULL) {
            APRINTF_E("broker malloc: failed\n");
            goto init_error;
        }
        memset(atnvParam.broker, 0x00, strlen(nvp) + 1);
        memcpy(atnvParam.broker, nvp, strlen(nvp));
    } else {
        atnvParam.broker = APP_MALLOC(strlen(democonfigMQTT_BROKER_ENDPOINT) + 1);
        if (atnvParam.broker == NULL) {
            APRINTF_E("broker malloc: failed\n");
            goto init_error;
        }
        memset(atnvParam.broker, 0x00, strlen(democonfigMQTT_BROKER_ENDPOINT) + 1);
        memcpy(atnvParam.broker, democonfigMQTT_BROKER_ENDPOINT,
            strlen(democonfigMQTT_BROKER_ENDPOINT));
    }

    if (read_nvram_int(AWS_NVRAM_CONFIG_PORT, (int*)&port)) {
        port = AWS_CONFIG_PORT_DEF;
        if (write_nvram_int(AWS_NVRAM_CONFIG_PORT, port)) {
            APRINTF_E(" write aws broker port: failed\n");
            goto init_error;
        }
    }
    atnvParam.port = port;
    APRINTF_I("pal_app_get_atcmd_parameter Port (%d)\n", port);

    atnvParam.certCA = APP_MALLOC(CERT_MAX_LENGTH);
    if (atnvParam.certCA == NULL) {
        APP_FREE(atnvParam.broker);
        APRINTF_E("NV certCA malloc: failed\n");
        goto init_error;
    }

    memset(atnvParam.certCA, 0x00, CERT_MAX_LENGTH);
    cert_flash_read(SFLASH_ROOT_CA_ADDR1, atnvParam.certCA, CERT_MAX_LENGTH);

    if (!fpstatus) {
        atnvParam.cert = APP_MALLOC(CERT_MAX_LENGTH);
        if (atnvParam.cert == NULL) {
            APP_FREE(atnvParam.broker);
            APP_FREE(atnvParam.certCA);
            APRINTF_E("NV cert malloc: failed\n");
            goto init_error;
        }
        atnvParam.key = APP_MALLOC(CERT_MAX_LENGTH);
        if (atnvParam.key == NULL) {
            APP_FREE(atnvParam.broker);
            APP_FREE(atnvParam.certCA);
            APP_FREE(atnvParam.cert);
            APRINTF_E("NV key malloc: failed\n");
            goto init_error;
        }
        memset(atnvParam.cert, 0x00, CERT_MAX_LENGTH);
        memset(atnvParam.key, 0x00, CERT_MAX_LENGTH);
        cert_flash_read(SFLASH_CERTIFICATE_ADDR1, atnvParam.cert, CERT_MAX_LENGTH);
        cert_flash_read(SFLASH_PRIVATE_KEY_ADDR1, atnvParam.key, CERT_MAX_LENGTH);
    }

    set_atnvParam = 1;
    return 0;
    init_error:
    APRINTF_E("AT initialization failed\n");
    return 1;
#elif defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AZURE_IOT__)
    uint32_t port;

    if (set_atnvParam)
        return 1;

    if (read_nvram_int(AZURE_NVRAM_CONFIG_PORT, (int *)&port)) {
        port = AZURE_CONFIG_PORT_DEF;
        if (write_nvram_int(AZURE_NVRAM_CONFIG_PORT, port)) {
            APRINTF_E(" write Azure server port: failed\n");
            goto init_error;
        }
    }
    atnvParam.port = port;
    APRINTF_I("pal_app_get_atcmd_parameter Port (%d)\n", port);

    atnvParam.certCA = APP_MALLOC(CERT_MAX_LENGTH);
    if (atnvParam.certCA == NULL) {
        APP_FREE(atnvParam.broker);
        APRINTF_E("NV certCA malloc: failed\n");
        goto init_error;
    }

    memset(atnvParam.certCA, 0x00, CERT_MAX_LENGTH);
    cert_flash_read(SFLASH_ROOT_CA_ADDR1, atnvParam.certCA, CERT_MAX_LENGTH);

    set_atnvParam = 1;
    return 0;
init_error:
    APRINTF_E("AT initialization failed\n");
    return 1;
#endif  /* __SUPPORT_ATCMD__ */
    return 0;
}

/**
 ****************************************************************************************
 * @brief Network connection check with ping
 * @param[in] ipaddrstr ping ip address
 * @return  void
 ****************************************************************************************
 */
#if defined(__SUPPORT_ATCMD__)
int8_t pal_app_check_internet_with_ping(char *ipaddrstr)
{
    UINT8 result_str[128] = { 0, };
    INT32 len = 0;
    INT32 wait = 0;
    INT32 interval = 0;
    INT32 ping_interface = 0; /* default wlan0 */
    UINT32 ipaddr = 0;
    UINT32 count = 0;
    UINT32 average, time_min, ping_max, transmitted, reply_count;

    extern unsigned int da16x_ping_client(int, char*, unsigned long, unsigned long*
        , unsigned long*, int, unsigned long, int, int, int, char*);

    /* result string */
    transmitted = reply_count = 0;
    average = time_min = ping_max = 0;
    ipaddr = (uint32_t)iptolong(ipaddrstr);

    count = 4; /* only 2 */
    len = 32; /* default */
    wait = 8000; /* default 4sec */
    interval = 1000; /* interval default 1sec */

    /* If station interface */
    ping_interface = 0; /* WLAN0_IFACE; */

    /* ping client api execution with nodisplay as 1 and getting the string of result */
    da16x_ping_client(ping_interface, NULL, ipaddr, NULL, NULL, len, count, wait,
        interval, 1, (char*)result_str);

    /* parsing the result string */
    sscanf((char*)result_str, "%u,%u,%u,%u,%u", &transmitted, &reply_count, &average,
        &time_min, &ping_max);

    if (reply_count > 0) { /* Success */
        APRINTF_I("Ping reply is ok\n");
        return 1;
    }
    APRINTF_I("Ping reply is fail\n");
    return 0;
}
#endif

void pal_app_check_ping(void)
{
#if defined (__SUPPORT_ATCMD__)
    int pingCheck;
    /* Check Ping */
    read_nvram_int(APP_NVRAM_CONFIG_PING_CHECK, &pingCheck);

    if (pingCheck != 1) {
        if (pal_app_check_internet_with_ping(DEFAULT_PING_IP) == 1) {
            APRINTF_I("[ Ping-check] OK \n\n");
            write_nvram_int(APP_NVRAM_CONFIG_PING_CHECK, 1);
        } else {
            APRINTF_I("[ Ping-check] fail \n\n");
            /* need factory reset */
            app_iot_at_command_status(ATCMD_Status_network_Fail);

            /* Wait for system-reboot */
            while (1) {
                vTaskDelay(10);
            }
        }
    }
#endif
    return;
}

void pal_app_dpm_auto_start(void)
{
#if defined(__SUPPORT_ATCMD__)
    int sysmode;
    char *tmp_read = NULL;
    char user[32] = { 0, };

    /* MCU wake up port and pin check */
    tmp_read = read_nvram_string(NV_MCU_WAKEUP_PORT);
    if (tmp_read != NULL) {
        strncpy(user, tmp_read, strlen(tmp_read));
        mcu_wakeup_port = (int8_t)app_atcmd_mcu_wakeup_port((char*)user);
    }

    memset(user, 0, 32);
    tmp_read = read_nvram_string(NV_MCU_WAKEUP_PIN);

    if (tmp_read != NULL) {
        strncpy(user, tmp_read, strlen(tmp_read));
        mcu_wakeup_pin = app_atcmd_mcu_wakeup_pin((char*)user);
    }

    APRINTF_I("\r\n[%s] mcu_wakeup_port=%d, mcu_wakeup_pin=0x%x \n\n"
        , __func__, mcu_wakeup_port, mcu_wakeup_pin);

    /* AWS + ATCMD, default set */
    if (mcu_wakeup_port < 0 || mcu_wakeup_pin <= 0) {
        if (mcu_wakeup_port < 0)
            mcu_wakeup_port = GPIO_UNIT_A;
        if (mcu_wakeup_pin <= 0)
            mcu_wakeup_pin = GPIO_PIN11;
        APRINTF_I("default set to mcu_wakeup_port=%d, mcu_wakeup_pin=0x%x \n\n", mcu_wakeup_port, mcu_wakeup_pin);
    }

    /* check whether configuration value OK or not */
    if (!app_at_feature_is_valid()) {
        APRINTF_E("\ninvalid APP feature...can't start APP Platform thread...check again\n\n");
        if (dpm_mode_is_enabled()) {
            dpm_mode_disable();
            /* save provisioning flag for DPM mode */
            write_nvram_int(APP_NVRAM_CONFIG_DPM_AUTO, 1);
            APRINTF_E("\nDPM mode is disabled\n");
        }

        app_iot_at_command_status(ATCMD_Status_need_configuration);
        vTaskDelete(NULL);
        return;
    } else {
        sysmode = getSysMode();
        if (sysmode != SYSMODE_AP_ONLY) {
            if (!dpm_mode_is_enabled()) {
                int dpm_auto;
                read_nvram_int(APP_NVRAM_CONFIG_DPM_AUTO, &dpm_auto);
                if (dpm_auto == 1) {
                    dpm_mode_enable();
                    reboot_func(SYS_REBOOT_POR);

                    /* Wait for system-reboot */
                    while (1)
                        vTaskDelay(10);
                }
            }
        }
    }
#endif  /* __SUPPORT_ATCMD__ */
    return;
}

void pal_app_start_provisioning(int32_t mode)
{
#if defined (__SUPPORT_ATCMD__)
#if defined (__SUPPORT_AWS_IOT__)
    if (mode == AWS_MODE_FPGEN)
        app_start_provisioning(AWS_MODE_FPATCMD);
    else
        app_start_provisioning(AWS_MODE_ATCMD);
#elif defined (__SUPPORT_AZURE_IOT__)
    app_start_provisioning(AZURE_MODE_ATCMD);
#endif
#else
    app_start_provisioning(mode);
#endif  /* __SUPPORT_ATCMD__ && __SUPPORT_AWS_IOT__ */
    return;
}

void pal_app_at_command_status(ATCMDScanResult _val)
{
#if defined(__SUPPORT_ATCMD__)
    app_iot_at_command_status(_val);
#else
    DA16X_UNUSED_ARG(_val);
#endif  /* __SUPPORT_ATCMD__ */
}

void pal_app_set_prov_feature(void)
{
#if defined (__SUPPORT_ATCMD__)
    set_prov_feature(APP_AT_CMD);
    APRINTF_I("[ APP-IOT AT-COMMAND ] \n\n");
#elif defined(__SUPPORT_SENSOR_REF__)
    APRINTF_S("[ APP-IOT Sensors ] \n\n");
    set_prov_feature(APP_SENSOR);
#else
    set_prov_feature(APP_DOORLOCK);
    APRINTF_S("[ APP-IOT Doorlock ] \n\n");
#endif  /* __SUPPORT_ATCMD__ */
    return;
}

int8_t pal_app_check_is_update_mcu(char *_url, OTA_UPDATE_CONFIG *config)
{
#if defined (__SUPPORT_ATCMD__)
    char *img_name = NULL;
    char *str;

    memset(atBuf, 0x00, 128);
    memcpy(atBuf, _url, strlen(_url));

    str = strtok(atBuf, (char*)"/");
    do {
        if (str == NULL) {
            APRINTF_I("OTA url is NULL break \n");
            break;
        }
        img_name = str;
    } while ((str = strtok(NULL, (char*)"/")) != NULL);
    if (strncasecmp(img_name, "MCU", 3) == 0) {
        config->update_type = OTA_TYPE_MCU_FW_STREAM;
        config->download_sflash_addr = 0;
        pal_app_at_command_status(ATCMD_Status_MCUOTA);
#if (SDK_MAJOR == 3) && (SDK_MINOR == 2) && (SDK_REVISION >= 5)
        memcpy(config->url, _url, strlen(_url));
        APRINTF_I(" %s url %s\n", "MCU_Stream", config->url);
#else
        memcpy(config->uri, _url, strlen(_url));
        APRINTF_I(" %s url %s\n", "MCU_Stream", config->uri);
#endif
        return 1;
    } else {
        return 0;
    }
#else
    DA16X_UNUSED_ARG(_url);
    DA16X_UNUSED_ARG(config);
    return 0;
#endif
}

int8_t pal_app_notify_mcu_ota(ota_update_type update_type, uint32_t status, uint32_t progress, uint32_t renew)
{
    APP_UNUSED_ARG(progress);
#if defined (__SUPPORT_ATCMD__)
    if (update_type != OTA_TYPE_MCU_FW_STREAM)
        return 0;
    if ((status == OTA_SUCCESS)) {
        pal_app_at_command_status(ATCMD_Status_STA_done);
#if defined (__SUPPORT_AWS_IOT__)
        if (write_nvram_int(AWS_NVRAM_CONFIG_OTA_RESULT, AWS_OTA_RESULT_OK))
            APRINTF_I("write AWS_NVRAM_CONFIG_OTA_RESULT failed\n");
#elif defined(__SUPPORT_AZURE_IOT__)
        if (write_nvram_int(AZURE_NVRAM_CONFIG_OTA_RESULT, AZURE_OTA_RESULT_OK))
            APRINTF_I("write AZURE_NVRAM_CONFIG_OTA_RESULT failed\n");
#endif
        vTaskDelay(1);
        APRINTF_I("Succeeded to replace with new MCU FW.\n");
        reboot_func(SYS_REBOOT);
    } else {
#if defined (__SUPPORT_AWS_IOT__)
        if (write_nvram_int(AWS_NVRAM_CONFIG_OTA_RESULT, AWS_OTA_RESULT_NG))
            APRINTF_I("write AWS_NVRAM_CONFIG_OTA_RESULT failed\n");
#elif defined(__SUPPORT_AZURE_IOT__)
        if (write_nvram_int(AZURE_NVRAM_CONFIG_OTA_RESULT, AZURE_OTA_RESULT_NG))
            APRINTF_I("write AZURE_NVRAM_CONFIG_OTA_RESULT failed\n");
#endif
        vTaskDelay(1);
        APRINTF_I("Failed to replace with new FW. (Err : 0x%02x)\n", status);
        if (renew == 0) {
            APRINTF_I(">>> System Reboot !!!\n");
            reboot_func(SYS_REBOOT);
        }
    }
    return 1;
#else
    APP_UNUSED_ARG(update_type);
    APP_UNUSED_ARG(status);
    APP_UNUSED_ARG(renew);
    return 0;
#endif
}

uint8_t pal_app_controlCommand(app_dpm_info_rtm *_rtmData)
{
#if defined (__SUPPORT_ATCMD__)
    int32_t subsCnt;
    uint32_t wait_cnt = 0;
    char *atSubsCmd = NULL;
    APP_UNUSED_ARG(_rtmData);

    subsCnt = app_at_get_registered_subscribe_count();
    if ((subsCnt > 0) && (subsCnt > userCommand)) {
        atSubsCmd = app_at_get_cloud_cmd_payload(userCommand);
        APRINTF_I("\r\nsend \"%s %s\" to MCU\r\n", CMD_SERVER_DATA, atSubsCmd);
        wait_MCU_resp(1);
        if ((mcu_wakeup_port >= 0) && (mcu_wakeup_pin > 0x00))
            atcmd_mcu_wakeup(mcu_wakeup_port, mcu_wakeup_pin);
        memset(atBuf, 0, sizeof(atBuf));
        da16x_sprintf(atBuf, "\r\n+%sIOT=%s %s\r\n", PLATFORM, CMD_SERVER_DATA, atSubsCmd);
        vTaskDelay(5);
        APP_PRINTF_ATCMD(atBuf, RETRY_ACK_CHECK_SKIP);
        while (get_MCU_resp() == 1) {
            if (wait_cnt > MAX_RESPONSE_WAIT_COUNT) {
                APRINTF_I("\r\n [%s] break loop out(=%d) \r\n", __func__, wait_cnt);
                break;
            }
            vTaskDelay(1);
            wait_cnt++;
        }
    }
    return 1;
#else
    APP_UNUSED_ARG(_rtmData);
    return 0;
#endif  /* __SUPPORT_ATCMD__ */
}

int8_t pal_app_atc_work(app_dpm_info_rtm *_rtmData)
{
#if defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AWS_IOT__)
    MQTTStatus_t rc = app_get_mqtt_status();
    uint32_t wait_cnt = 0;
    BaseType_t xStatus = pdPASS;
    app_dpm_info_rtm *data = (app_dpm_info_rtm*)_rtmData;

    if (rc != MQTTSuccess)
        xStatus = pdFAIL;

    APRINTF_I("*************************************************************************"
        "**************\n");

    APRINTF_I("\r\nsend \"%s %s\" to MCU\r\n", CMD_TO_MCU, APP_CONTROL_ATUPDATE_PREFIX);
    wait_MCU_resp(2); /* shadow response wait */
    if ((mcu_wakeup_port >= 0) && (mcu_wakeup_pin > 0x00))
        atcmd_mcu_wakeup((uint8_t)mcu_wakeup_port, mcu_wakeup_pin);

    memset(atBuf, 0, sizeof(atBuf));
    da16x_sprintf(atBuf, "\r\n+AWSIOT=%s %s\r\n", CMD_TO_MCU, APP_CONTROL_ATUPDATE_PREFIX);
    vTaskDelay(5);
    APP_PRINTF_ATCMD(atBuf, RETRY_ACK_CHECK_SKIP);
    while (get_MCU_resp() == 2) {
        if (wait_cnt > MAX_RESPONSE_WAIT_COUNT) {
            APRINTF_I("\r\n [%s] break loop out(=%d) \r\n", __func__, wait_cnt);
            break;
        }
        vTaskDelay(1);
        wait_cnt++;
    }

    APRINTF_I("*************************************************************************"
        "**************\n");

    if (sizeof(DEFAULT_SHADOW_NAME) > 1)
        da16x_sprintf(topic_buf, "%s%s%s%s%s%s", SHADOW_PREFIX, app_get_registered_thing_name()
            , SHADOW_NAMED_ROOT, DEFAULT_SHADOW_NAME, SHADOW_OP_UPDATE, "");
    else
        da16x_sprintf(topic_buf, "%s%s%s%s%s", SHADOW_PREFIX, app_get_registered_thing_name()
            , SHADOW_CLASSIC_ROOT, SHADOW_OP_UPDATE, "");

    pal_app_making_shadow_payload(payload_buf, sizeof(payload_buf));

    xStatus = xPublishToTopic(app_get_mqtt_context(),
        topic_buf,
        strlen(topic_buf),
        payload_buf,
        strlen(payload_buf));
    if (xStatus != pdPASS) {
        APRINTF_E("publish (shadow sensor update) NG\n");
        rc = MQTTSendFailed;
    } else {
        APRINTF_I("publish (shadow sensor update) OK -  payload: \"%s\"\n", payload_buf);
    }

    if (data && (xStatus == pdPASS)) {
        char *ota_url = app_get_ota_url();

        data->FOTAStat = (uint32_t)getOTAStat();
        if (getOTAStat() == (uint32_t)OTA_STAT_JOB_READY && strlen(ota_url) > 0)
            da16x_sprintf((char*)data->FOTAUrl, "%s", ota_url);
        else if (getOTAStat() == (INT32)OTA_STAT_JOB_CONFIRMED && strlen(ota_url) > 0)
            memset(data->FOTAUrl, 0, sizeof(data->FOTAUrl));

        APRINTF_I("last user Timer ID = %d\n", data->tid);
        APRINTF_I("last doorOpenFlag state: \"%s\"\n", data->doorOpen ? "true" : "false");
        APRINTF_I("last FOTA Stat: %d\n", getOTAStat());
        /* check printable code */
        if (ota_url[0] > 0x7F)
            APRINTF_I("last FOTA Url: \"???\"\n");
        else
            APRINTF_I("last FOTA Url: \"%s\"\n", ota_url);
    }

    if (rc != MQTTSuccess) {
        APRINTF_I("[%s] reconnecting...\n", __func__);
        app_aws_dpm_app_connect(data);
    }

    return 1;
#elif defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AZURE_IOT__)
    uint32_t wait_cnt = 0;

    if (userCommand != Cmd_Idle)
        APRINTF_I("user CMD (=%d: 255-idle, 0-off, 1-on)\n", userCommand);

    /* first - send "update" cmd to MCU */
    APRINTF_I("\r\nsend \"%s %s\" to MCU\r\n", CMD_TO_MCU, APP_CONTROL_ATUPDATE_PREFIX);

    wait_MCU_resp(2);       /* shadow response wait */

    if ((mcu_wakeup_port >= 0) && (mcu_wakeup_pin > 0x00))
        atcmd_mcu_wakeup((uint8_t)mcu_wakeup_port, mcu_wakeup_pin);

    memset(atBuf, 0, sizeof(atBuf));
    da16x_sprintf(atBuf, "\r\n+AZUIOT=%s %s\r\n", CMD_TO_MCU, APP_CONTROL_ATUPDATE_PREFIX);
    vTaskDelay(5);

#if defined(AT_ACK_CHECK)
    APP_PRINTF_ATCMD(atBuf, RETRY_ACK_CHECK);
#else
    APP_PRINTF_ATCMD(atBuf, RETRY_ACK_CHECK_SKIP);
#endif

    while (get_MCU_resp() == 2) {
        if (wait_cnt > MAX_RESPONSE_WAIT_COUNT) {
            APRINTF_I("\r\n [%s] break loop out(=%d) \r\n", __func__, wait_cnt);
            break;
        }
        vTaskDelay(1);
        wait_cnt++;
    }

    APRINTF_I("\n======================================================================="
    "================\n");
    return 1;
#else
    APP_UNUSED_ARG(_rtmData);
    return 0;
#endif  /* __SUPPORT_ATCMD__ && __SUPPORT_AWS_IOT__ */
}

int8_t pal_is_support_mcu_update(void)
{
#if defined (__SUPPORT_ATCMD__)
    return 1;
#else
    return 0;
#endif
}

#if defined(__SUPPORT_AWS_IOT__)
char* pal_app_get_broker_endpoint(char *broker)
{
#if defined(__SUPPORT_ATCMD__)
    pal_app_get_atcmd_parameter();
    return atnvParam.broker;
#else
    return broker;
#endif  /* __SUPPORT_ATCMD__ */
}

uint32_t pal_app_get_broker_port(uint32_t port)
{
#if defined(__SUPPORT_ATCMD__)
    pal_app_get_atcmd_parameter();
    return atnvParam.port;
#else
    return port;
#endif  /* __SUPPORT_ATCMD__ */
}

char* pal_app_get_cert_cert(char *cert)
{
#if defined(__SUPPORT_ATCMD__)
    pal_app_get_atcmd_parameter();
    return atnvParam.cert;
#else
    return cert;
#endif  /* __SUPPORT_ATCMD__ */
}

char* pal_app_get_cert_key(char *key)
{
#if defined(__SUPPORT_ATCMD__)
    pal_app_get_atcmd_parameter();
    return atnvParam.key;
#else
    return key;
#endif  /* __SUPPORT_ATCMD__ */
}

int8_t pal_app_event_cb(char *payload, int32_t len, bool *connect, bool *control,
bool *updatesensor)
{
#if defined (__SUPPORT_ATCMD__)
    int32_t subItemCnt = 0;
    int i;
    APRINTF_I("pal_app_event_cb %s\n", payload);

    if (strncmp(payload, AWS_CONTROL_OTA, strlen(AWS_CONTROL_OTA)) == 0) {
        APRINTF_I("confirmOTA comm\n");
        setOTAStat(OTA_STAT_JOB_CONFIRMED);
        userCommand = Cmd_Idle;
    } else if (strncmp(payload, AWS_CONTROL_CONNECT, strlen(AWS_CONTROL_CONNECT))
        == 0 && *connect == false) {
        APRINTF_I("connect comm\n");
        *connect = true;
        userCommand = Cmd_Idle;
    } else if (strncmp(payload, AWS_CONTROL_SENSOR, strlen(AWS_CONTROL_SENSOR)) == 0) {
        APRINTF_I("updateSensor comm\n");
        *updatesensor = true;
        userCommand = Cmd_Idle;
    } else {
        subItemCnt = app_at_get_registered_subscribe_count();
        if (subItemCnt > 0) {
            for (i = 0; i < subItemCnt; i++) {
                char *atRegisteredSubscribe;
                char *foundStr = NULL;
                atRegisteredSubscribe = app_at_get_registered_subscribe(i);
                if (strlen(atRegisteredSubscribe) > 0) {
                    foundStr = strstr(payload, atRegisteredSubscribe);
                    if (foundStr && (*control == false)) {
                        APRINTF_I("dynamic subscription command (\"%s\")\n", atRegisteredSubscribe);
                        app_at_set_cloud_cmd_payload(payload, len);
                        *control = true;
                        userCommand = i;
                        break;
                    }
                }
            }
        }
    }
    return 1;
#else
    APP_UNUSED_ARG(payload);
    APP_UNUSED_ARG(len);
    APP_UNUSED_ARG(connect);
    APP_UNUSED_ARG(control);
    APP_UNUSED_ARG(updatesensor);
    return 0;
#endif  /* __SUPPORT_ATCMD__ */
}

void pal_app_publish_user_contents(char *_payload)
{
#if defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AWS_IOT__)
    MQTTStatus_t rc = app_get_mqtt_status();
    BaseType_t xStatus = pdPASS;
    char *pub_topic;
    int k = 0;
    char *params[10] = { 0, };

    memset(payload_buf, 0, sizeof(payload_buf));
    memset(topic_buf, 0, sizeof(topic_buf));
    memcpy(payload_buf, _payload, strlen(_payload));

    pub_topic = read_nvram_string(APP_NVRAM_CONFIG_PTOPIC);
    da16x_sprintf(topic_buf, "%s%s", app_get_registered_thing_name(), pub_topic);

    params[k] = strtok(payload_buf, " ");

    while (params[k] != NULL) {
        /* APRINTF("%s\n", params[k]); */
        k++;
        params[k] = strtok(NULL, " ");
    }

    xStatus = xPublishToTopic(app_get_mqtt_context(),
        topic_buf, strlen(topic_buf),
        _payload, strlen(_payload));

    if (xStatus != pdPASS) {
        APRINTF_E("publish (contents update) NG - topic : %s payload: %s\n", topic_buf, _payload);
        rc = MQTTSendFailed;
    } else {
        APRINTF_I("publish (contents update) OK - topic : %s payload: %s\n", topic_buf, _payload);
        rc = MQTTSuccess;
    }

    app_set_mqtt_status(rc);
#else
    APP_UNUSED_ARG(_payload);
#endif
    return;
}

int8_t pal_app_reg_subscription_topic(void)
{
#if defined (__SUPPORT_ATCMD__)
    BaseType_t xStatus = pdPASS;
    char *sub_topic;

    memset(topic_buf, 0, sizeof(topic_buf));
    sub_topic = read_nvram_string(APP_NVRAM_CONFIG_STOPIC);
    da16x_sprintf(topic_buf, "%s%s", app_get_registered_thing_name(), sub_topic);
    xStatus = xSubscribeToTopic(app_get_mqtt_context()
        , topic_buf
        , strlen(topic_buf));
    if (xStatus == pdPASS)
        return 1;
    else
        return 2;
#else
    return 0;
#endif  /* __SUPPORT_ATCMD__ */
}
#endif  /* __SUPPORT_AWS_IOT__ */

#if defined (__SUPPORT_AZURE_IOT__)
int8_t pal_app_event_cb(const unsigned char *payload, size_t size, unsigned char **response, size_t *response_size,
    int32_t *result, bool *connect_flag)
{
#if defined (__SUPPORT_ATCMD__)
    uint32_t resPayloadLen = 0;
    char resPayload[32] = { 0, };
    int32_t subItemCnt = 0;
    uint8_t i;

    if (strncmp((char*)payload + 1, AZURE_CONTROL_OTA, strlen(AZURE_CONTROL_OTA)) == 0) {
        APRINTF_I("confirmOTA comm\n");

        setOTAStat(OTA_STAT_JOB_CONFIRMED);
        if (write_nvram_int(AZURE_NVRAM_CONFIG_OTA_STATE, AZURE_OTA_STATE_UNKOWN))
            APRINTF_E("write AZURE_NVRAM_CONFIG_OTA_STATE failed");
        userCommand = Cmd_Idle;
        *result = 1;
    } else if (strncmp((char*)payload + 1, AZURE_CONTROL_CONNECT, strlen(AZURE_CONTROL_CONNECT))
        == 0) {
        APRINTF_I("connect comm\n");
        if (*connect_flag) {
            *result = -1;
            return 1;
        }
        pal_app_atc_work(NULL);
        resPayloadLen = strlen(AZURE_RESPONSE_CONNECT) + 2;
        snprintf(resPayload, resPayloadLen + 1, "\"%s\"", AZURE_RESPONSE_CONNECT);

        *response_size = strlen(resPayload);
        *response = APP_MALLOC(*response_size);
        (void)memcpy(*response, resPayload, *response_size);
        *result = 200;

        atcmd_twins.doorlock_state = Door_Idle;
        *connect_flag = TRUE;
        userCommand = Cmd_Idle;
    } else {
        subItemCnt = app_at_get_registered_subscribe_count();
        if (subItemCnt > 0) {
            for (i = 0; i < subItemCnt; i++) {
                char *atRegisteredSubscribe;
                char *foundStr = NULL;
                atRegisteredSubscribe = app_at_get_registered_subscribe(i);
                if (strlen(atRegisteredSubscribe) > 0) {
                    foundStr = strstr((char*)payload, atRegisteredSubscribe);
                    if (foundStr && (app_get_control_flag() == FALSE)) {
                        APRINTF_I("dynamic subscription command (\"%s\")", atRegisteredSubscribe);
                        app_at_set_cloud_cmd_payload((char*)payload + 1, (int32_t)size - 2);
                        app_set_control_flag(TRUE);
                        userCommand = i; /* set index */
                        break;
                    }
                }
            }
        }
        if (app_get_control_flag() == TRUE) {
            char *resp = NULL;
            app_at_set_resp_string(NULL, 0);
            pal_app_controlCommand(NULL);
            if (resp_string[0] != 0)
                resp = resp_string;
            else
                resp = NULL;
            if (resp != NULL) {
                resPayloadLen = strlen(resp) + 2;
                snprintf(resPayload, resPayloadLen + 1, "\"%s\"", resp);

                *response_size = strlen(resPayload);
                *response = APP_MALLOC(*response_size);
                (void)memcpy(*response, resPayload, *response_size);
                *result = 200;
            } else {
                APRINTF_I("unsupported comm\n");
                const char deviceMethodResponse[] = "{ }";
                *response_size = sizeof(deviceMethodResponse) - 1;
                *response = APP_MALLOC(*response_size);
                (void)memcpy(*response, deviceMethodResponse, *response_size);

                *result = -1;
            }
            userCommand = Cmd_Idle;
            app_set_control_flag(FALSE);
            pal_app_atc_work(NULL);
        } else {
            *result = -1;
        }
    }

    APRINTF_I("Command response : %.*s \r\n", *response_size, *response);

    if (*result != -1)
        app_send_twins("method callback");
    return 1;
#else
    APP_UNUSED_ARG(payload);
    APP_UNUSED_ARG(size);
    APP_UNUSED_ARG(response);
    APP_UNUSED_ARG(response_size);
    APP_UNUSED_ARG(result);
    APP_UNUSED_ARG(connect_flag);
    return 0;
#endif  /* __SUPPORT_ATCMD__ */
}

char* palATDoorlockSerializeToJson(Doorlock *doorlock)
{
#if defined (__SUPPORT_ATCMD__)
    char *result;
    char *nvOtaFileVer;
    char *nvmcuOtaFileVer;
    int i;

    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);

    for (i = 0; i < gShadowCount; i++) {
        if (registeredShadow[i].type == SHADOW_JSON_INT32) {
            (void)json_object_set_number(root_object, registeredShadow[i].pKey,
                *(int*)registeredShadow[i].pData);
        } else if (registeredShadow[i].type == SHADOW_JSON_STRING) {
            (void)json_object_set_string(root_object, registeredShadow[i].pKey,
                (char*)registeredShadow[i].pData);
            APRINTF_I("registerShadow String data (%s) (%s), \n", registeredShadow[i].pKey, registeredShadow[i].pData);
        } else if (registeredShadow[i].type == SHADOW_JSON_FLOAT) {
            (void)json_object_set_number(root_object, registeredShadow[i].pKey,
                *(float*)registeredShadow[i].pData);
        } else {
            APRINTF_I("registerShadow Wrong data (%s), \n", registeredShadow[i].pKey);
        }
    }

#if defined(__SUPPORT_OTA__)
    doorlock->otaUpdate = (int)getOTAStat();
    (void)json_object_set_number(root_object, "OTAupdate", doorlock->otaUpdate);

    if (!strcmp(app_get_ota_result(), (char*)AZURE_UPDATE_OTA_OK))
        doorlock->otaResult = AZURE_UPDATE_OTA_OK;
    else if (!strcmp(app_get_ota_result(), (char*)AZURE_UPDATE_OTA_NG))
        doorlock->otaResult = AZURE_UPDATE_OTA_NG;
    else
        doorlock->otaResult = AZURE_UPDATE_OTA_UNKNOWN;
    (void)json_object_set_string(root_object, "OTAresult", doorlock->otaResult);

    nvOtaFileVer = read_nvram_string(AZURE_NVRAM_CONFIG_CURRENT_OTA_VERSION);
    if (nvOtaFileVer == NULL)
        doorlock->otaVersion = AZURE_REPORT_VERSION_NONE;
    else
        doorlock->otaVersion = nvOtaFileVer;
    (void)json_object_set_string(root_object, "OTAversion", doorlock->otaVersion);

    nvmcuOtaFileVer = read_nvram_string(AZURE_NVRAM_CONFIG_CURRENT_MCUOTA_VERSION);
    if (nvmcuOtaFileVer == NULL)
        doorlock->mcuotaVersion = AZURE_REPORT_VERSION_NONE;
    else
        doorlock->mcuotaVersion = nvmcuOtaFileVer;
    (void)json_object_set_string(root_object, "MCUOTAversion", doorlock->mcuotaVersion);
#endif

    result = json_serialize_to_string(root_value);
    json_value_free(root_value);
    if (result == NULL)
        return (char*)1;
    return result;
#else
    APP_UNUSED_ARG(doorlock);
    return 0;
#endif
}
#endif

#if defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AWS_IOT__)
static void pal_app_making_shadow_payload(char *payload, uint32_t lpayload)
{
    int8_t i;
    char *ptr = payload;

    memset(ptr, 0x00, lpayload);
    da16x_sprintf(ptr, "{\"state\":{\"reported\":{");
    ptr = payload + strlen(payload);
    for (i = 0; i < gShadowCount; i++) {
        if (registeredShadow[i].type == SHADOW_JSON_INT32) {
            if (i != 0) {
                da16x_sprintf(ptr, ",");
                ptr = payload + strlen(payload);
            }
            da16x_sprintf(ptr, "\"%s\":%d", registeredShadow[i].pKey,
                *(int*)registeredShadow[i].pData);
            ptr = payload + strlen(payload);
        } else if (registeredShadow[i].type == SHADOW_JSON_STRING) {
            if (i != 0) {
                da16x_sprintf(ptr, ",");
                ptr = payload + strlen(payload);
            }
            da16x_sprintf(ptr, "\"%s\":\"%s\"", registeredShadow[i].pKey,
                (char*)registeredShadow[i].pData);
            ptr = payload + strlen(payload);
        } else if (registeredShadow[i].type == SHADOW_JSON_FLOAT) {
            if (i != 0) {
                da16x_sprintf(ptr, ",");
                ptr = payload + strlen(payload);
            }
            da16x_sprintf(ptr, "\"%s\":%f", registeredShadow[i].pKey,
                *(float*)registeredShadow[i].pData);
            ptr = payload + strlen(payload);
        } else {
            APRINTF_I("registerShadow Wrong data (%s)\n", registeredShadow[i].pKey);
        }
    }
    da16x_sprintf(ptr, ",");
    ptr = payload + strlen(payload);
    da16x_sprintf(ptr, "\"OTAupdate\":%d,", getOTAStat());
    ptr = payload + strlen(payload);
    da16x_sprintf(ptr, "\"OTAresult\":\"%s\"", app_get_ota_result());
    ptr = payload + strlen(payload);
    da16x_sprintf(ptr, "}},\"clientToken\":\"%s-0\"}", app_get_registered_thing_name());

    return;
}
#endif

#if defined (__SUPPORT_ATCMD__) && defined (__SUPPORT_AZURE_IOT__)
void app_at_set_resp_string(char *new_string, uint32_t string_size)
{
    if (new_string == NULL) {
        memset(resp_string, 0x00, RESP_STR_SIZE);
        return;
    }

    if (string_size < (RESP_STR_SIZE))
        memcpy(resp_string, new_string, string_size);
    else
        APRINTF_I("\r\nMethod resp string size error\r\n");
    return;
}

struct atcmd_twin_params* app_at_get_twins(void)
{
    return (struct atcmd_twin_params*)&atcmd_twins;
}

void app_at_set_twins(struct atcmd_twin_params *param)
{
    atcmd_twins.battery = param->battery;
    atcmd_twins.temperature = param->temperature;
    atcmd_twins.doorState = param->doorState;
    atcmd_twins.doorStateChange = param->doorStateChange;
}

#endif
#endif  /* __SUPPORT_AWS_IOT__ || __SUPPORT_AZURE_IOT__ */

