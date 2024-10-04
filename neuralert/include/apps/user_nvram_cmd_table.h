/**
 ****************************************************************************************
 *
 * @file user_nvram_cmd_table.h
 *
 * @brief Define for user NVRAM operation command table
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

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "common_config.h"


//JW: Added this flag for run 0 vs. run 1 implementation
/// NVRAM name of MQTT run flag
#define MQTT_NVRAM_CONFIG_RUN_FLAG		"MQTT_RUN_FLAG"
/// NVRAM name for int-based run flag
#define NVRAM_CONFIG_RUN_FLAG           "RUN_FLAG"

/// NVRAM string value structure
typedef struct _user_conf_str {
    /// Parameter name (DA16X_USER_CONF_STR)
    int  id;
    /// NVRAM save name
    char nvram_name[24];
    /// Maximum length of the string value
    int  max_length;
} user_conf_str;

/// NVRAM integer value structure
typedef struct _user_conf_int {
    /// Parameter name (DA16X_USER_CONF_INT)
    int  id;
    /// NVRAM save name
    char nvram_name[24];
    /// Minimum value
    int  min_value;
    /// Maximum value
    int  max_value;
    /// Default value
    int  def_value;
} user_conf_int;

/// User Configurations (for string value)
typedef enum {
    DA16X_CONF_STR_USER_START = DA16X_CONF_STR_MAX,
    DA16X_CONF_STR_TEST_PARAM,

#if defined (__SUPPORT_MQTT__)
    DA16X_CONF_STR_MQTT_BROKER_IP,
    DA16X_CONF_STR_MQTT_SUB_TOPIC,
    DA16X_CONF_STR_MQTT_SUB_TOPIC_ADD,
    DA16X_CONF_STR_MQTT_SUB_TOPIC_DEL,
    DA16X_CONF_STR_MQTT_PUB_TOPIC,
    DA16X_CONF_STR_MQTT_USERNAME,
    DA16X_CONF_STR_MQTT_PASSWORD,
    DA16X_CONF_STR_MQTT_WILL_TOPIC,
    DA16X_CONF_STR_MQTT_WILL_MSG,
    DA16X_CONF_STR_MQTT_SUB_CLIENT_ID,
    DA16X_CONF_STR_MQTT_PUB_CLIENT_ID,
  #if defined (__MQTT_TLS_OPTIONAL_CONFIG__)
    DA16X_CONF_STR_MQTT_TLS_SNI,
  #endif // __MQTT_TLS_OPTIONAL_CONFIG__
#endif // (__SUPPORT_MQTT__)

#if defined (__SUPPORT_ZERO_CONFIG__)
    DA16X_CONF_STR_ZEROCONF_MDNS_HOSTNAME,
  #if defined (__SUPPORT_DNS_SD__)
    DA16X_CONF_STR_ZEROCONF_SRV_NAME,
    DA16X_CONF_STR_ZEROCONF_SRV_PROT,
    DA16X_CONF_STR_ZEROCONF_SRV_TXT,
  #endif // (__SUPPORT_DNS_SD__)
#endif // (__SUPPORT_ZERO_CONFIG__)

#if defined (__SUPPORT_ATCMD_TLS__)
    DA16X_CONF_STR_ATCMD_TLSC_CA_CERT_NAME_0,
    DA16X_CONF_STR_ATCMD_TLSC_CA_CERT_NAME_1,
    DA16X_CONF_STR_ATCMD_TLSC_CERT_NAME_0,
    DA16X_CONF_STR_ATCMD_TLSC_CERT_NAME_1,
    DA16X_CONF_STR_ATCMD_TLSC_HOST_NAME_0,
    DA16X_CONF_STR_ATCMD_TLSC_HOST_NAME_1,
    DA16X_CONF_STR_ATCMD_TLSC_PEER_IPADDR_0,
    DA16X_CONF_STR_ATCMD_TLSC_PEER_IPADDR_1,
#endif // (__SUPPORT_ATCMD_TLS__)

#if defined (__SUPPORT_ATCMD_MULTI_SESSION__)
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_0,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_1,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_2,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_3,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_4,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_5,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_6,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_7,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_8,
    DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_9,
#endif // (__SUPPORT_ATCMD_MULTI_SESSION__)

    // Legacy value retained for backwards compatibility, 
    // use DA16X_CONF_INT_RUN_FLAG instead
	LEGACY_DA16X_CONF_STR_MQTT_RUN_FLAG, 

    DA16X_CONF_STR_FINAL_MAX
} DA16X_USER_CONF_STR;

/// User Configurations (for integer value)
typedef enum {
    DA16X_CONF_INT_TEST_PARAM = DA16X_CONF_INT_MAX,

#if defined (__SUPPORT_MQTT__)
    DA16X_CONF_INT_MQTT_SUB,
    DA16X_CONF_INT_MQTT_PUB,
    DA16X_CONF_INT_MQTT_AUTO,
    DA16X_CONF_INT_MQTT_PORT,
    DA16X_CONF_INT_MQTT_QOS,
    DA16X_CONF_INT_MQTT_TLS,
    DA16X_CONF_INT_MQTT_WILL_QOS,
    DA16X_CONF_INT_MQTT_PING_PERIOD,
    DA16X_CONF_INT_MQTT_CLEAN_SESSION,
    DA16X_CONF_INT_MQTT_SAMPLE,
    DA16X_CONF_INT_MQTT_VER311,
    DA16X_CONF_INT_MQTT_TLS_INCOMING,
    DA16X_CONF_INT_MQTT_TLS_OUTGOING,
    DA16X_CONF_INT_MQTT_TLS_AUTHMODE,
    DA16X_CONF_INT_MQTT_TLS_NO_TIME_CHK,
#endif // (__SUPPORT_MQTT__)

#if defined (__SUPPORT_ZERO_CONFIG__)
    DA16X_CONF_INT_ZEROCONF_MDNS_REG,
  #if defined (__SUPPORT_DNS_SD__)
    DA16X_CONF_INT_ZEROCONF_SRV_REG,
    DA16X_CONF_INT_ZEROCONF_SRV_PORT,
  #endif // (__SUPPORT_DNS_SD__)
#endif // (__SUPPORT_ZERO_CONFIG__)

#if defined (__SUPPORT_ATCMD_TLS__)
    DA16X_CONF_INT_ATCMD_TLS_CID_0,
    DA16X_CONF_INT_ATCMD_TLS_CID_1,
    DA16X_CONF_INT_ATCMD_TLS_ROLE_0,
    DA16X_CONF_INT_ATCMD_TLS_ROLE_1,
    DA16X_CONF_INT_ATCMD_TLS_PROFILE_0,
    DA16X_CONF_INT_ATCMD_TLS_PROFILE_1,
    DA16X_CONF_INT_ATCMD_TLSC_INCOMING_LEN_0,
    DA16X_CONF_INT_ATCMD_TLSC_INCOMING_LEN_1,
    DA16X_CONF_INT_ATCMD_TLSC_OUTGOING_LEN_0,
    DA16X_CONF_INT_ATCMD_TLSC_OUTGOING_LEN_1,
    DA16X_CONF_INT_ATCMD_TLSC_AUTH_MODE_0,
    DA16X_CONF_INT_ATCMD_TLSC_AUTH_MODE_1,
    DA16X_CONF_INT_ATCMD_TLSC_LOCAL_PORT_0,
    DA16X_CONF_INT_ATCMD_TLSC_LOCAL_PORT_1,
    DA16X_CONF_INT_ATCMD_TLSC_PEER_PORT_0,
    DA16X_CONF_INT_ATCMD_TLSC_PEER_PORT_1,
#endif // (__SUPPORT_ATCMD_TLS__)

#if defined (__SUPPORT_ATCMD_MULTI_SESSION__)
    DA16X_CONF_INT_ATCMD_NW_TR_CID_0,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_1,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_2,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_3,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_4,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_5,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_6,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_7,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_8,
    DA16X_CONF_INT_ATCMD_NW_TR_CID_9,

    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_0,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_1,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_2,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_3,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_4,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_5,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_6,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_7,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_8,
    DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_9,

    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_0,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_1,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_2,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_3,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_4,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_5,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_6,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_7,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_8,
    DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_9,

    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_0,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_1,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_2,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_3,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_4,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_5,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_6,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_7,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_8,
    DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_9,
#endif // (__SUPPORT_ATCMD_MULTI_SESSION__)

#if defined(__SUPPORT_OTA__)
    DA16X_CONF_INT_OTA_TLS_AUTHMODE,
#endif //(__SUPPORT_OTA__)

    DA16X_CONF_INT_RUN_FLAG,
    DA16X_CONF_INT_FINAL_MAX
} DA16X_USER_CONF_INT;


int user_set_str(int name, char *value, int cache);
int user_set_int(int name, int value, int cache);
int user_get_str(int name, char *value);
int user_get_int(int name, int *value);

#endif    /* __USER_CONFIG_H__ */

/* EOF */
