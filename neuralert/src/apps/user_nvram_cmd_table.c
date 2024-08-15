/**
 ****************************************************************************************
 *
 * @file user_nvram_cmd_table.c
 *
 * @brief Entry point to add user operation using by defined API format
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


#include "sdk_type.h"

#include "da16x_system.h"
#include "common_def.h"
#include "user_nvram_cmd_table.h"
#include "command_net.h"

#if defined (__SUPPORT_MQTT__)
#include "mqtt_client.h"
#endif // (__SUPPORT_MQTT__)

#if defined (__SUPPORT_ZERO_CONFIG__)
#include "zero_config.h"
#endif    // __SUPPORT_ZERO_CONFIG__

#if defined (__SUPPORT_ATCMD_TLS__)
#include "atcmd.h"
#include "atcmd_tls_client.h"
#endif // __SUPPORT_ATCMD_TLS__

#if defined(__SUPPORT_OTA__)
#include "ota_update.h"
#include "ota_update_http.h"
#endif //(__SUPPORT_OTA__)

const user_conf_str user_config_str_with_nvram_name[] = {
  { DA16X_CONF_STR_TEST_PARAM,                "TEST_PARAM_STR",               0                                },

#if defined (__SUPPORT_MQTT__)
  { DA16X_CONF_STR_MQTT_BROKER_IP,            MQTT_NVRAM_CONFIG_BROKER,       MQTT_BROKER_MAX_LEN              },
  { DA16X_CONF_STR_MQTT_SUB_TOPIC,            MQTT_NVRAM_CONFIG_SUB_TOPIC,    MQTT_TOPIC_MAX_LEN               },
  { DA16X_CONF_STR_MQTT_SUB_TOPIC_ADD,        "",                             MQTT_TOPIC_MAX_LEN               },
  { DA16X_CONF_STR_MQTT_SUB_TOPIC_DEL,        "",                             MQTT_TOPIC_MAX_LEN               },
  { DA16X_CONF_STR_MQTT_PUB_TOPIC,            MQTT_NVRAM_CONFIG_PUB_TOPIC,    MQTT_TOPIC_MAX_LEN               },
  { DA16X_CONF_STR_MQTT_USERNAME,             MQTT_NVRAM_CONFIG_USERNAME,     MQTT_USERNAME_MAX_LEN            },
  { DA16X_CONF_STR_MQTT_PASSWORD,             MQTT_NVRAM_CONFIG_PASSWORD,     MQTT_PASSWORD_MAX_LEN            },
  { DA16X_CONF_STR_MQTT_WILL_TOPIC,           MQTT_NVRAM_CONFIG_WILL_TOPIC,   MQTT_TOPIC_MAX_LEN               },
  { DA16X_CONF_STR_MQTT_WILL_MSG,             MQTT_NVRAM_CONFIG_WILL_MSG,     MQTT_WILL_MSG_MAX_LEN            },
  { DA16X_CONF_STR_MQTT_SUB_CLIENT_ID,        MQTT_NVRAM_CONFIG_SUB_CID,      MQTT_CLIENT_ID_MAX_LEN           },
  { DA16X_CONF_STR_MQTT_PUB_CLIENT_ID,        MQTT_NVRAM_CONFIG_PUB_CID,      MQTT_CLIENT_ID_MAX_LEN           },
  #if defined (__MQTT_TLS_OPTIONAL_CONFIG__)
  { DA16X_CONF_STR_MQTT_TLS_SNI,              MQTT_NVRAM_CONFIG_TLS_SNI,      MQTT_BROKER_MAX_LEN              },
  #endif // __MQTT_TLS_OPTIONAL_CONFIG__
#endif // __SUPPORT_MQTT__

#if defined (__SUPPORT_ZERO_CONFIG__)
  { DA16X_CONF_STR_ZEROCONF_MDNS_HOSTNAME,    ZEROCONF_MDNS_HOSTNAME,         ZEROCONF_DNS_SD_SRV_NAME_LEN     },
  #if defined (__SUPPORT_DNS_SD__)
  { DA16X_CONF_STR_ZEROCONF_SRV_NAME,         ZEROCONF_DNS_SD_SRV_NAME,       ZEROCONF_DNS_SD_SRV_NAME_LEN     },
  { DA16X_CONF_STR_ZEROCONF_SRV_PROT,         ZEROCONF_DNS_SD_SRV_PROT,       ZEROCONF_DNS_SD_SRV_PROT_LEN     },
  { DA16X_CONF_STR_ZEROCONF_SRV_TXT,          ZEROCONF_DNS_SD_SRV_TXT,        ZEROCONF_DNS_SD_SRV_TXT_LEN      },
  #endif // (__SUPPORT_DNS_SD__)
#endif // (__SUPPORT_ZERO_CONFIG__)

#if defined (__SUPPORT_ATCMD_TLS__)
  { DA16X_CONF_STR_ATCMD_TLSC_CA_CERT_NAME_0, ATCMD_TLSC_NVR_CA_CERT_NAME_0,  ATCMD_TLSC_NVR_CA_CERT_NAME_LEN  },
  { DA16X_CONF_STR_ATCMD_TLSC_CA_CERT_NAME_1, ATCMD_TLSC_NVR_CA_CERT_NAME_1,  ATCMD_TLSC_NVR_CA_CERT_NAME_LEN  },
  { DA16X_CONF_STR_ATCMD_TLSC_CERT_NAME_0,    ATCMD_TLSC_NVR_CERT_NAME_0,     ATCMD_TLSC_NVR_CERT_NAME_LEN     },
  { DA16X_CONF_STR_ATCMD_TLSC_CERT_NAME_1,    ATCMD_TLSC_NVR_CERT_NAME_1,     ATCMD_TLSC_NVR_CERT_NAME_LEN     },
  { DA16X_CONF_STR_ATCMD_TLSC_HOST_NAME_0,    ATCMD_TLSC_NVR_HOST_NAME_0,     ATCMD_TLSC_NVR_HOST_NAME_LEN     },
  { DA16X_CONF_STR_ATCMD_TLSC_HOST_NAME_1,    ATCMD_TLSC_NVR_HOST_NAME_1,     ATCMD_TLSC_NVR_HOST_NAME_LEN     },
  { DA16X_CONF_STR_ATCMD_TLSC_PEER_IPADDR_0,  ATCMD_TLSC_NVR_PEER_IPADDR_0,   ATCMD_TLSC_NVR_PEER_IPADDR_LEN   },
  { DA16X_CONF_STR_ATCMD_TLSC_PEER_IPADDR_1,  ATCMD_TLSC_NVR_PEER_IPADDR_1,   ATCMD_TLSC_NVR_PEER_IPADDR_LEN   },
#endif // (__SUPPORT_ATCMD_TLS__)

#if defined (__SUPPORT_ATCMD_MULTI_SESSION__)
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_0, ATCMD_NVR_NW_TR_PEER_IPADDR_0,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_1, ATCMD_NVR_NW_TR_PEER_IPADDR_1,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_2, ATCMD_NVR_NW_TR_PEER_IPADDR_2,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_3, ATCMD_NVR_NW_TR_PEER_IPADDR_3,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_4, ATCMD_NVR_NW_TR_PEER_IPADDR_4,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_5, ATCMD_NVR_NW_TR_PEER_IPADDR_5,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_6, ATCMD_NVR_NW_TR_PEER_IPADDR_6,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_7, ATCMD_NVR_NW_TR_PEER_IPADDR_7,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_8, ATCMD_NVR_NW_TR_PEER_IPADDR_8,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
  { DA16X_CONF_STR_ATCMD_NW_TR_PEER_IPADDR_9, ATCMD_NVR_NW_TR_PEER_IPADDR_9,  ATCMD_NVR_NW_TR_PEER_IPADDR_LEN  },
#endif // (__SUPPORT_ATCMD_MULTI_SESSION__)
  { LEGACY_DA16X_CONF_STR_MQTT_RUN_FLAG,			MQTT_NVRAM_CONFIG_RUN_FLAG,		MQTT_CLIENT_ID_MAX_LEN	}, //Replaced by DA16X_CONF_INT_RUN_FLAG

  { 0, "", 0 }
};

user_conf_int user_config_int_with_nvram_name[] = {
  { DA16X_CONF_INT_TEST_PARAM,           "TEST_PARAM_INT",                 0,        1,        0                              },

#if defined (__SUPPORT_MQTT__)
  { DA16X_CONF_INT_MQTT_SUB,             "",                               0,        1,        0                              },
  { DA16X_CONF_INT_MQTT_PUB,             "",                               0,        1,        0                              },
  { DA16X_CONF_INT_MQTT_AUTO,            MQTT_NVRAM_CONFIG_AUTO,           0,        1,        MQTT_CONFIG_AUTO_DEF           },
  { DA16X_CONF_INT_MQTT_PORT,            MQTT_NVRAM_CONFIG_PORT,           0,        65535,    MQTT_CONFIG_PORT_DEF           },
  { DA16X_CONF_INT_MQTT_QOS,             MQTT_NVRAM_CONFIG_QOS,            0,        2,        MQTT_CONFIG_QOS_DEF            },
  { DA16X_CONF_INT_MQTT_TLS,             MQTT_NVRAM_CONFIG_TLS,            0,        1,        MQTT_CONFIG_TLS_DEF            },
  { DA16X_CONF_INT_MQTT_WILL_QOS,        MQTT_NVRAM_CONFIG_WILL_QOS,       0,        2,        MQTT_CONFIG_QOS_DEF            },
  { DA16X_CONF_INT_MQTT_PING_PERIOD,     MQTT_NVRAM_CONFIG_PING_PERIOD,    0,        86400,    MQTT_CONFIG_PING_DEF           },
  { DA16X_CONF_INT_MQTT_CLEAN_SESSION,   MQTT_NVRAM_CONFIG_CLEAN_SESSION,  0,        1,        MQTT_CONFIG_CLEAN_SESSION_DEF  },
  { DA16X_CONF_INT_MQTT_SAMPLE,          MQTT_NVRAM_CONFIG_SAMPLE,         0,        1,        0                              },
  { DA16X_CONF_INT_MQTT_VER311,          MQTT_NVRAM_CONFIG_VER311,         0,        1,        MQTT_CONFIG_VER311_DEF         },
  { DA16X_CONF_INT_MQTT_TLS_INCOMING,    MQTT_NVRAM_CONFIG_TLS_INCOMING,   (2*1024), (8*1024), MQTT_CONFIG_TLS_INCOMING_DEF   },
  { DA16X_CONF_INT_MQTT_TLS_OUTGOING,    MQTT_NVRAM_CONFIG_TLS_OUTGOING,   (2*1024), (8*1024), MQTT_CONFIG_TLS_OUTGOING_DEF   },
  { DA16X_CONF_INT_MQTT_TLS_AUTHMODE,    MQTT_NVRAM_CONFIG_TLS_AUTHMODE,   0,        2,        MQTT_CONFIG_TLS_AUTHMODE_DEF   },
  { DA16X_CONF_INT_MQTT_TLS_NO_TIME_CHK, MQTT_NVRAM_CONFIG_TLS_NO_TIME_CHK,0,        1,        MQTT_CONFIG_TLS_NO_TIME_CHK_DEF}, // debug purpose
#endif // (__SUPPORT_MQTT__)

#if defined (__SUPPORT_ZERO_CONFIG__)
  { DA16X_CONF_INT_ZEROCONF_MDNS_REG,    ZEROCONF_MDNS_REG,                0,        1,        0                              },
  #if defined (__SUPPORT_DNS_SD__)
  { DA16X_CONF_INT_ZEROCONF_SRV_REG,     ZEROCONF_DNS_SD_SRV_REG,          0,        1,        0                              },
  { DA16X_CONF_INT_ZEROCONF_SRV_PORT,    ZEROCONF_DNS_SD_SRV_PORT,         1,        9000,     9000                           },
  #endif // (__SUPPORT_DNS_SD__)
#endif // (__SUPPORT_ZERO_CONFIG__)

#if defined (__SUPPORT_ATCMD_TLS__)
  { DA16X_CONF_INT_ATCMD_TLS_CID_0,      ATCMD_TLS_NVR_CID_0,              -1,       ATCMD_TLS_MAX_ALLOW_CNT,          -1     },
  { DA16X_CONF_INT_ATCMD_TLS_CID_1,      ATCMD_TLS_NVR_CID_1,              -1,       ATCMD_TLS_MAX_ALLOW_CNT,          -1     },
  { DA16X_CONF_INT_ATCMD_TLS_ROLE_0,     ATCMD_TLS_NVR_ROLE_0,             -1,       1,                                -1     },
  { DA16X_CONF_INT_ATCMD_TLS_ROLE_1,     ATCMD_TLS_NVR_ROLE_1,             -1,       1,                                -1     },
  { DA16X_CONF_INT_ATCMD_TLS_PROFILE_0,  ATCMD_TLS_NVR_PROFILE_0,          -1,       ATCMD_TLSC_MAX_ALLOW_NVR_PROFILE, -1     },
  { DA16X_CONF_INT_ATCMD_TLS_PROFILE_1,  ATCMD_TLS_NVR_PROFILE_1,          -1,       ATCMD_TLSC_MAX_ALLOW_NVR_PROFILE, -1     },
  { DA16X_CONF_INT_ATCMD_TLSC_INCOMING_LEN_0,    ATCMD_TLSC_NVR_INCOMING_LEN_0,      ATCMD_TLSC_MIN_INCOMING_LEN, ATCMD_TLSC_MAX_INCOMING_LEN, ATCMD_TLSC_DEF_INCOMING_LEN },
  { DA16X_CONF_INT_ATCMD_TLSC_INCOMING_LEN_1,    ATCMD_TLSC_NVR_INCOMING_LEN_1,      ATCMD_TLSC_MIN_INCOMING_LEN, ATCMD_TLSC_MAX_INCOMING_LEN, ATCMD_TLSC_DEF_INCOMING_LEN },
  { DA16X_CONF_INT_ATCMD_TLSC_OUTGOING_LEN_0,    ATCMD_TLSC_NVR_OUTGOING_LEN_0,      ATCMD_TLSC_MIN_OUTGOING_LEN, ATCMD_TLSC_MAX_OUTGOING_LEN, ATCMD_TLSC_DEF_OUTGOING_LEN },
  { DA16X_CONF_INT_ATCMD_TLSC_OUTGOING_LEN_1,    ATCMD_TLSC_NVR_OUTGOING_LEN_1,      ATCMD_TLSC_MIN_OUTGOING_LEN, ATCMD_TLSC_MAX_OUTGOING_LEN, ATCMD_TLSC_DEF_OUTGOING_LEN },
  { DA16X_CONF_INT_ATCMD_TLSC_AUTH_MODE_0,  ATCMD_TLSC_NVR_AUTH_MODE_0,    pdFALSE,  pdTRUE,  pdFALSE                         },
  { DA16X_CONF_INT_ATCMD_TLSC_AUTH_MODE_1,  ATCMD_TLSC_NVR_AUTH_MODE_1,    pdFALSE,  pdTRUE,  pdFALSE                         },
  { DA16X_CONF_INT_ATCMD_TLSC_LOCAL_PORT_0, ATCMD_TLSC_NVR_LOCAL_PORT_0,   0,        65535,   0                               },
  { DA16X_CONF_INT_ATCMD_TLSC_LOCAL_PORT_1, ATCMD_TLSC_NVR_LOCAL_PORT_1,   0,        65535,   0                               },
  { DA16X_CONF_INT_ATCMD_TLSC_PEER_PORT_0,  ATCMD_TLSC_NVR_PEER_PORT_0,    0,        65535,   0                               },
  { DA16X_CONF_INT_ATCMD_TLSC_PEER_PORT_1,  ATCMD_TLSC_NVR_PEER_PORT_1,    0,        65535,   0                               },
#endif // (__SUPPORT_ATCMD_TLS__)

#if defined (__SUPPORT_ATCMD_MULTI_SESSION__)
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_0, ATCMD_NVR_NW_TR_CID_0, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_1, ATCMD_NVR_NW_TR_CID_1, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_2, ATCMD_NVR_NW_TR_CID_2, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_3, ATCMD_NVR_NW_TR_CID_3, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_4, ATCMD_NVR_NW_TR_CID_4, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_5, ATCMD_NVR_NW_TR_CID_5, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_6, ATCMD_NVR_NW_TR_CID_6, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_7, ATCMD_NVR_NW_TR_CID_7, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_8, ATCMD_NVR_NW_TR_CID_8, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },
  { DA16X_CONF_INT_ATCMD_NW_TR_CID_9, ATCMD_NVR_NW_TR_CID_9, ATCMD_SESS_NONE, ATCMD_SESS_UDP_SESSION, ATCMD_SESS_NONE         },

  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_0,       ATCMD_NVR_NW_TR_LOCAL_PORT_0,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_1,       ATCMD_NVR_NW_TR_LOCAL_PORT_1,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_2,       ATCMD_NVR_NW_TR_LOCAL_PORT_2,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_3,       ATCMD_NVR_NW_TR_LOCAL_PORT_3,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_4,       ATCMD_NVR_NW_TR_LOCAL_PORT_4,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_5,       ATCMD_NVR_NW_TR_LOCAL_PORT_5,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_6,       ATCMD_NVR_NW_TR_LOCAL_PORT_6,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_7,       ATCMD_NVR_NW_TR_LOCAL_PORT_7,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_8,       ATCMD_NVR_NW_TR_LOCAL_PORT_8,  0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_LOCAL_PORT_9,       ATCMD_NVR_NW_TR_LOCAL_PORT_9,  0,  65535, 0 },

  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_0,        ATCMD_NVR_NW_TR_PEER_PORT_0,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_1,        ATCMD_NVR_NW_TR_PEER_PORT_1,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_2,        ATCMD_NVR_NW_TR_PEER_PORT_2,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_3,        ATCMD_NVR_NW_TR_PEER_PORT_3,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_4,        ATCMD_NVR_NW_TR_PEER_PORT_4,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_5,        ATCMD_NVR_NW_TR_PEER_PORT_5,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_6,        ATCMD_NVR_NW_TR_PEER_PORT_6,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_7,        ATCMD_NVR_NW_TR_PEER_PORT_7,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_8,        ATCMD_NVR_NW_TR_PEER_PORT_8,   0,  65535, 0 },
  { DA16X_CONF_INT_ATCMD_NW_TR_PEER_PORT_9,        ATCMD_NVR_NW_TR_PEER_PORT_9,   0,  65535, 0 },

  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_0, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_0, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_1, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_1, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_2, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_2, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_3, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_3, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_4, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_4, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_5, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_5, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_6, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_6, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_7, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_7, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_8, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_8, 0, 0, 1 },
  { DA16X_CONF_INT_ATCMD_NW_TR_MAX_ALLOWED_PEER_9, ATCMD_NVR_NW_TR_MAX_ALLOWED_PEER_9, 0, 0, 1 },
#endif // (__SUPPORT_ATCMD_MULTI_SESSION__)
#if defined(__SUPPORT_OTA__)
{ DA16X_CONF_INT_OTA_TLS_AUTHMODE,	OTA_HTTP_NVRAM_TLS_AUTHMODE,	0,	3,	OTA_HTTP_TLS_AUTHMODE_DEF },
#endif //(__SUPPORT_OTA__)

    /// Auto-run flag. 
    ///-1 == uninitialized; 
    /// 0 == don't auto-run; 
    /// 1 == auto-run
    { DA16X_CONF_INT_RUN_FLAG,  NVRAM_CONFIG_RUN_FLAG,  -1,  1,  -1},
    { 0, "", 0, 0, 0 }
};

const user_conf_str *user_conf_str_table = user_config_str_with_nvram_name;
const user_conf_int *user_conf_int_table = user_config_int_with_nvram_name;

int user_set_str(int name, char *value, int cache)
{
    const user_conf_str *cmd_ptr = NULL;
    int result = CC_FAILURE_NOT_SUPPORTED;

    for (cmd_ptr = user_conf_str_table; cmd_ptr->id; cmd_ptr++) {
        if (name == cmd_ptr->id) {
            if (strlen(cmd_ptr->nvram_name) == 0) {
                break;
            }

            if (value == NULL) {
                if (cache) {
                    delete_tmp_nvram_env(cmd_ptr->nvram_name);
                } else {
                    delete_nvram_env(cmd_ptr->nvram_name);
                }

                return CC_SUCCESS;
            }

            if (cmd_ptr->max_length > 0 && strlen(value) > (unsigned int)cmd_ptr->max_length) {
                return CC_FAILURE_STRING_LENGTH;
            }

            if (cache) {
                write_tmp_nvram_string(cmd_ptr->nvram_name, value);
            } else {
                write_nvram_string(cmd_ptr->nvram_name, value);
            }

            result = CC_SUCCESS;
            break;
        }
    }

    switch (name) {
    case DA16X_CONF_STR_TEST_PARAM:
        PRINTF("USER_CONFIG: SET_STR TEST!\n");
        break;

#if defined (__SUPPORT_MQTT__)
    case DA16X_CONF_STR_MQTT_SUB_TOPIC_ADD:
        if (mqtt_client_add_sub_topic(value, cache)) {
            result = CC_FAILURE_INVALID;
        }

        break;

    case DA16X_CONF_STR_MQTT_SUB_TOPIC_DEL:
        if (mqtt_client_del_sub_topic(value, cache)) {
            result = CC_FAILURE_INVALID;
        }

        break;
#endif // (__SUPPORT_MQTT__)
    }

    return result;
}

int user_set_int(int name, int value, int cache)
{
    const user_conf_int *cmd_ptr = NULL;
    int result = CC_FAILURE_NOT_SUPPORTED;

    for (cmd_ptr = user_conf_int_table; cmd_ptr->id; cmd_ptr++) {
        if (name == cmd_ptr->id) {
            if (strlen(cmd_ptr->nvram_name) == 0) {
                break;
            }

            if (   (cmd_ptr->min_value || cmd_ptr->max_value)
                && (value < cmd_ptr->min_value || value > cmd_ptr->max_value)) {
                return CC_FAILURE_RANGE_OUT;
            }

            if (cache) {
                if (value == cmd_ptr->def_value) {
                    delete_tmp_nvram_env(cmd_ptr->nvram_name);
                } else {
                    write_tmp_nvram_int(cmd_ptr->nvram_name, value);
                }
            } else {
                if (value == cmd_ptr->def_value) {
                    delete_nvram_env(cmd_ptr->nvram_name);
                } else {
                    write_nvram_int(cmd_ptr->nvram_name, value);
                }
            }

            result = CC_SUCCESS;
            break;
        }
    }

    switch (name) {
    case DA16X_CONF_INT_TEST_PARAM:
        PRINTF("USER_CONFIG: SET_INT TEST!\n");
        break;

#if defined (__SUPPORT_MQTT__)
    case DA16X_CONF_INT_MQTT_SUB:
        if (get_run_mode() != SYSMODE_AP_ONLY) {
            if (value == CC_VAL_ENABLE) {
                if (mqtt_client_is_running() == TRUE) {
                    mqtt_client_force_stop();
                    mqtt_client_stop_sub();
                }
                
                if (mqtt_client_start_sub() == 0) {
                    result = CC_SUCCESS;
                } else {
                    result = CC_FAILURE_UNKNOWN;
                }
            } else {
                if (mqtt_client_is_running() == TRUE) {
                    mqtt_client_force_stop();
                    mqtt_client_stop_sub();
                }

                result = CC_SUCCESS;
            }
        }
        break;

    case DA16X_CONF_INT_MQTT_PUB:
        result = CC_FAILURE_NOT_SUPPORTED;
        break;

    case DA16X_CONF_INT_MQTT_AUTO:
        if (value == 1) {
            if (cache) {
                write_tmp_nvram_int(cmd_ptr->nvram_name, MQTT_INIT_MAGIC);
            } else {
                write_nvram_int(cmd_ptr->nvram_name, MQTT_INIT_MAGIC);
            }
        }

        break;
#endif // (__SUPPORT_MQTT__)
    }

    return result;
}

int user_get_str(int name, char *value)
{
    const user_conf_str *cmd_ptr = NULL;
    char *nvram_string = NULL;
    int result = CC_FAILURE_NOT_SUPPORTED;

    for (cmd_ptr = user_conf_str_table; cmd_ptr->id; cmd_ptr++) {
        if (name == cmd_ptr->id) {
            if (strlen(cmd_ptr->nvram_name) == 0) {
                break;
            }

            nvram_string = read_nvram_string(cmd_ptr->nvram_name);

            if (nvram_string == NULL) {
                result = CC_FAILURE_NO_VALUE;
            } else {
                strcpy(value, nvram_string);
                result = CC_SUCCESS;
            }

            break;
        }
    }

    switch (name) {
    case DA16X_CONF_STR_TEST_PARAM:
        PRINTF("USER_CONFIG: GET_STR TEST!\n");
        break;
    }

    return result;
}

int user_get_int(int name, int *value)
{
    const user_conf_int *cmd_ptr = NULL;
    int result = CC_FAILURE_NOT_SUPPORTED;
    int nvram_int;

    for (cmd_ptr = user_conf_int_table; cmd_ptr->id; cmd_ptr++) {
        if (name == cmd_ptr->id) {
            if (strlen(cmd_ptr->nvram_name) == 0) {
                break;
            }

            if (read_nvram_int(cmd_ptr->nvram_name, &nvram_int) < 0) {
                *value = cmd_ptr->def_value;
            } else {
                *value = nvram_int;
            }

            result = CC_SUCCESS;
            break;
        }
    }

    switch (name) {
    case DA16X_CONF_INT_TEST_PARAM:
        PRINTF("USER_CONFIG: GET_INT TEST!\n");
        break;

#if defined (__SUPPORT_MQTT__)
    case DA16X_CONF_INT_MQTT_SUB:
        *value = mqtt_client_check_sub_conn();
        break;

    case DA16X_CONF_INT_MQTT_PUB:
        *value = mqtt_client_check_sub_conn();
        break;
#endif // (__SUPPORT_MQTT__)
    case DA16X_CONF_INT_RUN_FLAG:

        // Check if the value has not yet been initialized
        // in which case, we want to check if the legacy str value has been set
        // otherwise default to 0
        if (*value == -1) {
            PRINTF("RUN MODE not initialized; checking against legacy config\n");

            // Retrieve the legacy string config value
            char str[50]; 
            da16x_get_config_str(LEGACY_DA16X_CONF_STR_MQTT_RUN_FLAG, str);
            PRINTF("NVRam legacy runFlag: %s\r\n",str);

            // If the legacy value was set to 1, initialized our new config to 1,
            // otherwise default to 0
            if(strcmp(str,"runFlag=1") == 0)
            {
                *value = 1;
            } else {
                *value = 0;
            }

            // Store the config value back, so that we don't have to go through
            // legacy recovery next time
            da16x_set_config_int(DA16X_CONF_INT_RUN_FLAG, *value);
        }
    }

    return result;
}

/* EOF */
