/**
 *****************************************************************************************
 * @file     mqtt_client.h
 * @brief    Header file - MQTT Client main controller
 *****************************************************************************************
 */

/*
Copyright (c) 2014 Roger Light <roger@atchoo.org>

All rights reserved. This program and the accompanying materials
are made available under the terms of the Eclipse Public License v1.0
and Eclipse Distribution License v1.0 which accompany this distribution.

The Eclipse Public License is available at
   http://www.eclipse.org/legal/epl-v10.html
and the Eclipse Distribution License is available at
  http://www.eclipse.org/org/documents/edl-v10.php.

Contributors:
   Roger Light - initial implementation and documentation.
   
Copyright (c) 2019-2022 Modified by Renesas Electronics.
*/

#ifndef	__MQTT_CLIENT_H__
#define	__MQTT_CLIENT_H__

#include "iface_defs.h"
#include "driver.h"
#include "da16x_system.h"
#include "user_nvram_cmd_table.h"
#include "common_def.h"
#include "command_net.h"
#include "application.h"
#include "user_dpm_api.h"
#include "mosquitto.h"
#include "mosquitto_internal.h"
#include "da16x_compile_opt.h"

#undef __TEST_SET_SLP_DELAY__

/// To check whether MQTT parameters in NVRAM is saved normally
#define MQTT_INIT_MAGIC					0x7E7EBEAF

/// mqtt_client module thread's stack size
#define MQTT_CLIENT_THREAD_STACK_SIZE	(1024 * 4)
/// MAX length of TLS certificate for MQTT
#define MQTT_CERT_MAX_LENGTH			(1024 * 2)

/// MQTT Subscriber thread name
#define MQTT_SUB_THREAD_NAME			"MQTT_Sub"
/// MQTT Publisher thread name
#define MQTT_PUB_THREAD_NAME			"MQTT_Pub"
/// MQTT Restart part's thread name
#define MQTT_RESTART_THREAD_NAME		"MQTT_Restart"
/// MQTT Parameter set name for DPM user memory
#define MQTT_PARAM_NAME					"MQTT_PARAM"

/// MQTT Version Name V3.1
#define MQTT_VER_31						"3.1"
/// MQTT Version Name V3.1.1
#define MQTT_VER_311					"3.1.1"

/// NVRAM name of broker address
#define MQTT_NVRAM_CONFIG_BROKER		"MQTT_BROKER"
/// NVRAM name of broker port
#define MQTT_NVRAM_CONFIG_PORT			"MQTT_PORT"
/// NVRAM name of publisher topic
#define MQTT_NVRAM_CONFIG_PUB_TOPIC		"MQTT_PUB_TOPIC"
/// NVRAM name of subscriber topic
#define MQTT_NVRAM_CONFIG_SUB_TOPIC		"MQTT_SUB_TOPIC"
/// NVRAM name of the number of subscriber topic
#define MQTT_NVRAM_CONFIG_SUB_TOPIC_NUM	"MQTT_SUB_TOPIC_NUM"
/// NVRAM name of MQTT QoS
#define MQTT_NVRAM_CONFIG_QOS			"MQTT_QOS"
/// NVRAM name of MQTT TLS use
#define MQTT_NVRAM_CONFIG_TLS			"MQTT_TLS"
/// NVRAM name of MQTT username
#define MQTT_NVRAM_CONFIG_USERNAME		"MQTT_USERNAME"
/// NVRAM name of MQTT password
#define MQTT_NVRAM_CONFIG_PASSWORD		"MQTT_PASSWORD"
/// NVRAM name of MQTT will topic
#define MQTT_NVRAM_CONFIG_WILL_TOPIC	"MQTT_WILL_TOPIC"
/// NVRAM name of MQTT will message
#define MQTT_NVRAM_CONFIG_WILL_MSG		"MQTT_WILL_MSG"
/// NVRAM name of MQTT will QoS
#define MQTT_NVRAM_CONFIG_WILL_QOS		"MQTT_WILL_QOS"
/// NVRAM name of MQTT auto_start use
#define MQTT_NVRAM_CONFIG_AUTO			"MQTT_AUTO"
/// NVRAM name of Sending PINGREQ period
#define MQTT_NVRAM_CONFIG_PING_PERIOD	"MQTT_PING_PERIOD"
/// NVRAM name of MQTT Service ID
#define MQTT_NVRAM_CONFIG_SERVICE_ID	"MQTT_SERVICE_ID"
/// NVRAM name of MQTT Device ID
#define MQTT_NVRAM_CONFIG_DEVICE_ID		"MQTT_DEVICE_ID"
/// NVRAM name of MQTT TOKEN
#define MQTT_NVRAM_CONFIG_TOKEN			"MQTT_TOKEN"
/// NVRAM name of MQTT data type
#define MQTT_NVRAM_CONFIG_DATA_TYPE		"MQTT_DATA_TYPE"
/// NVRAM name of MQTT clean_session use
#define MQTT_NVRAM_CONFIG_CLEAN_SESSION	"MQTT_CLEAN_SESSION"
/// NVRAM name of MQTT sample use
#define MQTT_NVRAM_CONFIG_SAMPLE		"MQTT_SAMPLE"
/// NVRAM name of MQTT API version
#define MQTT_NVRAM_CONFIG_API_VERSION	"MQTT_API_VER"
/// NVRAM name of MQTT subscriber client ID
#define MQTT_NVRAM_CONFIG_SUB_CID		"MQTT_SUB_CID"
/// NVRAM name of MQTT publisher client ID
#define MQTT_NVRAM_CONFIG_PUB_CID		"MQTT_PUB_CID"
/// NVRAM name of MQTT version 3.1.1 use
#define MQTT_NVRAM_CONFIG_VER311		"MQTT_VER311"
/// NVRAM name of MQTT TLS incoming
#define	MQTT_NVRAM_CONFIG_TLS_INCOMING	"MQTT_TLS_INCOMING"
/// NVRAM name of MQTT TLS outgoing
#define	MQTT_NVRAM_CONFIG_TLS_OUTGOING	"MQTT_TLS_OUTGOING"
/// NVRAM name of MQTT TLS auth mode
#define MQTT_NVRAM_CONFIG_TLS_AUTHMODE	"MQTT_TLS_AUTHMODE"
/// NVRAM name of MQTT TLS no time check option
#define MQTT_NVRAM_CONFIG_TLS_NO_TIME_CHK	"MQTT_TLS_NO_TIME_CHK"
#if defined (__MQTT_TLS_OPTIONAL_CONFIG__)
/// NVRAM name of MQTT TLS alpn
#define MQTT_NVRAM_CONFIG_TLS_ALPN		"MQTT_TLS_ALPN"
/// NVRAM name of the number of TLS alpn
#define MQTT_NVRAM_CONFIG_TLS_ALPN_NUM	"MQTT_TLS_ALPN_NUM"
/// NVRAM name of MQTT TLS SNI
#define MQTT_NVRAM_CONFIG_TLS_SNI		"MQTT_TLS_SNI"
/// NVRAM name of the number of cipher suits
#define MQTT_NVRAM_CONFIG_TLS_CSUIT_NUM "MQTT_TLS_CSUIT_NUM"
/// NVARM name of cipher suit list
#define MQTT_NVRAM_CONFIG_TLS_CSUITS	"MQTT_TLS_CSUIT"
#endif // __MQTT_TLS_OPTIONAL_CONFIG__

/// Default port number of the broker
#define MQTT_CONFIG_PORT_DEF			1883
/// Default Qos value
#define MQTT_CONFIG_QOS_DEF				0
/// Default TLS value
#define MQTT_CONFIG_TLS_DEF				0
/// Default clean_session value
#define MQTT_CONFIG_CLEAN_SESSION_DEF	1
/// Default sending PINGREQ period (sec.)
#define	MQTT_CONFIG_PING_DEF			600
/// Default auto_start value
#define	MQTT_CONFIG_AUTO_DEF			0
/// Default MQTT version (0 = 3.1, 1 = 3.1.1)
#define MQTT_CONFIG_VER311_DEF			0

#if defined	(__SUPPORT_ATCMD__)
#define	MQTT_CONFIG_TLS_INCOMING_DEF	(1024 * 6)
#else
#define	MQTT_CONFIG_TLS_INCOMING_DEF	(1024 * 4)
#endif // __SUPPORT_ATCMD__
#define	MQTT_CONFIG_TLS_OUTGOING_DEF	(1024 * 4)
#define MQTT_CONFIG_TLS_AUTHMODE_DEF	1
#define MQTT_CONFIG_TLS_NO_TIME_CHK_DEF	0

/// Max length of device ID
#define MQTT_DEVICE_ID_MAX_LEN			64

#if defined (__MQTT_TLS_OPTIONAL_CONFIG__)
#define MQTT_TLS_MAX_ALPN				3
#define MQTT_TLS_ALPN_MAX_LEN			24

/// Max number of TLS cipher suits
#define MQTT_TLS_MAX_CSUITS				17
#endif // __MQTT_TLS_OPTIONAL_CONFIG__

/// Topic max length
#define MQTT_TOPIC_MAX_LEN				64
/// Max number of topic
#define MQTT_MAX_TOPIC					4
/// Max length of PUBLISH message (both Sub and Pub)
#if defined	(__SUPPORT_ATCMD__)
#define MQTT_MSG_MAX_LEN				(1024 * 2)
#else
#define MQTT_MSG_MAX_LEN				(1024 * 3)
#endif // (__SUPPORT_ATCMD__)
/// Max length of will message
#define MQTT_WILL_MSG_MAX_LEN			64
/// Max length of username for log-in
#define MQTT_USERNAME_MAX_LEN			128	// Increased to accommodate AWS authenticator
/// Max length of password for log-in
#define MQTT_PASSWORD_MAX_LEN			160
/// Max length of client ID
#define MQTT_CLIENT_ID_MAX_LEN			40

/// Max number of MQTT reconnection
#define MQTT_CONN_MAX_RETRY				6
/// Max number of MQTT reconnection without DPM sleep in DPM mode
#define MQTT_RESTART_MAX_RETRY			3
/// MQTT Reconnection cycle (sec.)
#define MQTT_CONN_RETRY_PERIOD			5
/// Awaiting PINGRESP timeout (sec.)
#define	MQTT_WAIT_COUNT_PING			6
/// Max waiting time of MQTT DPM sleep
#define MQTT_CLIENT_DPM_SLEEP_TIMEOUT	30

/// Max length of data type
#define MQTT_DATA_TYPE_MAX_LEN			16
/// Max length of token
#define MQTT_TOKEN_MAX_LEN				24
/// Max length of API version
#define MQTT_API_VERSION_MAX_LEN		24
/// Max length of Broker address
#define MQTT_BROKER_MAX_LEN				64


/// PING status : None
#define PING_STS_NONE					0
/// PING status : PINGREQ sent (Awaiting PINGRESP)
#define PING_STS_SENT					1
/// PING status : PINGRESP received
#define PING_STS_RECEIVED				2

/// Thread status : Disconnected
#define THREAD_STS_NONE					0x0
/// Thread status : Subscriber is connected only
#define THREAD_STS_SUB_INIT_DONE		0x1
/// Thread status : Publisher is connected only
#define THREAD_STS_PUB_INIT_DONE		0x2
/// Thread status : Both Subscriber and Publisher atr connected
#define THREAD_STS_ALL_DONE				0x3


struct mosquitto;
struct mosquitto_message;

/// MQTT Client internal configuration structurer
struct mosq_config
{
	/// Subscriber Client ID
	char	*sub_id;
	/// MQTT Protocol (3 - v3.1 / 4 - v3.1.1)
	int		protocol_version;
	/// Sending PINGREQ period (sec.)
	int		keepalive;
	/// Broker address
	char	*host;
	/// Broker port number
	int		port;
	/// MQTT QoS
	int		qos;
	/// Message should be retained if 1.
	char	retain;
	/// Publisher topic
	char	*topic;
	/// Binded broker address
	char	*bind_address;
	/// Max inflight-message enable
	unsigned int max_inflight;
	/// Broker log-in ID
	char	*username;
	/// Broker log-in password
	char	*password;
	/// Will topic
	char	*will_topic;
	/// Will message
	char	*will_payload;
	/// Will message length
	long	will_payloadlen;
	/// Will QoS
	int		will_qos;
	/// Make the client Will retained if 1
	char	will_retain;
	/// TLS use
	char	insecure;
	/// TLS CA Certificate
	char	*cacert_ptr;
    size_t  cacert_buflen;
    size_t  cacert_len;
	/// TLS Client Certificate
	char	*cert_ptr;
    size_t  cert_buflen;
    size_t  cert_len;
	/// TLS Client Private Key
	char	*priv_key_ptr;
    size_t  priv_key_buflen;
    size_t  priv_key_len;
	/// TLS Client DH Param
	char	*dh_param_ptr;
    size_t  dh_param_buflen;
    size_t  dh_param_len;
#if defined (__MQTT_TLS_OPTIONAL_CONFIG__)
	/// TLS alpn
	char	**alpn;
	/// Number of TLS alpn
	int		alpn_count;
	/// TLS SNI
	char	*sni;
	/// TLS cipher suits
	int		*cipher_suits;
	/// TLS cipher suits count
	int		cipher_suits_count;
#endif // __MQTT_TLS_OPTIONAL_CONFIG__
	/// Clean session use
	char	clean_session;
	/// Subscriber topic(s)
	char	**topics;
	/// The number of subscriber topic
	int		topic_count;
};

/// MQTT parameters for DPM memory
typedef struct mqttParam
{
	/// MQTT Auto-start function use on booting
	int		mqtt_auto;
	/// Subscriber client ID
	char	sub_client_id[MQTT_CLIENT_ID_MAX_LEN + 1];
	/// MQTT device ID
	char	device_id[MQTT_DEVICE_ID_MAX_LEN + 1];
	/// MQTT API version
	char	api_version[MQTT_API_VERSION_MAX_LEN + 1];
	/// Broker address from NVRAM by users
	char	host[MQTT_BROKER_MAX_LEN + 1];
	/// Real IP address if the host is domain type
	char	host_ip[MQTT_BROKER_MAX_LEN + 1];
	/// Broker port number
	int		port;
	/// Publisher topic
	char	topic[MQTT_TOPIC_MAX_LEN + 1];
	/// Subscriber topic(s)
	char	topics[MQTT_MAX_TOPIC][MQTT_TOPIC_MAX_LEN + 1];
	/// The number of subscriber topic
	int		topic_count;
	/// MQTT QoS
	int		qos;
	/// Broker log-in ID
	char	username[MQTT_USERNAME_MAX_LEN + 1];
	/// Broker log-in password
	char	password[MQTT_PASSWORD_MAX_LEN + 1];
	/// Will message
	char	will_payload[MQTT_WILL_MSG_MAX_LEN + 1];
	/// Will topic
	char	will_topic[MQTT_TOPIC_MAX_LEN + 1];
	/// Will QoS
	int		will_qos;
	/// Sending PINGREQ period (sec.)
	int		keepalive;
	/// Max inflight-message enable
	int		max_inflight;
	/// Clean session use
	int		clean_session;
	/// MQTT Protocol (3 - v3.1 / 4 - v3.1.1)
	int		protocol_version;
	
	char	data_type[16];
	/// TLS use
	char	insecure;
	/// TLS CA Certificate
	char	*cacert_ptr;
	/// TLS Client Certificate
	char	*cert_ptr;
	/// TLS Client Private Key
	char	*priv_key_ptr;
	/// Subscriber reconnection count
	int		sub_connect_retry_count;
	/// PUBLISH message's ID
	int		pub_msg_id;
	/// Subscriber local port
	int		my_sub_port_no;
	/// Subscriber local port (previous)
	int		my_sub_port_no_prev;
	/// TLS time-check (validation) use
	int		tls_time_check_option;
	/// TLS Auth mode
	int		tls_authmode;
	/// TLS incoming size
	int		tls_incoming_size;
	/// TLS ougoing size
	int		tls_outgoing_size;
#if defined (__MQTT_TLS_OPTIONAL_CONFIG__)
	/// TLS alpn protocol count
	int		tls_alpn_cnt;
	/// TLS alpn
	char 	tls_alpn[MQTT_TLS_MAX_ALPN][MQTT_TLS_ALPN_MAX_LEN + 1];
	/// TLS SNI
	char	tls_sni[MQTT_BROKER_MAX_LEN + 1];
	/// TLS cipher suits
	int		tls_csuits[MQTT_TLS_MAX_CSUITS];
	/// TLS number of sipher suits
	int		tls_csuits_cnt;
#endif // __MQTT_TLS_OPTIONAL_CONFIG__
#if defined ( __MQTT_CONN_STATUS__ )
		/// Connection status flag to MQTT Borker
	char	broker_conn_done;
#endif	// __MQTT_CONN_STATUS__
#if defined (__TEST_SET_SLP_DELAY__)
	int 	temp_val_wait;
#endif
#if defined (__MQTT_EMUL_CMD__)
	int 	q2_rx_no_pubrec_tx;
	int 	q2_rx_discard_pubrel_rx;
	int 	q2_rx_no_pubcomp_tx;
	int		q1_tx_discard_puback_rx;
	int 	q2_tx_discard_pubrec_rx;
	int		q2_tx_no_pubrel_tx;
#endif // __MQTT_EMUL_CMD__
} mqttParamForRtm;

typedef struct _mqtt_client_thread
{
	char		thread_name[32];
	TaskHandle_t	thread;
	UINT		trigger;
	UCHAR		*stack;
} mqtt_client_thread;

extern UINT	pubPingStatus;
extern UINT	subPingStatus;

extern void mqtt_client_set_conn_cb(void (*user_cb)(void));
extern void mqtt_client_set_pub_cb(void (*user_pub_cb)(int));
extern void mqtt_client_set_msg_cb(void (*user_cb)(const char *buf, int len, const char *topic));
extern void mqtt_client_set_disconn_cb(void (*user_cb)(void));
extern void mqtt_client_set_subscribe_cb(void (*user_cb)(void));
extern void mqtt_client_set_unsubscribe_cb(void (*user_cb)(void));
extern void ut_mqtt_sem_take_recur(mqtt_mutex_t* mq_mutex, TickType_t wait_opt);
extern void ut_mqtt_sem_create(mqtt_mutex_t* mq_mutex);
extern void ut_mqtt_sem_delete(mqtt_mutex_t* mq_mutex);

void	client_config_cleanup(struct mosq_config *cfg);

int		client_opts_set(struct mosquitto *mosq, struct mosq_config *cfg);

int		client_connect(struct mosquitto *mosq, struct mosq_config *cfg);

void	mqtt_client_generate_port(void);

int		mqtt_client_initialize_config(void);

int	my_publish(struct mosquitto *mosq, void *obj, int result);

void	mqtt_sub_on_connect(struct mosquitto *mosq, void *obj, int result);

int mqtt_client_get_topic_count(void);

int mqtt_client_get_pub_msg_id(void);

int mqtt_client_cfg_sync_rtm(int cfg_str_param, char* str_val, 
							int cfg_int_param, int num_val);

int mqtt_client_is_cfg_dpm_mem_intact(void);

void mqtt_client_dpm_port_filter_set(void);

void mqtt_client_dpm_port_filter_clear(void);

/**
 ****************************************************************************************
 * @brief Start the MQTT Client thread
 * @return MOSQ_ERR_SUCCESS if succeed, error code else.
 ****************************************************************************************
 */
int 	mqtt_client_start(void);

/**
 ****************************************************************************************
 * @brief Terminate the MQTT Client thread
 * @return 0 - succeess / -1 - No publisher session operating
 ****************************************************************************************
 */
int		mqtt_client_stop(void);

USHORT	is_mqtt_client_thd_alive(void);
int		mqtt_client_start_sub(void);
int		mqtt_client_stop_sub(void);
void	mqtt_client_force_stop(void);
UINT8	mqtt_client_is_running(void);

/**
 ****************************************************************************************
 * @brief Send MQTT PUBLISH message
 * @param[in] top MQTT Topic (if NULL, previously configured topic is used)
 * @param[in] publish Message payload (Max length : 3072 bytes)
 * @return 0 - succeed / -1 - mqtt not connected / -2 - in-flight msg exists
 *        -3 - mqtt topic missing / -other - see mqtt_client_error_code
 ****************************************************************************************
 */
int		mqtt_client_send_message(char *top, char *publish);

/**
 ****************************************************************************************
 * @brief Send MQTT PUBLISH message with timeout check
 * @param[in] top MQTT Topic (if NULL, previously configured topic is used)
 * @param[in] publish Message payload (Max length : 3072 bytes)
 * @param[in] timeout timeout to wait for QoS messages
 * @return 0 - succeed to send / -1 - failure / -2 - timeout
 ****************************************************************************************
 */
int		mqtt_client_send_message_with_qos(char *top, char *publish, ULONG timeout);

/**
 ****************************************************************************************
 * @brief Set the publisher topic
 * @param[in] topic MQTT Publisher Topic (Max length: 64 bytes)
 * @return 0 - succeed to set / -1 - failure
 ****************************************************************************************
 */
UINT	mqtt_client_add_pub_topic(char *topic);

/**
 ****************************************************************************************
 * @brief Add a subscriber topic
 * @param[in] topic MQTT Subscriber Topic (Max length: 64 bytes, Max number of topic: 4)
 * @param[in] cache Save to cache only if 1 
 * @return 0 - succeed to add / -1 - failure
 ****************************************************************************************
 */
UINT	mqtt_client_add_sub_topic(char *topic, int cache);

/**
 ****************************************************************************************
 * @brief Delete a subscriber topic
 * @param[in] topic MQTT Subscriber Topic to delete
 * @param[in] cache Save to cache only if 1
 * @return 0 - entry found, delete success / 100 - entry not found / 
 *         101 - nvram driver call error
 ****************************************************************************************
 */
UINT	mqtt_client_del_sub_topic(char *topic, int cache);

/**
 ****************************************************************************************
 * @brief Unsubscribe from the topic specified
 * @param[in] topic MQTT Subscriber Topic to unsubscribe
 * @return 0 - succeed to unsubscribe / other - failure (see mqtt_client_error_code)
 ****************************************************************************************
 */
int	mqtt_client_unsub_topic(char *topic);

/**
 ****************************************************************************************
 * @brief Set the broker information (broker ip address & port)
 * @param[in] broker_ip Broker address (IP and domain both available)
 * @param[in] port Broker port number
 * @return 0 - succeed to set / -1 - failure
 ****************************************************************************************
 */
UINT	mqtt_client_set_broker_info(char *broker_ip, int port);

/**
 ****************************************************************************************
 * @brief Set auto_start parameter
 * @param[in] auto_start Automatic startup at system boot if 1.
 * @return 0 - succeed to set / -1 - failure
 ****************************************************************************************
 */
UINT	mqtt_client_set_auto_start(int auto_start);

/**
 ****************************************************************************************
 * @brief Initialize all configuration. 
 * @return 0 - succeed to set / -1 - failure
 ****************************************************************************************
 */
int		mqtt_client_config_initialize(void);

UINT	mqtt_client_config(int argc, char *argv[]);

/**
 ****************************************************************************************
 * @brief Register callback function called when MQTT message PUBLISH is done
 * a message completely
 * @param[in] user_cb Callback function to register
 ****************************************************************************************
 */
void	mqtt_pub_callback_set(void (*user_cb)(void));

/**
 ****************************************************************************************
 * @brief Register callback function called when MQTT Subscriber is connected completely
 * @param[in] user_cb Callback function to register
 ****************************************************************************************
 */
void	mqtt_sub_callback_set(void (*user_cb)(void));

/**
 ****************************************************************************************
 * @brief Register callback function called when MQTT Subscriber is disconnected
 * @param[in] user_cb Callback function to register
 ****************************************************************************************
 */
void	mqtt_sub_disconn_cb_set(void (*user_cb)(void));

#if defined (__MQTT_CLEAN_SESSION_MODE_SUPPORT__)
/**
 ****************************************************************************************
 * @brief Register callback function called when MQTT Subscriber is disconnected by 
 * receiving a message with invalid length if the connection is clean_session=0 and qos>0.
 * On receipt of this callback, application needs to clear the message in Broker 
 * by connecting with clean_sesion=1.
 * @param[in] user_cb Callback function to register
 ****************************************************************************************
 */
void	mqtt_sub_disconn2_cb_set(void (*user_cb)(void));
#endif // __MQTT_CLEAN_SESSION_MODE_SUPPORT__

/**
 ****************************************************************************************
 * @brief Register callback function called when MQTT Subscriber receive a PUBLISH
 * @param[in] user_cb Callback function to register
 * @param[in] buf PUBLISH message received
 * @param[in] len PUBLISH message length
 * @param[in] topic The topic of PUBLISH message
 ****************************************************************************************
 */
void	mqtt_msg_callback_set(void (*user_cb)(const char *buf, int len, const char *topic));

/**
 ****************************************************************************************
 * @brief Register callback function called when SUBSCRIBE request to a topic is finished
 * @param[in] user_cb Callback function to register
 ****************************************************************************************
 */
void	mqtt_subscribe_callback_set(void (*user_cb)(void));

/**
 ****************************************************************************************
 * @brief Register callback function called when UNSUBSCRIBE request is finished
 * @param[in] user_cb Callback function to register
 ****************************************************************************************
 */
void	mqtt_unsubscribe_callback_set(void (*user_cb)(void));


void	mqtt_client_create_thread(void * pvParameters);
void	mqtt_client_terminate_thread(void);

/**
 ****************************************************************************************
 * @brief Check whether the MQTT Client is connected or not
 * @return true(1) - Connected / false(0) - Not connected
 ****************************************************************************************
 */
int		mqtt_client_check_conn(void);
int		mqtt_client_check_sub_conn(void);

#define mqtt_sub_create_thread			mqtt_client_start_sub
#define mqtt_sub_term_thread			mqtt_client_stop_sub

#define mqtt_pub_create_thread			mqtt_client_start_pub
#define mqtt_pub_term_thread			mqtt_client_stop_pub

#define mqtt_pub_send_msg				mqtt_client_send_message
#define mqtt_pub_send_msg_with_qos		mqtt_client_send_message_with_qos

#define mqtt_auto_start					mqtt_client_create_thread
#define mqtt_client_termination			mqtt_client_terminate_thread

#define mqtt_sub_callback_set			mqtt_client_set_conn_cb
#define mqtt_pub_callback_set			mqtt_client_set_pub_cb
#define mqtt_msg_callback_set			mqtt_client_set_msg_cb
#define mqtt_sub_disconn_cb_set			mqtt_client_set_disconn_cb
#if defined (__MQTT_CLEAN_SESSION_MODE_SUPPORT__)
#define mqtt_sub_disconn2_cb_set		mqtt_client_set_disconn2_cb
#endif
#define mqtt_subscribe_callback_set		mqtt_client_set_subscribe_cb
#define mqtt_unsubscribe_callback_set	mqtt_client_set_unsubscribe_cb

#endif	/* __MQTT_CLIENT_H__ */
