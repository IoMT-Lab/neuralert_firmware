/**
 ****************************************************************************************
 *
 * @file neuralert.c
 *
 * @brief Main application code
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
#include "common_def.h"
#include "common_config.h"
#include "da16x_network_common.h"
#include "da16x_system.h"
#include "da16x_time.h"
#include "da16200_map.h"
#include "iface_defs.h"
#include "lwip/sockets.h"
#include "task.h"
#include "user_dpm.h"
#include "user_dpm_api.h"
#ifdef CFG_USE_RETMEM_WITHOUT_DPM
#include "user_retmem.h"
#endif //CFG_USE_RETMEM_WITHOUT_DPM
#include "mqtt_client.h"
#include "user_nvram_cmd_table.h"
#include "util_api.h"
#include "limits.h"
#include "W25QXX.h"
#include "Mc363x.h"
#include "adc.h"
#include "common.h"
// FreeRTOSConfig included for info about tick timing
#include "app_common_util.h"
#include "json.h"
#include "user_version.h"
#include "da16x_cert.h"


/**
 ********************************************************************************
 * MACROS AND DEFINITIONS
 *******************************************************************************
 */

/* Bit operators */
#define SET_BIT(src, bit)				(src |= bit)
#define CLR_BIT(src, bit)				(src &= (~bit))
#define BIT_SET(src, bit)				((src & bit) == bit)

/* Events */
#define USER_TERMINATE_TRANSMISSION_EVENT		(1 << 0)
#define USER_BOOTUP_EVENT						(1 << 1)
#define USER_WAKEUP_BY_RTCKEY_EVENT				(1 << 2)
#define USER_MISSED_RTCKEY_EVENT				(1 << 3)
#define USER_ATTEMPT_TRANSMIT_EVENT				(1 << 4)
#define USER_WIFI_CONNECT_COMPLETE_EVENT		(1 << 5)
#define USER_SLEEP_READY_EVENT					(1 << 6)

/* Process list */
#define USER_PROCESS_HANDLE_RTCKEY				(1 << 0)
#define USER_PROCESS_HANDLE_TIMER				(1 << 1)
#define USER_PROCESS_SEND_DATA					(1 << 2)
#define USER_PROCESS_MQTT_TRANSMIT				(1 << 3)
#define USER_PROCESS_MQTT_STOP					(1 << 4)
#define USER_PROCESS_WATCHDOG					(1 << 5)
#define	USER_PROCESS_WATCHDOG_STOP				(1 << 6)
#define USER_PROCESS_BLOCK_MQTT					(1 << 7)
#define USER_PROCESS_BOOTUP						(1 << 8)
#define USER_PROCESS_SEMAPHORE_ERROR			(1 << 9)

/* Retention Memory */
#define USER_RTM_DATA_TAG						"uRtmData"

/* Provisioning key */
#define USER_PROVISIONING_KEY "E72ttV0Ydp3"

/*
 * Accelerometer buffer (AB) setup
 */
// # of times the write function will attempt the write/verify cycle
// This includes the first try and subsequent retries
#define AB_WRITE_MAX_ATTEMPTS 4
// # of times the erase sector function will attempt the erase/verify cycle
#define AB_ERASE_MAX_ATTEMPTS 3


/*
 * Defines used by the MQTT transmission
 * The maximum sent in one packet should be a multiple of the acceleromter
 * FIFO buffer trip point (currently 32) since data is stored in blocks
 * consisting of FIFO reads
 *
 */
#define SAMPLES_PER_FIFO				32

// length of the timesync string
// timesync is structured as: "2025.02.12 17:30:05 (GMT +0:00) 000029405"
#define MAX_TIMESYNC_LENGTH				80


// How long to wait for a WIFI AND MQTT connection each time the tx task starts up
// This is just a task for how long it takes to connect -- not transmit.
// The premise is that if it takes too long to connect, the wifi connection is probably pretty
// poor and we should wait until later. 30 Seconds is pretty reasonable.  This is based on the
// Inteprod implementation (which basically kept the wifi on for 30 seconds regardless of
// the transmission) and still achieved a 5 day battery lifetime.
// This has been implemented as a software watchdog -- hints the change in name
#define WATCHDOG_TIMEOUT_SECONDS 30

// How long to wait for the software watchdog to shutdown.
#define WATCHDOG_STOP_TIMEOUT_SECONDS 5


// MQTT as of 5.0 prohibits retries when using a clean session (which we do).
// The low power da16200 and da16600 seem to struggle with maintaining a clean TCP connection.
// This causes packets to not pass the checksum at the broker (or the PUBACK isn't received).
// Either way, given that if we can start the MQTT client, that means a TLS handshake occurred,
// and the wifi was strong enough to get the connection established.  In this scenario, and
// since getting the data to the broker is critical in our application, we would like to "retry"
// by restarting the MQTT client when a packet timeout occurs.  Since the TLS handshake can
// incur a 15 second delay, we will limit the number of retries per transmit cycle and
// NOT per packet.
#define MQTT_MAX_ATTEMPTS_PER_TX 3



// Trigger value for the accelerometer to start MQTT transmission
//  16 ~= 1 minutes
//  80 ~= 5 minutes
// ***NOTE*** because the MQTT task WIFI activity appears to interfere
// with SPI bus functions and especially the erase that happens every
// 16 pages of FLASH writing, it is believed that having this be a
// multiple of 16 will reduce the interference because the AXL task
// erases the next flash sector before it starts the MQTT task
// That means that the MQTT task will have 16 AXL interrupt times to
// operate in before the next erase sector time
#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_FAST 16 // 1 minute (assuming 7 Hz AXL sampling rate)
#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_SLOW 80 // 5 minutes (assuming 7 Hz AXL sampling rate)


// This value if for the first transmission, which we want to occur relatively quickly after bootup.
// each FIFO trigger is about 4 seconds, so if the following is set to 3, it will start the transmit
// process within approximately 12 seconds.
#define MQTT_FIRST_TRANSMIT_TRIGGER_FIFO_BUFFERS 2

// This value is for switching between fast and slow transmit intervals.  When the number of unsuccessful
// transmissions is less than this value, the fast mode will be used.  Otherwise, the slow mode.
#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_TEST 10 // about 10 fast attempts before we revert to slow

// This is how long to wait for MQTT to stop before shutting the wifi down
// regardless of whether MQTT has cleanly exited.
#define MQTT_STOP_TIMEOUT_SECONDS 3

// How many accelerometer FIFO blocks to send in each JSON packet
// (This is arbitrary but limited by the max MQTT packet size)
// The larger this value, the fewer JSON packets need to be sent
// and the shorter the MQTT transmission cycle will be.
// In Stage6c, Step 6 development, the following times were observed:
// (With an inter-packet
// delay of 2 seconds and total post_transmission delays of 3 seconds.)
// Packet size 10 (14 packets) took 47 seconds
// Packet size 20 (7 packets) took 31 seconds (JSON packet ~= 10000 bytes)
// Packet size 30 (5 packets) took 28 seconds (JSON packet ~= 15000 bytes)
// Make sure the JSON message buffer is big enough!
// Note that during testing it was discovered that 30 block packets
// were causing errors in the MQTT publish client.  We didn't try anything
// between 20 and 30, so a little larger might be possible.
// #define FIFO_BLOCKS_PER_PACKET 24 // used in 1.9
#define FIFO_BLOCKS_PER_PACKET 5 // Smaller numbers work better, used in 1.10

// How many actual samples to be sent in each JSON packet
// # samples in the Accel FIFO times blocks per JSON packet
// The most individual samples we'll transmit in one JSON packet
#define MAX_SAMPLES_PER_PACKET	(FIFO_BLOCKS_PER_PACKET * SAMPLES_PER_FIFO)
// How long to wait for the PUBACK from the broker before giving up
// when sending MQTT message with QOS 1 or 2
// (Note that in SDK call, timeout is in 10s of milliseconds)
//#define MQTT_QOS_TIMEOUT_MS 5000 //-- this was in the 1.9 release
//  JW: Note, that there is currently a race condition in SDK 3.2.8.1 (and 3.2.2.0).
//  This race condition has been solved by adding a delay in:
// mqtt_client_send_message_with_qos(...) in sub_client.c
// In that function, the QoS uses 10s of ticks each loop (not 10s of milliseconds)
#define MQTT_QOS_TIMEOUT_MS 2000 // 1000 sometimes misses a PUBACK
// How long to wait for the MQTT client to subscribe to topics prior to giving up
// If we can't subscribe (for whatever reason) it is going to be really hard to
// publish.
#define MQTT_SUB_TIMEOUT_SECONDS 5

#define	TEST_DBG_DUMP(tag, buf, siz)	thread_peripheral_dump( # buf , buf , siz)


#ifdef CFG_USE_RETMEM_WITHOUT_DPM


/*
 * User data area in retention memory
 *
 * Note entire area is set to zero at boot time when it is allocated
 *******************************************************************/
typedef struct userData {

	// *****************************************************
	// Timekeeping for FIFO read cycles
	// *****************************************************
	ULONG FIFO_reads_this_power_cycle;
	// The following variable keeps track of the most recent timestamp
	// assigned to a FIFO buffer
	__time64_t last_FIFO_read_time_ms;
	// The following records when we last owk
	__time64_t last_wakeup_msec;

	// *****************************************************
	// MQTT transmission info
	// *****************************************************
	int MQTT_pub_msg_id;   // serial number for messages (must be an int)
	unsigned int MQTT_transmission_number; // serial number for a transmission event (multiple messages)
	unsigned int MQTT_stats_connect_attempts;	// stats: # times we tried
	unsigned int MQTT_stats_connect_fails;	// # times we've failed to connect
	unsigned int MQTT_stats_packets_sent;	// # times we've successfully sent a packet
	unsigned int MQTT_stats_packets_sent_last; // the value of MQTT_stats_packets_sent the last time we checked.
	unsigned int MQTT_stats_retry_attempts; // # times we've attempted a transmit retry
	unsigned int MQTT_stats_transmit_success;	// # times we were successful at uploading all the data
	unsigned int MQTT_attempts_since_tx_success;  // # times MQTT has consecutively failed to transmit
	int MQTT_tx_attempts_remaining; // # times MQTT can attempt to send this packet
	unsigned int MQTT_inflight;			// indicates the number of inflight messages

	// Time synchronization information
	// A time snapshot is taken on the first successful MQTT connection
	// and provided in all subsequent transmissions in order to synchronize
	int16_t MQTT_timesync_captured;		// 0 if not captured; 1 otherwise
	char MQTT_timesync_current_time_str[MAX_TIMESYNC_LENGTH];	// local time in string

	// *****************************************************
	// Unique device identifier
	// *****************************************************
	char Device_ID[10];			// Unique device identifier used for publish & subscribe

	// *****************************************************
	// Accelerometer info
	// *****************************************************
	unsigned int ACCEL_read_count;			// how many FIFO reads total
	unsigned int ACCEL_transmit_trigger;	// count of FIFOs to start transmit
	unsigned int ACCEL_missed_interrupts;	// How many times we detected full FIFO by polling

	// *****************************************************
	// External data flash (AB memory) statistics
	// *****************************************************
	unsigned int write_fault_count;		// # of times we failed to verify flash write since power up
	unsigned int write_retry_count;		// # of times we needed to retry the write since power up
	unsigned int write_attempt_events[AB_WRITE_MAX_ATTEMPTS];	// histogram of how often we had to retry

	unsigned int erase_attempts;			// # of times we attempted to erase sector
	unsigned int erase_fault_count;		// # of times we failed to verify flash erase since power up
	unsigned int erase_retry_count;		// # of times we needed to retry the erase since power up
	unsigned int erase_attempt_events[AB_ERASE_MAX_ATTEMPTS];	// histogram of how often we had to retry


	// *****************************************************
	// Accelerometer buffer management
	// *****************************************************
	AB_INDEX_TYPE next_AB_write_position;
	_AB_transmit_map_t AB_transmit_map[AB_TRANSMIT_MAP_SIZE]; //

} UserDataBuffer;
#endif

/* Process event type */
enum process_event {
	PROCESS_EVENT_TERMINATE,
	PROCESS_EVENT_CONTINUE,
};


/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */
// static int timer_id = 0;
/*
 * Process tracking bitmap
 */
static UINT32 processLists = 0;					// bitmap of active processes
static TaskHandle_t xTask = NULL;
static TaskHandle_t user_MQTT_task_handle = NULL;  // task handle of the user MQTT transmit task
static TaskHandle_t user_MQTT_stop_task_handle = NULL;  // task handle of the user MQTT stop task
static TaskHandle_t user_watchdog_task_handle = NULL; // task handle of the user watchdog task

/*
 * Information about why we're awake.  Among other things, used to figure out
 * how best to assign a timestamp to the accelerometer readings
 *
 *  Reasons we've wakened are:
 *     1. Power-on boot/initialization
 *     2. Wakened from sleep by accelerometer interrupt (RTC_KEY event)
 *     3. Timer (legacy from original Dialog design)
 *
 *  Once we are awake, we may receive an accelerometer interrupt (RTCKEY in timer event)
 *  or the event loop may discover by polling that we've missed an accelerometer
 *  interrupt and trigger an accelerometer read cycle.
 *
 *  So there are three ways that we might find ourselves reading the accelerometer:
 *     1. Wakened from sleep
 *     2. Received an interrupt while still awake (usually when doing MQTT transmission)
 *     3. Missed interrupt detected by event loop (usually when doing MQTT transmission)
 *
 */
static UINT8 isSysNormalBoot = pdFALSE;			// pdTRUE when power-on boot (first time boot)
// note that the following versions of these are turned on and off so that
// the accelerometer knows why it's reading and can assign an appropriate timestamp
static UINT8 isPowerOnBoot = pdFALSE;			// pdTRUE when power-on boot (first time boot)
static UINT8 isAccelerometerWakeup = pdFALSE;	// pdTRUE when wakened from sleep by accelerometer interrupt
static UINT8 isAccelerometerInterrupt = pdFALSE;// pdTRUE when interrupt while awake
static UINT8 isAccelerometerTimeout = pdFALSE;	// pdTRUE when missed interrupt detected


/*
 * Semaphore to coordinate between the accelerometer main task and
 * the MQTT transmit task with respect to accessing the flash.
 * This is used to provide exclusive access
 * to the flash so we don't accidentally overlap each other
 */
SemaphoreHandle_t Flash_semaphore = NULL;
/*
 * Semaphore to coordinate user data in NVRAM.
 * This is used so we don't accidentally execute simultaneous read/writes
 */
SemaphoreHandle_t User_semaphore = NULL;
/*
 * Semaphore to coordinate between processLists between threads.
 */
SemaphoreHandle_t Process_semaphore = NULL;

/*
 * Temporary storage for accelerometer samples to be transmitted
 * Created at transmit time from the stored FIFO structures
 */
static accelDataStruct accelXmitData[MAX_SAMPLES_PER_PACKET];



/*
 * Area in which to compose JSON packet
 * see spreadsheet for sizing
 */
#define MAX_JSON_STRING_SIZE 10000 // double our usual packet size
char mqttMessage[MAX_JSON_STRING_SIZE];



#ifdef CFG_USE_RETMEM_WITHOUT_DPM
/*
 * Pointer to the user data area in retention memory
 */
static UserDataBuffer *pUserData = NULL;
#endif

// Macros for setting, clearing, and checking system states
// Only for use by the set & clear functions, which also
// make sure the LED reflects the new state information
#define SET_SYSTEM_STATE_BIT(bit)		(pUserData->system_state_map |= bit)
#define CLR_SYSTEM_STATE_BIT(bit)		(pUserData->system_state_map &= (~bit))
#define BIT_SYSTEM_STATE_SET(bit)		((pUserData->system_state_map & bit) == bit)

// Macros for converting from RTC clock ticks (msec * 32768) to microseconds
// and milliseconds
#define CLK2US(clk)			((((unsigned long long )clk) * 15625ULL) >> 9ULL)
#define CLK2MS(clk)			((CLK2US(clk))/1000ULL)


/*
 * Application Program Interface
 *******************************************************************************
 */
void user_start_MQTT_client(); // used to start the MQTT client from user_apps.c


/*
 * STATIC FUNCTIONS DEFINITIONS (forward declarations)
 *******************************************************************************
 */
static int clear_AB_position(int, int);
static void user_create_MQTT_task(void);
static void user_create_MQTT_stop_task(void);
static int user_mqtt_send_message(void);
static int take_semaphore(SemaphoreHandle_t *);
static void timesync_snapshot(void);
static void user_reboot(void);


/*
 * EXTERN FUNCTIONS DEFINITIONS
 *******************************************************************************
 */
extern void da16x_time64_sec(__time64_t *p, __time64_t *cur_sec);
extern int get_current_rssi(int iface);
extern void start_LED_timer(); //TODO: remove this?
extern unsigned char get_fault_count(void);
extern void system_control_wlan_enable(uint8_t onoff);
extern int fc80211_set_app_keepalivetime(unsigned char tid, unsigned int sec,
										void (*callback_func)(unsigned int tid));


/**
 *******************************************************************************
 * @brief Function to get the milliseconds since power on
 *******************************************************************************
 */
static void user_time64_msec_since_poweron(__time64_t *cur_msec) {
	unsigned long long time_ms;
	unsigned long long rtc;

	rtc = RTC_GET_COUNTER();

	time_ms = CLK2MS(rtc); /* msec. */

	*cur_msec = time_ms; /* msec */
}


/**
 *******************************************************************************
 * @brief Function to manage what is displayed on the LEDs,
 * 			based on the current system state and alerts
 *******************************************************************************
 */
static void notify_user_LED()
{

	// JW: This functionality has been replaced.  We don't want patients to get alerts
	// based on device functionality.
#if 0
	// First, check for a system alert, since they take
	// precedence over the basic states
	if (0 != pUserData->system_alert_map)
	{
		// Check alerts in order of priority
		if (SYSTEM_ALERT_BIT_ACTIVE(USER_ALERT_FATAL_ERROR))
		{
			// Fast red blink
			setLEDState(RED, LED_FAST, 200, 0, LED_OFF, 0, 200);
		}
		JW: This chunk of code is deprecated
		else if (SYSTEM_ALERT_BIT_ACTIVE(USER_ALERT_BATTERY_LOW))
		{
			// Slow yellow blink
			setLEDState(YELLOW, LED_SLOW, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_ALERT_BIT_ACTIVE(USER_ALERT_AB_STORAGE_LOW))
		{
			// Slow yellow blink
			setLEDState(YELLOW, LED_FAST, 200, 0, LED_OFF, 0, 200);
		}
		else // Unkown alert?
		{
			// Slow yellow blink
			setLEDState(PURPLE, LED_FAST, 200, RED, LED_FAST, 200, 200);
		}

	}  // If alert is active
	else
	{
		// No alerts, so do system states in order of priority
		if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_INTERNAL_ERROR))
		{
			setLEDState(PURPLE, LED_ON, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_BATTERY_EXHAUSTED))
		{
			setLEDState(RED, LED_SLOW, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_PROVISIONING))
		{
			// Fast yellow blink
			setLEDState(CYAN, LED_SLOW, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_WIFI_CONNECT_FAILED))
		{
			// Fast yellow blink
			setLEDState(YELLOW, LED_FAST, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_WIFI_CONNECTING))
		{
			// Slow blue blink
			setLEDState(BLUE, LED_SLOW, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_WIFI_CONNECTED))
		{
			setLEDState(GREEN, LED_ON, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_WIFI_TRANSMITTING))
		{
			setLEDState(GREEN, LED_SLOW, 200, 0, LED_OFF, 0, 200);
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_POWER_ON_BOOTUP))
		{
			setLEDState(BLUE, LED_FAST, 200, 0, LED_OFF, 0, 200);
		}
		else // No known state
		{
			setLEDState(0, LED_OFF, 0, 0, LED_OFF, 0, 200);
		}
	}// system state
#endif // JW: deprecated 10.4

#if 0
	if (0 != pUserData->system_state_map)
		if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_WIFI_CONNECT_FAILED))
		{
			setLEDState(YELLOW, LED_FAST, 200, 0, LED_OFFX, 0, 200); // fast blink yellow
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_WIFI_CONNECTED))
		{
			setLEDState(GREEN, LED_FAST, 200, 0, LED_OFFX, 0, 200); // fast blink green
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_POWER_ON_BOOTUP))
		{
			setLEDState(BLUE, LED_FAST, 200, 0, LED_OFFX, 0, 200); // fast blink blue
		}
		else if (SYSTEM_STATE_BIT_ACTIVE(USER_STATE_NO_DATA_COLLECTION))
		{
			setLEDState(CYAN, LED_ONX, 200, 0, LED_OFFX, 0, 200); // steady cyan
		}
		else // No known state
		{
			setLEDState(BLACK, LED_OFFX, 0, 0, LED_OFFX, 0, 200);
		}
	else
	{
		setLEDState(BLACK, LED_OFFX, 0, 0, LED_OFFX, 0, 200);
	}
#endif//JW: deprecated 10.14

	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process bit", __func__);
	} else {
		if (BIT_SET(processLists, USER_PROCESS_BOOTUP))
		{
			setLEDState(BLUE, LED_SLOW, 200, 0, LED_OFFX, 0, 3600);
		}
		else // No known state
		{
			setLEDState(BLACK, LED_OFFX, 0, 0, LED_OFFX, 0, 200);
		}
		xSemaphoreGive(Process_semaphore);
	}



}


/**
 *******************************************************************************
 * @brief Check if the system is blocking mqtt access (done at bootup)
 *******************************************************************************
 */

UINT8 check_mqtt_block()
{
	UINT8 ret = pdTRUE;
	// check if the system state is clear
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore", __func__); // return pdTRUE
	} else {
		if (BIT_SET(processLists, USER_PROCESS_BLOCK_MQTT)) {
			ret = pdTRUE;
		} else {
			ret = pdFALSE;
		}
		xSemaphoreGive(Process_semaphore);
	}

	return ret;
}



/**
 *******************************************************************************
 * @brief Process similar to strncpy but makes sure there is a
 * terminating null
 * copies up to (max_len - 1) characters and adds a null terminator
 * *******************************************************************************
 */
void user_text_copy(UCHAR *to_string, const UCHAR *from_string, const int max_len)
{
	int i;
	for (i=0; (i < (max_len-1)); i++)
	{
		if(from_string[i] == 0x00)
			break;
		else
			to_string[i] = from_string[i];
	}
	to_string[i] = 0x00;
}



/**
 ****************************************************************************************
 * @brief mqtt_client callback function for processing PUBLISH messages \n
 * Users register a callback function to process a PUBLISH message. \n
 * In this example, when mqtt_client receives a message with payload "1",
 * it sends MQTT PUBLISH with payload "DA16K status : ..." to the broker connected.
 * @param[in] buf the message paylod
 * @param[in] len the message paylod length
 * @param[in] topic the topic this mqtt_client subscribed to
 ****************************************************************************************
 */
void my_app_mqtt_msg_cb(const char *buf, int len, const char *topic)
{
    DA16X_UNUSED_ARG(len);

#if 0
    BaseType_t ret;

    PRINTF(CYAN_COLOR "[MQTT_SAMPLE] Msg Recv: Topic=%s, Msg=%s \n" CLEAR_COLOR, topic, buf);

    if (strcmp(buf, "reply_needed") == 0) {
        if ((ret = my_app_send_to_q(NAME_JOB_MQTT_TX_REPLY, &tx_reply, APP_MSG_PUBLISH, NULL)) != pdPASS ) {
            PRINTF(RED_COLOR "[%s] Failed to add a message to Q (%d)\r\n" CLEAR_COLOR, __func__, ret);
        }
    } else if (strncmp(buf, APP_UNSUB_HDR, 6) == 0) {
        if ((ret = my_app_send_to_q(NAME_JOB_MQTT_UNSUB, NULL, APP_MSG_UNSUB, buf)) != pdPASS ) {
            PRINTF(RED_COLOR "[%s] Failed to add a message to Q (%d)\r\n" CLEAR_COLOR, __func__, ret);
        }
    } else {
        return;
    }
#endif
}

void user_mqtt_pub_cb(int mid)
{
	PRINTF("\n Neuralert: [%s] MQTT PUB callback, clearing inflight", __func__);

	// See user_mqtt_client_send_message_with_qos for what we're doing here.
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		// do nothing, a timeout will occur and the packet will be re-transmitted
	} else {
		pUserData->MQTT_inflight = 0; // clear the inflight message counter
		xSemaphoreGive(User_semaphore);
	}
	vTaskDelay(1);
}

void user_mqtt_conn_cb(void)
{
	PRINTF("\nNeuralert: [%s] MQTT CONNECTED",__func__);
	// Now that the mqtt connection is established, we can create our MQTT transmission task
	//vTaskDelay(10);
	user_create_MQTT_task();
	vTaskDelay(1);
}

void my_app_mqtt_sub_cb(void)
{
#if 0
	topic_count++;
    if (dpm_mode_is_enabled() && topic_count == mqtt_client_get_topic_count()) {
        my_app_send_to_q(NAME_JOB_MQTT_PERIODIC_PUB_RTC_REGI, NULL, APP_MSG_REGI_RTC, NULL);
    }
#endif
}




/**
 *******************************************************************************
 * @brief Send factory reset btn pushed once
 *******************************************************************************
 */
int user_factory_reset_btn_onetouch(void)
{
	PRINTF("\n\n**** Factory Reset Button One Touch ***\n\n");
	return pdTRUE;
}


/**
 *******************************************************************************
 * @brief Process to clear the accelerometer buffer management
 * transmit map location, making sure it's done with exclusive access
 *  Returns pdTRUE if unable to gain exclusive access
 *  returns pdFALSE otherwise
 *******************************************************************************
 */
static int clear_AB_position(int start_location, int end_location)
{
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking user semaphore");
		return pdTRUE;
	} else {
		// clear transmit map from start to end
		for (int i=start_location; i<=end_location; i++)
		{
			CLR_AB_POS(pUserData->AB_transmit_map, i);
		}
		xSemaphoreGive(User_semaphore);
	}

	return pdFALSE;
}



/**
 *******************************************************************************
 * @brief Send terminate transmission event to the user task
 *******************************************************************************
 */
void user_terminate_transmission_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_TERMINATE_TRANSMISSION_EVENT, eSetBits);
	}
}



/**
 *******************************************************************************
 * @brief Send boot-up event to the user task
 *******************************************************************************
 */
static void user_send_bootup_event_message(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_BOOTUP_EVENT, eSetBits);
	}
}

/**
 *******************************************************************************
 * @brief Send wake-up by RTC_WAKE_UP key event to the user task
 *******************************************************************************
 */
static void user_wakeup_by_rtckey_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_WAKEUP_BY_RTCKEY_EVENT, eSetBits);
	}
}


/**
 *******************************************************************************
 * @brief Send RTC_WAKE_UP key event during handling RTC timer event
 *******************************************************************************
 */
static void user_rtckey_event_in_timer_handle(void)
{
	isAccelerometerInterrupt = pdTRUE;			// tell AXL why it's reading

	if (xTask) {
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;

		xTaskNotifyIndexedFromISR(xTask, 0, USER_MISSED_RTCKEY_EVENT,
									eSetBits, &xHigherPriorityTaskWoken);

		Printf("========== RTCKEY event in timer handle =======\n");  //FRSDEBUG
	}
}


/**
 *******************************************************************************
 * @brief Send WLAN connection event
 *******************************************************************************
 */
void user_wifi_connection_complete_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_WIFI_CONNECT_COMPLETE_EVENT, eSetBits);
	}
}



/**
 *******************************************************************************
 * @brief Send sleep-ready event to the user task
 *******************************************************************************
 */
static void user_sleep_ready_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_SLEEP_READY_EVENT, eSetBits);
	}
}



/**
 *******************************************************************************
 * @brief Handle the RTC_WAKE_UP event during running.
 *******************************************************************************
 */
void user_rtc_wakeup_key_event_triggered(void)
{
	user_rtckey_event_in_timer_handle();
}


/**
 *******************************************************************************
 * @brief Handle wifi connection completed.
 *******************************************************************************
 */
void user_wifi_connection_completed(void)
{
	PRINTF("\n**Neuralert: [%s] do we care about this any more?\n", __func__); // FRSDEBUG

	user_wifi_connection_complete_event();
}



/*
 * Get battery voltage
 *
 * returns voltage as a float
 *
 * Note ADC has to have been configured during boot time.
 * See function config_pin_mux in user_main.c
 */
static int get_battery_voltage()
{
	/*
	 * Battery voltage
	 */
	uint16_t adcData;
	int adcDataNorm;
	uint16_t write_data;
	uint32_t data;

	// Set Battery voltage enable
	write_data = GPIO_PIN10;
	GPIO_WRITE(gpioa, GPIO_PIN10, &write_data, sizeof(uint16_t));

	vTaskDelay(5);

	// Read battery voltage
	// Note that due to a voltage divider, this measurement
	// isn't the actual voltage.  The divider ratio is, according to Nicholas Joseph, 54/25.
	// JW has confirmed this is probably correct, 140 * 54 / 25 = 302 (3.02 Volts) which seems about right.
 	//
	DRV_ADC_READ(hadc, DA16200_ADC_CH_0, (UINT32 *)&data, 0);
	adcData = (data >> 4) & 0xFFF;
	adcDataNorm = (adcData * VREF) / 4095;


	// turn off battery measurement circuit
	write_data = 0;
	GPIO_WRITE(gpioa, GPIO_PIN10, &write_data, sizeof(uint16_t));   /* disable battery input */

	return adcDataNorm;
}

/**
 *******************************************************************************
 * @brief send one JSON packet from the intermediate samples buffer
 *
 * typedef struct
	{
		int16_t Xvalue;					//!< X-Value * 1000
		int16_t Yvalue;					//!< Y-Value * 1000
		int16_t Zvalue;					//!< Z-Value * 1000
		__time64_t accelTime;               //!< time when accelleromter data taken
	} accelDataStruct;
 *
 *
 *
 *******************************************************************************
 */

int send_json_packet(const int count, const unsigned int transmission, const int sequence)
{

	int return_status = 0;
	int i;
	unsigned char str[80], str2[40];		// temp working strings for assembling
	int16_t Xvalue;
	int16_t Yvalue;
	int16_t Zvalue;
	char device_id[10];
	char timesync[MAX_TIMESYNC_LENGTH];


#ifdef __TIME64__
	__time64_t now;
#else
	time_t now;
#endif /* __TIME64__ */
	struct tm;
	char nowStr[20];
	unsigned char fault_count;

	// get data from user memory
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore",__func__);
		PRINTF("\n Neuralert: [%s] transmit %d:%d unsuccessful", __func__, transmission, sequence);
		return pdTRUE;
	} else {
		strcpy(device_id, pUserData->Device_ID);
		strcpy(timesync, pUserData->MQTT_timesync_current_time_str);
		xSemaphoreGive(User_semaphore);
	}


	/*
	 * JSON preamble
	 */
	strcpy(mqttMessage,"{\r\n\t\"state\":\r\n\t{\r\n\t\t\"reported\":\r\n\t\t{\r\n");
	/*
	 * MAC address of device - stored in retention memory
	 * during the bootup event
	 */
	sprintf(str,"\t\t\t\"id\": \"%s\",\r\n", device_id);
	strcat(mqttMessage,str);


	/* New timesync field added 1/26/23 per ECO approved by Neuralert
	 * Format:
	 *
	 *	The current first JSON packet has this format:
	 *	{ "state": { "reported": { "id": "EB345A", "meta": {},
	 *	"bat": 140,
	 *	"accX": [6 6 6 6 6 6 6 6 6 6 6 6 5 6 6 6 6 6 6 6 5 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 5 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6
	 *
	 *	"timesync": "2023.01.17 12:53:55 (GMT 00:00) 0656741",
	 *
	 *	where the data consists of the current date and time in “local” time, as configured when WIFI is set up.
	 *	The last field (0656741) is an internal timestamp in milliseconds since power-on that corresponds to the
	 *	current local time.  This will make it possible to align timestamps from different devices.
	 *
	 *	Note that the “current” local date and time is that returned from an SNTP server on the Internet
	 *	and is subject to internet lag and internal processing times.  None of which matter practically speaking.
	 *
	 */

	sprintf(str,"\t\t\t\"timesync\": \"%s\",\r\n", timesync);
	strcat(mqttMessage,str);

	/* get battery value */
	int adcData = get_battery_voltage();
	sprintf(str,"\t\t\t\"bat\": %d,\r\n",adcData);
	strcat(mqttMessage,str);

	// META FIELD DEFINITIONS HERE
	// Meta data field -- a json for whatever we want.
	strcat(mqttMessage,"\t\t\t\"meta\":\r\n\t\t\t{\r\n");
	/*
	 * Meta - MAC address of device - stored in retention memory
	 * during the bootup event
	 */
	sprintf(str,"\t\t\t\t\"id\": \"%s\",\r\n",device_id);
	strcat(mqttMessage,str);
	/*
	* Meta - Firmware Version
	*/
	sprintf(str,"\t\t\t\t\"ver\": \"%s\",\r\n", USER_VERSION_STRING);
	strcat(mqttMessage, str);
	/*
	 * Meta - Transmission sequence #
	 */
	sprintf(str,"\t\t\t\t\"trans\": %d,\r\n", transmission);
	strcat(mqttMessage, str);
	/*
	 * Meta - Message sequence this transmission
	 */
	sprintf(str,"\t\t\t\t\"seq\": %d,\r\n", sequence);
	strcat(mqttMessage, str);
	/*
	* Meta - Battery value this transmission
	*/
	sprintf(str,"\t\t\t\t\"bat\": %d,\r\n",adcData);
	strcat(mqttMessage, str);

	/*
	* Meta - Fault count at this transmission
	*/
	fault_count = get_fault_count();
	sprintf(str,"\t\t\t\t\"count\": %d,\r\n", (uint16_t)fault_count);
	strcat(mqttMessage, str);

	/*
	* Meta - time (in minutes) since boot-up
	*/
	__time64_t current_time;
	user_time64_msec_since_poweron(&current_time);
	sprintf(str,"\t\t\t\t\"mins\": %ld,\r\n", (uint32_t)(current_time/60000));
	strcat(mqttMessage, str);

	/*
	* Meta - time (in minutes) since boot-up
	*/
	int rssi = get_current_rssi(WLAN0_IFACE);
	sprintf(str,"\t\t\t\t\"rssi\": %d,\r\n", rssi);
	strcat(mqttMessage, str);

	/*
	 * Meta - Remaining free heap size (to monitor for significant leaks)
	 */
	sprintf(str,"\t\t\t\t\"mem\": %d\r\n", xPortGetFreeHeapSize());
	strcat(mqttMessage, str);

	// End meta data field (close bracket)
	strcat(mqttMessage,"\t\t\t},\r\n");


	/*
	 *  Accelerometer X values
	 */
	sprintf(str,"\t\t\t\"accX\": [");
	strcat(mqttMessage,str);
	for(i=0;i<count;i++)
	{
		Xvalue = accelXmitData[i].Xvalue;
		sprintf(str,"%d ",Xvalue);
		strcat(mqttMessage,str);
	}
	sprintf(str,"],\r\n");
	strcat(mqttMessage,str);

	/*
	 *  Accelerometer Y values
	 */
	sprintf(str,"\t\t\t\"accY\": [");
	strcat(mqttMessage,str);
	for(i=0;i<count;i++)
	{
		Yvalue = accelXmitData[i].Yvalue;
		sprintf(str,"%d ",Yvalue);
		strcat(mqttMessage,str);
	}
	sprintf(str,"],\r\n");
	strcat(mqttMessage,str);

	/*
	 *  Accelerometer Z values
	 */
	sprintf(str,"\t\t\t\"accZ\": [");
	strcat(mqttMessage,str);
	for(i=0;i<count;i++)
	{
		Zvalue = accelXmitData[i].Zvalue;
		sprintf(str,"%d ",Zvalue);
		strcat(mqttMessage,str);
	}
	sprintf(str,"],\r\n");
	strcat(mqttMessage,str);

	/*
	 *  Timestamps
	 */
	sprintf(str,"\t\t\t\"ts\": [");
	strcat(mqttMessage,str);
	// Note - as of 9/2/22 timestamps are in milliseconds,
	// measured from the time the device was booted.
	// So the largest expected timestamp will be at 5 days:
	// 5 days x 24 hours x 60 minutes x 60 seconds x 1000 milliseconds
	// = 432,000,000 msec
	// which will transmit as 432000000
	for(i=0;i<count;i++)
	{
		now = accelXmitData[i].accelTime;
		// Break the timestamp into millions and remainder
		// to be able to use sprintf, which doesn't handle
		// 64-bit numbers.
		uint64_t num1 = ((now/1000000) * 1000000);
		uint64_t num2 = now - num1;
		uint32_t num3 = num2;
		sprintf(nowStr,"%03ld",(long)(now/1000000));
		sprintf(str2,"%06ld ",num3);
		strcat(nowStr,str2);
		strcat(mqttMessage,nowStr);
	}
	sprintf(str,"]\r\n");
	strcat(mqttMessage,str);

	/*
	 * Closing braces
	 */
	strcat(mqttMessage,"\r\n\t\t}\r\n\t}\r\n}\r\n");

	unsigned int packet_len = strlen(mqttMessage);
	PRINTF("\n Neuralert: [%s]: %d total message length", __func__, packet_len);
	// Sanity check in case some future person expands message
	// without increasing buffer size
	if (packet_len > MAX_JSON_STRING_SIZE)
	{
		PRINTF("\nNeuralert: [%s] JSON packet size too big: %d with limit %d", __func__, packet_len, (int)MAX_JSON_STRING_SIZE);
		user_reboot();
	}


	return_status = user_mqtt_send_message();

	if(return_status == 0)
	{
		PRINTF("\n Neuralert: [%s], transmit %d:%d successful", __func__, transmission, sequence);
	}
	else
	{
		PRINTF("\n Neuralert: [%s] transmit %d:%d unsuccessful", __func__, transmission, sequence);
	}
	vTaskDelay(3); // to display the print statements
	return return_status;
}

/**
 *******************************************************************************
 * @brief Helper function to create a printable string from a long long
 *        because printf doesn't handle long longs
 *        Format is just digits.  No commas or anything.
 *******************************************************************************
 */
void time64_string (char *timestamp_str, const __time64_t *timestamp)
{
	__time64_t timestamp_copy;
	char nowStr[20];
	char str2[20];

	// split into high order and lower order pieces
	timestamp_copy = *timestamp;
	uint64_t num1 = ((timestamp_copy/1000000) * 1000000);	// millions
	uint64_t num2 = timestamp_copy - num1;
	uint32_t num3 = num2;

	sprintf(nowStr,"%03ld",(uint32_t)(timestamp_copy/1000000));
	sprintf(str2,"%06ld",num3);
	strcat(nowStr,str2);
	strcpy(timestamp_str,nowStr);

}

/**
 *******************************************************************************
 * @brief Helper function to create a printable string from a long long
 *        time in msec, because printf doesn't handle long longs
 *******************************************************************************
 */
void time64_msec_string (char *time_str, const __time64_t *time_msec)
{

	ULONG time_milliseconds;
	ULONG time_seconds;
	ULONG time_minutes;
	ULONG time_hours;
	ULONG time_days;

	time_seconds = (ULONG)(*time_msec / (__time64_t)1000);
	time_milliseconds = (ULONG)(*time_msec
				  - ((__time64_t)time_seconds * (__time64_t)1000));
	time_minutes = (ULONG)(time_seconds / (ULONG)60);
	time_hours = (ULONG)(time_minutes / (ULONG)60);
	time_days = (ULONG)(time_hours / (ULONG)24);

	time_seconds = time_seconds % (ULONG)60;
	time_minutes = time_minutes % (ULONG)60;
	time_hours = time_hours % (ULONG)24;

	sprintf(time_str,
			"%u Days plus %02u:%02u:%02u.%03u",
		time_days,
		time_hours,
		time_minutes,
		time_seconds,
		time_milliseconds);
}

/**
 *******************************************************************************
 * @brief Helper function to create a printable string from a long long
 *        time in seconds, because printf doesn't handle long longs
 *******************************************************************************
 */
void time64_seconds_string(char *time_str, const __time64_t *time_msec)
{

	ULONG time_milliseconds;
	ULONG time_tenths;
	ULONG time_seconds;

	time_seconds = (ULONG)(*time_msec / (__time64_t)1000);
	time_milliseconds = (ULONG)(*time_msec
				  - ((__time64_t)time_seconds * (__time64_t)1000));
	// At this point, time_milliseconds is between 0 and 999
	// We want x.x format, round to nearest tenth
	// so - for instance, 849 becomes 899 becomes .8
	//                and 850 becomes 900 becomes .9

	PRINTF(" [time64_seconds_string] intermediate msec: %lu", time_milliseconds);

	time_tenths = time_milliseconds + (ULONG)50;
	PRINTF(" [time64_seconds_string] intermediate tenths before division: %lu", time_tenths);
	time_tenths = time_tenths / (ULONG)100;

	sprintf(time_str,
			"%u.%u seconds  %u milliseconds",
		time_seconds,
		time_tenths,
		time_milliseconds);
}




/**
 *******************************************************************************
 * @brief calculate the timestamp for all a sample read from the accelerometer FIFO
 *      by using it's relative position in the buffer.
 *
 *   FIFO_ts 			- is the timestamp assigned to the FIFO buffer when it was read
 *   FIFO_ts_prev		- is the timestamp assigned to the previous FIFO buffer when it was read
 *   offset         	- is the position in the FIFO
 *   FIFO_samples		- is the total number of samples in the FIFO
 *   adjusted_timestamp - is the calculated timestamp to be assigned to the sample
 *                    		with this offset
 *
 *******************************************************************************
 */
void calculate_timestamp_for_sample(const __time64_t *FIFO_ts, const __time64_t *FIFO_ts_prev, int offset,
	int FIFO_samples, __time64_t *adjusted_timestamp)
{
	//__time64_t scaled_timestamp;	// times 1000 for more precise math
	__time64_t scaled_timestamp_prev; // times 1000 for more precise math
	__time64_t scaled_offsettime;	// the time offset of this sample * 1000
	__time64_t adjusted_scaled_timestamp;
	__time64_t rounded_offsettime;

	__time64_t scaled_timestamp = *FIFO_ts;
	scaled_timestamp = scaled_timestamp * (__time64_t)1000;

	scaled_timestamp_prev = *FIFO_ts_prev;
	scaled_timestamp_prev = scaled_timestamp_prev * (__time64_t)1000;

	scaled_offsettime = ((scaled_timestamp - scaled_timestamp_prev) * (__time64_t)(offset + 1)) / (__time64_t)FIFO_samples;

	// offset time * 1000
	adjusted_scaled_timestamp = scaled_timestamp_prev + scaled_offsettime;
	// back to msec.  We round by adding 500 usec before dividing
	rounded_offsettime = (adjusted_scaled_timestamp + (__time64_t)500)
			                  / (__time64_t)1000;

	*adjusted_timestamp = rounded_offsettime;
}

/**
 *******************************************************************************
 * @brief create a table of accelerometer data for transmission in one packet
 *
 * typedef struct
	{
		int16_t Xvalue;					//!< X-Value * 1000
		int16_t Yvalue;					//!< Y-Value * 1000
		int16_t Zvalue;					//!< Z-Value * 1000
		__time64_t accelTime;           //!< time when accelerometer data taken
	} accelDataStruct;
 *
 *returns -1 if an error
 *returns number of samples in data array otherwise
 *
 *******************************************************************************
 */
static int assemble_packet_data (packetDataStruct *packet_data_ptr)
{



	// Initialize packet data
	packet_data_ptr->start_block = packet_data_ptr->next_start_block;
	packet_data_ptr->end_block = INVALID_AB_ADDRESS;
	packet_data_ptr->num_blocks = 0;
	packet_data_ptr->num_samples = 0;
	packet_data_ptr->done_flag = pdFALSE;

	PRINTF("\n\n Neuralert: [%s] assembling packet data starting at %d", __func__, packet_data_ptr->start_block);
	vTaskDelay(3);

	HANDLE SPI = NULL;
	SPI = flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\n Neuralert: [%s] MAJOR SPI ERROR: Unable to open SPI bus handle", __func__);
		vTaskDelay(1);
		return pdTRUE;
	}

	// Note that start_block may be less than or greater than end_block
	// depending on whether we wrap around or not
	// If we're only sending one block, then start_block will be
	// equal to end_block
	int blocknumber = packet_data_ptr->start_block;

	int done = pdFALSE;
	while (!done)
	{
		// calculate the buffer gap
		int buffer_gap = 0;
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
			return pdTRUE;
		} else {
			AB_INDEX_TYPE write_position = pUserData->next_AB_write_position;
			xSemaphoreGive(User_semaphore);

			// calculate the buffer gap
			if (blocknumber >= write_position){
				buffer_gap = blocknumber - write_position;
			} else {
				buffer_gap = AB_FLASH_MAX_PAGES - (write_position - blocknumber);
			}
		}

		// check if the buffer gap is too close for comfort
		if (buffer_gap <= AB_TRANSMIT_SAFETY_GAP){
			packet_data_ptr->done_flag = pdTRUE;
			done = pdTRUE;
			break;
		}

		// check if the next sizeof(_AB_transmit_map_t) bits in the transmit queue are zeros
		// this is done by confirming we are in a new sizeof(_AB_transmit_map_t) chunk of data
		// the easiest way to do this is to check whether one position higher was the last element
		// in a chunk of data.  This is because we are traversing the queue in reverse.
		int data_to_transmit = pdFALSE; // assume there is no data until proven otherwise
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
			return pdTRUE; // error, exit
		} else {
			if (pUserData->AB_transmit_map[blocknumber / sizeof(_AB_transmit_map_t)] != 0)
			{
				data_to_transmit = pdTRUE;
			}
			xSemaphoreGive(User_semaphore);
		}



		if (!data_to_transmit) // the next sizeof(_AB_transmit_map_t) bits are zero
		{
			blocknumber = blocknumber - (int)(sizeof(_AB_transmit_map_t));
			if (blocknumber < 0) {
				blocknumber = blocknumber + AB_FLASH_MAX_PAGES;
			}
		}
		else // there is data to transmit in the sizeof(_AB_transmit_map_t) range near blocknumber
		{
			// find out if this blocknumber is ready for transmission (it could be a neighbor)
			int transmit_flag = pdFALSE;
			if (take_semaphore(&User_semaphore)) {
				PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
				return pdTRUE;
			} else {
				transmit_flag = POS_AB_SET(pUserData->AB_transmit_map, blocknumber);
				xSemaphoreGive(User_semaphore);
			}

			if (transmit_flag) {
				// The current blocknumber is ready for transmission, Read the block from Flash
				// For each block, assemble the XYZ data and assign a timestamp
				// based on the block timestamp and the samples relation to
				// when that timestamp was taken
				// Calculate address of next sector to write
				ULONG blockaddr = (ULONG) AB_FLASH_BEGIN_ADDRESS +
								  ((ULONG) AB_FLASH_PAGE_SIZE * (ULONG) blocknumber);
				int read_success = pdFALSE;
				accelBufferStruct FIFOblock;
				int retry_count = 0;
				for (retry_count = 0; retry_count < 3; retry_count++)
				{
					FIFOblock.num_samples = 0;
					if (take_semaphore(&Flash_semaphore)) {
						PRINTF("\n Neuralert [%s] error taking flash semaphore", __func__);
						// do nothing (we have a retry loop)
					} else {

						int spi_status = pageRead(SPI, blockaddr, (UINT8 *) &FIFOblock, sizeof(accelBufferStruct));
						xSemaphoreGive(Flash_semaphore);
						if(spi_status < 0){
							PRINTF("\n Neuralert: [%s] unable to read block %d addr: %x\n", __func__, blocknumber, blockaddr);
							// do nothing (we have a retry loop)
						} else {
							read_success = pdTRUE;
						}
						vTaskDelay(3);
					}

					if(FIFOblock.num_samples > 0)
					{
						break;
					}
				}


				if (!read_success) {
					return pdTRUE; // failed to read, exit.
				} else if (retry_count > 0)
				{
					PRINTF("\n Neuralert [%s] retried read %d times", __func__, retry_count);
				}

				// Only process FIFOblock if the data is real -- otherwise, skip block and proceed.
				if (FIFOblock.num_samples > 0)
				{
					char block_timestamp_str[20];
					// add the samples to the transmit array
					__time64_t block_timestamp = FIFOblock.accelTime;
					__time64_t block_timestamp_prev = FIFOblock.accelTime_prev;
					__time64_t sample_timestamp;
					time64_string (block_timestamp_str, &block_timestamp);
					for (int i = 0; i<FIFOblock.num_samples; i++)
					{
						accelXmitData[packet_data_ptr->num_samples].Xvalue = (int16_t)FIFOblock.Xvalue[i];
						accelXmitData[packet_data_ptr->num_samples].Yvalue = (int16_t)FIFOblock.Yvalue[i];
						accelXmitData[packet_data_ptr->num_samples].Zvalue = (int16_t)FIFOblock.Zvalue[i];
						calculate_timestamp_for_sample(&block_timestamp, &block_timestamp_prev,
								i, FIFOblock.num_samples, &sample_timestamp);
						accelXmitData[packet_data_ptr->num_samples].accelTime = sample_timestamp;
						packet_data_ptr->num_samples++;
					}

					packet_data_ptr->num_blocks++;
					if (packet_data_ptr->num_blocks == FIFO_BLOCKS_PER_PACKET)
					{
						done = pdTRUE;
					}
				} // FIFOblock.num_samples > 0
			} //transmit_flag

			// step to next block -- during transmission, we go backwards
			blocknumber--;
			if (blocknumber < 0) {
				blocknumber = blocknumber + AB_FLASH_MAX_PAGES;
			}

		} // data_to_transmit
	} // while loop for processing data

	flash_close(SPI);

	packet_data_ptr->next_start_block = blocknumber;
	packet_data_ptr->end_block = (blocknumber + 1) % AB_FLASH_MAX_PAGES; // One block larger than the next start block
	PRINTF("\n Neuralert: [%s] %d samples assembled from %d blocks",
			__func__, packet_data_ptr->num_samples, packet_data_ptr->num_blocks);

	return pdFALSE;
}


/**
 *******************************************************************************
 * @brief Parse a message received from the MQTT broker via subscribe
 *    callback
 *
 *    Format of downlink is:
 *    {message:<cmd1> <cmd2> <cmd3> <cmd4>}
 *    with up to four commands
 *
 *	  To shut down the device and erase all accelerometer data,
 *	  send:
 *    {message: <Unique device id>}
 *    where the device id is the abbreviated MAC address used to
 *    identify this specific device.
 *******************************************************************************
 */

int parseDownlink(char *buf, int len)
{
	int i, j,k, argc;
	//int ptrCommand,status;
	//int tempInt;
	char str1[20];
	char commands[4][90];
	//char *argvTmp[4] = {&commands[0][0], &commands[1][0], &commands[2][0], &commands[3][0]};

	PRINTF("parseDownlink %s %d\n",buf,len);

	// Scan for "message:" keyword
	// overruns buffer if first keyword > 19
	for(i=0,j=0; i < len; i++)
	{
		if( (buf[i] == '{') || (buf[i] == '\"') || (buf[i] == ' ') || (buf[i] == '\r') || (buf[i] == '\n') )  /* ignore these characters */
			 continue;
		if(buf[i] == ':')
		{
			str1[j] = '\0';
			 break;
		}
//		PRINTF("i: %d j: %d %c\r\n",i,j,buf[i]);
		 str1[j++] = buf[i];
	}
	if(strcmp(str1,"message") != 0)
	{
		PRINTF("Neuralert: [%s] message keyword missing in downlink command! %s", __func__, str1);
		return (FALSE);
	}

//	PRINTF("message in JSON packet! %s\r\n",str1);
	// Parse up to four separate commands in one packet
	for(i++,j=0, k=0, argc=0; i < len; i++)
	{
//		PRINTF("i: %d 0x%02X %c\r\n",i,buf[i],buf[i]);
		if( (buf[i] == '\"') || (buf[i] == '\r') || (buf[i] == '\n') )
			continue;
		if(buf[i] == '}')
			break;
//		PRINTF("i: %d j: %d k: %d %c\r\n",i,j,k,buf[i]);
		commands[j][k++] = buf[i];
		if(buf[i] == ' ')
		{
			k--;
			commands[j][k] = '\0';
			j++;
			k  = 0;
			argc++;
		}
	}
	commands[j][k] = '\0';
	argc++;

	PRINTF("\n\nDownlink command(s) received: %d\r\n",argc);
	for(i=0;i<argc;i++)
	{
		PRINTF("   i: %d %s\r\n",i,&commands[i][0]);
//		if(i < 4)
//			PRINTF("i: %d %s\r\n",i,argvTmp[i]);
	}

#if 0 // leaving this here for now to work on the OTA provisioning.
	if(strcmp(&commands[0][0],"terminate") == 0)
	{
		if(argc < 2)
		{

			PRINTF("\n Neuralert: [%s] terminate command received without device identifier", __func__);
		}
		else if (strcmp(pUserData->Device_ID, &commands[1][0]) != 0)
		{
			PRINTF("\n Neuralert: [%s] terminate command with wrong device identifier: %s", __func__,
					&commands[1][0]);
		}
	}
#endif

	return TRUE;
}




/**
 *******************************************************************************
 * @brief Terminate the transmission of data over MQTT
 * A robust implementation that can be called at anytime to return the system
 * to the process of logging AXL data on a hardware interrupt.
 * There are lots of reasons a transmission can fail, the most likely reasons are
 * intermittent wifi and MQTT client/broker issues.  Since these can't be fixed on
 * the fly, we opt to terminate the tranmission and try again later.
 *******************************************************************************
 */

void user_terminate_transmit(void)
{

	int sys_wdog_id;
	sys_wdog_id = da16x_sys_watchdog_register(pdFALSE);

	// Setup a new task to stop the MQTT client -- it can hang and block the wifi disconnect below
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore");
		user_reboot();
	} else {
		SET_BIT(processLists, USER_PROCESS_MQTT_STOP);
		xSemaphoreGive(Process_semaphore);
	}
	user_create_MQTT_stop_task();
	int timeout = MQTT_STOP_TIMEOUT_SECONDS * 10;
	da16x_sys_watchdog_notify(sys_wdog_id);
	int val = pdTRUE;
	while (val && timeout > 0)
	{
		if (take_semaphore(&Process_semaphore)) {
			PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
			user_reboot();
		} else {
			val = BIT_SET(processLists, USER_PROCESS_MQTT_STOP);
			xSemaphoreGive(Process_semaphore);
		}
		vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms delay
		timeout--;
		da16x_sys_watchdog_notify(sys_wdog_id);
	}

	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		CLR_BIT(processLists, USER_PROCESS_MQTT_STOP);
		xSemaphoreGive(Process_semaphore);
	}
	da16x_sys_watchdog_notify(sys_wdog_id);


	// disconnect from wlan -- keep trying until a watchdog timeout.
	// note, the da16x_cli_reply command should work regardless, and if it doesn't a reboot (via watchdog timeout)
	// is probably necessary.
	UCHAR value_str[128];
    int ret = da16x_cli_reply("disconnect", NULL, value_str);
	while (ret < 0 || strcmp(value_str, "FAIL") == 0) {
		PRINTF(" [%s] Failed disconnect from AP 0x%x\n  %s\n", __func__, ret, value_str);
		vTaskDelay(pdMS_TO_TICKS(10));
		ret = da16x_cli_reply("disconnect", NULL, value_str);
	}
	da16x_sys_watchdog_notify(sys_wdog_id);

	system_control_wlan_enable(FALSE);

	// The following delay (vTaskDelay(10)) is necessary to prevent an LmacMain hard fault.  LmacMain is generated in a pre-compile
	// library in the sdk. It seems like the wifi disconnect returns before all the resources are shutdown, and if
	// we proceed without delay to triggering a sleep event, a hard fault will result.
	// UPDATE: After adding vTaskDelay(10) we saw a hard fault after a successful transmission after about 3500 transmission on one device.
	// No logging caught what caused the hard fault.  Our best guess is that the accelerometer task (running at a higher priority)
	// was triggered during the vTaskDelay(10); which kept running while the accelerometer was processed.  The accelerometer might have been
	// blocking the LmacMain tasks such that the following delay was made moot.  Then the radio was turned off prior to graceful shutdown.
	// To solve the hypothesized problem above, we now read the pUserData-> -- which is more than enough time for the
	// accelerometer task to complete and give the processor back to LmacMain to shut down before the rf is turned off.

	unsigned int temp1 = 0;
	unsigned int temp2 = 1; // must be set to a different value than temp1 for the following to be safe
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking user semaphore", __func__);
		// do nothing, logic still works (temp1 != temp2)
	} else {
		temp1 = pUserData->ACCEL_read_count;
		xSemaphoreGive(User_semaphore);
	}
	vTaskDelay(10); // This delay is NECESSARY -- DO NOT REMOVE (see description above)
	da16x_sys_watchdog_notify(sys_wdog_id);

	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		// do nothing, logic still works (temp1 != temp 2)
	} else {
		temp2 = pUserData->ACCEL_read_count;
		xSemaphoreGive(User_semaphore);
	}

	if (temp1 != temp2) {
		vTaskDelay(10); // This delay is also likely necessary to prevent the rare event also described above
	}

	// turn off rf
    da16x_sys_watchdog_notify(sys_wdog_id);
	wifi_cs_rf_cntrl(TRUE); // Might need to do this elsewhere

	// Clear the transmit process bit so the system can go to sleep
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		CLR_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);
		CLR_BIT(processLists, USER_PROCESS_MQTT_STOP);
		CLR_BIT(processLists, USER_PROCESS_WATCHDOG);
		CLR_BIT(processLists, USER_PROCESS_WATCHDOG_STOP);
		xSemaphoreGive(Process_semaphore);
	}

	da16x_sys_watchdog_unregister(sys_wdog_id);
}




/**
 *******************************************************************************
 * @brief A application-level version MQTT send message for qos
 * This function is a application-level implementation of:
 * mqtt_client_send_message_with_qos in sub_client.c
 * the version in sub_client.c has a race condition w.r.t. inflight messages.
 * Specifically, in SDK 3.2.8.1 (and earlier) the while loop exit condition would
 * be checked prior to the inflight messages variable being incremented.  Renesas
 * was contacted and the recommendation was to implement an application-level
 * solution using the mqtt_slient_set_pub_cb.  Consequently, to user this function
 * the pUserData->MQTT_inflight variable must be set to zero in the pub_cb function
 * *******************************************************************************
 */
static int user_mqtt_client_send_message_with_qos(char *top, char *publish, ULONG timeout)
{
	int status;
	int qos;
	da16x_get_config_int(DA16X_CONF_INT_MQTT_QOS, &qos);

	if (take_semaphore(&User_semaphore)) {
		return -3; // couldn't get the semaphore
	} else {
		if (pUserData->MQTT_inflight > 0){
			xSemaphoreGive(User_semaphore);
			return -1; // A message is supposedly inflight
		}
		pUserData->MQTT_inflight = 1;
		xSemaphoreGive(User_semaphore);
	}

	status = mqtt_pub_send_msg(top, publish);

	if (!status && qos >= 1)
	{
		while (timeout--)
		{
			if (take_semaphore(&User_semaphore)) {
				return -3; // couldn't get the semaphore
			} else {
				if (pUserData->MQTT_inflight == 0) {
					xSemaphoreGive(User_semaphore);
					return 0;
				}
				xSemaphoreGive(User_semaphore);
			}

			vTaskDelay(10);
		}

		// a timeout has occurred.  clear the inflight variable and exit.
		// we're going to kill the MQTT client next which will kill the actual inflight message
		if (take_semaphore(&User_semaphore)) {
			return -3;
		} else {
			pUserData->MQTT_inflight = 0;
			xSemaphoreGive(User_semaphore);
			return -2;			/* timeout */
		}

	}
	else if (!status && qos == 0)
	{
		if (take_semaphore(&User_semaphore)) {
			return -3;
		} else {
			pUserData->MQTT_inflight = 0;
			xSemaphoreGive(User_semaphore);
		}

		return 0;
	}
	else
	{
		if (take_semaphore(&User_semaphore)) {
			return -3;
		} else {
			pUserData->MQTT_inflight = 0; // clear inflight, we'll kill the client next anyway
			xSemaphoreGive(User_semaphore);
		}

		return -1;		/* error */
	}
}




/**
 *******************************************************************************
 * @brief Task to manage sending MQTT messages
 *******************************************************************************
 */
static int user_mqtt_send_message(void)
{
	int transmit_status = 0;

	unsigned long timeout_qos = 0;
	timeout_qos = (unsigned long) (pdMS_TO_TICKS(MQTT_QOS_TIMEOUT_MS) / 10);
	PRINTF("\n Neuralert: [%s] timeout qos: %lu\n", __func__, timeout_qos);

	// note, don't call mqtt_client_send_message_with_qos in sub_client.c
	// there is a race condition in that function.  We use the following which
	// is an application-level implementation using callbacks
	transmit_status = user_mqtt_client_send_message_with_qos(NULL, mqttMessage, timeout_qos);

	return transmit_status;
}



/**
 *******************************************************************************
 * @brief A task to execute the MQTT stop so it doesn't block turning off wifi
 *
 * A new task is needed to cleanly stop the MQTT task.  Basically, we would
 * prefer to cleanly shut down MQTT.  That process involves first turning off
 * MQTT, then turning off the wifi.  However, MQTT can get into a state where
 * it is blocking the wifi disconnect process.  In that scenario, we'll opt for
 * the nuclear option of disconnecting from wifi and MQTT will stop in it's due
 * course.  This task's only purpose is to stop MQTT.  It is called by
 * user_terminate_transmit() -- in that function a timeout exists to stop the
 * wifi regardless of whether MQTT has shutdown.
 *
 * Since this thread is known to hang, don't implement a watchdog -- it is being
 * monitored by user_terminate_transmit()
 *
 *******************************************************************************
 */
static void user_process_MQTT_stop(void* arg)
{
	PRINTF ("\n Neuralert: [%s] Forcibly Stopping MQTT Client", __func__);
    if (mqtt_client_is_running() == TRUE) {
        mqtt_client_force_stop();
        mqtt_client_stop();
    }

	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		CLR_BIT(processLists, USER_PROCESS_MQTT_STOP);
		xSemaphoreGive(Process_semaphore);
	}
    vTaskDelay(1);

    PRINTF ("\n Neuralert: [%s] MQTT Client Stopped", __func__);

    user_MQTT_stop_task_handle = NULL;
    vTaskDelete(NULL);
}




/**
 *******************************************************************************
 * @brief A simple application-level watchdog for ensuring the MQTT task gets
 * started correctly.  The transmission setup is done via callback initially.
 * First the wifi is started.  Once connected, the MQTT task is started. If
 * something happens and the MQTT task doesn't get started (for a variety of reasons)
 * the microcontroller will think an MQTT transmission is in place and block going
 * to sleep2. Once the MQTT task is up and running, there is no risk of hanging.
 * So the watchdog is in place to basically timeout if the MQTT task isn't started
 * within some period of time.
 *******************************************************************************
 */
static void user_process_watchdog(void* arg)
{

	int sys_wdog_id = da16x_sys_watchdog_register(pdFALSE);

	int timeout = WATCHDOG_TIMEOUT_SECONDS * 10;
	int flag = pdTRUE;
	while (flag && timeout > 0)
	{
		if (take_semaphore(&Process_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
			// do nothing if this happens, the timeout is still ticking down, so it will recover
		} else {
			flag = BIT_SET(processLists, USER_PROCESS_WATCHDOG);
			xSemaphoreGive(Process_semaphore);
		}
		vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms delay
		timeout--;
		da16x_sys_watchdog_notify(sys_wdog_id);
	}

	// The Process bit should have been cleared in user_process_send_MQTT_data()
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
	} else {
		int val = BIT_SET(processLists, USER_PROCESS_WATCHDOG);
		xSemaphoreGive(Process_semaphore);
		if (val) {
			PRINTF("\n WIFI and MQTT Connection Watchdog Timeout");

			if (take_semaphore(&User_semaphore)) {
				PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
				// do nothing, not a critical error, just an accounting error
			} else {
				pUserData->MQTT_stats_connect_fails++;
				xSemaphoreGive(User_semaphore);
			}
			da16x_sys_watchdog_notify(sys_wdog_id);
			da16x_sys_watchdog_suspend(sys_wdog_id);
			user_terminate_transmit();
			da16x_sys_watchdog_notify_and_resume(sys_wdog_id);
		}
		da16x_sys_watchdog_notify(sys_wdog_id);
	}

	PRINTF ("\n Neuralert: [%s] Stopping watchdog task", __func__);
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error obtaining process semaphore", __func__);
		user_reboot();
	} else {
		CLR_BIT(processLists, USER_PROCESS_WATCHDOG);
		CLR_BIT(processLists, USER_PROCESS_WATCHDOG_STOP);
		xSemaphoreGive(Process_semaphore);
	}

	da16x_sys_watchdog_unregister(sys_wdog_id);

	user_watchdog_task_handle = NULL;
	vTaskDelete(NULL);
}





/**
 *******************************************************************************
 * @brief A simple application-level watchdog for the wifi & MQTT that will
 * trigger a graceful teardown of the wifi & MQTT tasks if something hangs.
 * We expect we'll have some intermittent wifi issues from time to time.
 *******************************************************************************
 */
static int user_process_start_watchdog()
{

	if (user_watchdog_task_handle != NULL){
		PRINTF("\n Neuralert: [%s] Watchdog task already running -- Not starting transmission", __func__);
		return -2;
	}


	BaseType_t create_status;

	create_status = xTaskCreate(
			user_process_watchdog,
			"USER_WATCHDOG", 			// Task name
			(3072),						// stack size for comfort
			( void * ) NULL,  			// no parameter to pass
			(OS_TASK_PRIORITY_USER + 2),	// Make this lower than USER_READ task in user_apps.c
			&user_watchdog_task_handle);			// save the task handle

	if (create_status == pdPASS)
	{
		PRINTF("\n Neuralert: [%s] Watchdog task created\n", __func__);
		return 0;
	}
	else
	{
		PRINTF("\n Neuralert: [%s] Watchdog task failed to create -- Not starting transmission", __func__);
		return -1;
	}

}



/**
 *******************************************************************************
 * @brief Retry the transmission of data over MQTT
 * A robust implementation that can be called at anytime to return the system
 * to the process of logging AXL data on a hardware interrupt.
 * There are lots of reasons a transmission can fail, the most likely reasons are
 * intermittent wifi and MQTT client/broker issues.  Sometimes it might be best
 * to restart MQTT and try again.
 *******************************************************************************
 */

void user_retry_transmit(void)
{
	int sys_wdog_id = da16x_sys_watchdog_register(pdFALSE);

	// First, turn off the watchdog -- we'll need to restart it in a bit.
	// if we can't then let the hardware watchdog timeout.
	// there should be nothing blocking this for very long
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
	} else {
		SET_BIT(processLists, USER_PROCESS_WATCHDOG_STOP);
		CLR_BIT(processLists, USER_PROCESS_WATCHDOG);
		xSemaphoreGive(Process_semaphore);
	}
	da16x_sys_watchdog_notify(sys_wdog_id);

	// Wait until the watchdog is stopped (these aren't high-priority tasks,
	// so we may need to wait a bit.
	// If the watchdog doesn't stop, the hardware watchdog will handle a reset.
	int val = pdFALSE;
	while (!val) {
		int flag = take_semaphore(&Process_semaphore);
		xSemaphoreGive(Process_semaphore);
		if (flag) {
			PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
			// take no action, the watchdog will kill us eventually
		} else {
			val = BIT_SET(processLists, USER_PROCESS_WATCHDOG_STOP);
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	da16x_sys_watchdog_notify(sys_wdog_id);
	da16x_sys_watchdog_suspend(sys_wdog_id);

	// Check if MQTT is running -- if so, shut it down
    if (mqtt_client_is_running() == TRUE) {
        mqtt_client_force_stop();
        mqtt_client_stop();
    }
    da16x_sys_watchdog_notify_and_resume(sys_wdog_id);

	// Set the process bits for a clean retry
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		SET_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);
		SET_BIT(processLists, USER_PROCESS_WATCHDOG);
		xSemaphoreGive(Process_semaphore);
	}


	// Restart the application watchdog
	da16x_sys_watchdog_notify(sys_wdog_id);
	int ret = 0;
	ret = user_process_start_watchdog();
	da16x_sys_watchdog_notify(sys_wdog_id);
	da16x_sys_watchdog_suspend(sys_wdog_id);
	if (ret == 0)
	{
		// Restart the Transmission
		user_start_MQTT_client();
	}
	else {
		user_terminate_transmit(); // Something bad happened with the watchdog setup -- stop transmission
	}
	da16x_sys_watchdog_unregister(sys_wdog_id);
}





/**
 *******************************************************************************
 * @brief Task to transmit the next group of FIFO buffers that have been stored
 * in NV memory
 *******************************************************************************
 */
static void user_process_send_MQTT_data(void* arg)
{
	int status = 0;
	int msg_sequence;
	unsigned int msg_transmission;
	AB_INDEX_TYPE transmit_start_loc = INVALID_AB_ADDRESS;
	int request_stop_transmit = pdFALSE;		// loop control for packet transmit loop
	int request_retry_transmit = pdFALSE;
	int transmit_complete = pdFALSE;
	int transmit_attempts_remaining = 0;

	packetDataStruct packet_data;	// struct to capture packet meta data.

	// stats
	int packets_sent = 0;		// actually sent during this transmission interval
	int samples_sent = 0;

	// Start up watchdog
	int sys_wdog_id = da16x_sys_watchdog_register(pdFALSE);

	// We've successfully made it to the transmission task!
	// Clear the watchdog process bit that was monitoring for this task to start
	// The watchdog will shutdown itself later regardless.
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		CLR_BIT(processLists, USER_PROCESS_WATCHDOG);
		xSemaphoreGive(Process_semaphore);
		vTaskDelay(1);
	}

	// Wait until MQTT is actually connected before proceeding
	// it takes a couple of seconds from when the MQTT connected callback is called
	// (which is when this function that triggered this task generation)
	// and when the client will actually be ready to work.
	// This is because we want to wait for the unsub_topic to be non-zero
	// in mosq_sub->unsub_topic prior to starting up.  If we experience a timeout,
	// kill the transmission cycle.
	int timeout = MQTT_SUB_TIMEOUT_SECONDS * 10;
	da16x_sys_watchdog_notify(sys_wdog_id);
	while (!mqtt_client_check_conn() && timeout > 0){
		vTaskDelay(pdMS_TO_TICKS(100));
		timeout--;
		da16x_sys_watchdog_notify(sys_wdog_id);
	}
	if (timeout <= 0){
		goto end_of_task;
	}

	// Retrieve MQTT information from user data
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		goto end_of_task;
	} else {
		transmit_start_loc = pUserData->next_AB_write_position; // start where the NEXT write will take place
		pUserData->MQTT_inflight = 0; // This is a new transmission, clear any lingering inflight issues
		msg_transmission = ++pUserData->MQTT_transmission_number;  // Increment transmission #
		transmit_attempts_remaining = pUserData->MQTT_tx_attempts_remaining; // store how many attempts we have left

		// We are connected to WIFI and MQTT and should have wall clock
		// time from an SNTP server
		// On the first successful connection we take a time snapshot that
		// will be transmitted in the "timesync" JSON field so that the
		// end user can coordinate the left and right wrist datastreams
		if(pUserData->MQTT_timesync_captured == 0)
		{
			timesync_snapshot(); // no semaphore protection inside timesync_snapshot
			pUserData->MQTT_timesync_captured = 1;
		}

		xSemaphoreGive(User_semaphore);

		// non-protected setup variables
		msg_sequence = 0;

		vTaskDelay(1);
	}

	// If there is no valid transmit position, we've been wakened by mistake
	// or something else has gone wrong
	if (	(transmit_start_loc == INVALID_AB_ADDRESS)
			|| (transmit_start_loc < 0)
			|| (transmit_start_loc >= AB_FLASH_MAX_PAGES))
	{
		PRINTF("\n Neuralert: [%s] MQTT task found invalid transmit start location: %d", __func__, transmit_start_loc);
		goto end_of_task;
	}

	// Adjust transmit_start_loc to account for pUserData->next_AB_write_position
	// which provides the NEXT write, not previously written
	transmit_start_loc = transmit_start_loc - 1;
	if (transmit_start_loc < 0)
	{
		// AXL just wrapped around so our last position is the last place in memory.
		transmit_start_loc += AB_FLASH_MAX_PAGES;
	}

	PRINTF("\n Neuralert: [%s] MQTT transmit starting at %d", __func__, transmit_start_loc);



	// *****************************************************************
	//  MQTT transmission
	// *****************************************************************
	da16x_sys_watchdog_notify(sys_wdog_id);

	// Set up transmit loop parameters
	packet_data.next_start_block = transmit_start_loc;

	vTaskDelay(1);
	request_stop_transmit = pdFALSE;
	int packet_count = 0;
	// Execute transmissions until done or told to stop
	while ((request_stop_transmit == pdFALSE)
			&& (transmit_complete == pdFALSE))
	{
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
			//do nothing, we'll just re-use the last message id -- not ideal, but better than restarting.
		} else {
			extern mqttParamForRtm mqttParams;
			mqttParams.pub_msg_id = ++pUserData->MQTT_pub_msg_id; // load saved pub_msg_id into user data (next id number)
			xSemaphoreGive(User_semaphore);
		}
		PRINTF("\n Neuralert: [%s] current pub_msg_id = %d", __func__, mqtt_client_get_pub_msg_id());


		// assemble the packet into the user data
		packet_count++;
		if (assemble_packet_data(&packet_data)) {
			PRINTF("\n Neuralert: [%s] Error building packet data", __func__);
			packet_data.num_samples = 0;
		} else {
			PRINTF("\n Neuralert: [%s] MQTT packet %d:  Start: %d End: %d num blocks: %d num samples: %d", __func__,
					packet_count, packet_data.start_block, packet_data.end_block,
					packet_data.num_blocks, packet_data.num_samples);
		}


		if (packet_data.num_samples < 0)
		{
			PRINTF("\n Neuralert: [%s] packet data error - aborting", __func__);
			request_stop_transmit = pdTRUE;
		}
		else if (packet_data.num_samples == 0)
		{
			transmit_complete = pdTRUE;
		}
		else
		{
			msg_sequence++;
			da16x_sys_watchdog_notify(sys_wdog_id);
			da16x_sys_watchdog_suspend(sys_wdog_id);
			status = send_json_packet(packet_data.num_samples, msg_transmission, msg_sequence);
			da16x_sys_watchdog_notify_and_resume(sys_wdog_id);

			if(status == 0) //Transmission successful!
			{
				//we have succeeded in a transmission (bootup complete), so clear the bootup process  bit.
				if (take_semaphore(&Process_semaphore)) {
					PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
					vTaskDelay(3); // to ensure display
					// do nothing, we'll try to clear the process bit next transmission success.
				} else {
					CLR_BIT(processLists, USER_PROCESS_BOOTUP);
					xSemaphoreGive(Process_semaphore);
				}

				da16x_sys_watchdog_notify(sys_wdog_id);
				if (take_semaphore(&User_semaphore)) {
					PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
					vTaskDelay(3); // to ensure disply
					// do nothing, if we don't clear the transmit locations then they will be resent next time.
					// if attempts since tx success isn't updated, we'll stay in "fast" transmission mode longer
				} else {
					// Do the stats
					pUserData->MQTT_attempts_since_tx_success = 0; // must be protected by user semaphore
					pUserData->MQTT_stats_packets_sent++; // must be protected by user semaphore
					packets_sent++;		// Total packets sent this interval
					samples_sent += packet_data.num_samples;
					xSemaphoreGive(User_semaphore);


					// Clear the transmission map corresponding to blocks in the packet
					if (packet_data.start_block > packet_data.end_block)
					{
						if (clear_AB_position(packet_data.end_block, packet_data.start_block)) {
							PRINTF("\n Neuralert [%s] error taking user semaphore");
							//do nothing, the data will just be resent.
						}
						vTaskDelay(20); // give time for someone else to grab the user semaphore

					}
					else // there was a "wrap" in the buffer
					{
						if (clear_AB_position(0, packet_data.start_block)) {
							PRINTF("\n Neuralert [%s] error taking user semaphore");
							//do nothing, the data will just be resent.
						}
						vTaskDelay(20); // give time for another task to grab the user semaphore

						if (clear_AB_position(packet_data.end_block, (AB_FLASH_MAX_PAGES - 1))) {
							PRINTF("\n Neuralert [%s] error taking user semaphore");
							//do nothing, the data will just be resent.
						}
						vTaskDelay(20);

					}


				}
				da16x_sys_watchdog_notify(sys_wdog_id);

				vTaskDelay(5);
			}
			else if (transmit_attempts_remaining > 0){
				PRINTF("\n Neuralert: [%s] MQTT transmission %d:%d failed. Remaining attempts %d. Retry Transmission",
						__func__, msg_transmission, msg_sequence, transmit_attempts_remaining);

				request_stop_transmit = pdTRUE; // must set to true to exit loop
				request_retry_transmit = pdTRUE;
				if (take_semaphore(&User_semaphore)) {
					PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
					//do nothing, not a critical error -- just local logging, might keep retrying
				} else {
					pUserData->MQTT_stats_retry_attempts++;
					pUserData->MQTT_tx_attempts_remaining--;
					xSemaphoreGive(User_semaphore);
				}
			}
			else
			{
				PRINTF("\n Neuralert: [%s] MQTT transmission %d:%d failed. Remaining attempts %d. Ending Transmission",
						__func__, msg_transmission, msg_sequence, transmit_attempts_remaining);
				request_stop_transmit = pdTRUE;
			}
			vTaskDelay(1);

		} // num_samples > 0

		if (packet_data.done_flag == pdTRUE){
			transmit_complete = pdTRUE; // there is no more data after this packet.
		}
	} // while we have stuff to transmit (and not asking for transmission to stop)

	if(transmit_complete)
	{
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
			vTaskDelay(3); // small delay to show error
			// do nothing, just affects local logging
		} else {
			pUserData->MQTT_stats_transmit_success++;
			xSemaphoreGive(User_semaphore);
		}

		PRINTF("\n Neuralert: [%s] MQTT transmission %d complete.  %d samples in %d JSON packets",
				__func__, msg_transmission, samples_sent, packets_sent);
		vTaskDelay(3); // delay to ensure the print statement is seen
	}

	// Presumably we've finished sending and allowed time for a shutdown
	// command or other message back from the cloud
	// Turn off the RF section until next transmit interval
end_of_task:
	da16x_sys_watchdog_notify(sys_wdog_id);
	da16x_sys_watchdog_suspend(sys_wdog_id);

	if (request_retry_transmit){
		user_retry_transmit();
	}
	else
	{
		user_terminate_transmit();
	}

	da16x_sys_watchdog_unregister(sys_wdog_id);
	user_MQTT_task_handle = NULL;
	vTaskDelete(NULL);
}


void user_process_wifi_conn()
{
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
		// do nothing, this is a courtesy call back for debugging
	} else {
		int flag = BIT_SET(processLists, USER_PROCESS_MQTT_TRANSMIT);
		xSemaphoreGive(Process_semaphore);
		if (!flag) {
			PRINTF("\n MQTT transmit task already in progress", __func__);
		}
	}
}


/**
 *******************************************************************************
 *  user_start_MQTT_client: start the MQTT client
 *******************************************************************************
 */
void user_start_MQTT_client()
{

	// MQTT client is affected by the network state.  Since the network should be up at this point,
	// we have two scenarios:
	// 1) client is already running -- in which case we go straight to creating the transmit task
	// 2) the client is not running -- in which case we start the client and process the request on a callback.
	if (mqtt_client_check_conn()){
		PRINTF("\nNeuralert: [%s] MQTT client already started", __func__);
	} else {
		int status = 0;
		status = mqtt_client_start(); // starts the MQTT client

		if(status == 0)
		{
			PRINTF("\nNeuralert: [%s] MQTT client start success -- waiting for connection", __func__);
		}
		else
		{
			PRINTF("\nNeuralert: [%s] MQTT client start failed", __func__);
		}
	}
}


/**
 *******************************************************************************
 *@brief  user_start_data_tx: start up the wifi and create a software watchdog
 *This function is intended to be the entry point for our transmission task
 *The first thing that will happen is it turns on the radio and network connection
 *The sequence of events will be:
 *		start radio and wifi
 *		on wifi connection callback -> start MQTT client (in user_apps.c)
 *		on MQTT connection callback -> start the transmission task
 *		once transmission is completed-> start the terminate transmission task
 *
 *Since there are many ways that bringing the wifi up can hang, we start a
 *separate task that monitors for a process bit.  This process bit will be set
 *in the start transmission task.
 *******************************************************************************
 */
static void user_start_data_tx(){

	// Begin by setting the process bits for the watchdog and MQTT transmission. This
	// prevents sleeping.
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_terminate_transmit();
		return;
	} else {
		SET_BIT(processLists, USER_PROCESS_WATCHDOG);
		SET_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);
		xSemaphoreGive(Process_semaphore);
	}

	// start the software watchdog
	int ret = 0;
	ret = user_process_start_watchdog();
	if (ret != 0)
	{
		user_terminate_transmit(); // Something bad happened with the watchdog setup -- stop transmission
		return;
	}

	// Initialize the maximum number of retries here -- this can't be done in
	// user_create_MQTT_task() since that function will be called when we restart
	// the client.  We do it here because this is the entry point data transmission
	pUserData->MQTT_tx_attempts_remaining = MQTT_MAX_ATTEMPTS_PER_TX;


	PRINTF("\n ===== Starting WIFI =====\n\n");
	// Start the RF section power up
	wifi_cs_rf_cntrl(FALSE);

	char value_str[128] = {0, };
	da16x_cli_reply("select_network 0", NULL, value_str);

}



/**
 *******************************************************************************
 *@brief  user_create_MQTT_stop_task: create a new task for stopping MQTT
 *
 *******************************************************************************
 */
static void user_create_MQTT_stop_task()
{
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		int val = BIT_SET(processLists, USER_PROCESS_MQTT_STOP);
		xSemaphoreGive(Process_semaphore);
		if (!val){
			PRINTF("\n MQTT stop not requested");
			return;
		}
	}



	BaseType_t create_status;

	create_status = xTaskCreate(
			user_process_MQTT_stop,
			"USER_MQTT_STOP", 				// Task name
			(4*1024),					// healthy size
			( void * ) NULL,  			// no parameter to pass
			(OS_TASK_PRIORITY_USER + 2),	// Make this lower than USER_READ task in user_apps.c
			&user_MQTT_stop_task_handle);			// save the task handle

	if (create_status == pdPASS)
	{
		PRINTF("\n Neuralert: [%s] MQTT stop task created", __func__);
	}
	else
	{
		PRINTF("\n Neuralert: [%s] MQTT stop task failed to created", __func__);
	}
}



/**
 *******************************************************************************
 *@brief  user_create_MQTT_task: create a new task for MQTT transmission
 *******************************************************************************
 */
static void user_create_MQTT_task()
{
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		int val = BIT_SET(processLists, USER_PROCESS_MQTT_TRANSMIT);
		xSemaphoreGive(Process_semaphore);
		if (!val){
			PRINTF("\n MQTT transmit task already in progress", __func__);
			return;
		}
	}

	// Prior to starting to transmit data, we need to restore the message id state
	// so we have unique message ids for each client message
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		// do nothing, we'll over-write data at the broker, but no reason to crash the system
	} else {
		extern struct mosquitto	*mosq_sub;
		mosq_sub->last_mid = pUserData->MQTT_pub_msg_id;
		xSemaphoreGive(User_semaphore);
	}

	BaseType_t create_status = xTaskCreate(
			user_process_send_MQTT_data,
			"USER_MQTT", 				// Task name
			(6*1024),					//we've seen near 4K stack utilization in testing, 50% buffer for safety.
			( void * ) NULL,  			// no parameter to pass
			(OS_TASK_PRIORITY_USER + 2),	// Make this lower than USER_READ task in user_apps.c
			&user_MQTT_task_handle);			// save the task handle

	if (create_status == pdPASS)
	{
		PRINTF("\n Neuralert: [%s] MQTT transmit task created", __func__);
	}
	else
	{
		PRINTF("\n Neuralert: [%s] MQTT transmit task failed to create", __func__);
	}
}



/**
 *******************************************************************************
 * @brief Process for disabling the WIFI autoconnect feature in the SDK
 *******************************************************************************
 */
static int user_process_disable_auto_connection(void)
{
	int netProfileUse;
	PRINTF("\n**Neuralert: %s\n", __func__); // FRSDEBUG

	/* To skip automatic network connection. It will be effected when boot-up. */
	int ret = da16x_get_config_int(DA16X_CONF_INT_STA_PROF_DISABLED, &netProfileUse);
	if (ret != CC_SUCCESS || netProfileUse == pdFALSE) {
		ret = da16x_set_config_int(DA16X_CONF_INT_STA_PROF_DISABLED, pdTRUE);
		PRINTF("\n Neuralert [%s] Disabling automatic network connection", __func__);
		/* It's just to give some delay before going to sleep mode.
		 *  it will be called only once.
		 */
		vTaskDelay(3);
	}
	return ret;
}

/**
 *******************************************************************************
 * @brief Sets the SSID to correspond to "Neuralert_MACSTR"
 * returns:
 *		pdTRUE if there is an error
 *		pdFALSE otherwise
 *******************************************************************************
 */
static int user_set_ssid(void) {
	//
	// This overides the default in util_api.c factory_reset_ap_mode()
	char neuralert_ssid[MAX_SSID_LEN + 3];
	char tmp_buf[MAX_SSID_LEN] = { 0, };

	sprintf(tmp_buf, "%s", "Neuralert");
	memset(neuralert_ssid, 0, MAX_SSID_LEN + 3);

	if (gen_ssid(tmp_buf, WLAN0_IFACE, 1, neuralert_ssid, sizeof(neuralert_ssid)) == -1) {
		PRINTF("\n Neuralert [%s] ssid error", __func__);
		return pdTRUE;
	}
	write_nvram_string((const char *)NVR_KEY_SSID_1, neuralert_ssid);
	vTaskDelay(100);

	return pdFALSE;
}

/**
 *******************************************************************************
 * @brief Process for taking control of a semaphore
 * returns:
 *		pdTRUE if there is an error
 *		pdFALSE otherwise
 *******************************************************************************
 */
static int take_semaphore(SemaphoreHandle_t *ptr)
{
	int return_value = pdTRUE;

	if(*ptr != NULL)
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake(*ptr, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
				shared resource. */
			return_value = pdFALSE;
		}
		else
		{
			PRINTF("\n Neuralert [%s] unable to obtain semaphore", __func__);
		}
	}
	else
	{
		PRINTF("\n Neuralert [%s] semaphore not initialized", __func__);
	}
	return return_value;
}



/**
 *******************************************************************************
 * @brief Process for initializing the accelerometer data buffering
 *  mechanism, including erasing a sector of the external data Flash
 *
 * Data is stored in a structure that holds the data read during
 * one FIFO read event.
 * Because the external flash memory is written with a "Page write"
 * command, each FIFO storage structure starts on a 256-byte page
 * boundary.
 * As of this writing, the FIFO storage structure is less than 256 bytes
 * so each write is to the next page boundary.
 * Additionally, the flash must be erased before it can be written.
 * The smallest erase function is a 4096 byte sector (16 pages)
 *
 * See document "Neuralert accelerometer data buffer design" for
 * details of this design
 *
 * This function takes the User_semaphore for its entirety and should not
 * be used in parallel with any processes needing the User_semaphore.  Moreover
 * it will only give up the
 *
 * returns pdTRUE if initialization succeeds
 * returns pdFALSE is a problem happens with the Flash initialization
 *******************************************************************************
 */
static int user_process_initialize_AB(void)
{
	int spi_status;
	int erase_status;
	UINT8 rx_data[3];
	ULONG SectorEraseAddr;
	HANDLE SPI = NULL;
	int i;

	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore");
		return pdFALSE;
	}

	pUserData->write_fault_count = 0;
	pUserData->write_retry_count = 0;
	pUserData->erase_attempts = 0;
	pUserData->erase_fault_count = 0;
	pUserData->erase_retry_count = 0;

	for (i = 0; i < AB_WRITE_MAX_ATTEMPTS; i++)
	{
		pUserData->write_attempt_events[i] = 0;
	}
	for (i = 0; i < AB_ERASE_MAX_ATTEMPTS; i++)
	{
		pUserData->erase_attempt_events[i] = 0;
	}
	/*
	 * Initialize the SPI bus and the Winbond external flash
	 */

	// Get handle for the SPI bus
	SPI = flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\n Neuralert: [%s] SPI initalization error", __func__);
		xSemaphoreGive(User_semaphore);
		return pdFALSE;
	}

	// Do device initialization
	spi_status = w25q64Init(SPI, rx_data);
	if (!spi_status)
	{
		PRINTF("\n Neuralert: [%s] SPI initalization error", __func__);
		xSemaphoreGive(User_semaphore);
		return pdFALSE;
	}

	// Initialize accelerometer buffering pointers
	// Note that pointers are Indexes and not addresses
	// (i.e., start at 0 and increment by 1)

	// Next location to write
	pUserData->next_AB_write_position = 0;

	// Initialize all the positions to not needing transmission
	for (i = 0; i < AB_TRANSMIT_MAP_SIZE; i++)
	{
		pUserData->AB_transmit_map[i] = 0;
	}

	// Erase the first sector where the first data will be written
	SectorEraseAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)pUserData->next_AB_write_position);
	PRINTF("\n Neuralert: [%s] Erasing first Sector Location: %x",__func__, SectorEraseAddr);

	erase_status = eraseSector_4K(SPI, SectorEraseAddr);

	if(!erase_status)
	{
		PRINTF("\n Neuralert: [%s] SPI erase error", __func__);
		xSemaphoreGive(User_semaphore);
		return pdFALSE;
	}

	flash_close(SPI);

	xSemaphoreGive(User_semaphore);
	return pdTRUE;

}

#if 0
/**
 *******************************************************************************
 * @brief Process to write one block to the accelerometer buffer
 * memory (flash)
 *  Returns pdFALSE if unable to write
 *  Returns pdTRUE if able to write
 *******************************************************************************
 */
static int AB_write_block(HANDLE SPI, int blockaddress, accelBufferStruct *FIFOdata)
{
	int return_value = pdFALSE;
	int spi_status;

	if(Flash_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( Flash_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */
			// Now write the block
			spi_status = pageWrite(SPI, blockaddress, (UINT8 *)FIFOdata, sizeof(accelBufferStruct));
			if(spi_status < 0)
			{
				PRINTF(" **: Flash Write error\n"); //Fault error indication here
				return_value = pdFALSE;
			}
			else
			{
//				Printf(" **: Flash Write successful\n");
				return_value = pdTRUE;
			}

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( Flash_semaphore );
		}
		else
		{
			PRINTF("\n ***: Unable to obtain Flash semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***: semaphore not initialized!\n");
	}

	return return_value;

}
#endif

/**
 *******************************************************************************
 * @brief Process for erasing a 4k sector, with check to make sure and retry
 *
 *  This function was created because we sometimes would have an erase
 *  failure while the MQTT task was active.  The erase failure would lead
 *  to a write failure of the next FIFO buffer, which in turn wouldn't
 *  update the write location and would discard the data.  Since we
 *  only erase at the END of writing, this led to an infinite fail loop.
 *  This function is an attempt to solve that.  A second solution would be
 *  to recognize the problem on the write fail and fix it there, but it
 *  seemed messier.  Hopefully this solves the problem.
 *  We noticed the erase fail when running for extended periods of time
 *  such as overnight. FRS
 *******************************************************************************
 */
static int user_erase_flash_sector(HANDLE SPI, ULONG SectorEraseAddr)
{

	int rerunCount;
	int faultFlag = 1;
	int erase_status;
	UINT32 erase_mismatch_count;
	UCHAR FIFObytes[256];				// pointer used to access bytes of fifo

	int retry_count = 0;

	int erase_confirmed;

	// Our return status starts at ok until a problem occurs
	erase_status = TRUE;

	PRINTF("-------------------------------\n");
	PRINTF(" Erase sector location: %x\n", SectorEraseAddr);
	PRINTF("-------------------------------\n");

	erase_mismatch_count = 0;
	// do some logging
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		// do nothing, logging isn't critical
	} else {
		pUserData->erase_attempts++;  // total sectors we tried to erase
		xSemaphoreGive(User_semaphore);
	}

	// Note - as of 9/10/22 the erase sector was taking about 50 milliseconds
	// We have to be careful not to overrun the accelerometer interrupt here
	// but if it takes 3 or 4 tries, that's still only a few hundred
	// milliseconds out of a 2+ second budget
	erase_confirmed = pdFALSE;
	for(rerunCount = 0; (rerunCount < AB_ERASE_MAX_ATTEMPTS) & !erase_confirmed;
				rerunCount++)
	{
		retry_count++;

		erase_status = eraseSector_4K(SPI, (UINT32)SectorEraseAddr);

		if(!erase_status)
		{
			PRINTF("\n\n********* eraseSector_4K returned error *********\n");
		}

		if (take_semaphore(&Flash_semaphore)) {
			PRINTF("\n Neuralert [%s] error taking flash semaphore", __func__);
			erase_status = FALSE;
		} else {
			int spi_status = pageRead(SPI, (UINT32)SectorEraseAddr,
				(UINT8 *) (accelBufferStruct *)FIFObytes, sizeof(accelBufferStruct));
			xSemaphoreGive(Flash_semaphore);
			if(spi_status < 0){
				PRINTF("\n Neuralert: [%s] Flash readback error: %x\n", __func__, SectorEraseAddr);
				erase_status = FALSE;
			}

			// Note that the issue could be the read rather than the write
			// A read of all zeroes seems to be a read error while
			// a read of non-zero data seems to indicate an erase malfunction
			erase_mismatch_count = 0;
			for(int i = 0; i <= 10; i++)
			{
				if(FIFObytes[i] != 0xFF)
				{
					PRINTF("  *** ERASE FAILED byte %d  %x\n", i, FIFObytes[i]);
					erase_mismatch_count++;
				}
			}

			if(erase_mismatch_count > 0)
			{
				PRINTF("  ERASE FLASH MISMATCH COUNT: %d\n\n", erase_mismatch_count);
				faultFlag = 1;
				erase_mismatch_count = 0;
				vTaskDelay(pdMS_TO_TICKS(10));
			}
			else
			{
				PRINTF(" Flash erase & verification successful\n");
				faultFlag = 0;
				erase_confirmed = pdTRUE;
				break;
			} // erase 4k returned ok status
		} // semaphore was obtained
	} // for rerunCount < max retries

	//do some logging
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore",__func__);
		// do nothing, logging isn't critical
	} else {
		if(faultFlag != 0)
		{
			pUserData->erase_fault_count++;  // total write failures since power on
		}
		else if(retry_count > 1)
		{
			pUserData->erase_retry_count++;	 // total # of times retry worked
		}
		// Log how many times we succeeded on each attempt count; [0] is first try, etc.
		pUserData->erase_attempt_events[retry_count-1]++;

		xSemaphoreGive(User_semaphore);
	}

end_of_task:

	return erase_status;
}



/**
 *******************************************************************************
 * @brief Process for writing data to external data Flash
 * One FIFO buffer structure is written to the next available
 * flash location.
 * See document "Neuralert accelerometer data buffer design" for
 * details of this design
 *
 *******************************************************************************
 */
static int user_process_write_to_flash(accelBufferStruct *pFIFOdata, int *did_an_erase)
{

	ULONG NextWriteAddr;
	ULONG SectorEraseAddr;
	int rerunCount, faultFlag = 1;
	int erase_status;
	int write_status;
	UINT32 write_fail_count;
	accelBufferStruct checkFIFO;	// copy for readback check
	int write_index;
	int retry_count;

	HANDLE SPI = NULL;

	// Our return status starts at ok until a problem occurs
	write_status = pdTRUE;

	// Set erase happened status for user in case we exit early
	*did_an_erase = pdFALSE;

	write_fail_count = 0;

	// next_AB_write_position is the index of the next place to write a
	// block.  Starts at 0 and wraps around when we reach the end of the
	// accelerometer buffer region in flash
	// Upon entry to this function it should always point to a valid
	// write location
	// Writes are done to 256-byte pages and so are on addresses that
	// are multiples of 0x100

	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		return pdFALSE;
	} else {
		write_index = pUserData->next_AB_write_position;
		xSemaphoreGive(User_semaphore);
	}

	// Calculate address of next sector to write
	NextWriteAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
			((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)write_index);
	PRINTF("\n-------------------------------\n");
	PRINTF(" Next location to write: %d\n",write_index);
	PRINTF(" Flash Write ADDR: 0x%X\r\n", NextWriteAddr);
	PRINTF(" Data sequence # : %d\n", pFIFOdata->data_sequence);
	PRINTF(" Number samples  : %d\n", pFIFOdata->num_samples);
	PRINTF("-------------------------------\n");

	retry_count = 0;

	SPI = flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\nNeuralert: [%s] MAJOR SPI ERROR: Unable to open SPI bus handle", __func__);

	}

	for(rerunCount = 0; rerunCount < AB_WRITE_MAX_ATTEMPTS; rerunCount++)
	{
		retry_count++;

		// get the flash semaphore
		if (take_semaphore(&Flash_semaphore)) {
			PRINTF("\n Neuralert [%s] error taking flash semaphore", __func__);
			write_status = FALSE;
		} else {
			// write to flash
			int spi_status_write = pageWrite(SPI, NextWriteAddr, (UINT8 *) pFIFOdata, sizeof(accelBufferStruct));
			if(spi_status_write < 0){
				PRINTF("\n Neuralert: [%s] Flash Write error %x", __func__, NextWriteAddr);
				write_status = FALSE;
			}

			vTaskDelay(3); // small delay to settle down

			// Now read it back and see if it's the same
			int spi_status_read = pageRead(SPI, NextWriteAddr, (UINT8 *) &checkFIFO, sizeof(accelBufferStruct));
			if(spi_status_read < 0){
				PRINTF("\n Neuralert: [%s] Flash readback error: %x", __func__, NextWriteAddr);
				write_status = FALSE;
			}

			xSemaphoreGive(Flash_semaphore);

			// Read back what we just wrote and make sure it took
			// Note - as of 8/20/22, the WIFI and MQTT activity in other
			// tasks seems to interfere with the SPI bus.  As of this
			// writing it seems that we catch the mistake and correct it
			// on the first retry.
			for(int i = 0; i <= receivedFIFO.num_samples - 1; i++)
			{
				if((receivedFIFO.Xvalue[i] != checkFIFO.Xvalue[i])
						|| (receivedFIFO.Yvalue[i] != checkFIFO.Yvalue[i])
						|| (receivedFIFO.Zvalue[i] != checkFIFO.Zvalue[i]))
				{
					PRINTF("\n RECEIVED: %d     X: %d Y: %d Z: %d\r",
						i, receivedFIFO.Xvalue[i], receivedFIFO.Yvalue[i], receivedFIFO.Zvalue[i]);
					PRINTF("\n CHECK:    %d     X: %d Y: %d Z: %d\r",
						i, checkFIFO.Xvalue[i], checkFIFO.Yvalue[i], checkFIFO.Zvalue[i]);
					write_fail_count++;
				}
			} // for each sample in FIFO compare buffer

			if(write_fail_count > 0)
			{
				PRINTF("\n Neuralert: [%s] SPI FLASH ERROR COUNT: %d", __func__, write_fail_count);
				faultFlag = 1;
				write_fail_count = 0;
				vTaskDelay(pdMS_TO_TICKS(30));
			}
			else
			{
				faultFlag = 0;
				break;
			}
		}
	} // for rerunCount < max retries

	// do some logging
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore",__func__);
		//do nothing, the log error isn't critical to functionality
	} else {
		if(faultFlag != 0)
		{
			pUserData->write_fault_count++;  // total write failures since power on
		}
		else if(retry_count > 1)
		{
			pUserData->write_retry_count++;	 // total # of times retry worked
		}
		// Log how many times we succeeded on each attempt count; [0] is first try, etc.
		pUserData->write_attempt_events[retry_count-1]++;

		xSemaphoreGive(User_semaphore);
	}


	// Note if we had a write failure, we should skip the following update
	if(faultFlag != 0)
	{
		PRINTF("\n Neuralert: [%s] FIFO DATA WRITE FAILURE - SKIPPING POINTER UPDATE", __func__);
		goto end_of_task;
	}

	// update next write location
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%] error taking user semaphore", __func__);
		// we have written data to the flash, but can't record it is there.  This could be problematic.
		// so we're going to reboot fresh.
		user_reboot();
	} else {
		// Indicate the current write position is ready for transmission
		SET_AB_POS(pUserData->AB_transmit_map, pUserData->next_AB_write_position);
		// Increment the write position
		pUserData->next_AB_write_position = ((pUserData->next_AB_write_position + 1) % AB_FLASH_MAX_PAGES);
		xSemaphoreGive(User_semaphore);
	}

	// Increment the write index -- this is only for erasing flash below,
	// so no risk of it affecting the actual AB write location
	write_index = ((write_index+1) % AB_FLASH_MAX_PAGES);

	// If the next page we're going to write to is the start of a
	// new sector, we have to erase it to be ready
	if(	((write_index % AB_PAGES_PER_SECTOR) == 0)
		|| (write_index == 0))
	{
		*did_an_erase = pdTRUE;
		// Calculate address of next sector to write
		SectorEraseAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)write_index);
		PRINTF("  Sector filled. Location: %x Erasing next sector\n",
					SectorEraseAddr);

		erase_status = user_erase_flash_sector(SPI, SectorEraseAddr);

		if(!erase_status)
		{
			PRINTF("\n Neuralert: [%s] SPI erase error", __func__);
			write_status = FALSE;
		}
	} // if need to erase next sector

end_of_task:
	flash_close(SPI);  // See comments about SPI closing above
	return write_status;
}


/**
 *******************************************************************************
 * @brief Take a snapshot of "wall clock" time for the JSON timesync field
 *
 *  This helper funciton assumes that it is called after taking the
 *  User_semaphore.  It is unsafe otherwise.  There is no error checking for this.
 *  in the timesync_snapshore function.
 *******************************************************************************
 */
static void timesync_snapshot(void)
{

	struct tm *ts;
	char buf[MAX_TIMESYNC_LENGTH];
	char buf2[20];

	//__time64_t cur_msec;
	__time64_t cur_sec;
	__time64_t time_since_power_on;

	/*
	 * The format of the output string in the JSON packet is:
	 *
	 *   "timesync": "2023.01.17 12:53:55 (GMT 00:00) 0656741",
	 *
	 * where the data consists of the current date and time in “local” time,
	 * as configured when WIFI is set up.
	 * The last field (0656741) is an internal timestamp in milliseconds
	 * since power-on that corresponds to the current local time.
	 * This will make it possible to align timestamps from different devices.
	 *
	 * Note that the “current” local date and time is that returned from
	 * an SNTP server on the Internet and is subject to internet lag and
	 * internal processing times.
	 *
	 */

	// Get current time since power on in milliseconds
	user_time64_msec_since_poweron(&time_since_power_on);
	time64_string(buf2, &time_since_power_on);

	// Get current local time in seconds
	da16x_time64_sec(NULL, &cur_sec);

	// Convert to date and time
	ts = (struct tm *)da16x_localtime64(&cur_sec);
	// And get as a string
	da16x_strftime(buf, sizeof (buf), "%Y.%m.%d %H:%M:%S", ts);
	// And add time zone offset in case they configured this when
	// provisioning the device, and attach the time since power on.
	// NOTE: this function MUST be called after taking User_semaphore -- there is no semaphore protection
	// in this function.
	sprintf(pUserData->MQTT_timesync_current_time_str,
			"%s (GMT %+02d:%02d) %s",
				buf,   da16x_Tzoff() / 3600,   da16x_Tzoff() % 3600, buf2);

	vTaskDelay(3);
}



/**
 *******************************************************************************
 * @brief Process for reading some data from accelerometer
 *
 * 	Note that we can arrive here in one of three ways:
 * 	   1. wakened from sleep2
 * 	   2. interrupt while active (usually when transmitting MQTT)
 * 	   3. FIFO full detected by event loop polling
 *
 *******************************************************************************
 */
static int user_process_read_data(void)
{
	signed char rawdata[8];
	unsigned char fiforeg[2];
	int storestatus;
	unsigned int tx_trigger_threshold = MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_FAST;
	unsigned int tx_trigger_value = 0;
	uint8_t ISR_reason;
	__time64_t assigned_timestamp;		// timestamp to assign to FIFO reading
	ULONG ms_since_last_read;
	int erase_happened;		// tells us if an erase sector has happened

	// Make known to other processes that we are active
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		SET_BIT(processLists, USER_PROCESS_HANDLE_RTCKEY);
		xSemaphoreGive(Process_semaphore);
	}


	// Read entire FIFO contents until it is empty,
	// filling the data structure that we store each
	// interrupt

	// Read status register with FIFO content info
	fiforeg[0] = MC36XX_REG_STATUS_1;
	i2cRead(MC3672_ADDR, fiforeg, 1);



	// Note - need to make sure we don't read more than 32 here
	// Note that even though the FIFO interrupt threshold is something
	// like 30, there is a delay before we get to this point and we
	// are quite likely to read extra samples.
	int8_t dataptr = 0;
	while(((fiforeg[0] & 16) != 16) && (dataptr < MAX_ACCEL_FIFO_SIZE))
	{
		rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
		if (!i2cRead(MC3672_ADDR, rawdata, 6)) {
			PRINTF("\n Neuralert [%s] error reading data over i2c", __func__);
			// do nothing, the data is gone, but we might as well continue
		} else {
			receivedFIFO.Xvalue[dataptr] = rawdata[0];
			receivedFIFO.Yvalue[dataptr] = rawdata[2];
			receivedFIFO.Zvalue[dataptr] = rawdata[4];
			dataptr++;
		}

		// Reread status register with FIFO content info
		fiforeg[0] = MC36XX_REG_STATUS_1;
		i2cRead(MC3672_ADDR, fiforeg, 1);
	}

	// get a timestamp
	user_time64_msec_since_poweron(&assigned_timestamp);

	// Clear the interrupt pending in the accelerometer
	clear_intstate(&ISR_reason);

	// Set the number of data items in the buffer
	receivedFIFO.num_samples = dataptr;
	// Set an ever-increasing sequence number
	unsigned int write_fault_count = 0;
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking user semaphore", __func__);
		// this error could cause a hard fault in LmacMain when shutting down transmission.
		// solution is to reboot the system cleanly (to be overly safe).
		user_reboot();
	} else {
		pUserData->FIFO_reads_this_power_cycle++;
		pUserData->ACCEL_read_count++; // also used in terminate transmission (other thread)
		receivedFIFO.data_sequence = pUserData->ACCEL_read_count;
		receivedFIFO.accelTime = assigned_timestamp;
		receivedFIFO.accelTime_prev = pUserData->last_FIFO_read_time_ms;
		ms_since_last_read = assigned_timestamp - pUserData->last_FIFO_read_time_ms;
		pUserData->last_FIFO_read_time_ms = assigned_timestamp;
		write_fault_count = pUserData->write_fault_count;
		xSemaphoreGive(User_semaphore);

		PRINTF("\n Milliseconds since last AXL read: %u", ms_since_last_read);
		PRINTF("\n FIFO samples read: %d", dataptr);
	}



	// *****************************************************
	// Store the FIFO data structure into nonvol memory
	//
	// *** Note - as of 9/12/22 it's possible for the software
	//    to get into a permanent fail loop if a previous
	//    erase function failed.
	// *****************************************************
	storestatus = user_process_write_to_flash(&receivedFIFO, &erase_happened);
	if(storestatus != pdTRUE)
	{
		PRINTF("\n Neuralert: [%s] UNABLE TO WRITE DATA TO FLASH", __func__);
		PRINTF("\n Neuralert: [%s] FAULT COUNT: %d", __func__, write_fault_count);
	}
	if(erase_happened)
	{
		PRINTF("\n Neuralert: [%s] Erase sector happened", __func__);
	}

	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking user semaphore", __func__);
		// do nothing, just logging display
	} else {
		PRINTF("\n -------------------------------------------------\n");
		PRINTF(" Total FIFO blocks read since power on   : %d\n", pUserData->ACCEL_read_count);
		PRINTF(" Total FIFO write failures since power on: %d\n", pUserData->write_fault_count);
		PRINTF(" Total times a write retry was needed    : %d\n", pUserData->write_retry_count);
		PRINTF(" Total missed accelerometer interrupts   : %d\n", pUserData->ACCEL_missed_interrupts);
		PRINTF("-------------------------------------------------\n");
		PRINTF(" Total MQTT connect attempts             : %d\n", pUserData->MQTT_stats_connect_attempts);
		PRINTF(" Total MQTT connect fails                : %d\n", pUserData->MQTT_stats_connect_fails);
		PRINTF(" Total MQTT packets sent                 : %d\n", pUserData->MQTT_stats_packets_sent);
		PRINTF(" Total MQTT retry attempts               : %d\n", pUserData->MQTT_stats_retry_attempts);
		PRINTF(" Total MQTT transmit success             : %d\n", pUserData->MQTT_stats_transmit_success);
		PRINTF(" MQTT tx attempts since tx success       : %d\n", pUserData->MQTT_attempts_since_tx_success);
		PRINTF(" ------------------------------------------------\n");


		// See if it's time to transmit data, based on how many FIFO buffers we've
		// accumulated since the last transmission
		tx_trigger_value = ++pUserData->ACCEL_transmit_trigger;  // Increment FIFO stored counter

		if (pUserData->MQTT_attempts_since_tx_success <= MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_TEST) {
			tx_trigger_threshold = MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_FAST;
		} else {
			tx_trigger_threshold = MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_SLOW;
		}

		xSemaphoreGive(User_semaphore);
	}

	PRINTF(" ACCEL transmit trigger: %d of %d\n", tx_trigger_value, tx_trigger_threshold);


	//mqtt_started = pdFALSE;
	if(tx_trigger_value >= tx_trigger_threshold)
	{

		// increment MQTT attempt since tx success counter (we're going to try a transmission)
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
			// not critical to address this.  if we miss too many, then the system will delay entering the
			// "long" transmission interval.
		} else {
			pUserData->MQTT_attempts_since_tx_success++;
			xSemaphoreGive(User_semaphore);
		}


		// Check if MQTT is still active before starting again
		int val = pdTRUE;
		if (take_semaphore(&Process_semaphore)) {
			PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
			user_reboot();
		} else {
			val = BIT_SET(processLists, USER_PROCESS_MQTT_TRANSMIT);
			xSemaphoreGive(Process_semaphore);
		}

		if (val) {
			// MQTT is still active. check if we're making progress -- if not, stop the transmission
			if (take_semaphore(&User_semaphore)) {
				PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
				// do nothing, this will trigger a transmission termination event below, which is fine.
			} else {
				pUserData->ACCEL_transmit_trigger = 0; // reset the transmit trigger value
				unsigned int packets_sent = pUserData->MQTT_stats_packets_sent;
				if (packets_sent > pUserData->MQTT_stats_packets_sent_last) {
					pUserData->MQTT_stats_packets_sent_last = packets_sent; //
				} else {
					PRINTF("\n MQTT task still active and not making progress. Stopping transmission.");
					xSemaphoreGive(User_semaphore);
					user_terminate_transmit();
				}
				xSemaphoreGive(User_semaphore);
			}
		} else {
			// Send the event that will start the MQTT transmit task
			if (take_semaphore(&User_semaphore)) {
				PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
				// do nothing (try again next time)
			} else {
				pUserData->ACCEL_transmit_trigger = 0;
				pUserData->MQTT_stats_connect_attempts++;
				xSemaphoreGive(User_semaphore);

				user_start_data_tx(); // potentially needs the user semaphore
			}
		} // MQTT still active or not
	} // trigger value above threshold

	// Signal that we're finished so we can sleep
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
		// do nothing, the result will be the system doesn't go to sleep and try again next RTCKEY
	} else {
		CLR_BIT(processLists, USER_PROCESS_HANDLE_RTCKEY);
		xSemaphoreGive(Process_semaphore);
	}

	return 0;
}





/**
 *******************************************************************************
 * @brief Accelerometer initialization for our application
 *******************************************************************************
 */
void user_initialize_accelerometer(void)
{

	UINT32 intr_src;

	unsigned char fiforeg[2];
	signed char rawdata[8];
	int i = 0;

	uint8_t ISR_reason;

	mc3672Init();

	vTaskDelay(250); // a large delay for the accelerometer to get initialized

	// While loop to empty contents of FIFO before enabling RTC ISR  - NJ 6/30/2022
	fiforeg[0] = MC36XX_REG_STATUS_1;
	i2cRead(MC3672_ADDR, fiforeg, 1);
	while((fiforeg[0] & 16) != 16){
		rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
		i2cRead(MC3672_ADDR, rawdata, 6);
		fiforeg[0] = MC36XX_REG_STATUS_1;
		i2cRead(MC3672_ADDR, fiforeg, 1);
		i++;
	}
	PRINTF("\n Neuralert: [%s] FIFO buffer emptied %d samples", __func__, i);

	// Assign the initial timestamp.
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore");
		// do nothing, just internal logging
	} else {
		user_time64_msec_since_poweron(&(pUserData->last_FIFO_read_time_ms));
		xSemaphoreGive(User_semaphore);
	}

	// Clear any accelerometer interrupt that might be pending
	clear_intstate(&ISR_reason);

	//This seems to initialize the wake-up controller in the current sdk
	RTC_IOCTL(RTC_GET_RTC_CONTROL_REG, &intr_src);
	intr_src |= WAKEUP_INTERRUPT_ENABLE(1);
	RTC_IOCTL(RTC_SET_RTC_CONTROL_REG, &intr_src);

	PRINTF("\n Neuralert: [%s] Accelerometer initialized", __func__);
}


/**
 *******************************************************************************
 * @brief Process for boot-up event
 *******************************************************************************
 */
static int user_process_bootup_event(void)
{



	// initialize boot up process variables
	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		SET_BIT(processLists, USER_PROCESS_BOOTUP);
		SET_BIT(processLists, USER_PROCESS_BLOCK_MQTT);
		xSemaphoreGive(Process_semaphore);
	}


	PRINTF("\n Neuralert: [%s] bootup event", __func__);
	PRINTF("\n Software part number  :    %s", USER_SOFTWARE_PART_NUMBER_STRING);
	PRINTF("\n Software version      :    %s", USER_VERSION_STRING);
	PRINTF("\n Software build time   : %s %s", __DATE__ , __TIME__ );
	notify_user_LED();

	// Turn off wifi, if it somehow got turned on
	wifi_cs_rf_cntrl(TRUE);		// RF now off

#if 0
	// Whether WIFI is connected or not, see if we can obtain our
	// MAC address
	// Check our MAC string used as a unique device identifier
	int MACaddrtype = 0;
	memset(macstr, 0, 18);
	memset(MACaddr, 0,7);
	while (MACaddrtype == 0) { // we need the MACaddr -- without it, data packets will be wrong.
		MACaddrtype = getMACAddrStr(1, macstr);  // Hex digits string together
		vTaskDelay(10);
	}
	MACaddr[0] = macstr[9];
	MACaddr[1] = macstr[10];
	MACaddr[2] = macstr[12];
	MACaddr[3] = macstr[13];
	MACaddr[4] = macstr[15];
	MACaddr[5] = macstr[16];
	PRINTF("\n MAC address - %s (type: %d)", macstr, MACaddrtype);
	//vTaskDelay(10); // delay between obtaining the MAC addr and strcpy (below).  Delay for reset achieves this.
#endif

	// The following delay gives the user time to type in a
	// command, such as:
	//   "reset" to get to the ROM monitor to reflash the software
	//   "user" and "run 0" to go back to provisioning mode (still needs power down and up)
	PRINTF("\n Delay for reset", __func__);
	vTaskDelay(pdMS_TO_TICKS(10000));
	PRINTF("\n End reset delay", __func__);



	// Just in case the autoconnect got turned on, make sure it is off
	user_process_disable_auto_connection();

	// Initialize the accelerometer buffer external flash
	if (!user_process_initialize_AB()) {
		PRINTF("\n Neuralert: [%s] Accelerometer flash buffer NOT initialized", __func__);
		user_reboot();
	}
	PRINTF("\n Accelerometer flash buffer initialized");

	// Copy the MACaddr to the UserData.  Note, strcpy spins up a new thread with higher priority
	// and a hard fault appears to occur.  Make sure there is a delay between obtaining the MAC addr (above)
	// and before calling strcpy (below)

	// Initialize the user data
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking user semaphore", __func__);
		user_reboot();
	} else {
		// Store device ID
		strcpy (pUserData->Device_ID, MACaddr);

		// Initialize the accelerometer timestamp bookkeeping
		pUserData->last_FIFO_read_time_ms = 0;
		pUserData->last_wakeup_msec = 0;


		pUserData->ACCEL_missed_interrupts = 0;
		pUserData->ACCEL_transmit_trigger = MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS_FAST - MQTT_FIRST_TRANSMIT_TRIGGER_FIFO_BUFFERS;

		// Initialize the FIFO interrupt cycle statistics
		pUserData->FIFO_reads_this_power_cycle = 0;



		// Initialize the MQTT statistics
		pUserData->MQTT_stats_connect_attempts = 0;
		pUserData->MQTT_stats_connect_fails = 0;
		pUserData->MQTT_stats_packets_sent = 0;
		pUserData->MQTT_stats_retry_attempts = 0;
		pUserData->MQTT_stats_transmit_success = 0;
		pUserData->MQTT_transmission_number = 0;
		pUserData->MQTT_pub_msg_id = 0;
		pUserData->MQTT_stats_packets_sent_last = 0;
		pUserData->MQTT_inflight = 0;

		xSemaphoreGive(User_semaphore);
	}

	// Initialize the accelerometer and enable the AXL interrupt
	user_initialize_accelerometer();

#ifdef CFG_USE_SYSTEM_CONTROL
	// Disabling WLAN at the next boot.
	system_control_wlan_enable(FALSE);
#endif

	if (take_semaphore(&Process_semaphore)) {
		PRINTF("\n Neuralert [%s] error taking process semaphore", __func__);
		user_reboot();
	} else {
		CLR_BIT(processLists, USER_PROCESS_BLOCK_MQTT);
		xSemaphoreGive(Process_semaphore);
	}

	return pdFALSE;
}



/**
 *******************************************************************************
 * @brief Process all events
 *******************************************************************************
 */
static UCHAR user_process_event(UINT32 event)
{
	__time64_t awake_time = 0;
	__time64_t current_msec_since_boot;

	PRINTF("%s: Event: [%d]\n", __func__, event);

	// Power-on boot
	// This is only expected to happen once when the device is
	// activated
	if (event & USER_BOOTUP_EVENT) {

		isPowerOnBoot = pdTRUE;		// tell accelerometer what's up

		// Do one-time power-on stuff
		user_process_bootup_event();

		// Check if possible to sleep
		user_sleep_ready_event();
	}

	// Wakened from low-power sleep by accelerometer interrupt
	if (event & USER_WAKEUP_BY_RTCKEY_EVENT) {
		PRINTF("\n Wake by RTCKEY event");

		// log the time we woke up (close enough to accurate -- just for logging)
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking user semaphore");
			// do nothing, just logging
		} else {
			user_time64_msec_since_poweron(&(pUserData->last_wakeup_msec));
			xSemaphoreGive(User_semaphore);
		}

		isAccelerometerWakeup = pdTRUE;	// tell accelerometer why it's awake

		// We were wakened only by an accelerometer interrupt so
		// we can safely turn off the RF section. This shouldn't be necessary,
		// but is good practice to ensure we aren't draining the battery
		wifi_cs_rf_cntrl(TRUE);

		// Read accelerometer data and store until time to transmit
		user_process_read_data();

		// Check if possible to sleep
		user_sleep_ready_event();
	}


	// This event occurs when a missed accelerometer interrupt is detected (FIFO at threshold)
	if (event & USER_MISSED_RTCKEY_EVENT) {
		PRINTF("** %s RTCKEY in TIMER event\n", __func__); // FRSDEBUG

		// Accelerometer interrupt while we're transmitting
		// Read the data
		user_process_read_data();
		// And see if we are able to sleep
		user_sleep_ready_event();
	}


	if (event & USER_SLEEP_READY_EVENT) {
		UINT32 val = 0;
		if (take_semaphore(&Process_semaphore)) {
			PRINTF("\n Neuralert: [%s] error taking process semaphore\n", __func__);
			SET_BIT(val, USER_PROCESS_SEMAPHORE_ERROR); // don't let the system go to sleep
		} else {
			val = processLists;
			xSemaphoreGive(Process_semaphore);
		}

		PRINTF("USER_SLEEP_READY_EVENT: processLists: 0x%x\n", val);
		if (val == 0) {

			user_time64_msec_since_poweron(&current_msec_since_boot);

			if (take_semaphore(&User_semaphore)) {
				PRINTF("\n Neuralert: [%s] error taking user semaphore");
				// do nothing, just internal logging
			} else {
				// Get relative time since power on from the RTC time counter register
				awake_time = current_msec_since_boot - pUserData->last_wakeup_msec;
				PRINTF("Entering sleep1. msec awake %u \n\n", awake_time);
				xSemaphoreGive(User_semaphore);
			}
			vTaskDelay(5); // delay to let everything settle down before sleep 1

			// enter into sleep 1
			extern void fc80211_da16x_pri_pwr_down(unsigned char retention);
			fc80211_da16x_pri_pwr_down(TRUE);
		}
		else
		{
			user_time64_msec_since_poweron(&current_msec_since_boot);

			if (take_semaphore(&User_semaphore)) {
				PRINTF("\n Neuralert: [%s] error taking user semaphore");
				PRINTF("\n Unable to sleep. \n", __func__);
				// do nothing, just used for logging
			} else {
				awake_time = current_msec_since_boot - pUserData->last_wakeup_msec;
				xSemaphoreGive(User_semaphore);
				PRINTF("Unable to sleep. msec awake %u\n\n", awake_time);
			}
		}
	}

	/* Continue in process */
	return PROCESS_EVENT_CONTINUE;
}



/**
 ****************************************************************************************
 * @brief mqtt_client sample callback function for processing PUBLISH messages \n
 * Users register a callback function to process a PUBLISH message. \n
 * In this example, when mqtt_client receives a message with payload "1",
 * it sends MQTT PUBLISH with payload "DA16K status : ..." to the broker connected.
 * @param[in] buf the message paylod
 * @param[in] len the message paylod length
 * @param[in] topic the topic this mqtt_client subscribed to
 ****************************************************************************************
 */
void neuralert_mqtt_msg_cb(const char *buf, int len, const char *topic)
{
    DA16X_UNUSED_ARG(len);
    DA16X_UNUSED_ARG(buf);
    DA16X_UNUSED_ARG(topic);

    PRINTF("\n**Neuralert: %s \n", __func__); // FRSDEBUG
    //BaseType_t ret;

    //PRINTF(CYAN_COLOR "[MQTT_SAMPLE] Msg Recv: Topic=%s, Msg=%s \n" CLEAR_COLOR, topic, buf);

    //if (strcmp(buf, "reply_needed") == 0) {
    //    if ((ret = my_app_send_to_q(NAME_JOB_MQTT_TX_REPLY, &tx_reply, APP_MSG_PUBLISH, NULL)) != pdPASS ) {
    //        PRINTF(RED_COLOR "[%s] Failed to add a message to Q (%d)\r\n" CLEAR_COLOR, __func__, ret);
    //    }
    //} else if (strncmp(buf, APP_UNSUB_HDR, 6) == 0) {
    //    if ((ret = my_app_send_to_q(NAME_JOB_MQTT_UNSUB, NULL, APP_MSG_UNSUB, buf)) != pdPASS ) {
    //        PRINTF(RED_COLOR "[%s] Failed to add a message to Q (%d)\r\n" CLEAR_COLOR, __func__, ret);
    //    }
    //} else {
    //    return;
    //}
}

void neuralert_mqtt_pub_cb(int mid)
{
	PRINTF("\n**Neuralert: %s \n", __func__); // FRSDEBUG
	DA16X_UNUSED_ARG(mid);
    //xEventGroupSetBits(my_app_event_group, EVT_PUB_COMPLETE);
}

void neuralert_mqtt_conn_cb(void)
{
	PRINTF("\n**Neuralert: %s \n", __func__); // FRSDEBUG
    //topic_count = 0;
}

void neuralert_mqtt_sub_cb(void)
{
	PRINTF("\n**Neuralert: %s \n", __func__); // FRSDEBUG
	//topic_count++;
    //if (dpm_mode_is_enabled() && topic_count == mqtt_client_get_topic_count()) {
    //    my_app_send_to_q(NAME_JOB_MQTT_PERIODIC_PUB_RTC_REGI, NULL, APP_MSG_REGI_RTC, NULL);
    //}

}


/**
 ****************************************************************************************
 * @brief Initialize the user task
 *        This is called for three reasons:
 *           1. Power-on reset
 *           2. Wake from sleep by accelerometer interrupt
 *           3. Wake from sleep by timer (original timed architecture)
 ****************************************************************************************
 */
static void user_init(void)
{
	int runMode;
	char *pResultStr = NULL;

	runMode = get_run_mode();
 	pResultStr = read_nvram_string(NVR_KEY_SSID_0);
	if (runMode == SYSMODE_STA_ONLY && strlen(pResultStr))
	{
		UINT32 wakeUpMode;

		wakeUpMode = da16x_boot_get_wakeupmode();
		PRINTF("%s: Boot type: 0x%X (%d)\n", __func__, wakeUpMode, wakeUpMode);

#ifdef CFG_USE_RETMEM_WITHOUT_DPM
		/* Create the user retention memory and Initialize. */
		unsigned int result = user_retmmem_get(USER_RTM_DATA_TAG, (UCHAR **)&pUserData);
		if (result == 0) {
			PRINTF("\n Neuralert: [%s] allocating retention memory\n", __func__);
			// created a memory for the user data
			result = user_retmmem_allocate(USER_RTM_DATA_TAG, (void**)&pUserData, sizeof(UserDataBuffer));
			if (result == 0) {
				memset(pUserData, 0, sizeof(UserDataBuffer));
			} else {
				PRINTF("\n Neuralert: [%s] Failed to allocate retention memory", __func__);
				user_reboot();
			}
		}

#endif


		/*
		 * Create a semaphore to make sure each task can have exclusive
		 * access to flash AB data memory
		 */
		Flash_semaphore = xSemaphoreCreateMutex();
		if (Flash_semaphore == NULL)
		{
			PRINTF("\n Neuralert: [%s] Error creating Flash semaphore", __func__);
			user_reboot();
		}


		/*
		 * Create a semaphore to make sure each task can have exclusive access to
		 * the MQTT transmission statistics.  These statistics are printed and logged
		 * by the user read task and written by the mqtt transmission task
		 */
		User_semaphore = xSemaphoreCreateMutex();
		if (User_semaphore == NULL)
		{
			PRINTF("\n Neuralert: [%s] Error creating stats semaphore", __func__);
			user_reboot();
		}

		/*
		 * Create a semaphore to make sure each task can have exclusive access to
		 * the processList statistics.
		 */
		Process_semaphore = xSemaphoreCreateMutex();
		if (Process_semaphore == NULL)
		{
			PRINTF("\n Neuralert: [%s] Error creating process semaphore", __func__);
			user_reboot();
		}


		// Initialize the FIFO interrupt cycle statistics
		if (take_semaphore(&User_semaphore)) {
			PRINTF("\n Neuralert [%s] error taking user semaphore");
			// do nothing, it is just a variable used for local accounting and doesn't affect functionality
		} else {
			pUserData->FIFO_reads_this_power_cycle = 0;
			xSemaphoreGive(User_semaphore);
		}





		/*
		 * Dispatch different events depending on why we
		 * woke up
		 */
		switch (wakeUpMode) {
			// wake up for some reason
			case WAKEUP_SOURCE_EXT_SIGNAL:
			case WAKEUP_EXT_SIG_WITH_RETENTION:
				// Accelerometer interrupt woke us from sleep
				user_wakeup_by_rtckey_event();
				break;

			// wake up from a software reboot.
			case WAKEUP_RESET:
				isSysNormalBoot = pdTRUE;
				user_send_bootup_event_message();
				break;

			// A WAKEUP_SOURCE_POR will trigger this on the first bootup.  We don't know what will happen during
			// brownout and there are fears it could trigger a WAKEUP_SOURCE_POR.  If that happens, we want a clean
			// reboot.  Also, in case of a WATCHDOG or hard fault, we want a clean reboot.  The default case below
			// executes a software reboot (which will trigger a WAKEUP_RESET -- see above).
			default:
				user_reboot();
				break;

		}
	} else if (runMode == SYSMODE_AP_ONLY) {
		wifi_cs_rf_cntrl(FALSE);
		PRINTF("\n Neuralert [%s] system configured as an access point (wrong)", __func__);
		user_reboot();
	}
}


/**
 ****************************************************************************************
 * @brief perform a user called reboot
 *        There are lots of ways this app can crash -- we've handled the biggest issues
 *        but the real corner cases will just trigger a reboot.
 ****************************************************************************************
 */
static void user_reboot(void)
{
	vTaskDelay(100);
	reboot_func(SYS_REBOOT_POR);

	/* Wait for system-reboot */
	while (1) {
		vTaskDelay(10);
	}
}



/**
 ****************************************************************************************
 * @brief parsing data for received data from phone
 * @param[in] _recData  received data
 * @return  int
 ****************************************************************************************
 */
static int neuralert_provisioning_step1(const char *_recData, char *reply_buf, uint32_t *reply_len)
{
	int return_val = pdTRUE;
	cJSON *cur_json = NULL;


	PRINTF("Retrieving Device Request ... \r\n");
    cur_json = cJSON_Parse(_recData);

	if (cur_json->type != cJSON_Object) {
		PRINTF("[%s]: type: %d, expected %d\r\n", __func__, cur_json->type, cJSON_Object);
		goto end_of_task;
	}

	// Dictionary with an Entry corresponding to "KEY"
	cur_json = cur_json->child;
	while (cur_json != NULL) {
		if (cur_json->string == NULL) {
			PRINTF("[%s] key value not specified (must be KEY)\r\n", __func__);
			goto end_of_task;
		}


		if (strcmp(cur_json->string, "KEY") == 0) {
			if (cur_json->type != cJSON_String) {
				PRINTF("[%s]: value type: %d, expected %d\r\n", __func__,  cur_json->type, cJSON_String);
				goto end_of_task;
			}

			if (strcmp(cur_json->valuestring, USER_PROVISIONING_KEY) == 0) {
				PRINTF("Provided key matches, reply with device info\r\n");

				*reply_len = sprintf(reply_buf,"{\"software\":\"%s\",\"version\":\"%s\",\"build\":\"%s %s\",\"id\":\"%s\"}",
	USER_SOFTWARE_PART_NUMBER_STRING, USER_VERSION_STRING, __DATE__, __TIME__, MACaddr);

			} else {
				PRINTF("[%s] key is incorrect\r\n", __func__);
				goto end_of_task;
			}
		} else {
			PRINTF("[%s] Unknown string (must be KEY): %s\r\n", __func__, cur_json->string);
			goto end_of_task;
		}

		cur_json = cur_json->next;
	}

	return_val = pdFALSE;

end_of_task:

    cJSON_Delete(cur_json);
    return return_val;
}





/**
 ****************************************************************************************
 * @brief parsing data for received data from phone
 * @param[in] _recData  received data
 * @return  int
 ****************************************************************************************
 */
static int neuralert_provisioning_step2(const char *_recData)
{
	int return_val = pdTRUE;
	int ret = 0;
    cJSON *json_recv_data = NULL;
	cJSON *nvram_or_cert = NULL;
	cJSON *cur_json = NULL;
	cJSON *cert_json = NULL;


	PRINTF("Clearing NVRAM and CERTs ... \r\n");
	cert_flash_delete_all(); // clear all certs
	//We could clear the network configuration here if we want.


	PRINTF("Executing Provisioning ... \r\n");
    json_recv_data = cJSON_Parse(_recData);

	if (json_recv_data->type != cJSON_Object) {
		PRINTF("[%s]: type: %d, expected %d\r\n", __func__, json_recv_data->type, cJSON_Object);
		goto end_of_task;
	}

	// Top-level API supports specification of "CERT" or "NVRAM" keys (different APIs for each)
	nvram_or_cert = json_recv_data->child;
	while (nvram_or_cert != NULL) {
		if (nvram_or_cert->string == NULL) {
			PRINTF("[%s] key value not specified (must be NVRAM or CERT)\r\n", __func__);
			goto end_of_task;
		}

		if (nvram_or_cert->type != cJSON_Object) {
			PRINTF("[%s]: value type: %d, expected %d\r\n", __func__, nvram_or_cert->type, cJSON_Object);
			goto end_of_task;
		}


		if (strcmp(nvram_or_cert->string, "NVRAM") == 0) {
			// set cur_json to first key:value pair in NVRAM object, then iterate
			cur_json = nvram_or_cert->child;
			while (cur_json != NULL) {
				if (cur_json->string == NULL) {
					PRINTF("[%s] NVRAM key value not specified\r\n", __func__);
					goto end_of_task;
				}

				// In this implementation, we are writing directly to flash (not cache) this is to simplify the API
				// since the cache API doesn't allow for const char names in the current API.  Const Char will make the
				// API more readable/interpretable for the provisioning APP developer, at a small cost in provisioning
				// energy/time.
				if (cur_json->type == cJSON_Number) {
					PRINTF("NVRAM type: %d, key: %s, value: %d\r\n", cur_json->type, cur_json->string, cur_json->valueint);
					ret = write_nvram_int(cur_json->string, cur_json->valueint);
					if (ret != 0){
						PRINTF("[%s] NVRAM int write error: %d\r\n", __func__, ret);
						goto end_of_task;
					}
				} else if (cur_json->type == cJSON_String) {
					PRINTF("NVRAM type: %d, key: %s, value: %s\r\n", cur_json->type, cur_json->string, cur_json->valuestring);
					ret = write_nvram_string(cur_json->string, cur_json->valuestring);
					if (ret != 0){
						PRINTF("[%s] NVRAM string write error: %d\r\n", __func__, ret);
						goto end_of_task;
					}
				} else {
					PRINTF("[%s] Unknown type: %d\r\n", __func__, cur_json->type);
					goto end_of_task;
				}

				cur_json = cur_json->next;
			}
		} else if (strcmp(nvram_or_cert->string, "CERT") == 0) {
			// set cur_json to first key:value pair in CERT object, then iterate
			cur_json = nvram_or_cert->child;
			while (cur_json != NULL) {
				if (cur_json->string == NULL) {
					PRINTF("[%s] CERT key value not specified\r\n", __func__);
					goto end_of_task;
				}

				// all values will be cJSON_Objects in the CERT branch
				if (cur_json->type != cJSON_Object) {
					PRINTF("[%s]: CERT value type: %d, expected %d\r\n", __func__, cur_json->type, cJSON_Object);
					goto end_of_task;
				}

				da16x_cert_t cert_data = {{0, 0, 0, 0}, {0}};
				char *cert_ptr = NULL;

				cert_json = cur_json->child;
				while (cert_json != NULL) {
					if (cert_json->string == NULL) {
						PRINTF("[%s] CERT key value not specified\r\n", __func__);
						goto end_of_task;
					}

					if (strcmp(cert_json->string, "module") == 0) {
						PRINTF("CERT key: %s, value: %d\r\n", cert_json->string, cert_json->valueint);
						cert_data.info.module = (unsigned int)cert_json->valueint;
					} else if (strcmp(cert_json->string, "type") == 0) {
						PRINTF("CERT key: %s, value: %d\r\n", cert_json->string, cert_json->valueint);
						cert_data.info.type = (unsigned int)cert_json->valueint;
					} else if (strcmp(cert_json->string, "format") == 0) {
						PRINTF("CERT key: %s, value: %d\r\n", cert_json->string, cert_json->valueint);
						cert_data.info.format = (unsigned int)cert_json->valueint;
					} else if (strcmp(cert_json->string, "cert") == 0) {
						PRINTF("CERT key: %s, value: %s\r\n", cert_json->string, cert_json->valuestring);
						cert_ptr = cert_json->valuestring;
					} else if (strcmp(cert_json->string, "cert_len") == 0) {
						PRINTF("CERT key: %s, value: %d\r\n", cert_json->string, cert_json->valueint);
						cert_data.info.cert_len = (unsigned int)cert_json->valueint;
					} else {
						PRINTF("[%s] CERT Unknown key: %d\r\n", __func__, cert_json->type);
						goto end_of_task;
					}

					cert_json = cert_json->next;
				}

				ret = da16x_cert_write((int)cert_data.info.module, (int)cert_data.info.type, (int)cert_data.info.format,
					(unsigned char*)cert_ptr, cert_data.info.cert_len);
				if (ret != 0){
					PRINTF("[%s] Certificate write error: %d\r\n", __func__, ret);
					goto end_of_task;
				}


				cur_json = cur_json->next;
			}
		} else {
			PRINTF("[%s] Unknown string (must be NVRAM or CERT): %s\r\n", __func__, cur_json->string);
			goto end_of_task;
		}

		nvram_or_cert = nvram_or_cert->next;
	}

	return_val = pdFALSE;

end_of_task:

    cJSON_Delete(json_recv_data);
    return return_val;
}



/**
 ****************************************************************************************
 * @brief dump hex data from TCP
 *        This is used for provisioning only!
 ****************************************************************************************
 */
void tcp_hex_dump(const char *title, unsigned char *buf, size_t len)
{
#if defined (TCP_ENABLED_HEXDUMP)
	//extern void hex_dump(unsigned char *data, unsigned int length);

	if (len) {
		PRINTF("%s(%ld)\n", title, len);
		hex_dump(buf, len);
	}
#else
	DA16X_UNUSED_ARG(title);
	DA16X_UNUSED_ARG(buf);
	DA16X_UNUSED_ARG(len);
#endif // (TCP_ENABLED_HEXDUMP)

}



/**
 ****************************************************************************************
 * @brief start a tcp server for provisioning (only used for provisioning).
 *
 *
 ****************************************************************************************
 */
void tcp_server_thread(void *arg)
{
    DA16X_UNUSED_ARG(arg);

    int ret = 0;
    int listen_sock = -1;
    int client_sock = -1;
	int provisioning_step = 1; // starts at 1 (not 0)

    struct sockaddr_in server_addr;

    struct sockaddr_in client_addr;
    int client_addrlen = sizeof(struct sockaddr_in);

    int len = 0;
    unsigned char data_buffer[TCP_SERVER_DEF_BUF_SIZE] = {0x00,};

    memset(&server_addr, 0x00, sizeof(struct sockaddr_in));
    memset(&client_addr, 0x00, sizeof(struct sockaddr_in));

    PRINTF("[%s] Start of TCP Server sample\r\n", __func__);

    listen_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        PRINTF("[%s] Failed to create listen socket\r\n", __func__);
        goto end_of_task;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_SERVER_DEF_PORT);

    ret = bind(listen_sock, (struct sockaddr *)&server_addr,
               sizeof(struct sockaddr_in));
    if (ret == -1) {
        PRINTF("[%s] Failed to bind socket\r\n", __func__);
        goto end_of_task;
    }

    ret = listen(listen_sock, TCP_SERVER_BACKLOG);
    if (ret != 0) {
        PRINTF("[%s] Failed to listen socket of tcp server(%d)\r\n",
               __func__, ret);
        goto end_of_task;
    }

	unsigned char* msg_buf = NULL;
	unsigned char* reply_buf = NULL;
	uint32_t reply_len = 100; // hardcoded here
    while (1) {
        client_sock = -1;
        memset(&client_addr, 0x00, sizeof(struct sockaddr_in));
        client_addrlen = sizeof(struct sockaddr_in);

        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr,
                             (socklen_t *)&client_addrlen);
        if (client_sock < 0) {
            continue;
        }

        PRINTF("Connected client(%d.%d.%d.%d:%d)\r\n",
               (ntohl(client_addr.sin_addr.s_addr) >> 24) & 0xFF,
               (ntohl(client_addr.sin_addr.s_addr) >> 16) & 0xFF,
               (ntohl(client_addr.sin_addr.s_addr) >>  8) & 0xFF,
               (ntohl(client_addr.sin_addr.s_addr)      ) & 0xFF,
               (ntohs(client_addr.sin_port)));

    	// clear the message buffer if not NUll (should always be null)
    	if (msg_buf != NULL) {
    		vPortFree(msg_buf);
    		msg_buf = NULL;
    	}
    	uint32_t msg_len = 0;
    	uint32_t msg_pos = 0;
        while (1) {


            memset(data_buffer, 0x00, sizeof(data_buffer));

            PRINTF("< Read from client: ");

            len = recv(client_sock, data_buffer, sizeof(data_buffer), 0);
            if (len <= 0) {
                PRINTF("[%s] Failed to receive data(%d)\r\n", __func__, len);
                break;
            }
        	data_buffer[len] = '\0';

        	// Print the following for debugging purposes
        	PRINTF("%d bytes read\r\n", len);
        	tcp_hex_dump("Received data", (unsigned char *)data_buffer, len);

			// initialize new message
			if (msg_len == 0) {
				// store message length
				msg_len = (data_buffer[0] << 24) + (data_buffer[1] << 16) + (data_buffer[2] << 8) + data_buffer[3];
				PRINTF("Size of incoming message: (%d) (%x %x %x %x)\n", msg_len, data_buffer[0],
					data_buffer[1], data_buffer[2], data_buffer[3]);
				vTaskDelay(3);

				// initialize message buffer
				msg_buf = pvPortMalloc(msg_len + 1);
				memset(msg_buf, 0x00, sizeof(msg_buf));

				// copy the buffer data to msg_buf (minus the first four bytes)
				for (int i = 4; i < len; i++) {
					msg_buf[i-4] = data_buffer[i];
				}
				msg_pos += (len - 4);
			} else {

				// copy the buffer data to msg_buf
				for (int i = 0; i < len; i++) {
					msg_buf[i+msg_pos] = data_buffer[i];
				}
				msg_pos += len;
			}

        	// check if we have received all the data yet
        	if (msg_len == msg_pos) {
        		msg_buf[msg_pos] = '\0'; // signify end of message
        		break;
        	} else {
        		PRINTF("msg_len: %d, msg_pos: %d \n", msg_len, msg_pos);
        	}
        }

    	// Print results of the provisioning packet
    	PRINTF("%d bytes read\r\n", msg_len);
    	tcp_hex_dump("Received message", (unsigned char *)msg_buf, msg_len);


    	// execute provisioning
    	int fail_flag = pdTRUE;
    	if (msg_buf != NULL) {

    		if (provisioning_step == 1) {

    			// initialize the reply buffer
    			if (reply_buf != NULL) {
    				vPortFree(reply_buf);
    				reply_buf = NULL;
    			}
    			reply_buf = pvPortMalloc(reply_len + 1);
    			memset(reply_buf, 0x00, sizeof(reply_buf));

    			fail_flag = neuralert_provisioning_step1((char *)msg_buf, (char *)reply_buf, &reply_len);
    			reply_buf[reply_len] = '\0';

    			if (!fail_flag) {
    				// Send the provisioning packet back to the host (only if successful)
    				PRINTF("> Write to client: ");
    				len = send(client_sock, reply_buf, reply_len, 0);
    				vPortFree(reply_buf);
    				reply_buf = NULL;

    				if (len <= 0) {
    					PRINTF("[%s] Failed to send data\r\n", __func__);
    					continue;
    				}
    				PRINTF("%d bytes written\r\n", len);
    				provisioning_step = 2; // setup the next TCP exchange for provisioning
    			}

    			// make sure the reply buffer is freed
    			if (reply_buf != NULL) {
    				vPortFree(reply_buf);
    				reply_buf = NULL;
    			}
    		} else if (provisioning_step == 2) {
    			fail_flag = neuralert_provisioning_step2((char *)msg_buf);

    			if (!fail_flag) {
    				// Send the provisioning packet back to the host (only if successful)
    				PRINTF("> Write to client: ");
    				len = send(client_sock, msg_buf, msg_len, 0);
    				if (len <= 0) {
    					PRINTF("[%s] Failed to send data\r\n", __func__);
    					continue;
    				}
    				PRINTF("%d bytes written\r\n", len);
    				provisioning_step = 3;
    			}
    		}

    		// Free the message buffer -- we're done (regardless of success or fail)
    		vPortFree(msg_buf);
    		msg_buf = NULL;
    	}

    	if (fail_flag){
    		PRINTF("Error parsing json -- trying again\r\n");
    		continue;
    	}

    	// check if we're done provisioning
    	if (provisioning_step == 3) {
    		close(client_sock);
    		PRINTF("Disconnected client\r\n");
    		goto end_of_task;
    	}
    }

end_of_task:

    PRINTF("[%s] End of Provisioning application\r\n", __func__);

    close(listen_sock);
    close(client_sock);

	// Last thing to do is set the run flag to 2 since the device is now provisioned.
	ret = write_nvram_int("RUN_FLAG", 2);
	if (ret != 0){
		PRINTF("[%s] NVRAM int write error: %d\r\n", __func__, ret);
		goto end_of_task;
	}

	user_reboot();
	vTaskDelete(NULL); // will likely never get called, but leaving for posterity
}


/**
 ****************************************************************************************
 * @brief Entry function for APP provisioning
 * @param[in] _mode
 *    Generic SDK:1,
 *    Platform AWS: Generic:10, ATCMD:11, ...
 *    Platform AZURE: Generic:20, ATCMD:21, ...
 * @return  void
 ****************************************************************************************
 */
void app_start_provisioning(int32_t _mode)
{
    int status;
    TaskHandle_t TCPServerThreadPtr = NULL;



    // TCP provisioning
    /* Create provision TCP thread on Soft-AP mode */
    status = xTaskCreate(tcp_server_thread,
    APP_SOFTAP_PROV_NAME,
    APP_SOFTAP_PROV_STACK_SZ, (void*)NULL,
    OS_TASK_PRIORITY_USER + 3, &TCPServerThreadPtr);
    if (status != pdPASS) {
        APRINTF("[%s] Failed to create TCP svr thread\r\n", __func__);
    }

}

/**
 ****************************************************************************************
 * @brief SoftAP Provisioning application thread calling function
 * @param[in] arg - transfer information
 * @return  void
 *
 * Note: this code was adapted taken from app_provisioning_sample.c
 *
 * The Neuralert provisioning only supports SYSMODE_AP_ONLY ... no concurrent station+AP
 * support.
 ****************************************************************************************
 */
void softap_provisioning(void *arg)
{
    int sys_wdog_id = -1;
    int sysmode;

    DA16X_UNUSED_ARG(arg);

    sys_wdog_id = da16x_sys_watchdog_register(pdFALSE);

    da16x_sys_watchdog_notify(sys_wdog_id);


SOFTAP_MODE :

    da16x_sys_watchdog_suspend(sys_wdog_id);

	sysmode = getSysMode();
	if (SYSMODE_AP_ONLY == sysmode) {
		TaskHandle_t TCPServerThreadPtr = NULL;

		// TCP provisioning
		/* Create provision TCP thread on Soft-AP mode */
		int status = xTaskCreate(tcp_server_thread,
		APP_SOFTAP_PROV_NAME,
		APP_SOFTAP_PROV_STACK_SZ, (void*)NULL,
		OS_TASK_PRIORITY_USER + 3, &TCPServerThreadPtr);
		if (status != pdPASS) {
			APRINTF("[%s] Failed to create TCP svr thread\r\n", __func__);
		}
	} else {
		vTaskDelay(500);

		if (chk_network_ready(WLAN0_IFACE) != 1) {
			goto SOFTAP_MODE;
		}
	}

    da16x_sys_watchdog_notify_and_resume(sys_wdog_id);

    da16x_sys_watchdog_unregister(sys_wdog_id);

	vTaskDelete(NULL);
}





void neuralert_app(void *param)
{
	DA16X_UNUSED_ARG(param);

	uint32_t ulNotifiedValue;
	int storedRunFlag;
	unsigned char fiforeg[2];
	int sys_wdog_id = -1;

	// MQTT callback setup
    //mqtt_client_set_msg_cb(user_mqtt_msg_cb);
    mqtt_client_set_pub_cb(user_mqtt_pub_cb);
    mqtt_client_set_conn_cb(user_mqtt_conn_cb);
    //mqtt_client_set_subscribe_cb(my_app_mqtt_sub_cb);

	/* Get our task handle */
	xTask = xTaskGetCurrentTaskHandle();

	PRINTF("\n\n===========>Starting neuralert_app\r\n");
	PRINTF(" Software part number  :    %s\n", USER_SOFTWARE_PART_NUMBER_STRING);
	PRINTF(" Software version      :    %s\n", USER_VERSION_STRING);
	PRINTF(" Software build time   : %s %s\n", __DATE__ , __TIME__ );
	PRINTF(" User data size        : %u bytes\n", sizeof(UserDataBuffer));

	/*
	 * Start the LED timer
	 * Starting it here allows the blink to work as a user command
	 */
	start_LED_timer();

	/*
	 * Check the user run flag.  This is a means to have the app start
	 * but remain inactive while the user interacts with the console
	 * to do provisioning and possible debugging
	 * When ready to go, runflag must be set to 1
	 *
	 * NOTE - user_init() hasn't been called yet, so RETMEM and
	 * other things aren't set at this point.  So the system-state
	 * mechanism is not up and running.
	 *
	 */

	user_get_int(DA16X_CONF_INT_RUN_FLAG, &storedRunFlag);
	PRINTF("\n Neuralert: [%s] NVRam runFlag: [%i]\r",__func__, storedRunFlag);

	if (storedRunFlag < 0) {
		// This *should* be impossible, due to the additional logic for retrieving the config
		PRINTF("\n Neuralert: [%s] Run flag set to impossible value");
		user_reboot();
	} else {
		// We have made sure the value isn't negative, so copy it over to the global flag
		runFlag = storedRunFlag;
	}

	// Whether WIFI is connected or not, see if we can obtain our
	// MAC address
	// Check our MAC string used as a unique device identifier
	int MACaddrtype = 0;
	memset(macstr, 0, 18);
	memset(MACaddr, 0,7);
	while (MACaddrtype == 0) { // we need the MACaddr -- without it, data packets will be wrong.
		MACaddrtype = getMACAddrStr(1, macstr);  // Hex digits string together
		vTaskDelay(10);
	}
	MACaddr[0] = macstr[9];
	MACaddr[1] = macstr[10];
	MACaddr[2] = macstr[12];
	MACaddr[3] = macstr[13];
	MACaddr[4] = macstr[15];
	MACaddr[5] = macstr[16];
	PRINTF("\n MAC address - %s (type: %d)", macstr, MACaddrtype);
	//vTaskDelay(10); // delay between obtaining the MAC addr and strcpy (below).  Delay for reset achieves this.


	if(runFlag <= 1)
	{
		// State mechanism isn't up and running yet, so
		// signal LEDs manually
		setLEDState(RED, LED_SLOW, 200, 0, LED_OFFX, 0, 3600);

		// Set the SSID for provisioning (default is different than packaging in SDK 8.2) -- needs to reboot
		if (runFlag == 0) {
			if (!user_set_ssid()) {
				int ret = write_nvram_int("RUN_FLAG", 1); // next time we come up, the reboot won't occur
				if (ret != 0){
					PRINTF("[%s] NVRAM int write error: %d\r\n", __func__, ret);
				}
				vTaskDelay(100); // small delay for nvram write
			}
			user_reboot(); //Restart the system to reset the SSID for provisioning
		}

		// Allow time for console to settle
		vTaskDelay(pdMS_TO_TICKS(100));
		PRINTF("\n\n******** Waiting for run flag to be set to 1 ********\n\n");

		softap_provisioning(NULL);
	}


	while (1)
	{
		if(runFlag == 2)
		{
			break;
		}

		da16x_sys_watchdog_notify(sys_wdog_id);
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	/*
	 * Initialize resources and kick off initial event,
	 * depending on what has awakened us.  This should
	 * cause the event loop to immediately do something.
	 */
	user_init();

	/*
	 * Now that retention memory & the persistent user memory has been
	 * set up, see if we know our device ID yet.  (Happens during
	 * the initial boot event)
	 */
	if (take_semaphore(&User_semaphore)) {
		PRINTF("\n Neuralert: [%s] error taking user semaphore", __func__);
		// do nothing, just for internal display
	} else {
		if (strlen(pUserData->Device_ID) > 0)
		{
			PRINTF(" Unique device ID      : %s\n", pUserData->Device_ID);
		}
		else
		{
			PRINTF(" Unique device ID not acquired yet\n");
		}
		xSemaphoreGive(User_semaphore);
	}



	/*
	 * Check the battery voltage
	 * Note that in an MQTT transmission cycle, we won't check battery
	 * again until we have wakened from the first sleep after the cycle.
	 * So, maybe 30 seconds or so.
	 *
	 */
	uint16_t adcData = get_battery_voltage();
	PRINTF(" User data size        : %u bytes\n", sizeof(UserDataBuffer));
    PRINTF(" Battery reading       : %d\n",adcData);



	/*
	 * Event loop - this is the main engine of the application
	 */
	PRINTF("==Event loop:[");
	while (1)
	{
		/* Block and wait for a notification.
		 Bits in this RTOS task's notification value are set by the notifying
		 tasks and interrupts to indicate which events have occurred. */
		xTaskNotifyWaitIndexed(
								0,       			/* Wait for 0th notification. */
								0x00,               /* Don't clear any notification bits on entry. */
								ULONG_MAX,          /* Reset the notification value to 0 on exit. */
								&ulNotifiedValue,   /* Notified value pass out in ulNotifiedValue. */
								pdMS_TO_TICKS(50));   /* Timeout if no event to check on AXL - twice rate of AXL sampling rate. */


		//PRINTF("%s: NotifiedValue: 0x%X\n", __func__, ulNotifiedValue);
		PRINTF(".");   // Show [.......] for wait loop
		// See if we've been notified of an event or just timed out
		if (ulNotifiedValue != 0)
		{
			/* Process events */
			PRINTF("]\n");

			// If we've received a terminate downlink command, exit
			// message processing loop and effectively shut down.
			da16x_sys_watchdog_notify(sys_wdog_id);
			//quit = (user_process_event(ulNotifiedValue) == PROCESS_EVENT_TERMINATE); //JW: no more terminate event
			user_process_event(ulNotifiedValue);
		}
		else
		{
			// Timeout on event loop - check to see if the AXL is stalled
			fiforeg[0] = MC36XX_REG_STATUS_1;
			i2cRead(MC3672_ADDR, fiforeg, 1);
			// if the AXL threshold has been reached but we did not get the
			// notification, fire off our own notification
			if(fiforeg[0] & 0x40)
			{
				if (take_semaphore(&User_semaphore)) {
					PRINTF("\n Neuralert: [%s] error taking user semaphore");
					// do nothing, it just internal logging
				} else {
					pUserData->ACCEL_missed_interrupts++;
					xSemaphoreGive(User_semaphore);
				}

				PRINTF("]\n");
				PRINTF("%s: AXL FIFO at threshold! [%x]\n", __func__, fiforeg[0]);
				if (xTask) {
					xTaskNotifyIndexed(xTask, 0, USER_MISSED_RTCKEY_EVENT, eSetBits);
					isAccelerometerTimeout = pdTRUE;	// remember why we are reading accelerometer				}
				}
			}
		} // event wait timeout
	} // while (1)
}
