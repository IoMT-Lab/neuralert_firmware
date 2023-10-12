/**
 ****************************************************************************************
 *
 * @file tcp_client_sleep2_sample.c
 *
 * @brief TCP Client sample code on sleep2  mode
 *
 * Copyright (c) 2016-2022 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */
// following temporary for Fred

// DEBUG FRED FRSDEBUG
#define __MQTT_CLIENT_SAMPLE__
#define CFG_USE_RETMEM_WITHOUT_DPM
#if defined (__TCP_CLIENT_SLEEP2_SAMPLE__)

#include "sdk_type.h"
#include "common_def.h"
#include "common_config.h"
#include "da16x_network_common.h"
#include "da16x_system.h"
#include "da16x_time.h"
#include "da16200_map.h"
#include "iface_defs.h"
#include "lwip/sockets.h"
#include "sample_defs.h"
#include "task.h"
#include "lwip/sockets.h"
#include "user_dpm.h"
#include "user_dpm_api.h"
#ifdef CFG_USE_RETMEM_WITHOUT_DPM
#include "user_retmem.h"
#endif
#include "mqtt_client.h"
#include "user_nvram_cmd_table.h"
#include "util_api.h"
#include "limits.h"
//#include "spi_flash/spi_flash.h"
//#include "spi_flash.h"
#include "W25QXX.h"
#include "Mc363x.h"
#include "adc.h"
#include "common.h"
// FreeRTOSConfig included for info about tick timing
#include "app_common_util.h"

#include "user_version.h"
#include "buildtime.h"

#ifdef CFG_USE_SYSTEM_CONTROL
#include "system_start.h"
#endif

/*
 * MACROS AND DEFINITIONS
 *******************************************************************************
 */
#define SET_PROCESS_BIT(src, bit)				(src |= bit)
#define CLR_PROCESS_BIT(src, bit)				(src &= (~bit))
#define PROCESS_BIT_SET(src, bit)				((src & bit) == bit)

/* Events */
#define USER_TERMINATE_EVENT					(1 << 0)
#define USER_BOOTUP_EVENT						(1 << 1)
#define USER_WAKEUP_BY_RTCKEY_EVENT				(1 << 2)
#define USER_WAKEUP_BY_TIMER_EVENT				(1 << 3)
#define USER_RTCKEY_IN_TIMER_HANDLE_EVENT		(1 << 4)
#define USER_TIMER_IN_RTCKEY_HANDLE_EVENT		(1 << 5)
#define USER_CONNECT_COMPLETE_EVENT				(1 << 6)
#define USER_SLEEP_READY_EVENT					(1 << 7)
#define USER_ATTEMPT_TRANSMIT_EVENT				(1 << 8)

/* Process list */
#define USER_PROCESS_HANDLE_RTCKEY				(1 << 0)
#define USER_PROCESS_HANDLE_TIMER				(1 << 1)
#define USER_PROCESS_CONNECT_AP					(1 << 2)
#define USER_PROCESS_SEND_DATA					(1 << 3)
#define USER_PROCESS_MQTT_TRANSMIT				(1 << 4)

// System states for LED management
// The intent is that only one state will be active at one
// time
// Note that zero is the default "accelerometer wake/sleep" state
#define USER_STATE_POWER_ON_BOOTUP				(1 << 0)
#define USER_STATE_PROVISIONING					(1 << 1)
#define USER_STATE_WIFI_CONNECTING				(1 << 2)
#define USER_STATE_WIFI_CONNECTED				(1 << 3)
#define USER_STATE_WIFI_TRANSMITTING			(1 << 4)
#define USER_STATE_WIFI_CONNECT_FAILED			(1 << 5)
#define USER_STATE_BATTERY_EXHAUSTED			(1 << 6)
#define USER_STATE_INTERNAL_ERROR				(1 << 7)

// System alerts for LED management
// The intent is that more than one condition can occur
// at the same time and alerts are handled in order of
// increasing precedence.  So a higher-precedence alert will
// be displayed rather than a lower-precedence bit
//
// Note that, in general, alerts will take
// precedence over user state.
// So, for instance, if the battery is low,
// the LED will announce that rather than
// a WIFI connect state
//#define USER_ALERT_UNABLE_TO_CONNECT			(1 << 0)
#define USER_ALERT_BATTERY_LOW 					(1 << 1)
#define USER_ALERT_AB_STORAGE_LOW				(1 << 2)
#define USER_ALERT_SPARE_2 						(1 << 3)
#define USER_ALERT_FATAL_ERROR					(1 << 4)


/* USER RTC Timer */
//#define USER_DATA_TX_TIMER_SEC					(5 * 60)	// 5 Mins
//#define USER_DATA_TX_TIMER_SEC					(3 * 60)	// 3 Mins
#define USER_DATA_TX_TIMER_SEC					(90)	// 1.5 Mins
#define USER_RTC_TIMER_NAME						"uTimer"
#define USER_RTC_TIMER_ID						(5)
#define USER_RTC_TIMER_ID_MIN					(5)

/*
 * Accelerometer configuration
 */
// The accelerometer threshold sets the point at which the accelerometer
// will trigger an interrupt
//#define ACCEL_FIFO_threshold 30


/* TCP Server Information */
#if 0
#define TCP_CLIENT_DEF_PORT						10192
#define TCP_CLIENT_DEF_BUF_SIZE					(1024 * 1)
#define TCP_CLIENT_DEF_SERVER_IP_ADDR			"192.168.0.2"
#define TCP_CLIENT_DEF_SERVER_PORT				TCP_CLI_TEST_PORT
#endif
#define TCP_CLIENT_SLP2_PERIOD					((USER_DATA_TX_TIMER_SEC + 60) * 1000000)  // USER_DATA_TX_TIMER_SEC + 1 min
#if 0
#define TCP_CLIENT_TX_BUF_SIZE 					1024
#define USER_IP_ADDR_LEN						(32)
#endif

/* NVRAM Items */
#define SERVER_IP_NAME							"SERVER_IP"
#define SERVER_PORT_NAME						"SERVER_PORT"


/* Retention Memory */
#define USER_RTM_DATA_TAG						"uRtmData"
//#define USER_RTM_DATA_LEN						sizeof(accelBufferStruct)
// Retention memory is 48KB, so if the accel buffer structure is about 120 bytes,
// we can hold 256 buffers in about 30k.  So this is about 7,680 samples @14Hz,
// so we have about 548 seconds = 9.1 minutes of data
// ** NOTE setting this to 256 resulted in a hardware fault, presumably
//         because we exceeded available memory. 
//#define USER_RTM_DATA_MAX_CNT					(256) // buffer size that can be stored for 2 mins.

#define USER_RTM_DATA_MAX_CNT					(54) // buffer size that can be stored for 2 mins.
//#define USER_RTM_DATA_SIZE						(USER_RTM_DATA_LEN * USER_RTM_DATA_MAX_CNT)
#define USER_CONNECT_STATUS_REPLY_SIZE			(512)


/**
 *******************************************************************************
 * @brief MQTT broker configuration
 *******************************************************************************
 */
#if 0
static const char *cert_buffer0 =
    "-----BEGIN CERTIFICATE-----\n"
    "MIID+TCCAuGgAwIBAgIJANqqHCazDkkOMA0GCSqGSIb3DQEBCwUAMIGSMQswCQYD\n"
    "VQQGEwJVUzETMBEGA1UECAwKQ2FsaWZvcm5pYTEUMBIGA1UEBwwLU2FudGEgQ2xh\n"
    "cmExFzAVBgNVBAoMDldpLUZpIEFsbGlhbmNlMR0wGwYDVQQDDBRXRkEgUm9vdCBD\n"
    "ZXJ0aWZpY2F0ZTEgMB4GCSqGSIb3DQEJARYRc3VwcG9ydEB3aS1maS5vcmcwHhcN\n"
    "MTMwMzExMTkwMjI2WhcNMjMwMzA5MTkwMjI2WjCBkjELMAkGA1UEBhMCVVMxEzAR\n"
    "BgNVBAgMCkNhbGlmb3JuaWExFDASBgNVBAcMC1NhbnRhIENsYXJhMRcwFQYDVQQK\n"
    "DA5XaS1GaSBBbGxpYW5jZTEdMBsGA1UEAwwUV0ZBIFJvb3QgQ2VydGlmaWNhdGUx\n"
    "IDAeBgkqhkiG9w0BCQEWEXN1cHBvcnRAd2ktZmkub3JnMIIBIjANBgkqhkiG9w0B\n"
    "AQEFAAOCAQ8AMIIBCgKCAQEA6TOCu20m+9zLZITYAhGmtxwyJQ/1xytXSQJYX8LN\n"
    "YUS/N3HG2QAQ4GKDh7DPDI13zhdc0yOUE1CIOXa1ETKbHIU9xABrL7KfX2HCQ1nC\n"
    "PqRPiW9/wgQch8Aw7g/0rXmg1zewPJ36zKnq5/5Q1uyd8YfaXBzhxm1IYlwTKMlC\n"
    "ixDFcAeVqHb74mAcdel1lxdagHvaL56fpUExm7GyMGXYd+Q2vYa/o1UwCMGfMOj6\n"
    "FLHwKpy62KCoK3016HlWUlbpg8YGpLDt2BB4LzxmPfyH2x+Xj75mAcllOxx7GK0r\n"
    "cGPpINRsr4vgoltm4Bh1eIW57h+gXoFfHCJLMG66uhU/2QIDAQABo1AwTjAdBgNV\n"
    "HQ4EFgQUCwPCPlSiKL0+Sd5y8V+Oqw6XZ4IwHwYDVR0jBBgwFoAUCwPCPlSiKL0+\n"
    "Sd5y8V+Oqw6XZ4IwDAYDVR0TBAUwAwEB/zANBgkqhkiG9w0BAQsFAAOCAQEAsNxO\n"
    "z9DXb7TkNFKtPOY/7lZig4Ztdu6Lgf6qEUOvJGW/Bw1WxlPMjpPk9oI+JdR8ZZ4B\n"
    "9QhE+LZhg6SJbjK+VJqUcTvnXWdg0e8CgeUw718GNZithIElWYK3Kh1cSo3sJt0P\n"
    "z9CiJfjwtBDwsdAqC9zV9tgp09QkEkav84X20VxaITa3H1QuK/LWSn/ORrzcX0Il\n"
    "10YoF6Hz3ZWa65mUoMzd8DYtCyGtcbYrSt+NMCqRB186PDQn5XBCytgF8VuiCyyk\n"
    "Z04hqHLzAFc21P9yhwKGi3BHD/Sep8fvr9y4VpMIqHQm2jaFPxY1VxhPSV+UHoE1\n"
    "fCPitIJTp/iXi7uXTQ==\n"
    "-----END CERTIFICATE-----\n";
static const char *cert_buffer1 =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIEBTCCAu2gAwIBAgICEEYwDQYJKoZIhvcNAQELBQAwgZIxCzAJBgNVBAYTAlVT\n"
    "MRMwEQYDVQQIDApDYWxpZm9ybmlhMRQwEgYDVQQHDAtTYW50YSBDbGFyYTEXMBUG\n"
    "A1UECgwOV2ktRmkgQWxsaWFuY2UxHTAbBgNVBAMMFFdGQSBSb290IENlcnRpZmlj\n"
    "YXRlMSAwHgYJKoZIhvcNAQkBFhFzdXBwb3J0QHdpLWZpLm9yZzAeFw0xMzA1MTAy\n"
    "MzQ0NTFaFw0yMzA1MDgyMzQ0NTFaMH4xCzAJBgNVBAYTAlVTMRMwEQYDVQQIDApD\n"
    "YWxpZm9ybmlhMRcwFQYDVQQKDA5XaS1GaSBBbGxpYW5jZTEfMB0GA1UEAwwWQ2xp\n"
    "ZW50IENlcnRpZmljYXRlIElETDEgMB4GCSqGSIb3DQEJARYRc3VwcG9ydEB3aS1m\n"
    "aS5vcmcwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDco7kQ4W2b/fBw\n"
    "2ZgSAXVWBCmJW0yax8K682kRiHlB1aJ5Im8rTEktZlPDQVhoO3Ur+Ij9Y8ukD3Hq\n"
    "CMa95T1a3Ly9lhDIME/VRqRgZRGa/FC/jkiK9u9SgIXPZqJk1s/JVG7a7deC8BvK\n"
    "iqIFhXoHl0N4nJxwVA8kag48dXbrTxrOH26C9qwU85iS/c1ozHJMmq052WPSQZII\n"
    "Sq8+VmxlCbbXxQ7kU2oZkxDqW3hMI3OPS80s8q4tMzuitvzFO0JvAHgh4IFyE7yg\n"
    "nIE7+lM9f3EwzECw9nEBdL7AvfnYLhlIEI8S8wZTUpnd8XA5Qs7Qa4rLNuqI273Z\n"
    "IWJLh9v/AgMBAAGjeDB2MA8GA1UdEwEB/wQFMAMCAQAwCwYDVR0PBAQDAgXgMBYG\n"
    "A1UdJQEB/wQMMAoGCCsGAQUFBwMCMB0GA1UdDgQWBBQtC2mx8POhZRfB+sj4wX1Z\n"
    "OzdrCzAfBgNVHSMEGDAWgBQLA8I+VKIovT5J3nLxX46rDpdngjANBgkqhkiG9w0B\n"
    "AQsFAAOCAQEAsvHJ+J2YTCsEA69vjSmsGoJ2iEXDuHI+57jIo+8qRVK/m1is1eiU\n"
    "AefExDtxxKTEPPtourlYO8A4i7oep9T43Be8nwVZdmxzfu14cdLKQrE+viCuHQTc\n"
    "BLSoAv6/SZa3MeIRkkDSPtCPTJ/Pp+VYPK8gPlc9pwEs/KLgFxK/Sq6RDNjTPs6J\n"
    "MChxi1iSdUES8mz3JDhQ2RQWVuibPorhgVqNTyXBjFUbYxTVl3hBCtK/Bd4IyFTX\n"
    "Li90XXHseNbj2sHu3PFBU7PG5mhKQMUOYqvQzEDIXT6SDA+PrepqrwXn/BL861K6\n"
    "GV4LcfKBg0HHdW9gJByZCN02igFTw56Kzg==\n"
    "-----END CERTIFICATE-----\n";
static const char *cert_buffer2 =
    "-----BEGIN RSA PRIVATE KEY-----\n"
    "MIIEpQIBAAKCAQEA3KO5EOFtm/3wcNmYEgF1VgQpiVtMmsfCuvNpEYh5QdWieSJv\n"
    "K0xJLWZTw0FYaDt1K/iI/WPLpA9x6gjGveU9Wty8vZYQyDBP1UakYGURmvxQv45I\n"
    "ivbvUoCFz2aiZNbPyVRu2u3XgvAbyoqiBYV6B5dDeJyccFQPJGoOPHV2608azh9u\n"
    "gvasFPOYkv3NaMxyTJqtOdlj0kGSCEqvPlZsZQm218UO5FNqGZMQ6lt4TCNzj0vN\n"
    "LPKuLTM7orb8xTtCbwB4IeCBchO8oJyBO/pTPX9xMMxAsPZxAXS+wL352C4ZSBCP\n"
    "EvMGU1KZ3fFwOULO0GuKyzbqiNu92SFiS4fb/wIDAQABAoIBAQDcnbCc2mt5AM98\n"
    "Z3aQ+nhSy9Kkj2/njDqAKIc0ituEIpNUwEOcbaj2Bk1W/W3iuyEMGHURuMmUgAUN\n"
    "WD0w/5j705+9ieG56eTJgts1r5mM+SHch+6tVQAz5GLn4N4cKlaWHyDBM/S77k47\n"
    "lacwEijUkkFaxm3+O27woEMf3OxNl24KmRenMYBhqcsoT4BYBw3Bh8xe+XN95rXj\n"
    "2BdIbr5+RWGc9Zsz4o5Wmd4mL/JvbKeohrsecien4TZRzWFku93XV5kie1c1aJy1\n"
    "nJ85bGJk4focmP/2ToxQysTbPYCxHVTIHuADK/qf9SGHJ9F7EBHE7+0isuwBbqOD\n"
    "OzS8rHdRAoGBAPCXlaHumEkLIRv3enhpHPBYxnDndNCtT1T6+Cuit/vfo6K6oA7p\n"
    "iUaej/GPZsDKXhayeTiEaq7QMinUtGkiCgGlVtXghXuCZz6KrH19W6wzC6Pbokmq\n"
    "BZak4LQcvGavt3VzjliAKLcdn6nQt/+bp/jKDJOKVbvb30sjS035Ah4zAoGBAOrF\n"
    "BgE9UTEnfQHIh7pyiM1DAomBbdrlRos8maQl26cHqUHN3+wy1bGHLzOjYFFoAasx\n"
    "eizw7Gudgbae28WIP1yLGrpt15cqVAvlCYmBtZ3C98FuT3FYgEEZpWNmE8Om+5UM\n"
    "td+mtMjonWAPkCYC+alqUZzeIs+CZs5CHKYCDqcFAoGBAOfkQv38GV2102jARJPQ\n"
    "RGtINaRXApmrod43s4Fjac/kAzVyiZk18PFXHUhnvlMt+jgIN5yIzMbHtsHo2SbH\n"
    "/zsM4MBuklm0G80FHjIp5HT6EksSA77amF5VdptDYzfaP4p+IYIdrKCqddzYZrCA\n"
    "mArMvAhs+iuCRhuG3is+SZNPAoGAHs6r8w2w0dp0tP8zkGvnN8hLVO//EnJzx2G0\n"
    "Z63wHQMMWu5BLCWflSRANW6C/SvAzE450hvralPI6cX+4PT4G5TFdSFk4RlU3hq4\n"
    "Has/wewLxv5Kvnz2l5Rd96U1gr8u1GhOlYKyxop/3FMuf050pJ6nBwa/WquqAfb6\n"
    "+23ZrmECgYEA6l0GFHwMFBNnpPuxHgYgS5+4g3+8DhZZIDc7IflBCBWF/ZwbM+nH\n"
    "+JSxiYYjvD7zIBhndqERcZ+fvbZTQ8oymr3j5AESM0ZfAHbft6IFQWjDUC3IDUF/\n"
    "4F0cUidFC8smu6Wa2tjvSIz7DfvmDsn1l+7s9qQvDxdyPas0IkL/v8w=\n"
    "-----END RSA PRIVATE KEY-----\n";
#endif

#define MQTT_SAMPLE_BROKER_IP	"public.mqtthq.com"
#define MQTT_SAMPLE_BROKER_PORT	1883
#define MQTT_SAMPLE_QOS			0
#define MQTT_SAMPLE_TLS			0
#define MQTT_SAMPLE_TOPIC_SUB	"Neuralert-fred2"
#define MQTT_SAMPLE_TOPIC_PUB	"Neuralert-fred1"

#if 0
// Dialog example from MQTT client example in SDK
#define MQTT_SAMPLE_BROKER_IP	"172.168.15.1"
#define MQTT_SAMPLE_BROKER_PORT	8883
#define MQTT_SAMPLE_QOS			2
#define MQTT_SAMPLE_TLS			1
#define MQTT_SAMPLE_TOPIC_SUB	"da16k"
#define MQTT_SAMPLE_TOPIC_PUB	"_da16k"
#endif

/*
 * Accelerometer buffer (AB) setup
 */

// # of times the write function will attempt the write/verify cycle
// This includes the first try and subsequent retries
#define AB_WRITE_MAX_ATTEMPTS 4
// # of times the erase sector function will attempt the erase/verify cycle
// ***NOTE*** there are two costs to multiple attempts.  The first is time.
// As of 9/12/22 it was taking 40-60 msec per all to the erase function.
// We need to stay inside the ~2 second AXL interrupt cycle so that we're
// ready when the next interrupt occurs
// The second cost is power.  The erase function uses a lot of power and
// so many erases will use more battery.
// When doing stress testing 9/11/22 to 9/13/22 we observed it taking as
// many as 5 attempts
// For instance, after running about 25 hours, we had these stats:
//  Total sector erase attempts             : 2712
//  Total times an erase retry was needed   : 2697
//  Total times succeeded on try 1          : 8
//  Total times succeeded on try 2          : 2659
//  Total times succeeded on try 3          : 30
//  Total times succeeded on try 4          : 7
//  Total times succeeded on try 5          : 8
// Although we need to figure out how to make this work on the first
// try, for the first release, we increase max attempts so that the
// software doesn't hang over 5 days for the customer
// On 9/13/22 the erase attempt was happening around 220 msec after wakeup
// and taking about 100 msec per erase/read cycle.  Soso in the worst
// case, 7 x 100 = 700 msec, which is still only about half the entire
// AXL FIFO interrupt time.
#define AB_ERASE_MAX_ATTEMPTS 3
/*
 * MQTT transmission setup
 * See spreadsheet for this calculation
 * One FIFO buffer
 */
/*
 * Defines used by the MQTT transmission
 * The maximum sent in one packet should be a multiple of the acceleromter
 * FIFO buffer trip point (currently 32) since data is stored in blocks
 * consisting of FIFO reads
 *
 */
#define SAMPLES_PER_FIFO					32

/*
 * Stage 2 test information
 * 14 samples/sec * 60 seconds/min * 5 minutes = 4200
 * 4200 samples/ 32 samples/FIFO = 131.25 FIFO buffers full
 * Rounding down = 131 FIFO buffers
 */
//#define FIFO_BUFFERS_PER_TRANSMIT_INTERVAL 131
//#define SAMPLES_PER_TRANSMIT_INTERVAL (SAMPLES_PER_FIFO * FIFO_BUFFERS_PER_TRANSMIT_INTERVAL)


// How long to wait for a WIFI connection each time the MQTT task starts up
// Note that 10 seconds was chosen arbitrarily early in development but
// it wasn't discovered until Release 1.8 that the caller was using milliseconds
// instead of seconds and so it was effectively an infinite wait.
// Observed time is about 5 seconds from the time the MQTT task starts
#define WIFI_CONNECT_TIMEOUT_SECONDS 10

// How many accelerometer FIFO blocks to send each time we wake up
// #define FIFO_BUFFERS_PER_TRANSMIT_INTERVAL 131

// See the discussion in the code about transmit timing and wanting to
// keep the total transmit task time to about 2 minutes.
// With WIFI & cloud startup time of 10 seconds plus post-transmission
// time waiting for response, let's gives ourselves 100 seconds
// of actual transmit time.
// As of 7/22/22 one JSON packet of 10 FIFO blocks was taking about 0.8 seconds
// to transmit, so 100 seconds gives us about 125 FIFO blocks max per transmit
// cycle.  But since there are 10 blocks per JSON packet, lets round down
// for now
// !!NOTE!! if this number is less than AXL trigger value for MQTT transmit,
//  the accelerometer will eventually overrun the transmit interval
// 120 takes about 30 seconds with 10 blocks per JSON packet transmit
// 9/13/22 F. Strathmann set at twice the MQTT transmit trigger so
// we can miss every other MQTT transmit interval and still be caught up
// As of 9/13/22, we do miss an occasional MQTT connect for no reason I
// can discern.  So this should keep us caught up if we have a WIFI connection
// In fall 2022, Sarah at Bricksimple reported that it was "slow" to catch up
// after missing WIFI for a while.
// 1/10/23 analysis and discussion with Marc Ryba, decided to expand to
// around 3 minute transmit interval in order to catch up faster.
// Analysis shows that by transmitting about 40 minutes worth of data
// per interval we can catch up 2 hours worth of buffered data in 15 minutes
// and catch up completely on the next interval.  So 20 minutes intead of
// more than an hour.
// So 40 minutes = 8 intervals = 144 * 8 = 1152 FIFO buffers per interval
//#define MAX_FIFO_BUFFERS_PER_TRANSMIT_INTERVAL 288
#define MAX_FIFO_BUFFERS_PER_TRANSMIT_INTERVAL 1152

// Trigger value for the accelerometer to start MQTT transmission
//  64 ~= 2 minutes
// 144 ~= 5 minutes
// 176 ~= 6 minutes
// 208 ~= 7 minutes
// 272 ~= 9 minutes
// ***NOTE*** because the MQTT task WIFI activity appears to interfere
// with SPI bus functions and especially the erase that happens every
// 16 pages of FLASH writing, it is believed that having this be a
// multiple of 16 will reduce the interference because the AXL task
// erases the next flash sector before it starts the MQTT task
// That means that the MQTT task will have 16 AXL interrupt times to
// operate in before the next erase sector time
//#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS 64
// 144 = 9 x 16 and just about 5 minutes at 29 samples / FIFO
#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS 144
//#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS 176
//#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS 208
//#define MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS 272

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
#define FIFO_BLOCKS_PER_PACKET 24

// How many actual samples to be sent in each JSON packet
// # samples in the Accel FIFO times blocks per JSON packet
// The most individual samples we'll transmit in one JSON packet
#define MAX_SAMPLES_PER_PACKET	(FIFO_BLOCKS_PER_PACKET * SAMPLES_PER_FIFO)
// How long to wait for the PUBACK from the broker before giving up
// when sending MQTT message with QOS 1 or 2
// (Note that in SDK call, timeout is in 10s of milliseconds)
#define MQTT_QOS_TIMEOUT_MS 5000

// How long to sleep after each JSON packet is sent to allow MQTT and WIFI
// activity to settle before attempting to read SPI bus again
// Note that at Release 1.7 (Dec 2022) we lowered this to 1/2 second and
// it caused problems with reliable transmission to AWS, as reported by
// Bricksimple.  So Release 1.8 restored it to 2 seconds.
//#define MQTT_INTER_PACKET_DELAY_MS 500
#define MQTT_INTER_PACKET_DELAY_MS 2000
// How long to sleep after completing MQTT transmission to allow cloud to respond
// and/or let transmission to complete
#define MQTT_POST_TRANSMISSION_DELAY_MS 2000
//#define MQTT_POST_TRANSMISSION_DELAY_MS 10000

// How long to sleep after powering down the RF section before allowing sleep
#define MQTT_POST_RF_POWER_OFF_DELAY_MS 1000



// Global area for building system error messages that
// will be logged
static UCHAR user_log_string_temp[USERLOG_STRING_MAX_LEN];
// How often to record statistics about operation.
// In accelerometer interrupt counts.  Interrupts happen
// about every 2.1 seconds.  Note that there is a limit to
// how many log entries are moved to flash every wake/sleep
// cycle.
// 28 == about every 1 minute (testing only)
// 140 = about every 5 minutes
#define AXL_LOG_STATS_TRIGGER_COUNT 144
// # of times the write function will attempt the write/verify cycle
 // This includes the first try and subsequent retries
 #define USERLOG_WRITE_MAX_ATTEMPTS 4
 // # of times the erase sector function will attempt the erase/verify cycle
 // On 9/13/22 the erase attempt was happening around 220 msec after wakeup
 // and taking about 100 msec per erase/read cycle.  Soso in the worst
 // case, 7 x 100 = 700 msec, which is still only about half the entire
 // AXL FIFO interrupt time.
#define USERLOG_ERASE_MAX_ATTEMPTS 3
 // Max entries to be held in retention memory
#define USERLOG_MAX_HOLD_ENTRIES 25
// Max number of log entries to write to flash each time we try to do this
// Note - this needs to take the write cycle time into account and the dynamics
// of the system and it's use of the SPI bus for Flash.
// We want to keep up with the frequency of logging so that we don't
// run out of space in retention memory, but we also don't want to
// tie up the system for too long so that we minimize our effect on the
// accelerometer task gathering data
// The million dollar question is where to insert the archiving
// function in the system.  It could possibly be a separate task, but
// until we sort out SPI bus interference, it seems safer to do it
// during an accelerometer interval when no erasing is being done
// and when no MQTT task is active.  So just tack it onto the end
// of the accelerometer wake/sleep cycle when we could otherwise go
// back to sleep.  As of this writing - 10/14/22 - the wake time is
// about 250-300 msec out of an approximately 2 second cycle.
// So if it's not an erase cycle or an MQTT cycle, we have at least
// one second to use for archiving things.  We also have to erase
// ahead, similar to the accelerometer task, so we have to allow
// time for that.  If we're going to be trying every cycle, then
// we know that we'll keep up with log production, so we don't need
// to archive too much per cycle.
#define MAX_LOG_ENTRIES_PER_ARCHIVE_INTERVAL 5


//#define SPI_FLASH_TXDATA_SIZE 256
//#define SPI_FLASH_RXDATA_SIZE 256
//#define SPI_FLASH_INPUT_DATA_SIZE 64
//#define SPI_FLASH_TEST_COUNT 3
//#define SPI_SFLASH_ADDRESS 0x1000

#define	TEST_DBG_DUMP(tag, buf, siz)	thread_peripheral_dump( # buf , buf , siz)
#if 0
static void thread_peripheral_dump(const char *title, unsigned char *buf,
		size_t len) {
	size_t i;

	PRINTF("%s (%d, %p)", title, len, buf);
	for (i = 0; i < len; i++) {
		if ((i % 32) == 0) {
			PRINTF("\n\t");
		} else if ((i % 4) == 2) {
			PRINTF("_");
		} else if ((i % 4) == 0) {
			PRINTF(" ");
		}
		PRINTF("%c%c", "0123456789ABCDEF"[buf[i] / 16],
				"0123456789ABCDEF"[buf[i] % 16]);
	}
	PRINTF("\n");
}
#endif

//void spi_flash_config_pin() {
//	_da16x_io_pinmux(PIN_DMUX, DMUX_SPIm);
//	_da16x_io_pinmux(PIN_EMUX, EMUX_SPIm);
//}


#ifdef CFG_USE_RETMEM_WITHOUT_DPM

// This sets how many accelerometer interrupt cycles we will collect
// timing for to establish the accelerometer chip sample frequency.
// As of 9/8/22, it appears that the nominal frequency stated in the
// chip specs can vary enough to throw off our timestamp calculation.
// So during the first N wake-from-sleep cycles after power-on,
// we measure the time for each FIFO buffer full and use this to
// calculate the average period per sample.
// This is then used by the MQTT task to calculate the timestamp
// value for each sample transmitted.
// This number should be less than the MQTT transmit trigger to insure
// that we've established the calibration information prior to
// starting MQTT transmission
#define AXL_CALIBRATION_CYCLES 30
typedef struct
	{
		__time64_t timestamp;			// assigned time of interrupt
		int			num_samples;		// how many samples were in the FIFO
	} AXL_calibration_data;


/*
 * Key value used to signal when a downlink command is received
 * requesting device shutdown
 */
#define MAGIC_SHUTDOWN_KEY 0xFACEBABE

/*
 * User data area in retention memory
 *
 * Note entire area is set to zero at boot time when it is allocated
 *******************************************************************/
typedef struct _userData {


	// *****************************************************
	// System state information (used to manage LEDs)
	// *****************************************************
	UINT32 system_state_map;		// bitmap for different system states
	UINT32 system_alert_map;		// bitmap for different alerts

	UINT32 ServerShutdownRequested;	// Set to a magic value when a terminate downlink is received

#if 0
	// Accelerometer data stored in Retention memory
	// Array of FIFO buffers captured in retention memory
	// for Stage5a-5d
	int FIFO_stash_index;		// Next stash location
	accelBufferStruct FIFOstash[USER_RTM_DATA_MAX_CNT];
#endif

	// *****************************************************
	// Accelerometer sample rate calibration data
	// *****************************************************
	// Count of entries we've collected
	int num_AXL_cal_entries;

	// Array of calibration info from initial accelerometer operation
	AXL_calibration_data AXL_cal_entry[AXL_CALIBRATION_CYCLES];

	// The result of the calibration.  Measured period of the
	// accelerometer's sampling period in microseconds.
	// Note that nominal is 1,000,000/14 = 71,429 usec.
	ULONG AXL_cal_sample_period_usec;
	int AXL_calibration_complete;		// pdTRUE if we've completed cal


	// *****************************************************
	// Timekeeping for FIFO read cycles
	// *****************************************************
	ULONG FIFO_reads_this_power_cycle;
	// The following variable keeps track of the most recent timestamp
	// assigned to a FIFO buffer
	__time64_t last_FIFO_read_time_ms;
	// The following variable keeps track of the most recent timestamp
	// from an actual wake from sleep.  This should be the most accurate
	// since the MQTT task is not running when this happens
	__time64_t last_accelerometer_wakeup_time_msec;
	// The following keeps track of how many times we've read the AXL
	// since we had a wakeup from sleep.  The reason is that we intend
	// to use this as part of the basis for interpolated timekeeping for missed
	// interrupts
	ULONG FIFO_reads_since_last_wakeup;
	// The following keeps track of how many actual accelerometer samples
	// we've read since we had a wakeup from sleep.  This times the sample
	// period tells us the expected arrival time of the sample that
	// caused the interrupt that got us here.
	ULONG FIFO_samples_since_last_wakeup;

	// The following tracks the go-to-sleep time
	__time64_t last_sleep_msec;


	// *****************************************************
	// MQTT transmission info
	// *****************************************************
	unsigned int MQTT_message_number;   // serial number for msgs
	unsigned int MQTT_connect_attempts;	// # times we tried
	unsigned int MQTT_connect_fails;	// # times we've failed to connect
	unsigned int MQTT_transmit_fails;	// # times a packet send failed
	unsigned int MQTT_dropped_data_events;	// # times MQTT had to skip data
	                               // because it was catching up to AXL

	// Time synchronization information
	// A time snapshot is taken on the first successful MQTT connection
	// and provided in all subsequent transmissions in order to synchronize
	// the left and right wrist device data streams
	__time64_t MQTT_timesync_timestamptime_msec;// Time snapshot in milliseconds
	__time64_t MQTT_timesync_localtime_msec;	// Corresponding local time snapshot in milliseconds
	int16_t MQTT_timesync_captured;		// 0 if not captured; 1 otherwise
	char MQTT_timesync_current_time_str[USERLOG_STRING_MAX_LEN];	// local time in string

	// *****************************************************
	// Unique device identifier
	// *****************************************************
	char Device_ID[7];			// Unique device identifier used for publish & subscribe

	// *****************************************************
	// Accelerometer info
	// *****************************************************
	unsigned int ACCEL_read_count;			// how many FIFO reads total
	unsigned int ACCEL_transmit_trigger;	// count of FIFOs to start transmit
	unsigned int ACCEL_missed_interrupts;	// How many times we detected full FIFO by polling
	unsigned int ACCEL_log_stats_trigger;	// count of FIFOs to log stats
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
	// System log statistics (external flash)
	// *****************************************************
	unsigned int total_log_entries;		// Total log entries written since power on
	unsigned int log_write_fault_count;		// # of times we failed to verify flash write since power up
	unsigned int log_write_retry_count;		// # of times we needed to retry the write since power up
	unsigned int log_write_attempt_events[AB_WRITE_MAX_ATTEMPTS];	// histogram of how often we had to retry

	unsigned int log_erase_attempts;			// # of times we attempted to erase sector
	unsigned int log_erase_fault_count;		// # of times we failed to verify flash erase since power up
	unsigned int log_erase_retry_count;		// # of times we needed to retry the erase since power up
	unsigned int log_erase_attempt_events[USERLOG_WRITE_MAX_ATTEMPTS];	// histogram of how often we had to retry



	// *****************************************************
	// Accelerometer buffer management
	// *****************************************************
	int16_t AB_initialized_flag;			// 0 if not initialized; 1 otherwise
	int8_t MQTT_internal_error;			// A genuine programming error, not transmit
	AB_INDEX_TYPE next_AB_write_position;
	AB_INDEX_TYPE next_AB_transmit_position;

	// *****************************************************
	// User logging buffer management
	// *****************************************************
	int16_t user_holding_log_initialized;
	int16_t user_log_initialized_flag;	// 0 if not initialized; 1 otherwise
	AB_INDEX_TYPE next_log_holding_position;
	AB_INDEX_TYPE oldest_log_holding_position;
	AB_INDEX_TYPE next_log_entry_write_position;
	AB_INDEX_TYPE oldest_log_entry_position;


	// User logging holding area
	// circular buffer with pointers as above
	USERLOG_ENTRY	user_log_entry[USERLOG_MAX_HOLD_ENTRIES];


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
static TaskHandle_t MQTT_task_handle = NULL;  // task handle of the MQTT transmit task

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
// the accelerometer knows why it's reading and can assign an aappropriate timestamp
static UINT8 isPowerOnBoot = pdFALSE;			// pdTRUE when poewr-on boot (first time boot)
static UINT8 isAccelerometerWakeup = pdFALSE;	// pdTRUE when wakened from sleep by accelerometer interrupt
static UINT8 isAccelerometerInterrupt = pdFALSE;// pdTRUE when interrupt while awake
static UINT8 isAccelerometerTimeout = pdFALSE;	// pdTRUE when missed interrupt detected



/*
 * Semaphore to coordinate between the accelerometer main task and
 * the MQTT transmit task.  This is used to provide exclusive access
 * to the AB management area so we don't accidentally overlap each other
 */
SemaphoreHandle_t AB_semaphore = NULL;
/*
 * Semaphore to coordinate between the accelerometer main task and
 * the MQTT transmit task with respect to accessing the flash.
 * This is used to provide exclusive access
 * to the flash so we don't accidentally overlap each other
 */
SemaphoreHandle_t Flash_semaphore = NULL;
/*
 * Semaphore to coordinate between users of the log holding area
 * in retention memory.
 */
SemaphoreHandle_t user_log_semaphore = NULL;

/*
 * Temporary storage for accelerometer samples to be transmitted
 * Created at transmit time from the stored FIFO structures
 */
// static int num_xmit_samples = 0;
static accelDataStruct accelXmitData[MAX_SAMPLES_PER_PACKET];

/*
 * Temporary storage to hold all the FIFO blocks to be transmitted
 * during one MQTT transmit cycle.
 * This is the workaround that allows the AXL task to set up data
 * for MQTT so that MQTT doesn't have to read from the SPI Flash
 */
#if 0
static int num_trans_blocks = 0;
static accelBufferStruct transmission_table[MAX_FIFO_BUFFERS_PER_TRANSMIT_INTERVAL];
#endif
/*
 * Area in which to compose JSON packet
 * see spreadsheet for sizing
 */
#define MAX_JSON_STRING_SIZE 20000
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
#define CLEAR_SYSTEM_STATE_BIT(bit)		(pUserData->system_state_map &= (~bit))
#define SYSTEM_STATE_BIT_ACTIVE(bit)	((pUserData->system_state_map & bit) == bit)

// Macros for setting, clearing, and checking system alerts
#define SET_SYSTEM_ALERT_BIT(bit)		(pUserData->system_alert_map |= bit)
#define CLEAR_SYSTEM_ALERT_BIT(bit)		(pUserData->system_alert_map &= (~bit))
#define SYSTEM_ALERT_BIT_ACTIVE(bit)	((pUserData->system_alert_map & bit) == bit)
#define ANY_SYSTEM_ALERT_ACTIVE()	(0 != pUserdata->system_alert_map))



#ifdef __TIME64__
//	__time64_t last_sleep_msec;
//	__time64_t last_sleep_sec;
#else
	time_t now;
#endif /* __TIME64__ */

/*
 * Time measurement for our processes
 * (under construction)
 */
//static struct tm last_sleep_time;
//static struct tm last_transmit_time;
// *localtime (const time_t *_timer);




/*
 * STATIC FUNCTIONS DEFINITIONS (forward declarations)
 *******************************************************************************
 */
static void user_process_timer_event(void);
static int user_process_connect_ap(void);
//static int check_connection_status(void);
static UCHAR user_process_check_wifi_conn(void);
static int get_AB_store_location(void);
static int get_AB_transmit_location(void);
static int update_AB_transmit_location(int new_location);
static int update_AB_write_location(int new_location);
static int AB_read_block(HANDLE SPI, UINT32 blockaddress, accelBufferStruct *FIFOdata);
static int flash_write_block(HANDLE SPI, int blockaddress, UCHAR *pagedata, int num_bytes);
static int get_holding_log_next_write_location(void);
static int get_holding_log_oldest_location(void);
static int update_holding_log_oldest_location(int new_location);
static int user_write_log_to_flash(USERLOG_ENTRY *pLogData, int *did_an_erase);
static int get_log_oldest_location(void);
static int get_log_store_location(void);
static void log_current_time(UCHAR *PrefixString);
static void timesync_snapshot(void);



/*
 * EXTERN FUNCTIONS DEFINITIONS
 *******************************************************************************
 */
extern void wifi_cs_rf_cntrl(int flag);
extern int fc80211_set_app_keepalivetime(unsigned char tid, unsigned int sec,
										void (*callback_func)(unsigned int tid));
void da16x_time64_sec(__time64_t *p, __time64_t *cur_sec);
void da16x_time64_msec(__time64_t *p, __time64_t *cur_msec);
void user_time64_msec_since_poweron(__time64_t *cur_msec);

// SDK MQTT function to set up received messages
void mqtt_client_set_msg_cb(void (*user_cb)(const char *buf, int len, const char *topic));

// Fred written functions to check on MQTT thread
extern int check_mqtt_client_thread_status(void);
//extern int check_mqtt_client_state(void);

// Fred written function in util_api.c
// Returns elapsed time since hardware boot for adjusting
// our interrupt time when we wake from an AXL interrupt
//extern ULONG MS_since_boot_time(void);

// When an accelerometer interrupt occurs and we're still awake,
// the interrupt takes this snapshot of the RTC clock
// so we know when it happened.  Processing will happen later,
// after the event works its way through the event processor
// and whatever other delays occur

long long user_accelerometer_interrupt_time; // RTC clock when interrupt happened
__time64_t user_accelerometer_interrupt_msec;  // msec since boot when interrupt happened

// Timekeeping for the polled accelerometer FIFO full detection
// (this is the workaround for missed accelerometer interrupts when
//  the MQTT task is active or the network is otherwise doing something)
//
__time64_t  user_AXL_poll_detect_RTC_clock;  // msec since boot when poll saw FIFO full
__time64_t  user_lower_AXL_poll_detect_RTC_clock;  // msec since boot when poll was negative
__time64_t  user_MQTT_start_msec;  // msec since boot when task started
__time64_t  user_MQTT_end_msec;  // msec since boot when task ended
ULONG user_MQTT_task_time_msec;   // run time msec


// Macros for converting from RTC clock ticks (msec * 32768) to microseconds
// and milliseconds
#define CLK2US(clk)			((((unsigned long long )clk) * 15625ULL) >> 9ULL)
#define CLK2MS(clk)			((CLK2US(clk))/1000ULL)


// LED timer and control functions
extern void start_LED_timer();
extern void setLEDState(uint8_t number1, uint8_t state1, uint16_t count1, uint8_t number2,  uint8_t state2, uint16_t count2, uint16_t secondsTotal);

/**
 *******************************************************************************
 * @brief Function to manage what is displayed on the LEDs,
 * 			based on the current system state and alerts
 *******************************************************************************
 */
static void notify_user_LED()
{

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

}

/**
 *******************************************************************************
 * @brief Set a system state bit and change LED colors
 *******************************************************************************
 */
#if 0
static void set_system_state(UINT32 state_to_set)
{
	// Set state in retention memory so it persists through
	// sleep cycles
	SET_SYSTEM_STATE_BIT(state_to_set);

	// Change the LED color if necessary
	notify_user_LED();

}
#endif

/**
 *******************************************************************************
 * @brief Set a system state bit as the only state and change LED colors
 *******************************************************************************
 */

static void set_sole_system_state(UINT32 state_to_set)
{
	// Set state in retention memory so it persists through
	// sleep cycles
	pUserData->system_state_map = state_to_set;

	// Change the LED color if necessary
	notify_user_LED();

}

/**
 *******************************************************************************
 * @brief Set a system alert bit and change LED colors
 *******************************************************************************
 */
static void set_system_alert(UINT32 alert_to_set)
{
	// Set state in retention memory so it persists through
	// sleep cycles
	SET_SYSTEM_ALERT_BIT(alert_to_set);

	// Change the LED color if necessary
	notify_user_LED();

}

/**
 *******************************************************************************
 * @brief Clear a system alert bit and change LED colors
 *******************************************************************************
 */
static void clear_system_alert(UINT32 alert_to_clear)
{
	// Set state in retention memory so it persists through
	// sleep cycles
	CLEAR_SYSTEM_ALERT_BIT(alert_to_clear);

	// Change the LED color if necessary
	notify_user_LED();
}


/**
 *******************************************************************************
 * @brief Process similar to strncpy but makes sure there is a
 * terminating null
 * copies up to (max_len - 1) characters and adds a null terminator
 * *******************************************************************************
 */
void user_text_copy(UCHAR *to_string, UCHAR *from_string, int max_len)
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
 *******************************************************************************
 * @brief Process to transfer log messages from the holding area
 * in retention memory to flash memory
 *  Returns pdFALSE if unable to write
 *  Returns pdTRUE if able to write
 *******************************************************************************
 */
static int user_archive_log_messages(int flush_all)
{
	int return_value = pdFALSE;
	int exit_requested = pdFALSE;
	AB_INDEX_TYPE check_location;
	AB_INDEX_TYPE write_holding_location;
	AB_INDEX_TYPE newest_location;
	AB_INDEX_TYPE oldest_holding_location;
	AB_INDEX_TYPE write_flash_location;
	AB_INDEX_TYPE newest_flash_location;
	AB_INDEX_TYPE oldest_flash_location;
	int total_holding_logs_stored;
	int total_holding_logs_free;
	int total_flash_logs_stored;
	int total_flash_logs_free;
	int num_entries_to_archive;
	AB_INDEX_TYPE last_holding_log_loc;
	AB_INDEX_TYPE entry_pointer;
	int archiving_finished;
	int num_archived;
	USERLOG_ENTRY temp_log_entry;		// one entry being sent to flash
	UCHAR time_string[20];
	int erase_was_done;
	int transferring_all;		// pdTRUE if we're doing the entire holding area

	__time64_t now;

	if(pUserData->user_log_initialized_flag != AB_MANAGEMENT_INITIALIZED)
	{
		PRINTF_RED("\n USER LOG NOT INITIALIZED\n");
		return pdFALSE;
	}

	if(pUserData->user_holding_log_initialized != AB_MANAGEMENT_INITIALIZED)
	{
		PRINTF_RED("\n USER LOG HOLDING NOT INITIALIZED\n");
		return pdFALSE;
	}

	// Figure out what is stored in retention memory
	oldest_holding_location = get_holding_log_oldest_location();
	if (oldest_holding_location < 0)
	{
		Printf("\n Unable to get user log oldest holding location\n");
		return pdFALSE;
	}
//	PRINTF("  Oldest holding log entry          : %d\n", oldest_holding_location);

	// If there is no data at this point, we can simply exit
	if(oldest_holding_location == INVALID_AB_ADDRESS)
	{
		PRINTF("\n *** user_archive_log_messages: nothing to do!\n");
		// We return TRUE because there is no error
		return pdTRUE;
	}

	// Figure out what is stored in the holding log in retention memory
	write_holding_location = get_holding_log_next_write_location();
	if (write_holding_location < 0)
	{
		Printf("\n Unable to get user holding log next write location\n");
		return pdFALSE;
	}

	// If we've archived what was stored in holding, oldest will have
	// caught up with next store location
	if(oldest_holding_location == write_holding_location)
	{
		PRINTF("user_archive_log_messages: head and tail the same. nothing to do!\n");
		// We return TRUE because there is no error
		return pdTRUE;
	}


	newest_location = write_holding_location - 1;
	if (newest_location < 0)
	{
		newest_location += USERLOG_MAX_HOLD_ENTRIES;
	}
	PRINTF("  Most recent holding log entry     : %d\n", newest_location);

	total_holding_logs_stored = (newest_location - oldest_holding_location) + 1;
	if (total_holding_logs_stored < 0)
	{
		// This involves a wraparound
		total_holding_logs_stored += USERLOG_MAX_HOLD_ENTRIES;
	}
	PRINTF("  Total holding log entries stored  : %d\n", total_holding_logs_stored);

	total_holding_logs_free = (oldest_holding_location - write_holding_location);
	if (total_holding_logs_free < 0)
	{
		// This involves a wraparound
		total_holding_logs_free += USERLOG_MAX_HOLD_ENTRIES;
	}
	PRINTF("  Total holding log entries free    : %d\n", total_holding_logs_free);


	if(!flush_all)
	{
		num_entries_to_archive = MAX_LOG_ENTRIES_PER_ARCHIVE_INTERVAL;
		if (num_entries_to_archive >= total_holding_logs_stored)
		{
			num_entries_to_archive = total_holding_logs_stored;
			transferring_all = pdTRUE;
		}
		else
		{
			transferring_all = pdFALSE;
		}
	}
	else
	{
		// Flushing everything in one shot
		num_entries_to_archive = total_holding_logs_stored;
		transferring_all = pdTRUE;
	}
//	PRINTF("  Holding log entries to archive    : %d\n", num_entries_to_archive);

	// Figure out ending position of extent being transmitted this cycle
	last_holding_log_loc = (oldest_holding_location + num_entries_to_archive) - 1;
	if(last_holding_log_loc >= USERLOG_MAX_HOLD_ENTRIES)
	{
		// Wrap around
		last_holding_log_loc -= USERLOG_MAX_HOLD_ENTRIES;
	}

	PRINTF("  Archiving %d entries from %d to %d\n",
			num_entries_to_archive, oldest_holding_location, last_holding_log_loc);

// xxxxxxxxxxxxxxxxxxx debug info
	// For debugging purposes, figure out what is stored in
	// the log flash area
	oldest_flash_location = get_log_oldest_location();
	if (oldest_flash_location < 0)
	{
		Printf("\n Unable to get flash log oldest location\n");
//		return pdFALSE;
	}
	PRINTF("  Oldest flash log entry          : %d\n", oldest_flash_location);

	// Figure out what is stored in the holding log in retention memory
	write_flash_location = get_log_store_location();
	if (write_flash_location < 0)
	{
		Printf("\n Unable to get user holding log next write location\n");
//		return pdFALSE;
	}
	PRINTF("  Next flash write location         : %d\n", write_flash_location);

	newest_flash_location = write_flash_location - 1;
	if (newest_flash_location < 0)
	{
		newest_flash_location += USERLOG_FLASH_MAX_PAGES;
	}
	PRINTF("  Most recent flash log written     : %d\n", newest_flash_location);

	total_flash_logs_stored = (newest_flash_location - oldest_flash_location) + 1;
	if (total_flash_logs_stored < 0)
	{
		// This involves a wraparound
		total_flash_logs_stored += USERLOG_FLASH_MAX_PAGES;
	}
	PRINTF("  Total flash log entries stored    : %d\n", total_flash_logs_stored);

	total_flash_logs_free = (oldest_flash_location - write_flash_location);
	if (total_flash_logs_free < 0)
	{
		// This involves a wraparound
		total_flash_logs_free += USERLOG_FLASH_MAX_PAGES;
	}
	PRINTF("  Total flash log entries free      : %d\n", total_flash_logs_free);

// xxxxxxxxxxxxxxxxxxx debug info


	// OK - now we know what the plan is.
	entry_pointer = oldest_holding_location;
	num_archived = 0;
	archiving_finished = pdFALSE;
	while (!archiving_finished)
	{
		// Use a separate semaphore take for each archive entry
		// (1) Obtain the retention memory entry to archive
		if(user_log_semaphore != NULL )
		{
			/* See if we can obtain the semaphore.  If the semaphore is not
				available wait 10 ticks to see if it becomes free. */
			if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
			{
				// We were able to obtain the semaphore and can now access the
				//	shared resource.

				// First grab a local copy of the log entry
				memcpy((UCHAR *)&temp_log_entry, (UCHAR *)&pUserData->user_log_entry[entry_pointer],
						sizeof(USERLOG_ENTRY));

				time64_string (time_string, &temp_log_entry.user_log_timestamp);
				PRINTF(">>Log entry [%d]: Type: %d Time: %s [%s]\n",
						entry_pointer,
						temp_log_entry.user_log_type,
						time_string,
						temp_log_entry.user_log_text);
				if(temp_log_entry.user_log_signature != USERLOG_SIGNATURE)
				{
					PRINTF(" ** Signature incorrect: %0x (should be %0x)\n",
							temp_log_entry.user_log_signature,
							USERLOG_SIGNATURE);
				}

				/* We have finished accessing the shared resource.  Release the
					semaphore. */
				xSemaphoreGive( user_log_semaphore );
			}
			else
			{
				PRINTF("\n ***user_archive_log_messages: Unable to obtain Flash semaphore\n");
			}
		}
		else
		{
			PRINTF("\n ***user_archive_log_messages: semaphore not initialized!\n");
		}

		// Note that this write will also erase the next sector
		// when necessary
		if(user_write_log_to_flash(&temp_log_entry, &erase_was_done))
		{
			num_archived++;
			if(num_archived >= num_entries_to_archive)
			{
				archiving_finished = pdTRUE;
			}
			entry_pointer++;
			if(entry_pointer >= USERLOG_MAX_HOLD_ENTRIES)
			{
				// Wrap around
				entry_pointer -= USERLOG_MAX_HOLD_ENTRIES;
			}
			// And update the oldest holding log entry pointer now
			// that it's been written to flash
			if(!update_holding_log_oldest_location(entry_pointer))
			{
				PRINTF("\n ***user_archive_log_messages: Unable to update holding log oldest location\n");
				archiving_finished = pdTRUE;
			}

		}
		else
		{
			// Unable to write to flash for some reason
			archiving_finished = pdTRUE;
		}
	} // for each holding log entry in archiving extent

	return return_value;
}




/**
 *******************************************************************************
 * @brief Process to write one log entry to the retention memory
 * holding area
 *  Returns pdFALSE if unable to write
 *  Returns pdTRUE if able to write
 *******************************************************************************
 */
static void user_write_log_holding_entry(const UCHAR entry_type, const UCHAR *message_text)
{
	AB_INDEX_TYPE check_location;
	AB_INDEX_TYPE write_location;
	__time64_t now;


	if(pUserData->user_holding_log_initialized != AB_MANAGEMENT_INITIALIZED)
	{
		PRINTF("\n USER LOG HOLDING NOT INITIALIZED\n");
		return;
	}

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */

			write_location = pUserData->next_log_holding_position;

//			PRINTF("\n Log entry: %d Type: %d [%s]\n",
//					write_location, entry_type, message_text);
			/*
			 * Get current time since activation
			 */
			da16x_time64_msec(NULL, &now);

			pUserData->user_log_entry[write_location].user_log_timestamp = now;
			pUserData->user_log_entry[write_location].user_log_signature = USERLOG_SIGNATURE;
			pUserData->user_log_entry[write_location].user_log_type = entry_type;
			user_text_copy(pUserData->user_log_entry[write_location].user_log_text,
					message_text, USERLOG_STRING_MAX_LEN);

			// Now update pointers, checking to make sure we're
			// not full.
			// This method always leaves one empty entry available
			// for the next log entry
			// Check for holding area full
			check_location = write_location + 1;
			if(check_location >= USERLOG_MAX_HOLD_ENTRIES)
			{
				check_location = 0;
			}
			// If this is the first log entry, then we're not full
			if(pUserData->oldest_log_holding_position == INVALID_AB_ADDRESS)
			{
				pUserData->oldest_log_holding_position = write_location;
				PRINTF("*** Setting first log holding position %d\n", write_location);
			}
			else
			{
				if(check_location == pUserData->oldest_log_holding_position)
				{
					// We've filled things
					PRINTF("\n Log holding position full - %d %d discarding oldest\n",
							check_location, pUserData->oldest_log_holding_position);
					pUserData->oldest_log_holding_position += 1;
					if(pUserData->oldest_log_holding_position >= USERLOG_MAX_HOLD_ENTRIES)
					{
						pUserData->oldest_log_holding_position = 0;
					}

				}
			}
			pUserData->next_log_holding_position = check_location;

//			PRINTF(" Holding oldest: %d  Next log: %d\n",
//					pUserData->oldest_log_holding_position, check_location);

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( user_log_semaphore );
		}
		else
		{
			Printf("\n ***user_write_log_holding_entry: Unable to obtain Flash semaphore\n");
		}
	}
	else
	{
		Printf("\n ***user_write_log_holding_entry: semaphore not initialized!\n");
	}

	return;

}


/**
 *******************************************************************************
 * @brief Log an error to nonvolatile memory
 *******************************************************************************
 */
static void user_log_error(const UCHAR *error_message)
{

// under construction - for now just show on console
	PRINTF_RED("\n**** System error:\n[%s]\n\n", error_message);
	if(USERLOG_ENABLED)
	{
		user_write_log_holding_entry(USERLOG_TYPE_ERROR, error_message);
	}
}

/**
 *******************************************************************************
 * @brief Log a system event to nonvolatile memory
 *******************************************************************************
 */
static void user_log_event(const UCHAR *event_message)
{

// under construction - for now just show on console
	PRINTF("**** System event: [%s]\n", event_message);
	if(USERLOG_ENABLED)
	{
		user_write_log_holding_entry(USERLOG_TYPE_INFORMATION, event_message);
	}
}



/**
 *******************************************************************************
 * @brief Send termination event to the user task
 *******************************************************************************
 */
static void user_terminate_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_TERMINATE_EVENT, eSetBits);
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
 * @brief Send wake-up by timer event to the user task
 *******************************************************************************
 */
static void user_wakeup_by_timer_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_WAKEUP_BY_TIMER_EVENT, eSetBits);
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

		xTaskNotifyIndexedFromISR(xTask, 0, USER_RTCKEY_IN_TIMER_HANDLE_EVENT,
									eSetBits, &xHigherPriorityTaskWoken);

		Printf("========== RTCKEY event in timer handle =======\n");  //FRSDEBUG
	}
}

#if 0
/**
 *******************************************************************************
 * @brief Send RTC timer event during handling RTC_WAKE_UP key event
 *******************************************************************************
 */
static void user_rtc_timer_callback(unsigned int param)
{
	PRINTF("  >>>> user_rtc_timer_callback: rtc timer expired(tid:%d)\n", param);
	if (xTask) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xTaskNotifyIndexedFromISR(xTask, 0, USER_TIMER_IN_RTCKEY_HANDLE_EVENT,
									eSetBits, &xHigherPriorityTaskWoken);
	} else {
		timer_id = param;
	}
}
#endif
/**
 *******************************************************************************
 * @brief Send WLAN connection event
 *******************************************************************************
 */
static void user_connection_complete_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_CONNECT_COMPLETE_EVENT, eSetBits);
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
 * @brief Send attempt transmit event to the user task
 *******************************************************************************
 */
static void user_attempt_transmit_event(void)
{
	if (xTask) {
		xTaskNotifyIndexed(xTask, 0, USER_ATTEMPT_TRANSMIT_EVENT, eSetBits);
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

	user_connection_complete_event();
}

#if 0
/**
 *******************************************************************************
 * @brief Process after wake-up
 *******************************************************************************
 */
static int user_process_create_rtc_timer(unsigned char tid, unsigned int sec)
{
	int ret = 0;

	PRINTF("\n**Neuralert: %s [%d seconds]\n", __func__, sec); // FRSDEBUG

	if (tid < USER_RTC_TIMER_ID_MIN) {
		PRINTF("%s: Failed to create rtc timer\n", __func__);
		return -1;
	}

	ret = fc80211_set_app_keepalivetime(tid, sec, user_rtc_timer_callback);
	if (ret != tid)
		PRINTF("%s: failed to create a rtc timer(%d)\n", __func__, ret);

	return ret;
}
#endif
/**
 ****************************************************************************************
 * @brief MQTT Basic Configuration Function \n
 * @subsection Parameters
 * - Broker IP Address
 * - Broker Port Number
 * - Qos
 * - Subscriber Topic
 * - Publisher Topic
 * - DPM Use
 * - SNTP Use (for TLS valid time)
 * - TLS Use
 * - TLS Root CA
 * - TLS Client Certificate
 * - TLS Client Private Key
 ****************************************************************************************
 */
static void mqtt_user_config(void)
{

	PRINTF("\n**Neuralert: configuring MQTT client\n"); // FRSDEBUG

	PRINTF("     Broker IP: %s", MQTT_SAMPLE_BROKER_IP);
	PRINTF("     Sub topic: %s", MQTT_SAMPLE_TOPIC_SUB);
	PRINTF("     Pub topic: %s", MQTT_SAMPLE_TOPIC_PUB);

	/* MQTT Setting */
    da16x_set_nvcache_int(DA16X_CONF_INT_MQTT_AUTO, 1);
    da16x_set_nvcache_str(DA16X_CONF_STR_MQTT_BROKER_IP, MQTT_SAMPLE_BROKER_IP);
    da16x_set_nvcache_int(DA16X_CONF_INT_MQTT_PORT, MQTT_SAMPLE_BROKER_PORT);
    da16x_set_nvcache_int(DA16X_CONF_INT_MQTT_QOS, MQTT_SAMPLE_QOS);
    da16x_set_nvcache_int(DA16X_CONF_INT_MQTT_TLS, MQTT_SAMPLE_TLS);
    da16x_set_nvcache_str(DA16X_CONF_STR_MQTT_SUB_TOPIC, MQTT_SAMPLE_TOPIC_SUB);
    da16x_set_nvcache_str(DA16X_CONF_STR_MQTT_PUB_TOPIC, MQTT_SAMPLE_TOPIC_PUB);
    da16x_set_nvcache_int(DA16X_CONF_INT_MQTT_PING_PERIOD, 60);

    /* DPM after Rebooting */
 //   da16x_set_nvcache_int(DA16X_CONF_INT_DPM, 1);

    da16x_set_nvcache_int(DA16X_CONF_INT_SNTP_CLIENT, 1);
    da16x_nvcache2flash();

    /* Input Certificate & Private Key */
//    cert_flash_write(SFLASH_ROOT_CA_ADDR1, (char *)cert_buffer0, strlen(cert_buffer0));
//    cert_flash_write(SFLASH_CERTIFICATE_ADDR1, (char *)cert_buffer1, strlen(cert_buffer1));
//    cert_flash_write(SFLASH_PRIVATE_KEY_ADDR1, (char *)cert_buffer2, strlen(cert_buffer2));
}


/*
 * Get battery voltage
 *
 * returns voltage as a float
 *
 * Note ADC has to have been configured during boot time.
 * See function config_pin_mux in user_main.c
 */
static float get_battery_voltage()
{
	/*
	 * Battery voltage
	 */
	uint16_t adcData;
	float adcDataFloat;
	uint16_t write_data;
	uint32_t data;

	// Set Battery voltage enable
	write_data = GPIO_PIN10;
	GPIO_WRITE(gpioa, GPIO_PIN10, &write_data, sizeof(uint16_t));

	vTaskDelay(5);

	// Read battery voltage
	// Note that due to a voltage divider, this measurement
	// isn't the actual voltage.  The divider ratio is,
	// according to Nicholas Joseph, 54/25.
	DRV_ADC_READ(hadc, DA16200_ADC_CH_0, (UINT32 *)&data, 0);
	adcData = (data >> 4) & 0xFFF;
	adcDataFloat = (adcData * VREF)/4095.0;

//	    PRINTF("Current ADC Value = 0x%x 0x%04X %d\r\n",data&0xffff,adcData, (uint16_t)(adcDataFloat * 100));

	// turn off battery measurement circuit
	write_data = 0;
	GPIO_WRITE(gpioa, GPIO_PIN10, &write_data, sizeof(uint16_t));   /* disable battery input */

	return adcDataFloat;
}

/**
 *******************************************************************************
 * @brief wait for MQTT connection with timeout in ms
 * return: 0 ; no error
 *******************************************************************************
 */
static int user_process_MQTT_wait_for_connection(int max_ms)
{
	int return_status = -1;
	int delay_time = 0;
	int delay_interval = 1000;	// ms to delay between checks
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===user_process_MQTT_wait_for_connection start");
#endif
	while (1)
	{
		// delay until mqtt is connected
		if (!mqtt_client_check_conn())
		{
			PRINTF("\r\n [%s] No MQTT Session ...\r\n", __func__);
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===user_process_MQTT_wait_for_connection waiting");
#endif

			// wait delay_interval ms
			vTaskDelay(pdMS_TO_TICKS(delay_interval));
			delay_time += delay_interval;
			if(delay_time > max_ms)
				break;
		}
		else
		{
			return_status = 0;
			break;
		}
	}

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===user_process_MQTT_wait_for_connection done");
#endif

	return return_status;

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
 *******************************************************************************
 */

int send_json_packet (int startAdd, int count, int msg_number, int sequence)
{
	int return_status = 0;
	int transmit_status = 0;
	int packet_len;
	static int mqttCount = 0;
	int status=0, statusCheck, i;
	unsigned char str[50],str2[20];		// temp working strings for assembling
	unsigned char rawdata[8];
	int16_t Xvalue;
	int16_t Yvalue;
	int16_t Zvalue;
//	int type;
	long timezone;
#ifdef __TIME64__
	__time64_t now;
	__time64_t nowSec;
#else
	time_t now;
#endif /* __TIME64__ */
	struct tm *ts;
	char buf[80], nowStr[20];
	uint32_t data;
	/*
	 * Battery voltage
	 */
	uint16_t adcData;
	float adcDataFloat;

	uint16_t write_data;


	/* WLAN0 */
#ifndef SILENT
	PRINTF("WLAN0 - %s\n", macstr);
#endif

#ifdef __TIME64__
	da16x_time64_msec(NULL, &now);
	da16x_time64_sec(NULL, &nowSec);
	uint64_t num1 = ((now/1000000) * 1000000);
	uint64_t num2 = now - num1;
	uint32_t num3 = num2;
//	PRINTF("now %lld - %lld %lld %ld\r\n",now/1000000,num1, num2, num3);
	sprintf(nowStr,"%ld",now/1000000);
//	PRINTF("%s\r\n",nowStr);
	sprintf(str2,"%06ld",num3);
//	PRINTF("%s\r\n",str2);
	strcat(nowStr,str2);
//	PRINTF("nows; %s\r\n",nowStr);
//	PRINTF("now %lld %lu %lu\r\n",now, (uint32_t)(now >> 32), (uint32_t)(now & 0xFFFFFFFF));
//	PRINTF("now %llX %08lX %08lX\r\n",now, (uint32_t)(now >> 32), (uint32_t)(now & 0xFFFFFFFF));
		ts = (struct tm *)da16x_localtime64(&nowSec);
#else
		now = da16x_time32(NULL);
		ts = da16x_localtime32(&now);
#endif /* __TIME64__ */
		da16x_strftime(buf, sizeof (buf), "%Y.%m.%d %H:%M:%S", ts);
#ifndef SILENT
//		PRINTF("- Current Time : %s (GMT %+02d:%02d)\n",   buf,   da16x_Tzoff() / 3600,   da16x_Tzoff() % 3600);
#endif

	/*
	 * Make sure that the MQTT client is still there
	 */
	statusCheck = mqtt_client_check_conn();
	if (!statusCheck)
	{
		user_log_error("**send_json_packet: MQTT connection down - aborting");
		return -1;
	}
	else
	{
	//	PRINTF("**Neuralert: send_json_packet: %d samples starting at %d\n",
	//			 count, startAdd); // FRSDEBUG

		/*
		 * JSON preamble
		 */
		strcpy(mqttMessage,"{\r\n\t\"state\":\r\n\t{\r\n\t\t\"reported\":\r\n\t\t{\r\n");
		/*
		 * MAC address of device - stored in retention memory
		 * during the bootup event
		 */
		sprintf(str,"\t\t\t\"id\": \"%s\",\r\n",pUserData->Device_ID);
		strcat(mqttMessage,str);
		/*
		 * Transmission sequence #
		 */
		sprintf(str,"\t\t\t\"trans\": %d,\r\n",msg_number);
		strcat(mqttMessage,str);
		/*
		 * Message sequence this transmission
		 */
		sprintf(str,"\t\t\t\"seq\": %d,\r\n",sequence);
		strcat(mqttMessage,str);

		/*
		 * On the first packet of a transmission interval, send
		 * battery status, time sync info, and any taps detected
		 */
		if(sequence == 1)
		{

			/* New timesync field added 1/26/23 per ECO approved by Neuralert
			 * Format:
			 *
			 *	The current first JSON packet has this format:
			 *	{ "state": { "reported": { "id": "EB345A",
			 *	"trans": 1,
			 *	"seq": 1,
			 *	"bat": 140,
			 *	"tap": 0,
			 *	"accX": [6 6 6 6 6 6 6 6 6 6 6 6 5 6 6 6 6 6 6 6 5 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 5 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6
			 *
			 *	This change will add the following field between the “seq” field and the “bat” field:
			 *
			 *	"timesync": "2023.01.17 12:53:55 (GMT 00:00) 0656741",
			 *
			 *	where the data consists of the current date and time in “local” time, as configured when WIFI is set up.  The last field (0656741) is an internal timestamp in milliseconds since power-on that corresponds to the current local time.  This will make it possible to align timestamps from different devices.
			 *
			 *	Note that the “current” local date and time is that returned from an SNTP server on the Internet and is subject to internet lag and internal processing times.
			 *
			 */
			time64_string(buf, &pUserData->MQTT_timesync_timestamptime_msec);
			sprintf(str,"\t\t\t\"timesync\": \"%s %s\",\r\n",
					pUserData->MQTT_timesync_current_time_str, buf);
			strcat(mqttMessage,str);


			/* get battery value */
			adcDataFloat = get_battery_voltage();
	//	    PRINTF("Current ADC Value: %d\n",(uint16_t)(adcDataFloat * 100));
			// Battery voltage in centivolts
			sprintf(str,"\t\t\t\"bat\": %d,\r\n",(uint16_t)(adcDataFloat * 100));
			strcat(mqttMessage,str);
			sprintf(str,"\t\t\t\"tap\": %d,\r\n",doubleTap);
			strcat(mqttMessage,str);
			doubleTap = 0;
		}

	//	PRINTF(">> JSON preamble: %s\n", mqttMessage); // FRSDEBUG

//		packet_len = strlen(mqttMessage);
//		PRINTF("\n**Neuralert: send_json_packet: %d length before accel values\n", packet_len); // FRSDEBUG

		/*
		 *  Accelerometer X values
		 */
		sprintf(str,"\t\t\t\"accX\": [");
		strcat(mqttMessage,str);
		for(i=0;i<count;i++)
		{
			Xvalue = accelXmitData[startAdd + i].Xvalue;
			sprintf(str,"%d ",Xvalue);
			strcat(mqttMessage,str);
		}
		sprintf(str,"],\r\n");
		strcat(mqttMessage,str);

		packet_len = strlen(mqttMessage);
//		PRINTF("\n**Neuralert: send_json_packet: %d length with X accel values\n", packet_len); // FRSDEBUG

		/*
		 *  Accelerometer Y values
		 */
		sprintf(str,"\t\t\t\"accY\": [");
		strcat(mqttMessage,str);
		for(i=0;i<count;i++)
		{
			Yvalue = accelXmitData[startAdd + i].Yvalue;
			sprintf(str,"%d ",Yvalue);
			strcat(mqttMessage,str);
		}
		sprintf(str,"],\r\n");
		strcat(mqttMessage,str);

		packet_len = strlen(mqttMessage);
//		PRINTF("\n**Neuralert: send_json_packet: %d length with X&Y accel values\n", packet_len); // FRSDEBUG

		/*
		 *  Accelerometer Z values
		 */
		sprintf(str,"\t\t\t\"accZ\": [");
		strcat(mqttMessage,str);
		for(i=0;i<count;i++)
		{
			Zvalue = accelXmitData[startAdd + i].Zvalue;
			sprintf(str,"%d ",Zvalue);
			strcat(mqttMessage,str);
		}
		sprintf(str,"],\r\n");
		strcat(mqttMessage,str);

		packet_len = strlen(mqttMessage);
//		PRINTF("\n**Neuralert: send_json_packet: %d length with X&Y&Z accel values\n", packet_len); // FRSDEBUG

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
			now = accelXmitData[startAdd + i].accelTime;
			// Break the timestamp into millions and remainder
			// to be able to use sprintf, which doesn't handle
			// 64-bit numbers.
			uint64_t num1 = ((now/1000000) * 1000000);
			uint64_t num2 = now - num1;
			uint32_t num3 = num2;
			sprintf(nowStr,"%ld",now/1000000);
			sprintf(str2,"%06ld ",num3);
			strcat(nowStr,str2);
			strcat(mqttMessage,nowStr);
		}
		sprintf(str,"]\r\n");
		strcat(mqttMessage,str);

		packet_len = strlen(mqttMessage);
//		PRINTF("\n**Neuralert: send_json_packet: %d length with all accel values\n", packet_len); // FRSDEBUG

		/*
		 * Closing braces
		 */
		strcat(mqttMessage,"\r\n\t\t}\r\n\t}\r\n}\r\n");

		packet_len = strlen(mqttMessage);
		PRINTF(">>send_json_packet: %d total message length\n", packet_len); // FRSDEBUG
		// Sanity check in case some future person expands message
		// without increasing buffer size
		if (packet_len > MAX_JSON_STRING_SIZE)
		{
			sprintf(user_log_string_temp, "** JSON packet size too big: %d with limit %d",
					packet_len, (int)MAX_JSON_STRING_SIZE );
			user_log_error(user_log_string_temp);
		}

		// Transmit with publish topic from NVRAM
//		transmit_status = mqtt_client_send_message(NULL, mqttMessage);
		transmit_status = 	mqtt_client_send_message_with_qos(NULL, mqttMessage,
								(MQTT_QOS_TIMEOUT_MS/10));

		if(transmit_status == 0)
		{
			PRINTF(" ==== send_json_packet: transmit %d:%d successful\n",
					msg_number, sequence); // FRSDEBUG
		}
		else
		{
			sprintf(user_log_string_temp, "**send_json_packet: transmit %d:%d failed (%d)",
					msg_number, sequence, transmit_status);
			user_log_error(user_log_string_temp);
		}

		return_status = transmit_status;

		// delay to allow message to be transmitted by the MQTT client task
		vTaskDelay(pdMS_TO_TICKS(500));

	} // If MQTT client is still connected

	return return_status;
}

/**
 *******************************************************************************
 * @brief Helper function to create a printable string from a long long
 *        because printf doesn't handle long longs
 *        Format is just digits.  No commas or anything.
 *******************************************************************************
 */
void time64_string (UCHAR *timestamp_str, __time64_t *timestamp)
{
	__time64_t timestamp_copy;
	char nowStr[20];
	char str2[20];

	// split into high order and lower order pieces
	timestamp_copy = *timestamp;
	uint64_t num1 = ((timestamp_copy/1000000) * 1000000);	// millions
	uint64_t num2 = timestamp_copy - num1;
	uint32_t num3 = num2;

	sprintf(nowStr,"%ld",(uint32_t)(timestamp_copy/1000000));
	sprintf(str2,"%06ld",num3);
	strcat(nowStr,str2);
	strcpy(timestamp_str,nowStr);

	return;
}

/**
 *******************************************************************************
 * @brief Helper function to create a printable string from a long long
 *        time in msec, because printf doesn't handle long longs
 *******************************************************************************
 */
void time64_msec_string (UCHAR *time_str, __time64_t *time_msec)
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
void time64_seconds_string (UCHAR *time_str, __time64_t *time_msec)
{

	ULONG time_milliseconds;
	ULONG time_tenths;
	ULONG time_seconds;
	UCHAR temp_string[40];

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
 * @brief calculate the timestamp for a sample read from the accelerometer FIFO
 *      by using it's relative position to the sample that caused the interrupt
 *      NOTE - this function assumes a 14Hz sample rate!
 *
 *   FIFO_timestamp - is the timestamp assigned to the FIFO buffer when it was read
 *   offset         - is the relative position in the FIFO from the timestamp
 *                    For instance, the immediately preceeding sample has
 *                    offset -1
 *   my_timestamp   - is the calculated timestamp to be assigned to the sample
 *                    with this offset
 *
 * NOTE - if CPU time is a concern with this math, the offsets could be precomputed
 *        in a table since they're always just multiples of 71.428571 msec.
 *******************************************************************************
 */
void calculate_timestamp_for_sample(__time64_t *FIFO_timestamp, int offset, __time64_t *adjusted_timestamp)
{
	__time64_t scaled_timestamp;	// times 1000 for more precise math
	__time64_t scaled_offsettime;	// the time offset of this sample * 1000
	__time64_t adjusted_scaled_timestamp;
	__time64_t rounded_offsettime;
	__time64_t inter_sample_period_usec;	// Amount we adjust for each sample in the block

	char input_str[20];
	char output_str[20];
	char scaled_time_str[20];
	char scaled_offset_str[20];
	char offset_str[20];

	scaled_timestamp = *FIFO_timestamp;
	scaled_timestamp = scaled_timestamp * (__time64_t)1000;
	time64_string (input_str, FIFO_timestamp);  // calling with address here
	time64_string (scaled_time_str, &scaled_timestamp);
//	PRINTF ("  Input time: %s  Scaled time: %s\n", input_str, scaled_time_str);

	// The math here is done * 1000 so as not to have large rounding
	// errors
	// If we have successfully calibrated this accelerometer,
	// use the calibrated value.  Otherwise, use the nominal 14Hz value
	if (pUserData->AXL_calibration_complete)
	{
		inter_sample_period_usec = pUserData->AXL_cal_sample_period_usec;
//		time64_string (scaled_time_str, &inter_sample_period_usec);
//		PRINTF ("  >>Using calibrated sample period: %s usec\n", scaled_time_str);
	}
	else
	{
		inter_sample_period_usec = (__time64_t)71429;	// 1000000 usec / 14
//		time64_string (scaled_time_str, &inter_sample_period_usec);
//		PRINTF ("  >>Using nominal sample period: %s usec\n", scaled_time_str);
	}
	scaled_offsettime = inter_sample_period_usec * (__time64_t)offset;
	// offset time * 1000
	adjusted_scaled_timestamp = scaled_timestamp + scaled_offsettime;
	// back to msec.  We round by adding 500 usec before dividing
	rounded_offsettime = (adjusted_scaled_timestamp + (__time64_t)500)
			                  / (__time64_t)1000;
	time64_string (offset_str, &rounded_offsettime);
//	PRINTF (" Offset index: %d  Offset time: %s\n", offset, offset_str);

	*adjusted_timestamp = rounded_offsettime;
	return;
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
		__time64_t accelTime;               //!< time when accelleromter data taken
	} accelDataStruct;
 *
 *returns -1 if an error
 *returns number of samples in data array otherwise
 *******************************************************************************
 */
static int assemble_packet_data (HANDLE SPI_handle, int start_block, int end_block)
{
	int blocknumber;		// 0-based block index in Flash
	ULONG blockaddr;			// physical address in flash
	int done = pdFALSE;
	__time64_t block_timestamp;
	__time64_t sample_timestamp;
	ULONG sample_timestamp_offset;
	int timesource;
	int timeoffset;
	accelBufferStruct FIFOblock;
	int num_samples;
	int num_blocks;
	int i;
	int retry_count;
	int timeoffsetindex;   	// +- position from where timestamp is assigned
	__time64_t inter_sample_period_usec;	// Amount we adjust for each sample in the block

	char scaled_time_str[20];

	char block_timestamp_str[20];
	char sample_timestamp_str[20];
	HANDLE SPI = NULL;

	PRINTF("**assembling packet data from %d to %d\n", start_block, end_block);

	SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		user_log_error("***MQTT transmit: MAJOR SPI ERROR: Unable to open SPI bus handle");
		return -1;
	}

	// Note that start_block may be less than or greater than end_block
	// depending on whether we wrap around or not
	// If we're only sending one block, then start_block will be
	// equal to end_block
	blocknumber = start_block;
	num_blocks = 0;
	num_samples = 0;
	while (!done)
	{
		// Read the block from Flash


		// For each block, assemble the XYZ data and assign a timestamp
		// based on the block timestamp and the samples relation to
		// when that timestamp was taken
		// Calculate address of next sector to write
		blockaddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)blocknumber);
		for (retry_count = 0; retry_count < 3; retry_count++)
		{
			if (!AB_read_block(SPI, blockaddr, &FIFOblock))
			{
				sprintf(user_log_string_temp, "assemble_packet_data: unable to read block %d addr: %x\n",
						blocknumber, blockaddr);
				user_log_error(user_log_string_temp);
				spi_flash_close(SPI);
				return -1;
			}
			else
			{
//				PRINTF(" assemble_packet_data: block %d read [%x] # samples: %d\n",
//						blocknumber, blockaddr, FIFOblock.num_samples);
			}
			if(FIFOblock.num_samples > 0)
			{
				break;
			}

		}
		if (retry_count > 0)
		{
			PRINTF(" assemble_packet_data: retried read %d times", retry_count);
		}

		// diagnostic if we don't read a real FIFO block
		if (FIFOblock.num_samples <= 0)
		{
			for(int i = 0; i < 3; i++)
			{
				PRINTF("  FIFO: %d     X: %d Y: %d Z: %d\n", i, FIFOblock.Xvalue[i], FIFOblock.Yvalue[i], FIFOblock.Zvalue[i]);
			} // for each sample in FIFO compare buffer
		}

		// add the samples to the transmit array
		num_blocks++;
		block_timestamp = FIFOblock.accelTime;
		time64_string (block_timestamp_str, &block_timestamp);
		timesource = FIFOblock.timestamp_sample;
//		PRINTF("  FIFO timesource: %d  timestamp: %s\n",
//				timesource, block_timestamp_str);

		if (pUserData->AXL_calibration_complete)
		{
			inter_sample_period_usec = pUserData->AXL_cal_sample_period_usec;
			time64_string (scaled_time_str, &inter_sample_period_usec);
//			PRINTF ("  >>Using calibrated sample period: %s usec\n", scaled_time_str);
		}
		else
		{
			inter_sample_period_usec = (__time64_t)71429;	// 1000000 usec / 14
			time64_string (scaled_time_str, &inter_sample_period_usec);
//			PRINTF ("  >>Using nominal sample period: %s usec\n", scaled_time_str);
		}

		for (i=0; i<FIFOblock.num_samples; i++)
		{
			accelXmitData[num_samples].Xvalue = FIFOblock.Xvalue[i];
			accelXmitData[num_samples].Yvalue = FIFOblock.Yvalue[i];
			accelXmitData[num_samples].Zvalue = FIFOblock.Zvalue[i];
			// Calculate the approximate time of this sample.
			// The timestamp for the FIFO block applies to "timesource" sample,
			// which is the sample that reached the FIFO threshold set
			// when we set up the accelerometer.  The offset will be negative
			// for earlier samples and positive for later ones.
			timeoffsetindex = i - timesource;
			// offset will be how many 14Hz intervals plus/minus from timestamp
			if (timeoffsetindex == 0)
			{
				// This is the sample that reached the FIFO threshold
				// and caused the interrupt
				sample_timestamp = block_timestamp;
			}
			else
			{
				// This is one of the other samples retrieved from the
				// accelerometer so we have to calculate what it's timestamp
				// should be based on it's position in the FIFO relative to
				// the sample that caused the interrupt
				calculate_timestamp_for_sample(&block_timestamp,
						timeoffsetindex, &sample_timestamp);
			}

			accelXmitData[num_samples].accelTime = sample_timestamp;
			num_samples++;
		}

		// See if we're done
		if (blocknumber == end_block)
		{
			done = pdTRUE;
		}
		else
		{
			// step to next block
			blocknumber++;
			if(blocknumber >= AB_FLASH_MAX_PAGES)
			{
				// wraparound
				blocknumber = 0;
			}
		}
	}

	spi_flash_close(SPI);
	PRINTF("**Assemble packet data: %d samples assembled from %d blocks\n",
			num_samples, num_blocks);

	return num_samples;
}



/**
 *******************************************************************************
 * @brief Process to connect to an AP that stored in NVRAM.
 * return: 0 ; no error
 *******************************************************************************
 */
static int user_process_connect_to_ap_and_wait(int max_wait_seconds)
{
	int	ret = 0;
	char value_str[128] = {0, };
	unsigned int start_time_ticks, cur_time_ticks, elapsed_time_ticks;
	unsigned int max_wait_msec;
	unsigned int max_wait_TICKS;
	unsigned int elapsed_time_msec;
#define INITIAL_CHECK_INTERVAL_MSEC 3000
#define CHECK_INTERVAL_MSEC 1000
	int connected;


//	SET_PROCESS_BIT(processLists, USER_PROCESS_CONNECT_AP);
	PRINTF("**Neuralert: %s\n", __func__); // FRSDEBUG
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===Enter connect_to_ap_and_wait");
#endif

	if (user_process_check_wifi_conn() == pdTRUE) {
		PRINTF("**Neuralert: user_process_connect_to_ap() already connected\n"); // FRSDEBUG

		/* Connection is already established */
//		user_connection_complete_event();
		return ret;
	}

	// use the internal command line interface to initiate the connect
	PRINTF("**Neuralert: user_process_connect_to_ap() attempting connection\n"); // FRSDEBUG
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===start of connection attempt");
#endif

	ret = da16x_cli_reply("select_network 0", NULL, value_str);
	if (ret < 0 || strcmp(value_str, "FAIL") == 0) {
		PRINTF(" [%s] Failed connect to AP (da16x_cli_reply) 0x%x %s\n", __func__, ret, value_str);
//		CLR_PROCESS_BIT(processLists, USER_PROCESS_CONNECT_AP);
		ret = -1;
		return ret;
	}

	// Wait an initial time since we know it will take a little time
	PRINTF(" [%s] Start initial check delay %d\n", __func__, INITIAL_CHECK_INTERVAL_MSEC);
	vTaskDelay(pdMS_TO_TICKS(INITIAL_CHECK_INTERVAL_MSEC));

	// Wait for connection with timeout
	start_time_ticks = xTaskGetTickCount();
	max_wait_msec = max_wait_seconds * 1000;
	max_wait_TICKS = pdMS_TO_TICKS(max_wait_msec);
	do {
		vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MSEC));

		PRINTF(" [%s] Checking WIFI\n", __func__);

		cur_time_ticks = xTaskGetTickCount();
		if (cur_time_ticks >= start_time_ticks) {
			elapsed_time_ticks = cur_time_ticks - start_time_ticks;
		} else {
			elapsed_time_ticks =  (((unsigned int) 0xFFFFFFFF) - start_time_ticks) + cur_time_ticks;
		}
		PRINTF(" [%s] elapsed time %d ticks (max %d ticks )\n", __func__, elapsed_time_ticks, max_wait_TICKS);

		connected = user_process_check_wifi_conn();

	} while (!connected && (elapsed_time_ticks <= max_wait_TICKS));

	if (connected)
		ret = 0;
	else
		ret = -1;
	return ret;
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
 *    {message:terminate <Unique device id>}
 *    where the device id is the abbreviated MAC address used to
 *    identify this specific device.
 *******************************************************************************
 */

int parseDownlink(char *buf, int len)
{
	int i, j,k, argc, ptrCommand,status;
	int tempInt;
	char str1[20];
	char commands[4][90];
	char *argvTmp[4] = {&commands[0][0], &commands[1][0], &commands[2][0], &commands[3][0]};

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
		sprintf(user_log_string_temp, "** message keyword missing in downlink command! %s",str1);
		user_log_error(user_log_string_temp);
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

	if(strcmp(&commands[0][0],"terminate") == 0)
	{
		if(argc < 2)
		{
			user_log_error("  ** terminate command received without device identifier\n");
		}
		else if (strcmp(pUserData->Device_ID, &commands[1][0]) != 0)
		{
			sprintf(user_log_string_temp, "** terminate command with wrong device identifier: %s",
					&commands[1][0]);
			user_log_error(user_log_string_temp);
		}
		else
		{
			// We've been asked to shut down by the cloud server
			// Set a flag so that it happens in an orderly way and
			// under the control of the accelerometer task, who
			// knows what's what
			// We use a special value so guard against a random
			// occurrence.
			pUserData->ServerShutdownRequested = MAGIC_SHUTDOWN_KEY;
		}
	}

#if 0
	// Commands left over from Tim's version
	if(strcmp(&commands[0][0],"timezone") == 0)
	{
		tempInt = strtol(&commands[1][0], NULL, 10);
		PRINTF("Time Zone Downlink: %d\r\n",tempInt);
		tempInt = tempInt * 3600;
		set_timezone_to_rtm(tempInt);	/* to rtm */
		da16x_SetTzoff(tempInt);
		set_time_zone(tempInt);		/* to nvram */
		return TRUE;
	}
	if(strcmp(argvTmp[0],"ota_update") == 0)
	{
		cmd_ota_update(argc, argvTmp);
		return TRUE;
	}
#endif

	return TRUE;
}


/**
 *******************************************************************************
 * @brief Callback function to receive downlink messages from the MQTT
 * broker
 *******************************************************************************
 */
static void MQTT_message_receiver_cb (const char *buf, int len, const char *topic)
{
	MQTT_DBG_PRINT("\n  MQTT_message_receiver_cb called %s %d topic: %s\r\n\n",buf, len, topic);

	user_log_event("Downlink command received");
	user_log_event(buf);
	parseDownlink((char *)buf, len);
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
	int clear_to_transmit = FALSE;
	int samples_to_send;
	int send_start_addr;
	int msg_sequence;
	int whole_packets_to_send;
	int last_packet_size;
	int WIFI_status;
	int MQTT_status;
	int MQTT_client_state;
	int transmit_start_loc;
	int transmit_end_loc;
	int transmit_memory_size;	// last index before wraparound
	int next_write_position;	// next place AXL will write a block
	int last_write_position;	// last place AXL wrote a block
	int total_blocks_stored;	// total number of blocks available to send when wakened
	int total_blocks_free;		// total number of blocks AXL has free to write
	int num_blocks_to_send;		// number of FIFO blocks to transmit this interval
	int num_blocks_left_to_send;	// Countdown of what we have left to do

	int this_packet_start_block;	// where the current packet starts in AB
	int this_packet_end_block;		// where the current packet ends in AB
	int this_packet_num_blocks;		// how many FIFO blocks in the current packet
	int next_packet_start_block;	// where the subsequent packet will start
	int request_stop_transmit = pdFALSE;		// loop control for packet transmit loop
	int packet_count;				// packet send counter

	// stats
	int packets_sent = 0;		// actually sent during this transmission interval
	int blocks_sent = 0;
	int samples_sent = 0;

	int drop_data_skip_to_loc;		// start location of where we will transmit after dropping data

	UCHAR elapsed_sec_string[40];

#ifdef CFG_USE_RETMEM_WITHOUT_DPM
	UINT32 i;
#else
	int len = 0;
	char data_buffer[TCP_CLIENT_TX_BUF_SIZE] = {0x00,};
#endif

	// Let the other tasks know that we're active so we don't sleep
	SET_PROCESS_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);

	vTaskDelay(1);

	// Mark our start time
	user_time64_msec_since_poweron(&user_MQTT_start_msec);
	time64_string(elapsed_sec_string, &user_MQTT_start_msec);
	PRINTF("\n ===== MQTT start milliseconds %s\n\n", elapsed_sec_string);

	log_current_time("MQTT connection attempt. ");


//	PRINTF("\n**********************************************\n");
//	PRINTF("**Neuralert: user_process_send_MQTT_data\n"); // FRSDEBUG
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("=********** user_process_send_MQTT_data **********");
#endif
	vTaskDelay(1);

	// Set system state to WIFI connection attempt
//	set_sole_system_state(USER_STATE_WIFI_CONNECTING);

	// Retrieve the next transmit block location
	transmit_start_loc = get_AB_transmit_location();
	transmit_memory_size = AB_FLASH_MAX_PAGES;

	// If there is no valid transmit position, we've been wakened by mistake
	// or something else has gone wrong
	if (	(transmit_start_loc == INVALID_AB_ADDRESS)
			|| (transmit_start_loc < 0)
			|| (transmit_start_loc >= transmit_memory_size))
	{
		sprintf(user_log_string_temp, "MQTT task found invalid transmit start location: %d", transmit_start_loc);
		user_log_error(user_log_string_temp);
		set_sole_system_state(USER_STATE_INTERNAL_ERROR);
		goto end_of_task;
	}

	PRINTF("\n\n******  MQTT transmit starting at %d ******\n", transmit_start_loc);

	// So, notionally, we'll be wakened every 5 minutes or so
	// 5 minutes is 14Hz x 60 x 5 = 4200 AXL samples
	// 4200 samples / ~30 samples per FIFO block = 140 blocks
	//
	// Because of our flash storage scheme, data is stored in blocks
	// representing one read of the AXL FIFO.  (Note that we might
	// make this more efficient by having the AXL task store data in
	// RETMEM between interrupts and then aggregate samples to be stored.
	// But, because the timestamp is associated with the interrupt, that
	// would seriously complicate life.)
	//

	// Figure out total number of blocks that have been stored, based
	// on where AXL is about to write next.  The last block written
	// should have been just prior to this one
	// We are going to assume that we wouldn't have been wakened if
	// AXL hasn't already written some blocks to nonvol, so we won't
	// get carried away double-checking.  The data should be invalid anyway.
	next_write_position = get_AB_store_location();
	if (	(next_write_position < 0)
		|| 	(next_write_position >= transmit_memory_size))
	{
		sprintf(user_log_string_temp, "MQTT task found invalid AXL write location: %d", next_write_position);
		user_log_error(user_log_string_temp);
		set_sole_system_state(USER_STATE_INTERNAL_ERROR);
		goto end_of_task;
	}
	last_write_position = next_write_position - 1;
	if (last_write_position < 0)
	{
		// AXL just wrapped around so our last position is the last
		// place in memory.
		last_write_position += transmit_memory_size;
	}
	PRINTF("  MQTT last block written is %d\n", last_write_position);

	total_blocks_stored = (last_write_position - transmit_start_loc) + 1;
	if (total_blocks_stored < 0)
	{
		// This involves a wraparound
		total_blocks_stored += transmit_memory_size;
	}
	PRINTF("  MQTT total blocks stored %d\n", total_blocks_stored);

	total_blocks_free = (transmit_start_loc - next_write_position);
	if (total_blocks_free < 0)
	{
		// This involves a wraparound
		total_blocks_free += transmit_memory_size;
	}
	PRINTF("  MQTT total blocks free %d\n", total_blocks_free);

	// Check to see if we're getting close to running out of
	// accelerometer buffer storage due to being unable to transmit
	if(total_blocks_free < AB_FLASH_WARNING_THRESHOLD)
	{
		set_system_alert(USER_ALERT_AB_STORAGE_LOW);
	}
	else
	{
		clear_system_alert(USER_ALERT_AB_STORAGE_LOW);
	}
	// Check to see if we're behind the accelerometer and it will catch up.
	// To prevent this, we keep a sizable unused space between the next
	// AXL store location and the MQTT next transmit location.
	if(total_blocks_free < AB_FLASH_OVERLAP_LIMIT)
	{
		pUserData->MQTT_dropped_data_events++;

		APRINTF_E("\n***** AB STORAGE OVERLAP LIMIT %d EXCEEDED *****\n", AB_FLASH_OVERLAP_LIMIT);
		drop_data_skip_to_loc = transmit_start_loc + AB_FLASH_OVERLAP_LIMIT;
		if(drop_data_skip_to_loc >= transmit_memory_size)
		{
			// Wrap around
			drop_data_skip_to_loc -= transmit_memory_size;
		}
		APRINTF_E("***** Dropping data from page %d to %d *****\n",	transmit_start_loc, drop_data_skip_to_loc);
		if(!update_AB_transmit_location(drop_data_skip_to_loc))
		{
			APRINTF_E("\n MQTT: Unable to update AB transmit location\n");
		}
		else
		{
			APRINTF_E("\n MQTT: updated AB transmit location: %d ******\n\n",drop_data_skip_to_loc);
		}
		sprintf(user_log_string_temp, "** AB STORAGE OVERLAP LIMIT %d EXCEEDED", AB_FLASH_OVERLAP_LIMIT);
		user_log_error(user_log_string_temp);
		sprintf(user_log_string_temp, "** Dropping data from page %d to %d", transmit_start_loc, drop_data_skip_to_loc);
		user_log_error(user_log_string_temp);

		transmit_start_loc = drop_data_skip_to_loc;

		// Note we assume that this will be positive by correct sizing
		// of AB memory and the overlap limit
		total_blocks_stored = (last_write_position - transmit_start_loc) + 1;
		if (total_blocks_stored < 0)
		{
			total_blocks_stored += transmit_memory_size;
		}
		PRINTF("  Updated MQTT total blocks stored %d\n", total_blocks_stored);

		total_blocks_free = (transmit_start_loc - next_write_position);
		if (total_blocks_free < 0)
		{
			total_blocks_free += transmit_memory_size;
		}
		PRINTF("  Updated MQTT total blocks free %d\n", total_blocks_free);
	} // if we've caught up to the writer


	// AB memory is sized to store a max of about 2 hours of data
	// If we have no WIFI, the worst case is 14Hz x 60 seconds x 60 minutes x 2
	// or about 100,800 samples to transmit.  If there are about 30 samples
	// per FIFO block, we could have as many as 3,360 FIFO blocks to transmit
	// As of 7/22/22 it was taking about 0.8 second to transmit one JSON packet
	// of 10 FIFO blocks.  In this case it would take 336 JSON packets to
	// transmit 2 hours worth of data.  At 0.8 seconds per packet, that would
	// be 268.8 seconds or 4.48 minutes.
	//
	// But... we also have the cooperative task scenario to consider.
	// As of 7/22/22, when WIFI is available and the MQTT broker
	// is available, it takes about 5 or 10 seconds to get the WIFI up
	// and running and the MQTT client functional.  Plus we have a post-
	// transmission delay to allow the cloud to send us a message.
	// So in the worst case, sending all the stored data might cut it close
	// if we were fully backed up.
	//
	// So choose a chunk of data that we can transmit in, say,
	// two minutes, which will give plenty of breathing room and, hopefully,
	// allow the MQTT task to complete before the AXL task thinks it wants
	// us to start again.

	num_blocks_to_send = MAX_FIFO_BUFFERS_PER_TRANSMIT_INTERVAL;
	if (num_blocks_to_send > total_blocks_stored)
	{
		num_blocks_to_send = total_blocks_stored;
	}
	PRINTF("  MQTT blocks to transmit this cycle %d\n", num_blocks_to_send);

	// Figure out ending position of extent being transmitted this cycle
	transmit_end_loc = (transmit_start_loc + num_blocks_to_send) - 1;
	if(transmit_end_loc >= transmit_memory_size)
	{
		// Wrap around
		transmit_end_loc -= transmit_memory_size;
	}

	PRINTF("  MQTT transmitting %d blocks from %d to %d\n",
			num_blocks_to_send, transmit_start_loc, transmit_end_loc);


	// OK - now we know what the plan is.


	// Start the RF section power up (should have been done earlier)
	wifi_cs_rf_cntrl(FALSE);
	vTaskDelay(1);

	// Fire up the WIFI connection (RF should be powered up)
	// and wait around to see if we can connect
	// wait up to 10 seconds
	pUserData->MQTT_connect_attempts++;
	WIFI_status = user_process_connect_to_ap_and_wait(WIFI_CONNECT_TIMEOUT_SECONDS);
	if (WIFI_status != 0)
	{
		PRINTF("\n\n>>>>> ***** UNABLE TO CONNECT TO WIFI - stopping transmit ******\n\n");
		pUserData->MQTT_connect_fails++;
		user_log_event("** MQTT unable to connect to WIFI **");
		// Transition to a warning state
		set_sole_system_state(USER_STATE_WIFI_CONNECT_FAILED);
		goto end_of_task;
	}
	vTaskDelay(1);

//	PRINTF("\n============ MQTT transmit WIFI connected =======================\n");
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===MQTT transmit WIFI connected===");
#endif

	// So at this point, there should be a connection to the access point
	// If we're booting for the first time, the MQTT client task might
	// be running, but when in steady state we expect that it was shut
	// down when we left off from the last transmit.
	// So see what's going on and try to get it running again
	MQTT_status =  check_mqtt_client_thread_status();
	if (MQTT_status == 0)
	{
		PRINTF("**user_process_send_MQTT_data: MQTT client not started\n"); // FRSDEBUG
	}
	else
	{
		PRINTF("**user_process_send_MQTT_data: MQTT client already started\n"); // FRSDEBUG
	}

	// Observation of how this code plays out shows that if the MQTT client
	// thread is started but still not connected, we try to start it but
	// fail because it's already running.  So give it a little more than
	// than we did originally to see if this helps
	// Note - as of about 9/20/22, it helped.  No MQTT fail-to-connect
	// events were observed.
	vTaskDelay(pdMS_TO_TICKS(1500));

	if (mqtt_client_check_conn())
	{
		PRINTF("\n**Neuralert: MQTT client connected already\n"); // FRSDEBUG
		clear_to_transmit = TRUE;
	}
	else
	{
		// try to start it
		PRINTF("**Neuralert: attempting MQTT client start\n"); // FRSDEBUG

		// (Optional) Put MQTT config in NVRAM first
		// mqtt_user_config();

		// This only starts the MQTT task
		status = mqtt_client_start();
		if(status == 0)
		{
			PRINTF("**Neuralert: MQTT client started successfully\n"); // FRSDEBUG
			// delay to allow network to get established
			// time based on Fred guess but also MQTT programmer's guide example code
			vTaskDelay(pdMS_TO_TICKS(3000));
		}
		else
		{
			PRINTF("**Neuralert: MQTT client start failed\n"); // FRSDEBUG
			clear_to_transmit = FALSE;
		}
	}
	vTaskDelay(1);

	// So this is weirdly redundant.  Probably need a connect callback function
	// instead of all this if we can find one for MQTT
	if (!clear_to_transmit)
	{
		PRINTF("**Neuralert: going to wait for connection\n"); // FRSDEBUG
		// wait six seconds for connection
		// So this wait period is somewhat arbitrary.  On 9/12/22 it was
		// observed that a three second wait caused us to occasionally
		// miss a transmit interval because it looked like we gave up too
		// soon.  So I doubled the time to give it time to finish the connection.
		// 9/27/22 update - this seemed to work and we haven't been
		// missing any connect attempts due to timing
		status = user_process_MQTT_wait_for_connection(6000);
		if (status == 0)
		{
			PRINTF("**Neuralert: MQTT client connected\n"); // FRSDEBUG
			clear_to_transmit = TRUE;
		}
		else
		{
			PRINTF("\n**Neuralert: MQTT client NOT connected! aborting send\n"); // FRSDEBUG
			clear_to_transmit = FALSE;
		}
	}
	vTaskDelay(1);

	// *****************************************************************
	//  MQTT transmission
	// *****************************************************************
	if(!clear_to_transmit)
	{
		PRINTF("\n**Neuralert:unable to transmit\n"); // FRSDEBUG
		pUserData->MQTT_connect_fails++;
		request_stop_transmit = pdTRUE;
		// Transition to a Yellow blink
		user_log_event("** MQTT unable to connect to broker **");
		set_sole_system_state(USER_STATE_WIFI_CONNECT_FAILED);
	}
	else
	{
		// Let user know the happy news
//		set_sole_system_state(USER_STATE_WIFI_CONNECTED);
		// pause momentarily so they get a chance to see the LED
//		vTaskDelay(pdMS_TO_TICKS(1000));
		// So now we know that the MQTT client has been successfully
		// started.  Set up the callback function to receive downlink
		// messages
		mqtt_client_set_msg_cb(MQTT_message_receiver_cb);

		// We are connected to WIFI and MQTT and should have wall clock
		// time from an SNTP server
		// On the first successful connection we take a time snapshot that
		// will be transmitted in the "timesync" JSON field so that the
		// end user can coordinate the left and right wrist datastreams
		if(pUserData->MQTT_timesync_captured == 0)
		{
			timesync_snapshot();
			pUserData->MQTT_timesync_captured = 1;
		}


		// Our transmit extent was calculated above

		msg_sequence = 0;
		++pUserData->MQTT_message_number;  // Increment transmission #

		// See how big the last packet will be
		whole_packets_to_send = num_blocks_to_send / FIFO_BLOCKS_PER_PACKET;
		last_packet_size = num_blocks_to_send - (whole_packets_to_send * FIFO_BLOCKS_PER_PACKET);
		PRINTF("\n**Neuralert: send_data: sending %d FIFO blocks in %d whole packets\n",
				num_blocks_to_send, whole_packets_to_send); // FRSDEBUG
		if (last_packet_size > 0)
		{
			PRINTF("\n**Neuralert:            with a final packet of %d FIFO blocks\n",
				last_packet_size); // FRSDEBUG
		}
		// Construct data to send

		// Set up transmit loop parameters
		num_blocks_left_to_send = num_blocks_to_send;
		this_packet_start_block = transmit_start_loc;

		// Let user know what we're up to
//		set_sole_system_state(USER_STATE_WIFI_TRANSMITTING);

		vTaskDelay(1);
		request_stop_transmit = pdFALSE;
		packet_count = 0;
		while (	(num_blocks_left_to_send > 0)
				&& (pdFALSE == request_stop_transmit) )
		{
			packet_count++;

			// how many blocks in this packet
			if (num_blocks_left_to_send < FIFO_BLOCKS_PER_PACKET)
			{
				// Sending partial packet with what's left
				this_packet_num_blocks = num_blocks_left_to_send;
			}
			else
			{
				// Sending full size packet
				this_packet_num_blocks = FIFO_BLOCKS_PER_PACKET;
			}
			this_packet_end_block = (this_packet_start_block + this_packet_num_blocks) - 1;
			if(this_packet_end_block >= transmit_memory_size)
			{
				// Wrap around
				this_packet_end_block -= transmit_memory_size;
			}

			PRINTF("\n**MQTT packet %d:  Start: %d End: %d num blocks: %d\n",
					packet_count, this_packet_start_block, this_packet_end_block,
					this_packet_num_blocks);

			// Assemble data to be transmitted in this packet, including
			// calculating timestamps
			samples_to_send = assemble_packet_data(NULL, this_packet_start_block, this_packet_end_block);
			if (samples_to_send == 0)
			{
				request_stop_transmit = pdTRUE;
				user_log_error("** MQTT: no samples found for packet - aborting");
			}
			else
			{
				if (samples_to_send < 0)
				{
					user_log_error("** MQTT transmit: error returned from assemble_packet_data - aborting");
					request_stop_transmit = pdTRUE;
				}
				else
				{
					msg_sequence++;
					send_start_addr = 0;
					status = send_json_packet (send_start_addr, samples_to_send,
							pUserData->MQTT_message_number, msg_sequence);
					if(status == 0)
					{
						// Do stats
						packets_sent++;		// Total packets sent this interval
						blocks_sent += this_packet_num_blocks;
						samples_sent += samples_to_send;

//						PRINTF("**Neuralert: send_data: transmit %d:%d successful\n",
//								pUserData->MQTT_message_number, msg_sequence); // FRSDEBUG
		#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("===JSON packet sent");
		#endif
						PRINTF("Starting post-JSON packet transmit delay (%d msec)\n", MQTT_INTER_PACKET_DELAY_MS);
						// Because MQTT and WIFI activity seems to interfere seriously
						// with SPI bus activity, put a delay here to allow this
						// activity to settle down before attempting reading more
						// data from SPI flash
						vTaskDelay(pdMS_TO_TICKS(MQTT_INTER_PACKET_DELAY_MS));

						// Now that we've transmitted this packet, update the
						// MQTT transmit pointer to reflect the data transmitted
						// Note that we believe that we'll never catch up to the
						// Accelerometer store location because we maintain a sizable
						// buffer of free space between the two pointers.  See elsewhere
						// in this function for that logic.
						next_packet_start_block = this_packet_end_block + 1;
						if(next_packet_start_block >= transmit_memory_size)
						{
							// Wrap around
							next_packet_start_block -= transmit_memory_size;
						}
						if(!update_AB_transmit_location(next_packet_start_block))
						{
							user_log_error("** MQTT: Unable to update AB transmit location");
						}
						else
						{
							PRINTF("\n MQTT: updated AB transmit location: %d ******\n\n",next_packet_start_block);
						}
					}
					else
					{
						sprintf(user_log_string_temp, "MQTT transmission %d:%d failed. Ending transmission.",
								pUserData->MQTT_message_number, msg_sequence);
						user_log_error(user_log_string_temp);
						PRINTF_RED("\n**** %s\n", user_log_string_temp);
						request_stop_transmit = pdTRUE;
						pUserData->MQTT_transmit_fails++;
					}
					vTaskDelay(1);
				} // Able to assemble packets

				num_blocks_left_to_send -= this_packet_num_blocks;
				this_packet_start_block = this_packet_end_block + 1;
				if(this_packet_start_block >= transmit_memory_size)
				{
					// Wrap around
					this_packet_start_block -= transmit_memory_size;
				}
			} // Samples to send > 0
		} // while we have stuff to transmit
	} // if clear to transmit

	if(!request_stop_transmit)
	{
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
		printf_with_run_time("===Starting post-transmit delay");
#endif
		// delay to allow messages to be transmitted by the MQTT client task
		vTaskDelay(pdMS_TO_TICKS(MQTT_POST_TRANSMISSION_DELAY_MS));
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
		printf_with_run_time("===Done post-transmit delay");
#endif

		PRINTF(" ==== MQTT transmission %d complete.  %d samples in %d JSON packets",
				pUserData->MQTT_message_number,
				samples_sent,
				packets_sent);
		vTaskDelay(pdMS_TO_TICKS(2000));

		sprintf(user_log_string_temp,
				"MQTT transmission %d complete.  %d samples in %d JSON packets",
				pUserData->MQTT_message_number,
				samples_sent,
				packets_sent);
		user_log_event(user_log_string_temp);

	}
	else
	{
		sprintf(user_log_string_temp, "MQTT transmission %d inccomplete. %d samples sent in %d JSON packets ",
				pUserData->MQTT_message_number, samples_sent, packets_sent);
		user_log_error(user_log_string_temp);
		set_sole_system_state(USER_STATE_WIFI_CONNECT_FAILED);
		// delay to allow warning to be seen
		vTaskDelay(pdMS_TO_TICKS(MQTT_POST_TRANSMISSION_DELAY_MS));
	}

	// Presumably we've finished sending and allowed time for a shutdown
	// command or other message back from the cloud
	// Turn off the RF section until next transmit interval

	PRINTF("\n**Neuralert: user_process_send_data turning off RF section\n"); // FRSDEBUG
	wifi_cs_rf_cntrl(TRUE);

	// delay to allow things to settle
	vTaskDelay(pdMS_TO_TICKS(MQTT_POST_RF_POWER_OFF_DELAY_MS));
#if 0
	spi_flash_close(MQTT_SPI_handle);
#endif
end_of_task:

	// Stop the mqtt client
	mqtt_client_stop();

	// Power down the RF section
	// Note as of 7/11/22 this confuses the lan clients
	wifi_cs_rf_cntrl(TRUE);

#ifdef CFG_USE_SYSTEM_CONTROL
	// Disabling WLAN at the next boot. : This is an example.
	system_control_wlan_enable(FALSE);
#endif

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===user_process_send_MQTT_data finished");
#endif

	// Clear system state so LEDs are off, unless there's an alert
	set_sole_system_state(0);

	// Check to see if we received a device terminate downlink command
	// while we were connected to the MQTT broker
	if (pUserData->ServerShutdownRequested == MAGIC_SHUTDOWN_KEY)
	{
		PRINTF("\n\n\n**************************\n");
		PRINTF("  SHUTDOWN REQUESTED\n");
		PRINTF("**************************\n\n\n");
		user_log_event("** SHUTDOWN REQUESTED **");
		if (xTask) {
			xTaskNotifyIndexed(xTask, 0, USER_TERMINATE_EVENT, eSetBits);
		}
	}

	CLR_PROCESS_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);

	// Get our end time, elapsed time, and log it
	user_time64_msec_since_poweron(&user_MQTT_end_msec);

	user_time64_msec_since_poweron(&user_MQTT_end_msec);
	time64_string(elapsed_sec_string, &user_MQTT_end_msec);
	PRINTF("\n ===== MQTT end milliseconds %s\n", elapsed_sec_string);

	user_MQTT_task_time_msec = (ULONG)(user_MQTT_end_msec - user_MQTT_start_msec);
	PRINTF(" ==== MQTT elapsed milliseconds %lu\n", user_MQTT_task_time_msec);

	// Note the following two lines cause a hard fault.  I don't know why
//	sprintf(user_log_string_temp, "MQTT task elapsed time: %lu", user_MQTT_task_time_msec);
//	user_log_event(user_log_string_temp);

	/* Delete task */
	MQTT_task_handle = NULL;
	vTaskDelete(NULL);
}


/**
 *******************************************************************************
 *  user_create_MQTT_task: create a new task for MQTT transmission
 *******************************************************************************
 */
static void user_create_MQTT_task()
{
	BaseType_t create_status;
	UBaseType_t current_task_priority;	// priority of main task (Accelerometer task)

	// Get our priority
	current_task_priority = uxTaskPriorityGet((TaskHandle_t)NULL);

	create_status = xTaskCreate(
			user_process_send_MQTT_data,
			"MQTTtask", 				// Task name
			(4*1024),					// 4K stack size for comfort
			( void * ) NULL,  			// no parameter to pass
			(current_task_priority - 1),  // Make this a lower priority
//		  (tskIDLE_PRIORITY + 6), 		// one less than accelerometer
			&MQTT_task_handle);			// save the task handle

	if (create_status == pdPASS)
	{
		PRINTF(">>>>>>> MQTT transmit task created <<<<<<<<<"); // FRSDEBUG
	}
	else
	{
		user_log_error(">>>>>>> MQTT transmit task failed to create <<<<<<<<<");
	}
//	vTaskDelay(pdMS_TO_TICKS(3000));

	return;
}



/**
 *******************************************************************************
 *  Dummy function during stage5a to keep timer transmit happy
 *******************************************************************************
 */
static void user_process_send_data()
{

	PRINTF("\n\n>>>>>>>> TIMED VERSION user_process_send_data\n\n"); // FRSDEBUG
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===user_process_send_data SHOULD NOT BE RUNNING");
#endif
//	vTaskDelay(pdMS_TO_TICKS(3000));

	return;
}



/**
 *******************************************************************************
 * @brief Process to check wifi connection status.
 *******************************************************************************
 */
static UCHAR user_process_check_wifi_conn(void)
{
	char *status;
	UCHAR result = 0;  // should say pdFALSE (just sayin...)

//	PRINTF("**Neuralert: %s\n", __func__); // FRSDEBUG

	status = (char *)pvPortMalloc(USER_CONNECT_STATUS_REPLY_SIZE);
	if (status == NULL) {
		PRINTF("%s(%d): failed to allocate memory\n", __func__, __LINE__);
		return pdFALSE;
	}

	memset(status, 0, USER_CONNECT_STATUS_REPLY_SIZE);
	da16x_cli_reply("status", NULL, status);

//	PRINTF("\n**Neuralert: command line status string: [%s]\n", status);
	if (strstr(status, "wpa_state=COMPLETED"))
		result = pdTRUE;

	vPortFree(status);

	return result;
}

/**
 *******************************************************************************
 * @brief Process to connect to an AP that stored in NVRAM.
 * return: 0 ; no error
 *******************************************************************************
 */
static int user_process_connect_ap(void)
{
	int	ret = 0;
	char value_str[128] = {0, };
	
//stage5	SET_PROCESS_BIT(processLists, USER_PROCESS_CONNECT_AP);
	PRINTF("\n**Neuralert: %s\n", __func__); // FRSDEBUG

	if (user_process_check_wifi_conn() == pdTRUE) {
		PRINTF("\n**Neuralert: user_process_connect_ap() already connected\n"); // FRSDEBUG

		/* Connection is already established */
//		user_connection_complete_event();
		return ret;
	}

	// use the internal command line interface to connect
	PRINTF("\n**Neuralert: user_process_connect_ap() attempting connection\n"); // FRSDEBUG
	ret = da16x_cli_reply("select_network 0", NULL, value_str);
	if (ret < 0 || strcmp(value_str, "FAIL") == 0) {
		PRINTF(" [%s] Failed connect to AP 0x%x\n", __func__, ret, value_str);
//stage5		CLR_PROCESS_BIT(processLists, USER_PROCESS_CONNECT_AP);
	}

	return ret;
}

/**
 *******************************************************************************
 * @brief Process for the RTC timer event
 *******************************************************************************
 */
static void user_process_timer_event(void)
{
	int ret;

	PRINTF("\n**Neuralert: user_process_timer_event\n"); // FRSDEBUG
#if 0
	/* RF control: ON */
	wifi_cs_rf_cntrl(FALSE);

	/* Connect to the AP */
	ret = user_process_connect_ap();
	if (ret) {
		PRINTF("%s: Failed to make connection to an AP\n", __func__);
		/* Go to sleep2 mode due to connection failed. */
		user_sleep_ready_event();
	}
#endif
}

/**
 *******************************************************************************
 * @brief Process for disabling the WIFI autoconnect feature in the SDK
 *******************************************************************************
 */
static int user_process_disable_auto_connection(void)
{
	int ret, netProfileUse;
	PRINTF("\n**Neuralert: %s\n", __func__); // FRSDEBUG

	/* To skip automatic network connection. It will be effected when boot-up. */
	ret = da16x_get_config_int(DA16X_CONF_INT_STA_PROF_DISABLED, &netProfileUse);
	if (ret != CC_SUCCESS || netProfileUse == pdFALSE) {
		ret = da16x_set_config_int(DA16X_CONF_INT_STA_PROF_DISABLED, pdTRUE);
		PRINTF("\n****** Disabling automatic network connection\n\n");
		/* It's just to give some delay before going to sleep mode.
		 *  it will be called only once.
		 */
		vTaskDelay(3);
	}

	return ret;
}

/**
 *******************************************************************************
 * @brief Process for initializing the "holding" area of the
 *  user log mechanism.  This is intended to allow early
 *  initialization before the flash area is initialized
 *  so that we can catch early information during initial
 *  device activation.
 *
 * Log entries are held in retention memory until they are
 * periodically transferred to flash.
 *
 * See document "Neuralert system logging design" for
 * details of this design
 *
 * returns pdTRUE if initialization succeeds
 * returns pdFALSE is a problem happens with the Flash initialization
 *******************************************************************************
 */
static void user_process_initialize_user_log_holding(void)
{
	// Initialize buffer pointers
	// Note that pointers are Indexes and not addresses
	// (i.e., start at 0 and increment by 1)

	// Holding area in retention memory
	pUserData->next_log_holding_position = 0;
	pUserData->oldest_log_holding_position = INVALID_AB_ADDRESS;
	// Set the area-initialized flag
	pUserData->user_holding_log_initialized = AB_MANAGEMENT_INITIALIZED;

}




/**
 *******************************************************************************
 * @brief Process for initializing the user log mechanism
 *  including erasing a sector of the external data Flash
 *
 *  NOTE - this should be called AFTER the accelerometer
 *  buffering (AB) area has been initialized since that
 *  does flash initialization.
 *
 * Log entries are held in retention memory until they are
 * periodically transferred to flash.
 *
 * Because the external flash memory is written with a "Page write"
 * command, each log entryh starts on a 256-byte page
 * boundary.
 *
 * Additionally, the flash must be erased before it can be written.
 * The smallest erase function is a 4096 byte sector (16 pages)
 *
 * See document "Neuralert system logging design" for
 * details of this design
 *
 * returns pdTRUE if initialization succeeds
 * returns pdFALSE is a problem happens with the Flash initialization
 *******************************************************************************
 */
static int user_process_initialize_user_log(void)
{
	int spi_status;
	int erase_status;
//	int unlock_status;
	int init_status = TRUE;
	UINT8 rx_data[3];
	ULONG SectorEraseAddr;
	HANDLE SPI = NULL;		// Local SPI bus handle for this activity

	PRINTF(">>>Initializing user log in flash\n");
	vTaskDelay(pdMS_TO_TICKS(100));

	pUserData->log_write_fault_count = 0;
	pUserData->log_write_retry_count = 0;
	pUserData->log_erase_attempts = 0;
	pUserData->log_erase_fault_count = 0;
	pUserData->log_erase_retry_count = 0;
	for (int i = 0; i < USERLOG_WRITE_MAX_ATTEMPTS; i++)
	{
		pUserData->log_write_attempt_events[i] = 0;
		pUserData->log_erase_attempt_events[i] = 0;
	}
	/*
	 * Initialize the SPI bus
	 */
//	spi_flash_config_pin();	// Hack to reconfigure the pin multiplexing

	// Get handle for the SPI bus
	SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\n\n********* Initialize user log SPI flash open error *********\n");
		init_status = FALSE;
	}

	// Do device initialization
//	spi_status = w25q64Init(SPI, rx_data);

//	if (!spi_status)
//	{
//		PRINTF("\n\n********* SPI initalization error *********\n");
//		init_status = FALSE;
//	}

	// Initialize buffer pointers
	// Note that pointers are Indexes and not addresses
	// (i.e., start at 0 and increment by 1)

	// Holding area in retention memory
	// Note done in separate function
//	pUserData->next_log_holding_position = 0;
//	pUserData->oldest_log_holding_position = INVALID_AB_ADDRESS;

	// Next location to write IN FLASH
	pUserData->next_log_entry_write_position = 0;

	// Set the oldest log entry to an invalid value until
	// something is written to the log
	pUserData->oldest_log_entry_position = INVALID_AB_ADDRESS;

	// Erase the first sector where the first 16 log entries will be
	SectorEraseAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)pUserData->next_log_entry_write_position);
	Printf("\n  Erasing first log sector. Location: %x \n",SectorEraseAddr);
//	Printf("  Erasing Chip  \n");

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("======= about to erase log sector");
//	printf_with_run_time("======= about to erase chip");
#endif

	erase_status = eraseSector_4K(SPI, SectorEraseAddr);
//	erase_status = eraseChip(SPI);
//	vTaskDelay(pdMS_TO_TICKS(500));

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("======= done erasing log sector");
//	printf_with_run_time("======= done erasing chip");
#endif

	if(!erase_status)
	{
		PRINTF("\n\n********* User log SPI erase error *********\n");
		init_status = FALSE;
	}

	if (init_status == TRUE)
	{
		// Set the area-initialized flag
		pUserData->user_log_initialized_flag = AB_MANAGEMENT_INITIALIZED;
	}
	else
	{
		// Clear the area-initialized flag
		pUserData->user_log_initialized_flag = 0;

	}

	spi_status = spi_flash_close(SPI);
	return init_status;

}


#if 0
void AB_view_hex(char* _data, int32_t _length)
{
	for (int j=0; j < _length; j++)
	{
		PRINTF("0x%02x,", _data[j] & 0xFF);
	}
	PRINTF("\n");
}


// dump some data from flash
// You must supply the SPI handle and be in a session
static void AB_dump_data(HANDLE SPI, ULONG dumpaddr, int dumplength)
{
	int spi_status;
	UINT8 bytes[256];
	if (dumplength > 256)
	{
		PRINTF(" AB_dump_page max length 256 exceeded\n");
		return;
	}
	// Read data from flash
	spi_status = pageRead(SPI, dumpaddr, bytes, (UINT32)dumplength);

	if(spi_status < 0)
	{
		PRINTF("  AB_dump_page read error\n");
	}
	else
	{
		AB_view_hex(bytes, dumplength);
	}

}
#endif

#if 0
/**
 *******************************************************************************
 * @brief Dump the Winbond Flash status registers (diagnostic aid)
 *
 *******************************************************************************
 */
static void W25Q64_display_status_registers(HANDLE handler, char *text)
{
	UINT8 statreg1, statreg2, statreg3;  // for reading Winbond status registers
	int spi_status;

	statreg1 = 0xFF;
	statreg2 = 0xFF;
	statreg3 = 0xFF;
	spi_status = readStatReg1(handler, &statreg1);
	if(spi_status < 0)
	{
			PRINTF("  Error reading status register 1\n"); //Fault error indication here
	}
	spi_status = readStatReg2(handler, &statreg2);
	if(spi_status < 0)
	{
			PRINTF("  Error reading status register 2\n"); //Fault error indication here
	}
	spi_status = readStatReg3(handler, &statreg3);
	if(spi_status < 0)
	{
			PRINTF("  Error reading status register 3\n"); //Fault error indication here
	}

	PRINTF(">>>>> [%s] Status registers: %02X %02X %02X\n",
			text,
			statreg1, statreg2, statreg3);

}
#endif

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
 * returns pdTRUE if initialization succeeds
 * returns pdFALSE is a problem happens with the Flash initialization
 *******************************************************************************
 */
static int user_process_initialize_AB(void)
{
	int spi_status;
	int erase_status;
//	int unlock_status;
	int init_status = TRUE;
	UINT8 rx_data[3];
	ULONG SectorEraseAddr;
	HANDLE SPI = NULL;
	int i;

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
//	spi_flash_config_pin();	// Hack to reconfigure the pin multiplexing

	// Get handle for the SPI bus
	SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\n\n********* SPI initalization error *********\n");
		init_status = FALSE;
	}
//	vTaskDelay(1);

	// Do device initialization
	spi_status = w25q64Init(SPI, rx_data);

	if (!spi_status)
	{
		PRINTF("\n\n********* SPI initalization error *********\n");
		init_status = FALSE;
	}

	// Initialize accelerometer buffering pointers
	// Note that pointers are Indexes and not addresses
	// (i.e., start at 0 and increment by 1)

	// Next location to write
	pUserData->next_AB_write_position = 0;

	// Set the next transmit position to an invalid value until
	// there is something to transmit
	pUserData->next_AB_transmit_position = INVALID_AB_ADDRESS;

	// Erase the first sector where the first data will be written
	SectorEraseAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)pUserData->next_AB_write_position);
	PRINTF("  Erasing first Sector Location: %x \n",SectorEraseAddr);
//	Printf("  Erasing Chip  \n");

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("======= about to erase sector");
//	printf_with_run_time("======= about to erase chip");
#endif

	erase_status = eraseSector_4K(SPI, SectorEraseAddr);
//	erase_status = eraseChip(SPI);
//	vTaskDelay(pdMS_TO_TICKS(500));

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("======= done erasing sector");
//	printf_with_run_time("======= done erasing chip");
#endif

	if(!erase_status)
	{
		PRINTF("\n\n********* SPI erase error *********\n");
		init_status = FALSE;
	}

	if (init_status == TRUE)
	{
		// Set the area-initialized flag
		pUserData->AB_initialized_flag = AB_MANAGEMENT_INITIALIZED;
	}
	else
	{
		// Clear the area-initialized flag
		// This should prevent MQTT from trying to use it
		pUserData->AB_initialized_flag = 0;

	}

	spi_flash_close(SPI);

	return init_status;

}


/**
 *******************************************************************************
 * @brief Process for clearing the accelerometer data buffering
 *  mechanism, including erasing the entire acceleromater data
 *  previously accumulated
 *
 *  Although only intended for device shutdown, this function leaves
 *  the AB area in a pristine state that could be used for a new
 *  cycle.
 *
 *  This function assumes that no other task is attempting to
 *  access external flash and so does not use the semaphores to
 *  gain exclusive access.
 *
 *  Note that we do NOT erase the entire flash chip because we don't
 *  want to disturb the system log.
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
 * returns pdTRUE if initialization succeeds
 * returns pdFALSE is a problem happens with the Flash initialization
 *******************************************************************************
 */
static int user_process_clear_AB(void)
{
	int spi_status;
	int erase_status;
	int clear_status = TRUE;		// our function return
	UINT8 rx_data[3];
	ULONG SectorEraseAddr;
	HANDLE SPIhandle;
	AB_INDEX_TYPE next_AB_clear_position;
	int max_sectors;
	int sectors_erased = 0;

	/*
	 * Initialize the SPI bus and the Winbond external flash
	 */
//	spi_flash_config_pin();	// Hack to reconfigure the pin multiplexing

	// Get our own handle for the SPI bus
	SPIhandle = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPIhandle == NULL)
	{
		PRINTF("\n\n********* user_process_clear_AB: unable to obtain SPI handle *********\n");
		clear_status = FALSE;
	}
//	vTaskDelay(1);

	// Do device initialization
	spi_status = w25q64Init(SPIhandle, rx_data);

	if (!spi_status)
	{
		PRINTF("\n\n********* user_process_clear_AB: SPI initalization error *********\n");
		clear_status = FALSE;
	}

	// Reset the accelerometer buffering pointers
	// Note that pointers are Indexes and not addresses
	// (i.e., start at 0 and increment by 1)

	// Next location to write
	pUserData->next_AB_write_position = 0;

	// Set the next transmit position to an invalid value
	pUserData->next_AB_transmit_position = INVALID_AB_ADDRESS;


#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("== Starting to clear data buffering area");
#endif

	// As of 9/29/22, there are 3888 pages
	// 3888 pages / 16 sectors per page = 243 4k sectors
	max_sectors = AB_FLASH_MAX_PAGES / 16;
	sprintf(user_log_string_temp, "**** Shutting down - erasing %d sectors", max_sectors);
	user_log_event(user_log_string_temp);
	for(	next_AB_clear_position = 0;
			next_AB_clear_position < max_sectors;
			next_AB_clear_position++)
	{
		sectors_erased++;
		// Calculate sector address
		SectorEraseAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_SECTOR_SIZE * (ULONG)next_AB_clear_position);
		PRINTF("  Erasing sector %d: %x \n",sectors_erased, SectorEraseAddr);

		// observed times for erasing are about 40-50 msec
		erase_status = eraseSector_4K(SPIhandle, SectorEraseAddr);
//		vTaskDelay(pdMS_TO_TICKS(50));
		if(!erase_status)
		{
			PRINTF("\n********* user_process_clear_AB: SPI erase error *********\n");
			clear_status = FALSE;
		}
	} // FOR LOOP
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("== Finished clearing data buffering area");
#endif
	user_log_event("Finished clearing data buffering area");
	spi_status = spi_flash_close(SPIhandle);
	return clear_status;

}

/**
 *******************************************************************************
 * @brief Process to retrieve the user log buffer management
 * next-write location, making sure it's done with exclusive access
 *  Returns -1 if unable to gain exclusive access
 *  returns the log next write position otherwise
 *
 *******************************************************************************
 */
static int get_log_store_location(void)
{
	int return_value = -1;

	// Figure out what is stored in the flash log area
	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
				shared resource. */

			// Where our entry will go
			return_value = pUserData->next_log_entry_write_position;

			/* We have finished accessing the shared resource.  Release the
				semaphore. */
			xSemaphoreGive( user_log_semaphore );
		}
		else
		{
			PRINTF("\n ***archive_one_log_entry: Unable to obtain log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***archive_one_log_entry: semaphore not initialized!\n");
	}

	return return_value;
}

/**
 *******************************************************************************
 * @brief Process to retrieve the flash user log oldest location,
 * making sure it's done with exclusive access
 *  Returns -1 if unable to gain exclusive access
 *  returns the holding log oldest position otherwise
 *
 *******************************************************************************
 */
static int get_log_oldest_location(void)
{
	int return_value = -1;

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */
			return_value = pUserData->oldest_log_entry_position;

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( user_log_semaphore );
		}
		else
		{
			PRINTF("\n ***Unable to obtain user log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***user log semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to retrieve the user holding log next write location,
 * making sure it's done with exclusive access
 *  Returns -1 if unable to gain exclusive access
 *  returns the log next write position otherwise
 *
 *******************************************************************************
 */
static int get_holding_log_next_write_location(void)
{
	int return_value = -1;

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */
			return_value = pUserData->next_log_holding_position;

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( user_log_semaphore );
		}
		else
		{
			PRINTF("\n ***Unable to obtain user log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***user log semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to retrieve the user holding log oldest location,
 * making sure it's done with exclusive access
 *  Returns -1 if unable to gain exclusive access
 *  returns the holding log oldest position otherwise
 *
 *******************************************************************************
 */
static int get_holding_log_oldest_location(void)
{
	int return_value = -1;

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */
			return_value = pUserData->oldest_log_holding_position;

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( user_log_semaphore );
		}
		else
		{
			PRINTF("\n ***Unable to obtain user log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***user log semaphore not initialized!\n");
	}

	return return_value;

}
/**
 *******************************************************************************
 * @brief Process to update the user holding log management
 * oldest location, making sure it's done with exclusive access
 *
 *  Returns FALSE if unable to gain exclusive access
 *  returns TRUE otherwise
 *******************************************************************************
 */
static int update_holding_log_oldest_location(int new_location)
{
	int return_value = pdFALSE;

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
//			PRINTF("****** update_holding_log_oldest_location: new value %d ******\n",
//					new_location);
			/* We were able to obtain the semaphore and can now access the
				shared resource. */
			pUserData->oldest_log_holding_position = new_location;;

			/* We have finished accessing the shared resource.  Release the
				semaphore. */
			xSemaphoreGive( user_log_semaphore );
			return_value = pdTRUE;
		}
		else
		{
			PRINTF("\n ***Unable to obtain user log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***user log semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to update the user log management
 * oldest location, making sure it's done with exclusive access
 *
 *  Returns FALSE if unable to gain exclusive access
 *  returns TRUE otherwise
 *******************************************************************************
 */
static int update_log_oldest_location(int new_location)
{
	int return_value = pdFALSE;

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			PRINTF("****** update_log_oldest_location: new value %d ******\n", new_location);
			/* We were able to obtain the semaphore and can now access the
				shared resource. */
			pUserData->oldest_log_entry_position = new_location;;

			/* We have finished accessing the shared resource.  Release the
				semaphore. */
			xSemaphoreGive( user_log_semaphore );
			return_value = pdTRUE;
		}
		else
		{
			PRINTF("\n ***Unable to obtain user log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***user log semaphore not initialized!\n");
	}

	return return_value;

}
/**
 *******************************************************************************
 * @brief Process to update the user log management
 * next write (store) location, making sure it's done with exclusive access
 *
 *  Returns FALSE if unable to gain exclusive access
 *  returns TRUE otherwise
 *******************************************************************************
 */
static int update_log_store_location(int new_location)
{
	int return_value = pdFALSE;

	if(user_log_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( user_log_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
//			PRINTF("****** update_log_store_location: new value %d ******\n", new_location);
			/* We were able to obtain the semaphore and can now access the
				shared resource. */
			pUserData->next_log_entry_write_position = new_location;

			/* We have finished accessing the shared resource.  Release the
				semaphore. */
			xSemaphoreGive( user_log_semaphore );
			return_value = pdTRUE;
		}
		else
		{
			PRINTF("\n ***Unable to obtain user log semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***user log semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to retrieve the accelerometer buffer management
 * next-write location, making sure it's done with exclusive access
 *  Returns -1 if unable to gain exclusive access
 *  returns the AB next write position otherwise
 *
 *******************************************************************************
 */
static int get_AB_store_location(void)
{
	int return_value = -1;

	if(AB_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( AB_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */
			return_value = pUserData->next_AB_write_position;

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( AB_semaphore );
		}
		else
		{
			PRINTF("\n ***Unable to obtain AB semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***AB semaphore not initialized!\n");

	}

	return return_value;

}
/**
 *******************************************************************************
 * @brief Process to retrieve the accelerometer buffer management
 * next-transmit location, making sure it's done with exclusive access
 *  Returns -1 if unable to gain exclusie access
 *  returns the AB next write position otherwise
 *
 *******************************************************************************
 */
static int get_AB_transmit_location(void)
{
	int return_value = -1;

	if(AB_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( AB_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			/* We were able to obtain the semaphore and can now access the
	            shared resource. */
			return_value = pUserData->next_AB_transmit_position;

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( AB_semaphore );
		}
		else
		{
			PRINTF("\n ***Unable to obtain AB semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***AB semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to update the accelerometer buffer management
 * next-transmit location, making sure it's done with exclusive access
 *  Returns FALSE if unable to gain exclusive access
 *  returns TRUE otherwise
 *******************************************************************************
 */
static int update_AB_transmit_location(int new_location)
{
	int return_value = pdFALSE;

	// NOTE! this doesn't check to see if the MQTT task is running or
	// not.  It is expected that this function will be called under
	// the following circumstances:
	//   1. The first read by the AXL task when it has to set the first
	//      transmit.  MQTT should not be running then.
	//   2. The MQTT task when it has finished transmitting and needs
	//      to set up for the next transmit.  AXL should not have to
	//      be updating at that  point.
	//   3. (TBD) If the AXL task catches up to the MQTT task because
	//      the MQTT task has been unable to transmit
	if(AB_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( AB_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
	//		PRINTF("****** update_AB_transmit_location: new value %d ******\n", new_location);
			/* We were able to obtain the semaphore and can now access the
				shared resource. */
			pUserData->next_AB_transmit_position = new_location;

			/* We have finished accessing the shared resource.  Release the
				semaphore. */
			xSemaphoreGive( AB_semaphore );
			return_value = pdTRUE;
		}
		else
		{
			PRINTF("\n ***Unable to obtain AB semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***AB semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to update the accelerometer buffer management
 * next-write location, making sure it's done with exclusive access
 *  Returns FALSE if unable to gain exclusive access
 *  returns TRUE otherwise
 *
 *******************************************************************************
 */
static int update_AB_write_location(int new_location)
{
	int return_value = pdFALSE;

	// NOTE! this doesn't check to see if the MQTT task is running or
	// not.  It is expected that this function will be called under
	// the following circumstances:
	//   1. MQTT task is not running and we've just read from the AXL
	//   2. MQTT task is running but is transmitting some data that
	//      we recorded recently but not including what we've just
	//      recorded
	//   3. MQTT task is running but is transmitting old data that
	//      we recorded a while ago.  This is likely when we lose
	//      WIFI for a while
	//    4. (TBD) MQTT task is transmitting data where we plan to
	//       write or erase.  VERY IMPORTANT THAT WE DON"T LET THIS HAPPEWN

	if(AB_semaphore != NULL )
	{
		/* See if we can obtain the semaphore.  If the semaphore is not
			available wait 10 ticks to see if it becomes free. */
		if( xSemaphoreTake( AB_semaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
//			PRINTF("===update_AB_write_location: new value %d\n", new_location);
			/* We were able to obtain the semaphore and can now access the
				shared resource. */
			pUserData->next_AB_write_position = new_location;
			/* We were able to obtain the semaphore and can now access the
			    shared resource. */

			/* We have finished accessing the shared resource.  Release the
				semaphore. */
			xSemaphoreGive( AB_semaphore );
			return_value = pdTRUE;
		}
		else
		{
			PRINTF("\n ***Unable to obtain AB semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***AB semaphore not initialized!\n");
	}

	return return_value;

}
#if 0
/**
 *******************************************************************************
 * @brief Helper function to return TRUE if an array of bytes is all zeroes
 *  Returns pdFALSE any byte is non-zero
 *  Returns pdTRUE if all "len" bytes are zero
 *******************************************************************************
 */
static int zerobytes(UCHAR *bytes, int len)
{
	int i;
	for (i=0; i<len; i++)
	{
		if(bytes[i] != 0x00)
			return pdFALSE;
	}
	return pdTRUE;
}
#endif

/**
 *******************************************************************************
 * @brief Process to retrieve one block of data from a flash page
 *  Returns pdFALSE if unable to read
 *  Returns pdTRUE and the returned data if able to read
 *******************************************************************************
 */
static int flash_read_page_data(HANDLE SPI, UINT32 pageaddress, UCHAR *Pagedata, int Numbytes)
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
			// Now read the block
			spi_status = pageRead(SPI, pageaddress, (UINT8 *)Pagedata, Numbytes);

			if(spi_status < 0){
				Printf("  ***** flash_read_page_data error reading loc 0x%x\n", pageaddress);
				return_value = pdFALSE;
			}
			else
			{
				return_value = pdTRUE;
			}

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( Flash_semaphore );
		}
		else
		{
			Printf("\n ***flash_read_page_data: Unable to obtain Flash semaphore\n");
		}
	}
	else
	{
		Printf("\n ***flash_read_page_data: semaphore not initialized!\n");
	}

	return return_value;

}



/**
 *******************************************************************************
 * @brief Process to retrieve one block from the accelerometer buffer
 * memory (flash)
 *  Returns pdFALSE if unable to read
 *  Returns pdTRUE and the FIFO buffer if able to read
 *******************************************************************************
 */
static int AB_read_block(HANDLE SPI, UINT32 blockaddress, accelBufferStruct *FIFOdata)
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
			// Now read the block
			spi_status = pageRead(SPI, blockaddress, (UINT8 *)FIFOdata, sizeof(accelBufferStruct));

			if(spi_status < 0){
				PRINTF("  ***** AB_read_block error reading block 0x%x\n", blockaddress);
				return_value = pdFALSE;
			}
			else
			{
				return_value = pdTRUE;
			}

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( Flash_semaphore );
		}
		else
		{
			PRINTF("\n ***AB_read_block: Unable to obtain Flash semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***AB_read_block: semaphore not initialized!\n");
	}

	return return_value;

}

/**
 *******************************************************************************
 * @brief Process to write data to one page of the external data flash memory
 *
 *  Returns pdFALSE if unable to write
 *  Returns pdTRUE if able to write
 *******************************************************************************
 */
static int flash_write_block(HANDLE SPI, int blockaddress, UCHAR *pagedata, int num_bytes)
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
			spi_status = pageWrite(SPI, blockaddress, (UINT8 *)pagedata, num_bytes);
			if(spi_status < 0)
			{
				Printf(" **flash_write_block: Flash Write error\n"); //Fault error indication here
				return_value = pdFALSE;
			}
			else
			{
//				Printf(" **flash_write_block: Flash Write successful\n");
				return_value = pdTRUE;
			}

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( Flash_semaphore );
		}
		else
		{
			Printf("\n ***flash_write_block: Unable to obtain Flash semaphore\n");
		}
	}
	else
	{
		Printf("\n ***flash_write_block: semaphore not initialized!\n");
	}

	return return_value;

}

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
				PRINTF(" **AB_write_block: Flash Write error\n"); //Fault error indication here
				return_value = pdFALSE;
			}
			else
			{
//				Printf(" **AB_write_block: Flash Write successful\n");
				return_value = pdTRUE;
			}

			/* We have finished accessing the shared resource.  Release the
	            semaphore. */
			xSemaphoreGive( Flash_semaphore );
		}
		else
		{
			PRINTF("\n ***AB_write_block: Unable to obtain Flash semaphore\n");
		}
	}
	else
	{
		PRINTF("\n ***AB_write_block: semaphore not initialized!\n");
	}

	return return_value;

}
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

//	ULONG SectorEraseAddr;
	int spi_status;
	int rerunCount;
	int faultFlag = 1;
	int erase_status;
	UINT32 erase_mismatch_count;
//	accelBufferStruct checkFIFO;	// copy for readback check
	UCHAR FIFObytes[256];				// pointer used to access bytes of fifo
//	int write_index;
	int retry_count;
	int fault_happened;
	HANDLE MYSPI;   // experiment to try having own handle
	int erase_confirmed;

	// Our return status starts at ok until a problem occurs
	erase_status = TRUE;

	PRINTF("-------------------------------\n");
	PRINTF(" Erase sector location: %x\n", SectorEraseAddr);
	PRINTF("-------------------------------\n");

	fault_happened = 1;
	retry_count = 0;
	erase_mismatch_count = 0;
	pUserData->erase_attempts++;  // total sectors we tried to erase

	// Note - as of 9/10/22 the erase sector was taking about 50 milliseconds
	// We have to be careful not to overrun the accelerometer interrupt here
	// but if it takes 3 or 4 tries, that's still only a few hundred
	// milliseconds out of a 2+ second budget
	erase_confirmed = pdFALSE;
	for(rerunCount = 0; (rerunCount < AB_ERASE_MAX_ATTEMPTS) & !erase_confirmed;
				rerunCount++)
	{
		retry_count++;
		// Hack to make sure our gpio pin mux mapping is ok
		// (during development it seemed to go awry sometimes)
//		spi_flash_config_pin();

#if 0
		// Get our own handle to the SPI bus
		MYSPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
		if (MYSPI == NULL)
		{
			Printf("\n***ERASE SECTOR MAJOR SPI ERROR: Unable to open SPI bus handle\n\n");
			fault_happened = 1;
		}
//		else
//		{
#endif

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("======= about to erase sector");
#endif

		erase_status = eraseSector_4K(SPI, (UINT32)SectorEraseAddr);

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
		printf_with_run_time("======= done erasing sector");
#endif
		if(!erase_status)
		{
			PRINTF("\n\n********* eraseSector_4K returned error *********\n");
		}
//		else
//		{
//			// Now read some bytes to make sure it's erased
//			// Flash erase sets bits to all hex FF
//			// Since we have the handy read block function, use that
//			//
//			// This delay is some voodoo added to see why we're
//			// reading zeroes back all the time but getting it on the
//			// second attempt
//			vTaskDelay(pdMS_TO_TICKS(10));
//
//			// put something recognizable here (DEBUG)
//			for(int i = 0; i <= 10; i++)
//			{
//				FIFObytes[i] = 0xF0 + i;
//			}
#if 1
		// Get our own handle to the SPI bus
//		MYSPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
//		if (MYSPI == NULL)
//		{
//			Printf("\n***ERASE SECTOR MAJOR SPI ERROR: Unable to open SPI bus handle\n\n");
//			fault_happened = 1;
//		}
//		else
//		{
#endif

			if (pdFALSE == AB_read_block(SPI, (UINT32)SectorEraseAddr, (accelBufferStruct *)FIFObytes))
			{
				PRINTF("  Flash readback error: %x\n", SectorEraseAddr); //Fault error indication here
				erase_status = FALSE;
			}
//			spi_flash_close(MYSPI);


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
//				if( rerunCount != 2)
//				spi_flash_close(MYSPI);
			}
			else
			{
				PRINTF(" Flash erase & verification successful\n");
				faultFlag = 0;
//				spi_flash_close(MYSPI);
				erase_confirmed = pdTRUE;
				break;
			} // erase 4k returned ok status
	} // for rerunCount < max retries

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


end_of_task:

//	spi_flash_close(SPI);  // See comments about SPI closing above

	return erase_status;
}


/**
 *******************************************************************************
 * @brief Process for writing one log entry to external data Flash
 * One log entry structure is written to the next available
 * flash location.
 * See document "Neuralert system logging design" for
 * details of this design
 *
 *******************************************************************************
 */
static int user_write_log_to_flash(USERLOG_ENTRY *pLogData, int *did_an_erase)
{

	ULONG NextWriteAddr;
	ULONG SectorEraseAddr;
	int spi_status, rerunCount, faultFlag = 1;
	UINT8 *reg;
	int erase_status;
	int write_status;
	UINT32 i2c_status;
	UINT32 write_fail_count;
	USERLOG_ENTRY checkEntry;	// copy for readback check
	int write_index;
	int oldest_log_entry;
	int next_write_location;
	int retry_count;
	int fault_happened;
	HANDLE SPI = NULL;

	// Our return status starts at ok until a problem occurs
	write_status = TRUE;

	// Set erase happened status for user in case we exit early
	*did_an_erase = pdFALSE;

	// Retried the index of the next place to write a
	// block.  Starts at 0 and wraps around when we reach the end of the
	// log region in flash
	// Upon entry to this function it should always point to a valid
	// write location
	// Writes are done to 256-byte pages and so are on addresses that
	// are multiples of 0x100
	write_index = get_log_store_location();
	if (write_index < 0)
	{
		PRINTF("\n Unable to get flash log store location\n");
		return FALSE;
	}

	// Remember how many entries we wrote to flash
	pUserData->total_log_entries++;

	// Calculate address of next sector to write
	NextWriteAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
			((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)write_index);
//	PRINTF("----------------------------------------\n");
//	PRINTF(" Next flash log entry to write: %d\n",write_index);
//	PRINTF(" Flash Write ADDR                : 0x%X\r\n", NextWriteAddr);
	PRINTF(" Writing flash log entry: %d [0x%X]\n",write_index, NextWriteAddr);
//	PRINTF("----------------------------------------\n");

	fault_happened = 1;
	retry_count = 0;

	write_fail_count = 0;

	for(rerunCount = 0; rerunCount < USERLOG_WRITE_MAX_ATTEMPTS; rerunCount++)
	{
		retry_count++;

		// Note - the logic of the SPI access is that this log function
		// will only be called when no other user activity is happening.
		// I.e., it is designed to be activated after the accelerometer
		// code has wakened from sleep2, written data to flash, NOT
		// erased flash, and NOT started the MQTT task.
		// So we are guaranteed to be in a quiescent period

		// Get a handle to the SPI bus
		SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
		if (SPI == NULL)
		{
			PRINTF("\n***MAJOR SPI ERROR: Unable to open SPI bus handle\n\n");
			fault_happened = 1;
		}
		else
		{
//			PRINTF(" user_process_write_to_flash (1): SPI handle: %x\n", SPI);

			if(!flash_write_block(SPI, NextWriteAddr, (UCHAR *)pLogData, sizeof(USERLOG_ENTRY)))
			{
				PRINTF("  Flash Write error %x\n", NextWriteAddr); //Fault error indication here
			}
			else
			{
//				PRINTF("  Flash Write successful\n");
			}
//			vTaskDelay(1);

			// Now read it back and see if it's the same
			if (pdFALSE == flash_read_page_data(SPI, NextWriteAddr, (UCHAR *)&checkEntry, sizeof(USERLOG_ENTRY)))
			{
				PRINTF("  Flash readback error: %x\n", NextWriteAddr); //Fault error indication here
				write_status = FALSE;
			}

			// Read back what we just wrote and make sure it took
			// Note - as of 8/20/22, the WIFI and MQTT activity in other
			// tasks seems to interfere with the SPI bus.  As of this
			// writing it seems that we catch the mistake and correct it
			// on the first retry.
			// Further note - as of 10/26/22 Renesas supplied some patches
			// for SPI operation that seem to make SPI operation reliable
			if(pLogData->user_log_timestamp != checkEntry.user_log_timestamp)
			{
				PRINTF("  MISMATCHED TIMESTAMPS: %d vs %d\n", pLogData->user_log_timestamp,
						checkEntry.user_log_timestamp);
				write_fail_count++;
			} // for each sample in FIFO compare buffer

			if(write_fail_count > 0)
			{
				PRINTF("SPI FLASH ERROR COUNT: %d\n\n", write_fail_count);
				faultFlag = 1;
				write_fail_count = 0;
				vTaskDelay(pdMS_TO_TICKS(30));
				if( rerunCount != (USERLOG_WRITE_MAX_ATTEMPTS - 1))
					spi_flash_close(SPI);
			}
			else
			{
//				PRINTF(" Log flash write & verification successful\n");
				faultFlag = 0;
				break;
			}
		} // SPI handle not null
	} // for rerunCount < max retries

	if(faultFlag != 0)
	{
		pUserData->log_write_fault_count++;  // total write failures since power on
	}
	else if(retry_count > 1)
	{
		pUserData->log_write_retry_count++;	 // total # of times retry worked
	}
	// Log how many times we succeeded on each attempt count; [0] is first try, etc.
	pUserData->log_write_attempt_events[retry_count-1]++;


// Note that the SPI handle is not closed after the last attempt
// This is done later to allow more time per Nick
//	PRINTF(" user_process_write_to_flash (2): SPI handle: %x\n", SPI);

	// If we had a write failure, we skip the pointer update

	if(faultFlag != 0)
	{
		APRINTF_E("\n***** LOG WRITE FAILURE - SKIPPING POINTER UPDATE *****\n\n");
		goto end_of_task;
	}

	// If the oldest entry pointer is not set yet,
	// then set it to the place we just wrote
	// (should be first location in this case)
	oldest_log_entry = get_log_oldest_location();
//	PRINTF(" Oldest flash log entry       : %d\n",oldest_log_entry);
	if (oldest_log_entry == INVALID_AB_ADDRESS)
	{
		if(!update_log_oldest_location(write_index))
		{
			PRINTF("\n Unable to set log oldest location\n");
			goto end_of_task;
		}
		else
		{
			PRINTF(" ****** Setting oldest flash location %d ******\n",write_index);
		}

	}
	// Just some safeguard code that should not happen
	else if ((oldest_log_entry < 0)
			|| (oldest_log_entry >= USERLOG_FLASH_MAX_PAGES))
	{
		PRINTF("\n ***** error oldest flash log location invalid %d - resetting ****\n", oldest_log_entry);
		if(!update_log_oldest_location(write_index))
		{
			PRINTF("\n Unable to set log oldest location\n");
			goto end_of_task;
		}
	}

	// Set up for the next write
	write_index++;
	if(write_index >= USERLOG_FLASH_MAX_PAGES)
	{
		PRINTF(" Last log address reached. %d Resetting to zero\n",
				write_index);
		write_index = 0;
	}
	if(!update_log_store_location(write_index))
	{
		PRINTF("\n Unable to set log next write location\n");
//			goto end_of_task;
	}
	else
	{
//		PRINTF(" === Writing to flash next write location updated: %d\n", write_index);
	}

	//	vTaskDelay(pdMS_TO_TICKS(50));

	// Check to see if we've caught up with the oldest location
	// This is not intended to happen during a typical five-day
	// device operation.
	if (write_index == oldest_log_entry)
	{
		PRINTF("\n ***** User log full!  Discarding oldest %d ***\n", oldest_log_entry);
		oldest_log_entry++;
		if(oldest_log_entry >= USERLOG_FLASH_MAX_PAGES)
		{
			oldest_log_entry = 0;
		}

		if(!update_log_oldest_location(oldest_log_entry))
		{
			PRINTF("\n Unable to set log oldest location\n");
		}
	}


	// If the next page we're going to write to is the start of a
	// new sector, we have to erase it to be ready

	if(	((write_index % AB_PAGES_PER_SECTOR ) == 0)
		|| (write_index == 0))
	{
		*did_an_erase = pdTRUE;
		// Calculate address of next sector to write
		SectorEraseAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)write_index);
		PRINTF("  Log sector filled. Location: %x Erasing next sector\n",
					SectorEraseAddr);

		pUserData->log_erase_attempts++;

		erase_status = user_erase_flash_sector(SPI, SectorEraseAddr);
#if 0
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("======= about to erase sector");
#endif

		erase_status = eraseSector_4K(SPI, SectorEraseAddr);

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("======= done erasing sector");
#endif
#endif
		if(!erase_status)
		{
			PRINTF("\n\n********* Sector erase error *********\n");
			write_status = FALSE;
		}
	} // if need to erase next sector

end_of_task:
	spi_flash_close(SPI);  // See comments about SPI closing above
	return write_status;
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
	int spi_status, rerunCount, faultFlag = 1;
	UINT8 *reg;
	int erase_status;
	int write_status;
	UINT32 i2c_status;
	UINT32 write_fail_count;
	accelBufferStruct checkFIFO;	// copy for readback check
	int write_index;
	int transmit_index;
	int retry_count;
	int fault_happened;
	HANDLE SPI = NULL;

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("======= writing FIFO buff to flash");
#endif

	// Our return status starts at ok until a problem occurs
	write_status = TRUE;

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
	write_index = get_AB_store_location();
	if (write_index < 0)
	{
		user_log_error("** Unable to get AB store location **");
		return FALSE;
	}
//	Printf("==Next AB store location: %d\n", write_index);

	// Calculate address of next sector to write
	NextWriteAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
			((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)write_index);
	PRINTF("-------------------------------\n");
	PRINTF(" Next location to write: %d\n",write_index);
	PRINTF(" Flash Write ADDR: 0x%X\r\n", NextWriteAddr);
	PRINTF(" Data sequence # : %d\n", pFIFOdata->data_sequence);
	PRINTF(" Number samples  : %d\n", pFIFOdata->num_samples);
	PRINTF(" Timestamp sample: %d\n", pFIFOdata->timestamp_sample);
	PRINTF("-------------------------------\n");

	fault_happened = 1;
	retry_count = 0;

	SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		user_log_error("***MAJOR SPI ERROR: Unable to open SPI bus handle");
		fault_happened = 1;
	}

	for(rerunCount = 0; rerunCount < AB_WRITE_MAX_ATTEMPTS; rerunCount++)
	{
		retry_count++;
		// Hack to make sure our gpio pin mux mapping is ok
		// (during development it seemed to go awry sometimes)
//		spi_flash_config_pin();

		// Get a handle to the SPI bus (should we keep the original handle?)
//		SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
//		if (SPI == NULL)
//		{
//			Printf("\n***MAJOR SPI ERROR: Unable to open SPI bus handle\n\n");
//			fault_happened = 1;
//		}
//		else
		{
//			PRINTF(" user_process_write_to_flash (1): SPI handle: %x\n", SPI);

			if(!AB_write_block(SPI, NextWriteAddr, pFIFOdata))
			{
				sprintf(user_log_string_temp, "** Flash Write error %x", NextWriteAddr); //Fault error indication here
				user_log_error(user_log_string_temp);
			}
			else
			{
//				PRINTF("  Flash Write successful\n");
			}
//			vTaskDelay(1);
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("======= done writing FIFO to flash");
#endif

			// Now read it back and see if it's the same
			if (pdFALSE == AB_read_block(SPI, NextWriteAddr, &checkFIFO))
			{
				PRINTF("  Flash readback error: %x\n", NextWriteAddr); //Fault error indication here
				write_status = FALSE;
			}

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
						PRINTF("RECEIVED: %d     X: %d Y: %d Z: %d\r\n", i, receivedFIFO.Xvalue[i], receivedFIFO.Yvalue[i], receivedFIFO.Zvalue[i]);
						PRINTF("CHECK:    %d     X: %d Y: %d Z: %d\r\n", i, checkFIFO.Xvalue[i], checkFIFO.Yvalue[i], checkFIFO.Zvalue[i]);
						write_fail_count++;
					}
			} // for each sample in FIFO compare buffer

			if(write_fail_count > 0)
			{
				PRINTF("SPI FLASH ERROR COUNT: %d\n\n", write_fail_count);
				faultFlag = 1;
				write_fail_count = 0;
				vTaskDelay(pdMS_TO_TICKS(30));
//				if( rerunCount != (AB_WRITE_MAX_ATTEMPTS-1))
//					spi_flash_close(SPI);
			}
			else
			{
//				PRINTF(" Flash write & verification successful\n");
				faultFlag = 0;
				break;
			}
		} // SPI handle not null
	} // for rerunCount < max retries

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


// Note that the SPI handle is not closed after the last attempt
// This is done up the caller tree to allow more time per Nick
//	PRINTF(" user_process_write_to_flash (2): SPI handle: %x\n", SPI);

	// Note if we had a write failure, we should skip the following update

	if(faultFlag != 0)
	{
		user_log_error("***** FIFO DATA WRITE FAILURE - SKIPPING POINTER UPDATE *****");
		goto end_of_task;
	}

	// If the MQTT transmit pointer is not set yet,
	// then set it to the place we just wrote
	// (should be first location in this case)
	transmit_index = get_AB_transmit_location();
	if (transmit_index == INVALID_AB_ADDRESS)
	{
		if(!update_AB_transmit_location(write_index))
		{
			user_log_error("Unable to set AB transmit location");
			goto end_of_task;
		}
		else
		{
			PRINTF(" ****** Setting MQTT transmit location %d ******\n",write_index);
		}

	}
	// Just some safeguard code that should not happen
	else if ((transmit_index < 0)
			|| (transmit_index >= AB_FLASH_MAX_PAGES))
	{
		sprintf(user_log_string_temp, " ***** Error MQTT transmit point invalid %d - resetting ****", transmit_index);
		user_log_error(user_log_string_temp);
		if(!update_AB_transmit_location(write_index))
		{
			user_log_error("Unable to set AB transmit location");
			goto end_of_task;
		}
	}
	else // valid transmit location
	{
		// Normally the MQTT task will do the updating when it finishes transmitting
		// However, if we are catching up we could have a problem

		// See if we're about to catch up to the MQTT task transmit pointer
		// This will happen if we are unable to transmit for about 2 hours
		// In this case, we have to throw away MQTT data

		// NOTE - as of 8/23/22 the MQTT task does this checking and
		// maintains a reasonably sized guard band of unused flash
		// space to make sure the accelerometer has room to write before
		// the next time MQTT transmits.
		// It might make sense to have an additional check here in case
		// everything goes off the rails
		//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
		//xxxxxxxxxxxxxxxxxxxxxxxxxxxTO DO xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
		//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

	}


	// Set up for the next write
	write_index++;
//	Printf(" === Writing to flash next write location: %d\n", write_index);
	if(write_index >= AB_FLASH_MAX_PAGES)
	{		PRINTF(" Last AB address reached. %d Resetting to zero\n",
				write_index);
		write_index = 0;
	}
	if(!update_AB_write_location(write_index))	{
		user_log_error("Unable to set AB write location");
//			goto end_of_task;
	}	else
	{
//		Printf(" === Writing to flash next write location updated: %d\n", write_index);
	}

	//	vTaskDelay(pdMS_TO_TICKS(50));

	// If the next page we're going to write to is the start of a
	// new sector, we have to erase it to be ready

	if(	((write_index % AB_PAGES_PER_SECTOR ) == 0)
		|| (write_index == 0))
//	if((pUserData->next_AB_write_position % 4 ) == 0)
	{
		*did_an_erase = pdTRUE;
		// Calculate address of next sector to write
		SectorEraseAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)write_index);
		PRINTF("  Sector filled. Location: %x Erasing next sector\n",
					SectorEraseAddr);

		erase_status = user_erase_flash_sector(SPI, SectorEraseAddr);
#if 0
#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("======= about to erase sector");
#endif

		erase_status = eraseSector_4K(SPI, SectorEraseAddr);

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
			printf_with_run_time("======= done erasing sector");
#endif
#endif
		if(!erase_status)
		{
			user_log_error("********* SPI erase error *********");
			write_status = FALSE;
		}
	} // if need to erase next sector

end_of_task:
	spi_flash_close(SPI);  // See comments about SPI closing above
	return write_status;
}

/**
 *******************************************************************************
 * @brief Write the current "wall clock" time to the user log in flash
 *
 *******************************************************************************
 */
static void log_current_time(UCHAR *PrefixString)
{

__time64_t nowSec;
struct tm *ts;
char buf[USERLOG_STRING_MAX_LEN];
int len;
int timelen;

	da16x_time64_sec(NULL, &nowSec);
	ts = (struct tm *)da16x_localtime64(&nowSec);
	da16x_strftime(buf, sizeof (buf), "%Y.%m.%d %H:%M:%S", ts);
	timelen = strlen(buf);

	len = strlen(PrefixString);
	if ((len > 0) + (timelen + len < USERLOG_STRING_MAX_LEN))
	{
		sprintf(user_log_string_temp, "%s Current Time : %s (GMT %+02d:%02d)",
				PrefixString, buf,   da16x_Tzoff() / 3600,   da16x_Tzoff() % 3600);
	}
	else
	{
		sprintf(user_log_string_temp, "Current Time : %s (GMT %+02d:%02d)",
				buf,   da16x_Tzoff() / 3600,   da16x_Tzoff() % 3600);
	}

	user_log_event(user_log_string_temp);

}

/**
 *******************************************************************************
 * @brief Take a snapshot of "wall clock" time for the JSON timesync field
 *
 *******************************************************************************
 */
static void timesync_snapshot(void)
{

struct tm *ts;
char buf[USERLOG_STRING_MAX_LEN];
char timestamp_string[20];
__time64_t cur_msec;
__time64_t cur_sec;

	/*
	 * The format of the output string in the JSON packet is:
	 *
	 *   "timesync": "2023.01.17 12:53:55 (GMT 00:00) 0656741",
	 *
	 * where the data consists of the current date and time in “local” time,
	 * as configured when WIFI is set up.  T
	 * he last field (0656741) is an internal timestamp in milliseconds
	 * since power-on that corresponds to the current local time.
	 * This will make it possible to align timestamps from different devices.
	 *
	 * Note that the “current” local date and time is that returned from
	 * an SNTP server on the Internet and is subject to internet lag and
	 * internal processing times.
	 *
	 */

	// Get current time since power on (timestamp time) in milliseconds
	user_time64_msec_since_poweron(&pUserData->MQTT_timesync_timestamptime_msec);

	// Get current local time in milliseconds and seconds
	da16x_time64_msec(NULL, &cur_msec);
	pUserData->MQTT_timesync_localtime_msec = cur_msec;

	cur_sec = ((cur_msec + 500ULL) / 1000ULL); /* sec rounded*/

	// Convert to date and time
	ts = (struct tm *)da16x_localtime64(&cur_sec);
	// And get as a string
	da16x_strftime(buf, sizeof (buf), "%Y.%m.%d %H:%M:%S", ts);
	// And add time zone offset in case they configured this when
	// provisioning the device.
	sprintf(pUserData->MQTT_timesync_current_time_str,
			"%s (GMT %+02d:%02d)",
				buf,   da16x_Tzoff() / 3600,   da16x_Tzoff() % 3600);

	PRINTF("\n **** Time sync established ***\n");
	time64_string (timestamp_string, &pUserData->MQTT_timesync_timestamptime_msec);

	PRINTF("  Timestamp: %s  Local time: %s\n\n", timestamp_string,
			pUserData->MQTT_timesync_current_time_str);

	return;
}


/**
 *******************************************************************************
 * @brief Periodically log operation information
 *
 *******************************************************************************
 */
static void log_operating_info(void)
{
	float adcDataFloat;

	// Note that local ("wall clock") time isn't available unless we have
	// an internet connection, which typically only happens when
	// we're doing an MQTT transmission
	sprintf(user_log_string_temp, "Total FIFO blocks read since power on   : %d", pUserData->ACCEL_read_count);
	user_log_event(user_log_string_temp);
	if(pUserData->write_fault_count > 0)
	{
		sprintf(user_log_string_temp, "Total FIFO write failures since power on: %d", pUserData->write_fault_count);
		user_log_event(user_log_string_temp);
	}
	if(pUserData->write_retry_count > 0)
	{
		sprintf(user_log_string_temp, "Total times a write retry was needed    : %d", pUserData->write_retry_count);
		user_log_event(user_log_string_temp);
	}
	sprintf(user_log_string_temp, "Total missed accelerometer interrupts   : %d", pUserData->ACCEL_missed_interrupts);
	user_log_event(user_log_string_temp);

	sprintf(user_log_string_temp, "Total sector erase events               : %d", pUserData->erase_attempts);
	user_log_event(user_log_string_temp);
	if(pUserData->erase_retry_count > 0)
	{
		sprintf(user_log_string_temp, "Total times an erase retry was needed   : %d", pUserData->erase_retry_count);
		user_log_event(user_log_string_temp);
	}

	sprintf(user_log_string_temp, "Total MQTT connect attempts             : %d", pUserData->MQTT_connect_attempts);
	user_log_event(user_log_string_temp);
	sprintf(user_log_string_temp, "Total MQTT connect fails                : %d", pUserData->MQTT_connect_fails);
	user_log_event(user_log_string_temp);
	sprintf(user_log_string_temp, "Total MQTT packet transmit fails        : %d", pUserData->MQTT_transmit_fails);
	user_log_event(user_log_string_temp);
	if(pUserData->MQTT_dropped_data_events > 0)
	{
		sprintf(user_log_string_temp, "Total times transmit buffer wrapped     : %d", pUserData->MQTT_dropped_data_events);
		user_log_event(user_log_string_temp);
	}
	// Battery voltage
	adcDataFloat = get_battery_voltage();
	sprintf(user_log_string_temp, "Battery reading : %d",(uint16_t)(adcDataFloat * 100));
	user_log_event(user_log_string_temp);
}

/**
 *******************************************************************************
 * @brief Determine timestamp to use for this reading of the accelerometer FIFO
 *
 *        NOTE - this function is designed to be called exactly once
 *               each time a new FIFO is read from the accelerometer
 *               because we keep statistics on reads that we use to
 *               interpolate the interrupt time when it is otherwise
 *               impossible to determine.
 *******************************************************************************
 */
static void determine_timestamp(__time64_t *returned_timestamp, int num_samples)
{
	int i;
	int j;
	unsigned char time_buffer[50];
	ULONG time_since_boot_milliseconds;
	ULONG time_since_boot_seconds;
	ULONG time_since_boot_minutes;
	ULONG time_since_boot_hours;
	ULONG time_since_boot_days;
	long long interrupt_time;  // RTC clock ticks when busy interrupt occurred

	struct tm *current_time;
	__time64_t now;
	__time64_t nowSec;
	__time64_t nowrawmsec;		// RTC clock tick-based current time msec
	__time64_t latencyms;		// time from interrupt to "now"
	__time64_t timestamp_selected;		// return value

	__time64_t interrupt_time_RTC;   // Time of interrupt when awake in RTC ticks
	__time64_t interrupt_time_msec; // same in milliseconds

	__time64_t lost_time;		// difference between two early time snapshots

	__time64_t time_since_last_wakeup_msec;
//	__time64_t msec_since_time_basis;
//	ULONG average_msec_per_FIFO_read;    // average time per FIFO read when not waking from sleep
//	ULONG average_msec_per_AXL_sample;	// average time per sample when not waking from sleep

	// The following used to calculation accelerometer calibration
	__time64_t total_cal_time;			// total time over samples
	__time64_t cal_interrupt_time;		// time between two interrupts
	int	total_cal_samples;				// total # of samples
	__time64_t average_cal_period_usec;	// average period in usec

	// The following used to calculate the polled FIFO full timestamp
#define NOMINAL_SAMPLE_PERIOD_USEC 71429
	ULONG samples_since_timestamp;		// total samples read since we had a
										// solid wake-from-sleep timestamp
										// prior to this FIFO reading
	                                    // ! be careful with the "bonus" samples
	                                    // that collect while we're getting around
	                                    // to reading
	__time64_t usec_since_timestamp;	// Total calculated time since we had a solid timestamp
	__time64_t calculated_timestamp_usec;  // Assigned calculated timestamp in microseconds

	__time64_t average_timestamp_RTC;		// average of the before and after polled times
	__time64_t average_timestamp_msec;		// average of the before and after polled times
	__time64_t timestamp_error_msec;		// difference between extrapolated and bracketed


	// The following are temps used to hold __time64_t string values
	// since printf can't handle long longs
	char time_string[20];
	char time_string2[20];
	/*
	 * Get current time for reference and some statistics
	 * NOTE - the interrupt we're trying to assign a timestamp to has
	 *        happened a while ago, so this time is just for information
	 *
	 * Note also that the "system time" from the SDK is dependent on
	 * the time received by the SNTP task that could run at boot time
	 * and at every time the MQTT task connects to WIFI.  The SNTP time
	 * retrieved is in seconds and is subject to internet delays, so
	 * the clock time will be inexact for our purposes.
	 */
	da16x_time64_msec(NULL, &now);
	da16x_time64_sec(NULL, &nowSec);

//	PRINTF("\n*** Current da16xx time in msec: %u  sec: %u\n", now, nowSec);
	current_time = (struct tm *)da16x_localtime64(&nowSec);
	da16x_strftime(time_buffer, sizeof(time_buffer), "%Y.%m.%d %H:%M:%S", current_time);
//	PRINTF("Current time is: %s\n", time_buffer);

	// Get relative time since power on from the RTC time counter register
	user_time64_msec_since_poweron(&nowrawmsec);
//	PRINTF("\n*** Milliseconds since boot now: %u\n", nowrawmsec);

	/*
	 * There are three ways that we might find ourselves reading the accelerometer:
	 *    1. Wakened from low-power sleep
	 *    2. Received an interrupt while still awake (usually when doing MQTT transmission)
	 *           (but has been seen during power-on if the network takes too long)
	 *    3. Missed interrupt detected by event loop (usually when doing MQTT transmission)
	 *
	 *  We assign a timestamp differently for each event, as follows:
	 *    1. We use the time that was captured close to wakeup/boot time
	 *         (currently in user_init() function)
	 *    2. We use the time captured by the rtc_ext_cb() function during the
	 *          interrupt process.  See system_start.c.
	 *    3. We calculate what time the interrupt must have occurred by
	 *          extrapolating from the last hard measured time we were here.
	 *
	 *  As each timestamp is determined, we also check for possible missed data.
	 *  We decide this based on the relative time since we last read the FIFO
	 *  with a solid time basis divided by the number of FIFO reads since that
	 *  point.
	 */
//	isPowerOnBoot = pdFALSE;			// pdTRUE when poewr-on boot (first time boot)

	/*
	 * Wakened from low-power sleep
	 */
	if (pdTRUE == isAccelerometerWakeup)
	{
		// *****************************************************
		//    Wakened from sleep
		// *****************************************************

		PRINTF(">>> Timestamp basis: Accelerometer interrupt wakeup from sleep\n");

		// This is the time relative to power-on taken in user_init.
		// It's probably the most reliable number since at power on
		// we may or may not have WIFI to get internet time via SNTP
		// It's also a much smaller number since our active life is
		// about 5 days, so 5 days x 24 hours x 60 minutes x 60 seconds x 1000 msec
		//
		// During development, time was taken in user_main() but then about
		// 9/12/22 we were grabbed a time much earlier. (22 msec)
		time64_string (time_string, &user_raw_rtc_wakeup_time_msec);
//		PRINTF("\n*** Milliseconds since user_main() when wakeup happened: %s (testing)\n",
//				time_string);


		lost_time = user_raw_rtc_wakeup_time_msec - user_raw_launch_time_msec;
		time64_string (time_string, &user_raw_launch_time_msec);
//		time64_string (time_string2, &lost_time);
//		PRINTF("*** Milliseconds since main() when wakeup happened: %s diff %s (timestamp)\n",
//				time_string, time_string2);
		PRINTF("*** Milliseconds since main() when wakeup happened: %s (timestamp)\n",
				time_string);

		// Note - as of 9/12/22 there was a 22 msec difference between the two times
		// here.  So we used the earlier one as closer to the actual time that
		// the interrupt occurred and woke the DA16200 from sleep
		timestamp_selected = user_raw_launch_time_msec;

		time_since_boot_seconds = (ULONG)(user_raw_rtc_wakeup_time_msec / (__time64_t)1000);
		time_since_boot_milliseconds = (ULONG)(user_raw_rtc_wakeup_time_msec
						  - ((__time64_t)time_since_boot_seconds * (__time64_t)1000));
		time_since_boot_minutes = (ULONG)(time_since_boot_seconds / (ULONG)60);
		time_since_boot_hours = (ULONG)(time_since_boot_minutes / (ULONG)60);
		time_since_boot_days = (ULONG)(time_since_boot_hours / (ULONG)24);

		time_since_boot_seconds = time_since_boot_seconds % (ULONG)60;
		time_since_boot_minutes = time_since_boot_minutes % (ULONG)60;
		time_since_boot_hours = time_since_boot_hours % (ULONG)24;

		PRINTF("*** Time since boot when wakeup happened: %u Days plus %02u:%02u:%02u.%03u\n",
				time_since_boot_days,
				time_since_boot_hours,
				time_since_boot_minutes,
				time_since_boot_seconds,
				time_since_boot_milliseconds);

		latencyms = (__time64_t)(nowrawmsec - user_raw_rtc_wakeup_time_msec);
		PRINTF("*** Processing delay since wakeup interrupt in msec: %u\n\n", latencyms);

		// If we haven't completed accelerometer calibration yet,
		// do it now.  This is intended to be the first N cycles of
		// operation, prior to the first MQTT transmit cycle
		if (!pUserData->AXL_calibration_complete)
		{
			i = pUserData->num_AXL_cal_entries;
			pUserData->AXL_cal_entry[i].timestamp = timestamp_selected;
			pUserData->AXL_cal_entry[i].num_samples = num_samples;
			pUserData->num_AXL_cal_entries++;
			if(pUserData->num_AXL_cal_entries >= AXL_CALIBRATION_CYCLES)
			{
				pUserData->AXL_calibration_complete = pdTRUE;
				PRINTF("*** Accelerometer calibration complete. Samples: %d\n\n",
						pUserData->num_AXL_cal_entries);
				total_cal_time = (__time64_t)0;
				total_cal_samples = 0;
				// Use the interval between the 1st & 2nd interrupt, the 2nd & 3rd,
				// etc.
				for (j=1; j<AXL_CALIBRATION_CYCLES; j++)
				{
					cal_interrupt_time = pUserData->AXL_cal_entry[j].timestamp -
							pUserData->AXL_cal_entry[j-1].timestamp;
					total_cal_time += cal_interrupt_time;
					total_cal_samples += pUserData->AXL_cal_entry[j].num_samples;
					time64_string (time_string, &pUserData->AXL_cal_entry[j].timestamp);
					time64_string (time_string2, &cal_interrupt_time);
					PRINTF("   [%d] %s  %d (%s)\n", j, time_string,
							pUserData->AXL_cal_entry[j].num_samples, time_string2);
				}
				time64_string (time_string, &total_cal_time);
				PRINTF("   Total cal time   : %s \n",time_string);
				PRINTF("   Total cal samples: %d\n", total_cal_samples);

				average_cal_period_usec = total_cal_time * (__time64_t)1000; // Convert from msec to usec
				average_cal_period_usec = average_cal_period_usec / (__time64_t)total_cal_samples;
				time64_string (time_string, &average_cal_period_usec);
				PRINTF("   Average cal sample period: %s \n",time_string);
				sprintf(user_log_string_temp, "Accelerometer calibration complete. Average cal sample period: %s usec", time_string);
				user_log_event(user_log_string_temp);

				pUserData->AXL_cal_sample_period_usec = (ULONG)average_cal_period_usec;
				pUserData->AXL_calibration_complete = pdTRUE;
				// Do a sanity check by taking the most recent timestamp minus
				// the first timestamp and divide
			}
		}


#if 0
		// Keep track of the last time we had a wake from sleep
		ms_since_last_read = assigned_timestamp - pUserData->last_FIFO_read_time_ms;
		pUserData->last_FIFO_read_time_ms = assigned_timestamp;
		PRINTF(" >>Milliseconds since last AXL read: %u\n",
				ms_since_last_read);
#endif
		if(pUserData->last_accelerometer_wakeup_time_msec != 0)
		{
			time_since_last_wakeup_msec = timestamp_selected - pUserData->last_accelerometer_wakeup_time_msec;
//			PRINTF(" >>Milliseconds since last AXL wakeup: %u\n",
//					time_since_last_wakeup_msec);

		}
		// Set our time and remember how many new FIFO reads since then
		pUserData->last_accelerometer_wakeup_time_msec = timestamp_selected;
		pUserData->FIFO_reads_since_last_wakeup = 0;
		// If we had extra samples after the assigned timestamp, we need to
		// count those towards the time to the next interrupt
		// For instance, if the threshold is 28 and we read 30 samples
		// on this wakeup, the extra 2 count towards the time to fill
		// the next FIFO.  So if the polling detects the FIFO full at 28,
		// we will set the timestamp of the 28th new sample to 30 * period.
		if (num_samples > AXL_FIFO_INTERRUPT_THRESHOLD)
		{
			pUserData->FIFO_samples_since_last_wakeup =
					num_samples - AXL_FIFO_INTERRUPT_THRESHOLD;
		}
		else
		{
			pUserData->FIFO_samples_since_last_wakeup = 0;
		}

		// And clear the timestamps used by the FIFO-at-threshold logic
		// so they get fresh data when that happens
		user_lower_AXL_poll_detect_RTC_clock = 0;
		user_AXL_poll_detect_RTC_clock = 0;

	}  // If wakened from sleep
	else
	{
		// *****************************************************
		//    Not wakened from sleep
		//      - either an interrupt while running
		//         or a FIFO full detected by polling
		// *****************************************************

		pUserData->FIFO_reads_since_last_wakeup++;
//		PRINTF("*** FIFO reads since last time basis  : %u\n",
//				pUserData->FIFO_reads_since_last_wakeup);

		// and how many samples we've read since the last time we had a timestamp
		pUserData->FIFO_samples_since_last_wakeup += num_samples;
//		PRINTF("*** FIFO samples since last time basis  : %u\n",
//				pUserData->FIFO_samples_since_last_wakeup);

		// Note the following stuff was before the calibration of actual
		// wakes from sleep was done
//		msec_since_time_basis = (__time64_t)(nowrawmsec - pUserData->last_accelerometer_wakeup_time_msec);
//		PRINTF("*** Milliseconds since last time basis: %u\n", msec_since_time_basis);

//		average_msec_per_FIFO_read = msec_since_time_basis / pUserData->FIFO_reads_since_last_wakeup;
//		PRINTF("*** Average milliseconds per FIFO read: %u\n", average_msec_per_FIFO_read);

//		average_msec_per_AXL_sample = msec_since_time_basis / pUserData->FIFO_samples_since_last_wakeup;
//		PRINTF("*** Average milliseconds per AXL sample: %u\n", average_msec_per_AXL_sample);

		// This is when we're reading the AXL while awake
		// so we either got an interrupt or we timed out in the event loop
		if(pdTRUE == isAccelerometerInterrupt)
		{
			// *****************************************************
			//    Interrupted while running
			// *****************************************************
			PRINTF(">>> Timestamp basis: Accelerometer interrupt while running\n");
			// Get the time of the interrupt stored when interrupt occurred
			interrupt_time_RTC = user_accelerometer_interrupt_time;  // RTC clock ticks
			interrupt_time_msec = CLK2MS(interrupt_time_RTC); /* msec. */
			PRINTF("\n*** Accelerometer interrupt msec since boot: %u\n\n", interrupt_time_msec);

			timestamp_selected = interrupt_time_msec;
			PRINTF("\n*** Milliseconds since boot when interrupt happened: %u (timestamp)\n",
					timestamp_selected);
#if 0
		// Keep track of the last time we had a wake from sleep
		ms_since_last_read = assigned_timestamp - pUserData->last_FIFO_read_time_ms;
		pUserData->last_FIFO_read_time_ms = assigned_timestamp;
		PRINTF(" >>Milliseconds since last AXL read: %u\n",
				ms_since_last_read);
#endif

		} // Interrupt while running
		else
		{

			if(pdTRUE == isAccelerometerTimeout)
			{
				// *****************************************************
				//    FIFO full detected by polling
				// *****************************************************
				PRINTF(">>> Timestamp basis: Accelerometer missed interrupt detected\n");

				// We have two techniques here:
				//  1. extrapolate from the last time we had a wake from sleep
				//     event and a solid timestamp
				//  2. use the bracketing timestamps recorded in the
				//     event loop to establish a range and take the average
				//  We'll do both and compare
				//

				// First, calculate what the estimate of the time was
				// when we noticed the FIFO full.  (see the event loop)

				// If we have bracketing times, use the average of the two
				if (user_lower_AXL_poll_detect_RTC_clock > 0)
				{
					average_timestamp_RTC = (user_lower_AXL_poll_detect_RTC_clock
							+ user_AXL_poll_detect_RTC_clock) / (__time64_t)2;
				}
				else
				{
					// Use the timestamp from when we detected the FIFO full
					average_timestamp_RTC = user_AXL_poll_detect_RTC_clock;

				}
				// And convert from RTC ticks to msec
				PRINTF(" ---> Lower bracket time (msec) %u\n",
						CLK2MS(user_lower_AXL_poll_detect_RTC_clock));
				PRINTF(" ---> Upper bracket time (msec) %u\n",
						CLK2MS(user_AXL_poll_detect_RTC_clock));

				average_timestamp_msec = CLK2MS(average_timestamp_RTC);
				PRINTF(" ---> Bracketed msec since boot when FIFO threshold was reached: %u\n",
						average_timestamp_msec);


				// And now, calculate an extrapolated clock value from
				// previous timestamps
				if(pUserData->last_accelerometer_wakeup_time_msec != 0)
				{
					// Extrapolate using the calibrated inter-sample period
					// captured during the initial accelerometer calibration

					// How many samples since last wake from sleep, including
					// all the current FIFO
					samples_since_timestamp = pUserData->FIFO_samples_since_last_wakeup;
					PRINTF("*** Samples since last wake-from-sleep timestamp: %u \n",
							samples_since_timestamp);

					// Since we're trying to figure out how many samples to
					// get to the current FIFO sample that was at the FIFO threshold,
					// deduct any current samples that arrived after the threshold
					if (num_samples > AXL_FIFO_INTERRUPT_THRESHOLD)
					{
						samples_since_timestamp -=
								(num_samples - AXL_FIFO_INTERRUPT_THRESHOLD);
						PRINTF("*** adjusted Samples since last wake-from-sleep timestamp: %u \n",
								samples_since_timestamp);
					}

					// Now estimate how long it's been since that timestamp,
					// based on the calibrated accelerometer sampling rate
					// determined at power-on
					if (pUserData->AXL_calibration_complete)
					{
						// Use calibrated inter-sample period to extrapolate
						usec_since_timestamp = (__time64_t)pUserData->AXL_cal_sample_period_usec
								* (__time64_t)samples_since_timestamp;
					}
					else
					{
						// Calibration not complete - use nominal value
						// NOTE - this really shouldn't happen since
						// we don't expect the polled FIFO event to happen
						// until the MQTT task is active, long after calibration
						// occurs
						PRINTF("\n **** POLLED FIFO TIMESTAMP BASED ON NOMINAL VALUES ***\n");
						usec_since_timestamp = (__time64_t)NOMINAL_SAMPLE_PERIOD_USEC
								* (__time64_t)samples_since_timestamp;

					}


					PRINTF("*** Microseconds since timestamp: %u \n",
							usec_since_timestamp);

					// Calculate the new timestamp in microseconds
					calculated_timestamp_usec =
							(__time64_t)1000 * pUserData->last_accelerometer_wakeup_time_msec;
					calculated_timestamp_usec += usec_since_timestamp;

					// and finally, calculate the timestamp in milliseconds, rounded
					timestamp_selected =
							(calculated_timestamp_usec + (__time64_t)500) / 1000;

//					time64_string (time_string, &timestamp_selected);
					PRINTF("*** Calibrated milliseconds since boot when FIFO threshold was reached: %u (timestamp)\n",
							timestamp_selected);

					// See how different they are.
					timestamp_error_msec = average_timestamp_msec - timestamp_selected;
//					PRINTF("*** Difference in timestamp methods: %d \n",
//							timestamp_error_msec);


				} // have a previous wake-from-sleep timestamp to use
				else
				{
					// *****************************************************
					//   No previous wake-from-sleep timestamp  - this is a problem
					// *****************************************************
					APRINTF_E("\n***** NO PREVIOUS TIMESTAMP FOR TIMEOUT *****\n");

					// So just use the detection time estimate
					timestamp_selected = average_timestamp_msec;
				} // No previous timestamp to use

			} // FIFO threshold detected by event loop (timeout)
			else
			{
				// *****************************************************
				//    Unknown activation - this is a problem
				// *****************************************************
				APRINTF_E("\n***** UNABLE TO DETERMINE TIMESTAMP UNKNOWN REASON FOR WAKEUP *****\n");
				// Use current offset from boot
				// Get relative time since power on from the RTC time counter register
				user_time64_msec_since_poweron(&nowrawmsec);
				PRINTF("\n*** Current time since boot in msec: %u\n",
						nowrawmsec);

				timestamp_selected = nowrawmsec;
			} // Unknown reason - don't know how to assign timestamp
		} // not accelerometer interrupt while awake
	} // not wakened from sleep

#if 0
		// Keep track of the last time we had a wake from sleep
		ms_since_last_read = assigned_timestamp - pUserData->last_FIFO_read_time_ms;
		pUserData->last_FIFO_read_time_ms = assigned_timestamp;
		PRINTF(" >>Milliseconds since last AXL read: %u\n",
				ms_since_last_read);
#endif


end_of_func:

	// clear our reasons for activity flags
	isPowerOnBoot = pdFALSE;
	isAccelerometerWakeup = pdFALSE;
	isAccelerometerInterrupt = pdFALSE;
	isAccelerometerTimeout = pdFALSE;

	*returned_timestamp = timestamp_selected;

	return;
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
	int dataptr;
	int i;
	INT32 readstatus;
	int storestatus;
	int I2Cstatus;
	UINT32 erase_fail_count, write_fail_count;
	uint8_t ISR_reason;
	__time64_t assigned_timestamp;		// timestamp to assign to FIFO reading
	ULONG current_time_ms;
	ULONG ms_since_last_read;
	int archive_status;
	int erase_happened;		// tells us if an erase sector has happened
	int mqtt_started;		// tells us if we started the mqtt task
	int max_display;		// temp to figure out last active "try" position

	// Make known to other processes that we are active
	SET_PROCESS_BIT(processLists, USER_PROCESS_HANDLE_RTCKEY);

//#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
//	printf_with_run_time("Read AXL data");
//#endif


	// Read entire FIFO contents until it is empty,
	// filling the data structure that we store each
	// interrupt

	// Read status register with FIFO content info
	fiforeg[0] = MC36XX_REG_STATUS_1;
	I2Cstatus = i2cRead(MC3672_ADDR, fiforeg, 1);

	//TODO check I2Cstatus return value
//	PRINTF(" FIFO status 0X%x\r\n", fiforeg[0]);

	pUserData->FIFO_reads_this_power_cycle++;


	// Only show statistics if we're in a keep-awake cycle with multiple
	// read events
	if (pUserData->FIFO_reads_this_power_cycle > 1)
	{
//		PRINTF(" AXL reads this power cycle: %d \n",
//				pUserData->FIFO_reads_this_power_cycle);
	}

	// Note - need to make sure we don't read more than 32 here
	// Note that even though the FIFO interrupt threshold is something
	// like 30, there is a delay before we get to this point and we
	// are quite likely to read extra samples.
	dataptr = 0;
	while(((fiforeg[0] & 16) != 16) && (dataptr < MAX_ACCEL_FIFO_SIZE))
	{
		rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
		readstatus = i2cRead(MC3672_ADDR, rawdata, 6);

		receivedFIFO.Xvalue[dataptr] = (/*((unsigned short)rawdata[1] << 8)*/ + rawdata[0]);
		receivedFIFO.Yvalue[dataptr] = (/*((unsigned short)rawdata[3] << 8)*/ + rawdata[2]);
		receivedFIFO.Zvalue[dataptr] = (/*((unsigned short)rawdata[5] << 8)*/ + rawdata[4]);
		dataptr++;

//		PRINTF("%d     X: %d Y: %d Z: %d\r\n", dataptr, receivedFIFO.Xvalue[dataptr], receivedFIFO.Yvalue[dataptr], receivedFIFO.Zvalue[dataptr]);

		// Reread status register with FIFO content info
		fiforeg[0] = MC36XX_REG_STATUS_1;
		readstatus = i2cRead(MC3672_ADDR, fiforeg, 1);
		//TODO check I2Cstatus return value
		//PRINTF("FIFO STAT %d: 	%d\r\n", dataptr, fiforeg[0]);
	}

	// Clear the interrupt pending in the accelerometer
	clear_intstate(&ISR_reason);

	// Set the number of data items in the buffer
	receivedFIFO.num_samples = dataptr;
	// Set an ever-increasing sequence number
	pUserData->ACCEL_read_count++;  // Increment FIFO read #
	receivedFIFO.data_sequence = pUserData->ACCEL_read_count;
	// Tell MQTT which sample index is the one that we think
	// triggered the interrupt and therefore is the one
	// to associate with the timestamp
	receivedFIFO.timestamp_sample = (AXL_FIFO_INTERRUPT_THRESHOLD - 1);

	// Set the timestamp in milliseconds
	//
	//  There are two main choices here:
	//    1. time relative to power-on boot - with a range of 0 to 5 days or so.
	//         This time is based on the RTC time counter that runs at
	//          msec * 32768, although we store it in msec.
	//    2. wall clock time - based on SNTP activity.  This depends on having
	//       an internet connection and is a much larger number.  It's basically
	//        msec since Jan 1, 1970
	//  So, as of 9/1/22, we choose the relative time, taken as close to
	//  the interrupt as possible.  Note that we should
	//  be able to translate this to wall clock time when we get to the
	//  MQTT task, since it needs SNTP to operate and the time will be real
	//  The DA16x SDK keeps track of RTC offset from real time.  See the
	//  file da16x_time.c for helper functions and more information.
	//
	// Figure out what timestamp to use
	determine_timestamp(&assigned_timestamp, dataptr);

	receivedFIFO.accelTime = assigned_timestamp;

	// Figure out how long since the last interrupt or read time
	// If we've wakened from sleep via RTC interrupt, we check to
	// see if we've missed data.
	// If we've been interrupted while doing MQTT transmit, we
	// check the timing to see how much latency or jitter we have
//	current_time_ms = MS_since_boot_time();
	ms_since_last_read = assigned_timestamp - pUserData->last_FIFO_read_time_ms;
	pUserData->last_FIFO_read_time_ms = assigned_timestamp;
	PRINTF(" >>Milliseconds since last AXL read: %u\n",
			ms_since_last_read);

	// Display the values
//#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
//	printf_with_run_time(">>>>>>> FIFO data acquired");
//#endif

//	PRINTF(" FIFO read sequence %d\n", pUserData->ACCEL_read_count);
	PRINTF(" FIFO samples read: %d\n", dataptr);

#if 0
	for(i=0; i<dataptr; i++)
	{
		PRINTF("%d     X: %d Y: %d Z: %d\r\n", i, receivedFIFO.Xvalue[i],
				receivedFIFO.Yvalue[i], receivedFIFO.Zvalue[i]);
	}
	// give time for above print to finish before next interrupt
	vTaskDelay(5);
#endif

	// *****************************************************
	// Store the FIFO data structure into nonvol memory
	//
	// *** Note - as of 9/12/22 it's possible for the software
	//    to get into a permanent fail loop if a previous
	//    erase function failed.
	// *****************************************************
	storestatus = user_process_write_to_flash(&receivedFIFO, &erase_happened);
	if(storestatus != TRUE)
	{
		APRINTF_E("\n***** UNABLE TO WRITE DATA TO FLASH *****\n");
		APRINTF_E("\n***** FAULT COUNT: %d *****\n", pUserData->write_fault_count);
	}
	if(erase_happened)
	{
		PRINTF(" Erase sector happened\n");
	}

	PRINTF(" Total FIFO blocks read since power on   : %d\n", pUserData->ACCEL_read_count);
	if(pUserData->write_fault_count > 0)
	{
		PRINTF(" Total FIFO write failures since power on: %d\n", pUserData->write_fault_count);
	}
	if(pUserData->write_retry_count > 0)
	{
		PRINTF(" Total times a write retry was needed    : %d\n", pUserData->write_retry_count);
	}
	PRINTF(" Total missed accelerometer interrupts   : %d\n", pUserData->ACCEL_missed_interrupts);
	max_display = AB_WRITE_MAX_ATTEMPTS - 1;
	while (max_display > 0 &&
			 pUserData->write_attempt_events[max_display] == 0)
	{
		max_display--;
	}
//	for (i=0; i<AB_WRITE_MAX_ATTEMPTS; i++)
	if(max_display > 0)
	{
		for (i=0; i<=max_display; i++)
		{
			PRINTF(" Total times succeeded on try %d          : %d\n", (i+1), pUserData->write_attempt_events[i]);
		}
	}
		PRINTF(" Total sector erase events               : %d\n", pUserData->erase_attempts);
	if(pUserData->erase_retry_count > 0)
	{
		PRINTF(" Total times an erase retry was needed   : %d\n", pUserData->erase_retry_count);
	}
		max_display = AB_ERASE_MAX_ATTEMPTS - 1;
	while (max_display > 0 &&
			 pUserData->erase_attempt_events[max_display] == 0)
	{
		max_display--;
	}
//	for (i=0; i<AB_ERASE_MAX_ATTEMPTS; i++)
	if(max_display > 0)
	{
		for (i=0; i<=max_display; i++)
		{
			PRINTF(" Total times succeeded on try %d          : %d\n", (i+1), pUserData->erase_attempt_events[i]);
		}
	}
//	PRINTF(" Total times succeeded on first retry    : %d\n", pUserData->erase_retry_events[1]);
//	PRINTF(" Total times a second retry was needed   : %d\n", pUserData->erase_retry_events[2]);
	PRINTF(" ----------------------------------------\n");
	PRINTF(" Total MQTT connect attempts             : %d\n", pUserData->MQTT_connect_attempts);
	PRINTF(" Total MQTT connect fails                : %d\n", pUserData->MQTT_connect_fails);
	PRINTF(" Total MQTT packet transmit fails        : %d\n", pUserData->MQTT_transmit_fails);
	if(pUserData->MQTT_dropped_data_events > 0)
	{
		PRINTF(" Total times transmit buffer wrapped     : %d\n", pUserData->MQTT_dropped_data_events);
	}
	PRINTF(" ----------------------------------------\n");

	PRINTF(" Total log entries since power on        : %d\n", pUserData->total_log_entries);
	if(pUserData->log_write_fault_count > 0)
	{
		PRINTF(" Total log write failures since power on : %d\n", pUserData->log_write_fault_count);
	}
	if(pUserData->log_write_retry_count > 0)
	{
		PRINTF(" Total times a log write retry was needed: %d\n", pUserData->log_write_retry_count);
	}
	max_display = USERLOG_WRITE_MAX_ATTEMPTS - 1;
	while (max_display > 0 &&
			 pUserData->log_write_attempt_events[max_display] == 0)
	{
		max_display--;
	}
//	for (i=0; i<USERLOG_WRITE_MAX_ATTEMPTS; i++)
	if (max_display > 0)
	{
		for (i=0; i<=max_display; i++)
		{
			PRINTF(" Total times succeeded on try %d          : %d\n", (i+1), pUserData->log_write_attempt_events[i]);
		}
	}
		PRINTF(" Total log sector erase events           : %d\n", pUserData->log_erase_attempts);
	if(pUserData->log_erase_retry_count > 0)
	{
		PRINTF(" Total times a log erase retry was needed: %d\n", pUserData->log_erase_retry_count);
	}
		max_display = USERLOG_ERASE_MAX_ATTEMPTS - 1;
	while (max_display > 0 &&
			 pUserData->log_erase_attempt_events[max_display] == 0)
	{
		max_display--;
	}
//	for (i=0; i<USERLOG_ERASE_MAX_ATTEMPTS; i++)
	if(max_display > 0)
	{
		for (i=0; i<=max_display; i++)
		{
			PRINTF(" Total times succeeded on try %d          : %d\n", (i+1), pUserData->log_erase_attempt_events[i]);
		}
	}

	// See if it's time to log stats
	++pUserData->ACCEL_log_stats_trigger;  // Increment FIFO stored

	PRINTF(" ACCEL log stats trigger: %d of %d\n",
			pUserData->ACCEL_log_stats_trigger, (int)AXL_LOG_STATS_TRIGGER_COUNT);
	mqtt_started = pdFALSE;
	if(pUserData->ACCEL_log_stats_trigger >= AXL_LOG_STATS_TRIGGER_COUNT)
	{
		log_operating_info();
		// Reset our trigger counter
		pUserData->ACCEL_log_stats_trigger = 0;
	}

	// See if it's time to transmit data, based on how many FIFO buffers we've
	// accumulated since the last transmission
	++pUserData->ACCEL_transmit_trigger;  // Increment FIFO stored

	PRINTF(" ACCEL transmit trigger: %d of %d\n",
			pUserData->ACCEL_transmit_trigger, (int)MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS);
	mqtt_started = pdFALSE;
	if(pUserData->ACCEL_transmit_trigger >= MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS)
	{
		mqtt_started = pdTRUE;
		// Check if MQTT is still active before starting again
		if (PROCESS_BIT_SET(processLists, USER_PROCESS_MQTT_TRANSMIT))
		{
			user_log_error("****** MQTT task still active!!! Unable to start ******");
		}
		else
		{
			// Let the other tasks know that MQTT will be active so we don't sleep
			SET_PROCESS_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);

			// Send the event that will start the MQTT transmit task
			user_attempt_transmit_event();
		}

		// Reset our transmit trigger counter
		pUserData->ACCEL_transmit_trigger = 0;
	}
#ifdef CFG_USE_SYSTEM_CONTROL
	else if (pUserData->ACCEL_transmit_trigger == MQTT_TRANSMIT_TRIGGER_FIFO_BUFFERS - 1)
	{
		// Enable WLAN auto start at the next wake from sleep cycle
		// so that it's ready when we start the MQTT task
		system_control_wlan_enable(TRUE);
	}
#endif

	// Note - this is here because we had a lot of difficulty getting the
	// SPI write to be reliable.  Hence the retry stuff.  But it was also
	// suspected that closing the SPI bus too soon was a factor so it
	// was moved here.
//	spi_flash_close(SPI);

	// If we are at the end of the wake part of a wake/sleep cycle
	// and haven't done an accelerometer buffer sector erase in this
	// cycle and the MQTT task isn't running, then
	// give the logging mechanism a chance to move any log entries
	// buffered in retention memory to flash.  This should be able
	// to occur without any other interference.
	if(!erase_happened
		&& !PROCESS_BIT_SET(processLists, USER_PROCESS_MQTT_TRANSMIT))
	{
		archive_status = user_archive_log_messages(pdFALSE);
	}

// Delay to make sure that statistics show up on the console log
	vTaskDelay(pdMS_TO_TICKS(50));

	// Signal that we're finished so we can sleep

	CLR_PROCESS_BIT(processLists, USER_PROCESS_HANDLE_RTCKEY);

	return 0;
}





/**
 *******************************************************************************
 * @brief Accelerometer initialization for our application
 *******************************************************************************
 */
void user_initialize_accelerometer(void)
{

INT32 status;
UINT32 intr_src;

// Accelerometer initialization
int8_t Xvalue;
int8_t Yvalue;
int8_t Zvalue;

unsigned char fiforeg[2];
signed char rawdata[8];
int i = 0;

uint8_t ISR_reason;

//	#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
//	printf_with_run_time("Start AXL init");
//	#endif

	// Accelerometer API initialization
	// This sets the threshold at which we receive an interrupt
//	mc3672Init((int)ACCEL_FIFO_threshold);
	mc3672Init();

	// Why this huge delay?
	vTaskDelay(250);

	// While loop to empty contents of FIFO before enabling RTC ISR  - NJ 6/30/2022
	fiforeg[0] = MC36XX_REG_STATUS_1;
	status = i2cRead(MC3672_ADDR, fiforeg, 1);

	while((fiforeg[0] & 16) != 16){
		rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
		status = i2cRead(MC3672_ADDR, rawdata, 6);
		fiforeg[0] = MC36XX_REG_STATUS_1;
		status = i2cRead(MC3672_ADDR, fiforeg, 1);
		i++;
	}
	PRINTF("\n------>FIFO buffer emptied %d samples\n", i);

	//Enable RTC ISR - NJ 6/30/2022
	// Note that in original SDK this interrupt was enabled earlier.
	// See function config_ext_wakeup_resource();
	PRINTF("\n------>%s enabling accelerometer interrupt\n", __func__); // FRSDEBUG

	// Clear any accelerometer interrupt that might be pending
	clear_intstate(&ISR_reason);

	RTC_IOCTL(RTC_GET_RTC_CONTROL_REG, &intr_src);
	intr_src |= WAKEUP_INTERRUPT_ENABLE(1);
	RTC_IOCTL(RTC_SET_RTC_CONTROL_REG, &intr_src);

	#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("End AXL init");
	#endif

	user_log_event("Accelerometer initialized");

	return;
}
/**
 *******************************************************************************
 * @brief Process for boot-up event
 *******************************************************************************
 */
static int user_process_bootup_event(void)
{
	int ret, netProfileUse;
	int spi_status;
	int status;
	char MQTT_topic[MQTT_PASSWORD_MAX_LEN] = {0, }; // password has the max length in str type
	int MACaddrtype;
	UCHAR time_string[20];

	PRINTF("\n********** Neuralert bootup event ***********\n");
//	PRINTF("**Neuralert: %s\n", __func__); // FRSDEBUG

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
printf_with_run_time("Starting boot event process");
#endif

	set_sole_system_state(USER_STATE_POWER_ON_BOOTUP);

	// Do early initialization of the intermediate logging area
	// so that any information during bootup can be captured
	user_process_initialize_user_log_holding();

// Log the earliest timestamp we know.  This was taken in main()
// very soon after hardware initialization.  This is just to understand
// what the RTC clock says that early in the process.
	time64_string (time_string, &user_raw_launch_time_msec);
	sprintf(user_log_string_temp, "***Bootup event: time snapshot from main() %s ms",
				time_string);
	PRINTF("\n%s\n", user_log_string_temp);

	user_log_event(user_log_string_temp);

	sprintf(user_log_string_temp, "Software part number  :    %s", USER_SOFTWARE_PART_NUMBER_STRING);
	user_log_event(user_log_string_temp);
	sprintf(user_log_string_temp, "Software version      :    %s", USER_VERSION_STRING);
	user_log_event(user_log_string_temp);
	sprintf(user_log_string_temp, "Software build time   : %s %s", __DATE__ , __TIME__ );
	user_log_event(user_log_string_temp);


	// Enable WIFI on initial bootup.  This allows us to find out if
	// WIFI is available, if we want to.
	wifi_cs_rf_cntrl(FALSE);		// RF now on

	/* See if automatic network connection is disabled and do an
	 * connect to see if WIFI is available
	 */
	ret = da16x_get_config_int(DA16X_CONF_INT_STA_PROF_DISABLED, &netProfileUse);
	if (ret != CC_SUCCESS || netProfileUse == pdFALSE)
	{
		PRINTF("\n **** NETWORK AUTO-START IS ENABLED ****\n");
	}
	else
	{
		PRINTF("\n **** NETWORK AUTO-START IS DISABLED ****\n");
		PRINTF("     Attempting manual connection for bootup\n");
		/* Try to make connection manually. */
		ret = user_process_connect_ap();
	}


	// The following delay serves three purposes.  It was originally added because
	// the WIFI and MQTT startup activity seemed to interfere with
	// SPI bus activity and, in particular, interfered with the initial
	// erase of the external data flash.
	// The second purpose is to give the user time to type in a
	// command, such as:
	//   "reset" to get to the ROM monitor to reflash the software
	//   "user" and "run 0" to go back to provisioning mode (still needs power down and up)
	// The third purpose is to give the SDK enough time to establish whether
	// a WIFI connection is available and whether we can connect to the broker.
	// If we're unable to connect, then we should let the user know via
	// the LEDs
	PRINTF("\n...Delay for reset and stabilization...\n\n");
	vTaskDelay(pdMS_TO_TICKS(15000));
	PRINTF("\n...End stabilization delay...\n\n");
	
	// So at this point, any network connection activity should have
	// completed.  If net startup was enabled, it will have attempted
	// a connection to WIFI.  In that case, we may or may not have WIFI.
	// If MQTT is set to auto start, it will also have attempted a
	// connection to the MQTT broker configured
	// So we could add some activity here that checks for network connection
	// and alerts the user if no WIFI or MQTT connection has been made
	// but for now, we just turn off the auto-start of the network to keep
	// it from slowing down the accelerometer wake-from-low-power-sleep
	if (user_process_check_wifi_conn() == pdTRUE)
	{
		PRINTF("\n*** WIFI is connected ***\n");
		user_log_event("*** Successful WIFI connection ***");
		// Add the current wall clock time to the log for a starting reference
		log_current_time("Bootup WIFI. ");

		set_sole_system_state(USER_STATE_WIFI_CONNECTED);
	}
	else
	{
		PRINTF("\n*** Activation WIFI is not connected ***\n");
		user_log_event("*** WIFI connection not successful ***");
		// Add the current wall clock time to the log for a starting reference
		// Note that this will most likely be 1970, since we didn't have a
		// successful internet time connection.
		log_current_time("Bootup no WIFI. ");
		set_sole_system_state(USER_STATE_WIFI_CONNECT_FAILED);
		// Delay to allow it to be seen
		vTaskDelay(pdMS_TO_TICKS(3000));
	}

	// Whether WIFI is connected or not, see if we can obtain our
	// MAC address
	// Check our MAC string used as a unique device identifier
	// ***NOTE*** when we tried to do this earlier in the boot sequence
	// it caused a hard fault.  It hasn't been tested when WIFI doesn't connect
	memset(macstr, 0, 18);
	memset(MACaddr, 0,7);
	MACaddrtype = getMACAddrStr(1, macstr);  // Hex digits string together
	MACaddr[0] = macstr[9];
	MACaddr[1] = macstr[10];
	MACaddr[2] = macstr[12];
	MACaddr[3] = macstr[13];
	MACaddr[4] = macstr[15];
	MACaddr[5] = macstr[16];
	PRINTF(" MAC address - %s (type: %d)\n", macstr, MACaddrtype);

	strcpy (pUserData->Device_ID, MACaddr);
	PRINTF(" Unique device ID: %s\n", MACaddr);

	sprintf(user_log_string_temp, "Unique device ID: %s", MACaddr);
	user_log_event(user_log_string_temp);


	user_process_disable_auto_connection();

	// Initialize the accelerometer buffer external flash
	user_process_initialize_AB();

	user_log_event("*** Accelerometer flash buffering initialized");

	pUserData->ACCEL_missed_interrupts = 0;

	// Initialize the accelerometer calibration process
	pUserData->AXL_calibration_complete = pdFALSE;
	pUserData->num_AXL_cal_entries = 0;
	// Set the accelerometer sample period to nominal (14Hz sample rate)
	pUserData->AXL_cal_sample_period_usec = (ULONG)71429;

	// Initialize the FIFO interrupt cycle statistics
	pUserData->FIFO_reads_this_power_cycle = 0;

	// Initialize the accelerometer timestamp bookkeeping
	pUserData->last_accelerometer_wakeup_time_msec = 0;
	pUserData->FIFO_reads_since_last_wakeup = 0;
	pUserData->FIFO_samples_since_last_wakeup = 0;

	// Initialize the MQTT statistics
	pUserData->MQTT_message_number = 0;
	pUserData->MQTT_connect_attempts = 0;
	pUserData->MQTT_connect_fails = 0;
	pUserData->MQTT_transmit_fails = 0;
	pUserData->MQTT_dropped_data_events = 0;


	// Complete the log initialization process, including
	// erasing a flash sector
	user_process_initialize_user_log();

	// Initialize the accelerometer and enable the AXL interrupt
	user_initialize_accelerometer();
	
	// Close the SPI handle opened in AB init
//	spi_status = spi_flash_close(SPI);

#ifdef CFG_USE_SYSTEM_CONTROL
	// Disabling WLAN at the next boot. : This is an example.
	system_control_wlan_enable(FALSE);
#endif

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
printf_with_run_time("Finished boot event process");
#endif

	return ret;
}


/**
 *******************************************************************************
 * @brief Process all events
 *******************************************************************************
 */
static UCHAR user_process_event(UINT32 event)
{

//	PRINTF("%s: Event: [%d]\n", __func__, event);

	__time64_t awake_time;
	__time64_t current_msec_since_boot;
	UCHAR time_string[20];

	// Power-on boot
	// This is only expected to happen once when the device is
	// activated
	if (event & USER_BOOTUP_EVENT) {
//		PRINTF("\n**Neuralert: %s Bootup event\n", __func__); // FRSDEBUG

		isPowerOnBoot = pdTRUE;		// tell accelerometer what's up

//		user_process_create_rtc_timer(USER_RTC_TIMER_ID, USER_DATA_TX_TIMER_SEC);

		// Do one-time power-on stuff
		user_process_bootup_event();

		// Check if possible to sleep
		user_sleep_ready_event();
	}

	// Wakened from low-power sleep by accelerometer interrupt
	if (event & USER_WAKEUP_BY_RTCKEY_EVENT) {
		PRINTF("**%s: Wake by RTCKEY event\n", __func__); // FRSDEBUG

		isAccelerometerWakeup = pdTRUE;	// tell accelerometer why it's awake

		// We were wakened only by an accelerometer interrupt so
		// we can safely turn off the RF section
		wifi_cs_rf_cntrl(TRUE);

		// Read accelerometer data and store until time to transmit
		user_process_read_data();

		// Check if possible to sleep
		user_sleep_ready_event();
	}

	// Wakened by previously set time
	// This event is not used as of 9/1/22 (Stage 7a)
	if (event & USER_WAKEUP_BY_TIMER_EVENT) {
		PRINTF("** %s Wake by TIMER event\n", __func__); // FRSDEBUG
#if 0
		// Wakened by transmit timer, start the RF section power up
		wifi_cs_rf_cntrl(FALSE);
		// And start to connect to WIFI
		user_process_connect_ap();
#endif
	}

	// This event is when an accelerometer interrupt occurs while
	// we're not sleeping.  It's usually when the MQTT task is running
	// Note that the event loop also signals this event when it detects
	// a missed accelerometer interrupt (FIFO at threshold)
	if (event & USER_RTCKEY_IN_TIMER_HANDLE_EVENT) {
		PRINTF("** %s RTCKEY in TIMER event\n", __func__); // FRSDEBUG


		// Accelerometer interrupt while we're transmitting
		// Read the data
		user_process_read_data();
		// And see if we are able to sleep
		user_sleep_ready_event();
	}

	// A second unused event.  This was originally for a time MQTT transmit
	// activity.
	if (event & USER_TIMER_IN_RTCKEY_HANDLE_EVENT) {
		PRINTF("** %s TIMER in RTCKEY event\n", __func__); // FRSDEBUG
		// Transmit timer happens while we're reading the accelerometer
		// Start timer for next cycle
//ONHOLDSTAGE5a
//		user_process_create_rtc_timer(USER_RTC_TIMER_ID, USER_DATA_TX_TIMER_SEC);
		// And start the internet connection
		user_process_timer_event();
	}

	// A third unused event.  This was the connection complete from a timed
	// transmit event.  The timer event started a connection attempt and
	// when that completed, this event occurred.  It then triggered
	// the sending of data.  From Dialog's original example.
	if (event & USER_CONNECT_COMPLETE_EVENT) {
		PRINTF("** %s USER CONNECT COMPLETE event\n", __func__); // FRSDEBUG
		CLR_PROCESS_BIT(processLists, USER_PROCESS_CONNECT_AP);
		if (isSysNormalBoot) {
			user_process_disable_auto_connection();
		} else {
			// We're now connected to the internet, so start
			// to send data
			user_process_send_data();
		}
		user_sleep_ready_event();
	}


	// Transmit event triggered by accelerometer
	if (event & USER_ATTEMPT_TRANSMIT_EVENT) {
		PRINTF("** %s START TRANSMIT event\n", __func__); // FRSDEBUG

		// Let the other tasks know that we're active so we don't sleep
		SET_PROCESS_BIT(processLists, USER_PROCESS_MQTT_TRANSMIT);

		// Start the RF section power up
		wifi_cs_rf_cntrl(FALSE);

//#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
//	printf_with_run_time("======= ABOUT TO START MQTT TASK");
//#endif

		user_create_MQTT_task();

		// calling sleep ready only makes sense if the create task failed
		// for some reason
		user_sleep_ready_event();
	}

	if (event & USER_SLEEP_READY_EVENT) {
		PRINTF("  USER_SLEEP_READY_EVENT: processLists: %d [%x]\n",
				processLists, processLists);

#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("===USER SLEEP READY EVENT");
#endif

		if (processLists == 0) {
			user_time64_msec_since_poweron(&current_msec_since_boot);
			awake_time = current_msec_since_boot - pUserData->last_sleep_msec;
			time64_string (time_string, &awake_time);
//			time64_string (time_string, &pUserData->last_sleep_msec);
			PRINTF("Entering sleep2. msec since last sleep %s \n\n", time_string);
			vTaskDelay(3);
			// Get relative time since power on from the RTC time counter register
			user_time64_msec_since_poweron(&pUserData->last_sleep_msec);
//			PRINTF("\n*** Milliseconds since boot now: %u\n", nowrawmsec);

			// t\Turn off LEDs to save power while doing sleep/wake cycle
			// Note the expectation is that only alerts will show
			// during wake/sleep and then only important ones
			set_sole_system_state(0);

			dpm_sleep_start_mode_2(TCP_CLIENT_SLP2_PERIOD, TRUE);
		}
		else
		{
			user_time64_msec_since_poweron(&current_msec_since_boot);
			awake_time = current_msec_since_boot - pUserData->last_sleep_msec;
			PRINTF("%s: Unable to sleep. Awake %u msec\n", __func__, awake_time);
		}
	}

	if (event & USER_TERMINATE_EVENT) {
		PRINTF("\n**Neuralert: %s USER TERMINATE event\n", __func__); // FRSDEBUG
		user_log_event("**** USER TERMINATE EVENT ****");
		/* Terminate the process*/
		return PROCESS_EVENT_TERMINATE;
	}

	/* Continue in process */
	return PROCESS_EVENT_CONTINUE;
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
	UINT32 count;

	#if defined(__RUNTIME_CALCULATION__) && defined(XIP_CACHE_BOOT)
	printf_with_run_time("Start user_init");
#endif

	runMode = get_run_mode();
 	pResultStr = read_nvram_string(NVR_KEY_SSID_0);
	if (runMode == SYSMODE_STA_ONLY && strlen(pResultStr))
	{
		int result = 0;
		UINT32 wakeUpMode;

		wakeUpMode = da16x_boot_get_wakeupmode();
		PRINTF("%s: Boot type: 0x%X (%d)\n", __func__, wakeUpMode, wakeUpMode);

#ifdef CFG_USE_RETMEM_WITHOUT_DPM
		/* Create the user retention memory and Initialize. */
		result = user_retmmem_get(USER_RTM_DATA_TAG, (UCHAR **)&pUserData);
		if (result == 0) {
			PRINTF("\n**Neuralert: %s allocating retention memory\n", __func__); // FRSDEBUG
			// created a memory for the user data
			result = user_retmmem_allocate(USER_RTM_DATA_TAG, (void**)&pUserData, sizeof(UserDataBuffer));
			if (result == 0) {
				memset(pUserData, 0, sizeof(UserDataBuffer));
			} else {
				PRINTF("%s: Failed to allocate retention memory\n", __func__);
				user_log_error("Failed to allocate retention memory");
			}
		}
		else
		{
//			PRINTF("\n**Neuralert: %s retention memory retrieved\n", __func__); // FRSDEBUG
//			PRINTF("** Next MQTT message number: %d\n", pUserData->MQTT_message_number);
		}
#endif

		/*
		 * Create a semaphore to make sure each task can have exclusive
		 * access to the management pointers
		 */
		AB_semaphore = xSemaphoreCreateMutex();
		if (AB_semaphore == NULL)
		{
			user_log_error("****** Error creating AB semaphore *****");
		}
		else
		{
//			PRINTF("\n****** AB semaphore created *****\n");
		}

		/*
		 * Create a semaphore to make sure each task can have exclusive
		 * access to flash AB data memory
		 */
		Flash_semaphore = xSemaphoreCreateMutex();
		if (Flash_semaphore == NULL)
		{
			user_log_error("****** Error creating Flash semaphore *****");
		}
		else
		{
//			PRINTF("\n****** Flash semaphore created *****\n");
		}
		/*
		 * Create a semaphore to make sure each task can have exclusive
		 * access to the log holding area in retention memory
		 */
		user_log_semaphore = xSemaphoreCreateMutex();
		if (user_log_semaphore == NULL)
		{
			user_log_error("****** Error creating log holding semaphore *****");
		}
		else
		{
//			PRINTF("\n****** Log holding semaphore created *****\n");
		}

		// Initialize the FIFO interrupt cycle statistics
		pUserData->FIFO_reads_this_power_cycle = 0;

		/*
		 * Clear the shutdown-requested-by-server flag.  To request
		 * a shutdown, it has to be set to a magic value
		 */
		pUserData->ServerShutdownRequested = 0;


		/*
		 * Dispatch different events depending on why we
		 * woke up
		 */
		switch (wakeUpMode) {
		case WAKEUP_RESET:
		case WAKEUP_SOURCE_POR:
		case WAKEUP_WATCHDOG:
			// Power-on reset (once when device is activated)
			isSysNormalBoot = pdTRUE;
			user_send_bootup_event_message();
			break;

		case WAKEUP_SOURCE_EXT_SIGNAL:
		case WAKEUP_EXT_SIG_WITH_RETENTION:
			// Accelerometer interrupt woke us from sleep
			user_wakeup_by_rtckey_event();
			break;

		case WAKEUP_COUNTER_WITH_RETENTION:
			// Wakened from sleep by timer (currently not active)
//			user_process_create_rtc_timer(USER_RTC_TIMER_ID, USER_DATA_TX_TIMER_SEC);
			user_wakeup_by_timer_event();
			break;
		}
	} else if (runMode == SYSMODE_AP_ONLY) {
		wifi_cs_rf_cntrl(FALSE);
		// TODO: ???
	}
}

void user_log_run_time(void)
{
	__time64_t nowrawmsec;
	ULONG time_since_boot_milliseconds;
	ULONG time_since_boot_seconds;
	ULONG time_since_boot_minutes;
	ULONG time_since_boot_hours;
	ULONG time_since_boot_days;

	// Get milliseconds since we first powered up
	user_time64_msec_since_poweron(&nowrawmsec);

	time_since_boot_seconds = (ULONG)(nowrawmsec / (__time64_t)1000);
	time_since_boot_milliseconds = (ULONG)(nowrawmsec
					  - ((__time64_t)time_since_boot_seconds * (__time64_t)1000));
	time_since_boot_minutes = (ULONG)(time_since_boot_seconds / (ULONG)60);
	time_since_boot_hours = (ULONG)(time_since_boot_minutes / (ULONG)60);
	time_since_boot_days = (ULONG)(time_since_boot_hours / (ULONG)24);

	time_since_boot_seconds = time_since_boot_seconds % (ULONG)60;
	time_since_boot_minutes = time_since_boot_minutes % (ULONG)60;
	time_since_boot_hours = time_since_boot_hours % (ULONG)24;

	sprintf(user_log_string_temp,
			"*** Time since power on: %u Days plus %02u:%02u:%02u.%03u",
			time_since_boot_days,
			time_since_boot_hours,
			time_since_boot_minutes,
			time_since_boot_seconds,
			time_since_boot_milliseconds);
	user_log_event(user_log_string_temp);

}

/**
 ****************************************************************************************
 * @brief Shutdown event - either by downlink command or by
 *     battery below battery_exhausted setpoint
 ****************************************************************************************
 */
static void user_deinit(void)
{

	__time64_t awake_time;
	__time64_t current_msec_since_boot;

	UCHAR time_string[20];
	UCHAR value_str[256];
	int clear_AB_status;
	int ret;
	int archive_status;
	int status;
	__time64_t nowrawmsec;		// RTC clock tick-based current time msec

	/* Delete all resources */
	PRINTF("%s\n", __func__);

	// Set the nonvol RUN flag so that we enter command / provisioning
	// mode when we restart.  This preserves the initial part of the
	// log and makes sure we don't start running again.
	user_log_event("*** Setting runflag=0 ***");
	status = user_set_str(DA16X_CONF_STR_MQTT_RUN_FLAG, "runFlag=0", 0);

	// Put total run time in the log
	user_log_run_time();

	// Disconnect from WIFI
	ret = da16x_cli_reply("disconnect", NULL, value_str);
	if (ret < 0 || strcmp(value_str, "FAIL") == 0) {
		PRINTF(" [%s] Failed disconnect from AP 0x%x\n  %s\n", __func__, ret, value_str);
	}

	// Shut down the RF section
	wifi_cs_rf_cntrl(TRUE);

	/*
	 * A downlink command was received asking that we terminate
	 * The MQTT task should have been shut down before issuing
	 * the terminate request, so at this point we could stop the
	 * accelerometer interrupt, delete all the patient data
	 * in flash, and then go to sleep.
	 */

	clear_AB_status = user_process_clear_AB();
	if (!clear_AB_status)
	{
		sprintf(user_log_string_temp, "***** clear_AB returned an error *****");
		user_log_error(user_log_string_temp);
	}

#if 0
	user_time64_msec_since_poweron(&current_msec_since_boot);
	awake_time = current_msec_since_boot - pUserData->last_sleep_msec;
	time64_string (time_string, &awake_time);

	PRINTF("\n\n**************************************************\n");
//	PRINTF("Entering terminal sleep2. msec since last sleep %s \n\n", time_string);
	// Get relative time since power on from the RTC time counter register
	user_time64_msec_since_poweron(&nowrawmsec);
	PRINTF("\n*** Milliseconds since boot now: %u\n", nowrawmsec);
#endif

	// Do a final log entry, including battery level
	log_operating_info();

	// Archive all pending log messages
	archive_status = user_archive_log_messages(pdTRUE);

	// Turn off LEDs
	set_sole_system_state(0);

	vTaskDelay(pdMS_TO_TICKS(2000));

	// for now, enter a busy loop
	// unfortunately, this will run down the battery
	while (TRUE)
	{
//		watch_dog_kicking_example(WATCH_DOG_TIME_1ST, TRUE);	// rescale_time

		// Continue to archive any log messages - just in case
		archive_status = user_archive_log_messages(pdTRUE);

		vTaskDelay(pdMS_TO_TICKS(5000));
	}

//	dpm_sleep_start_mode_2(TCP_CLIENT_SLP2_PERIOD, TRUE);

}

void tcp_client_sleep2_sample(void *param)
{
	uint32_t ulNotifiedValue;
	UCHAR quit = FALSE;
	char tempstr[50];  // working temp string
	UCHAR eventreceived;  // result from event waiting
	unsigned char fiforeg[2];
	UINT32 i2c_status;
// Type of MAC address returned
//		MAC_SPOOFING,
//		NVRAM_MAC,
//		OTP_MAC
	int MACaddrtype;
	float adcDataFloat;


	/* Get our task handle */
	xTask = xTaskGetCurrentTaskHandle();


	PRINTF("\n\n===========>Starting tcp_client_sleep2_sample\r\n");
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
	tempstr[0] = 0x00;
	strcpy(tempstr,"not set");
	user_get_str(DA16X_CONF_STR_MQTT_RUN_FLAG,tempstr);
	PRINTF("===========>NVRam runFlag: [%s]\r\n",tempstr);
	if(strcmp(tempstr,"runFlag=1") == 0)
	{
		runFlag = 1;
	}

	if(runFlag == 0)
	{
		// State mechanism isn't up and running yet, so
		// signal LEDs manually
		setLEDState(CYAN, LED_SLOW, 200, 0, LED_OFF, 0, 200);

		// Allow time for console to settle
		vTaskDelay(pdMS_TO_TICKS(100));
		PRINTF("\n\n******** Waiting for run flag to be set ********\n\n"); 	}
		while (TRUE)
		{
			if(runFlag )
			{
				break;
			}
	//		watch_dog_kicking_example(WATCH_DOG_TIME_1ST, TRUE);	// rescale_time
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
	if (strlen(pUserData->Device_ID) > 0)
	{
		PRINTF(" Unique device ID      : %s\n", pUserData->Device_ID);
	}
	else
	{
		PRINTF(" Unique device ID not acquired yet\n");
	}


	/*
	 * Check the battery voltage
	 * Note that in an MQTT transmission cycle, we won't check battery
	 * again until we have wakened from the first sleep after the cycle.
	 * So, maybe 30 seconds or so.
	 *
	 */
	adcDataFloat = get_battery_voltage();
	PRINTF(" User data size        : %u bytes\n", sizeof(UserDataBuffer));
    PRINTF(" Battery reading       : %d\n",(uint16_t)(adcDataFloat * 100));

	if(adcDataFloat < VREF_EXHAUSTED)
	{
		// Battery done for - enter terminal battery state
		clear_system_alert(USER_ALERT_BATTERY_LOW);
		set_sole_system_state(USER_STATE_BATTERY_EXHAUSTED);
		PRINTF(" *** Battery exhausted ***\n");
		sprintf(user_log_string_temp, "*** Battery exhausted! Reading: %d Limit: %d",
				(uint16_t)(adcDataFloat * 100),
				(uint16_t)(VREF_EXHAUSTED * 100));
		user_log_error(user_log_string_temp);
		// Enter a shutdown state.
		user_terminate_event();
	}
	else if(adcDataFloat < VREF_LOW)
	{
		set_system_alert(USER_ALERT_BATTERY_LOW);
		PRINTF(" *** Battery low ***\n");
//		user_log_event("*** Battery low ***");
	}
	else
	{
		clear_system_alert(USER_ALERT_BATTERY_LOW);
	}


	/*
	 * Event loop - this is the main engine of the application
	 */
	PRINTF("==Event loop:[");
	while (quit == FALSE)
	{
		/* Block and wait for a notification.
		 Bits in this RTOS task's notification value are set by the notifying
		 tasks and interrupts to indicate which events have occurred. */
//		PRINTF("%s: About to wait on event:\n", __func__);
		eventreceived = xTaskNotifyWaitIndexed(
								0,       			/* Wait for 0th notification. */
								0x00,               /* Don't clear any notification bits on entry. */
								ULONG_MAX,          /* Reset the notification value to 0 on exit. */
								&ulNotifiedValue,   /* Notified value pass out in ulNotifiedValue. */
//								pdMS_TO_TICKS(250));   /* Timeout if no event to check on AXL. */
								pdMS_TO_TICKS(25));   /* Timeout if no event to check on AXL. */ //RTOS must check at half the sampling rate of accelerometer
//								portMAX_DELAY);     /* Block indefinitely. */

//		PRINTF("%s: NotifiedValue: 0x%X\n", __func__, ulNotifiedValue);
		PRINTF(".");   // Show [.......] for wait loop
		// See if we've been notified of an event or just timed out
		if (ulNotifiedValue != 0)
		{
			/* Process events */
			PRINTF("]\n");

			// If we've received a terminate downlink command, exit
			// the message processing loop and effectively shut down.
			quit = (user_process_event(ulNotifiedValue) == PROCESS_EVENT_TERMINATE);
		}
		else
		{
			// Timeout on event loop - check to see if the AXL is stalled
			fiforeg[0] = MC36XX_REG_STATUS_1;
			i2c_status = i2cRead(MC3672_ADDR, fiforeg, 1);
			// if the AXL threshold has been reached but we didn't get the
			// notification, fire off our own notification
			if(fiforeg[0] & 0x40)
			{
				// Mark the time when we noticed this
				// in RTC ticks
				user_AXL_poll_detect_RTC_clock = RTC_GET_COUNTER();
				pUserData->ACCEL_missed_interrupts++;
				PRINTF("]\n");
				PRINTF("%s: AXL FIFO at threshold! [%x]\n", __func__, fiforeg[0]);
				if (xTask) {
					xTaskNotifyIndexed(xTask, 0, USER_RTCKEY_IN_TIMER_HANDLE_EVENT, eSetBits);
					isAccelerometerTimeout = pdTRUE;	// remember why we are reading accelerometer				}

				}
			}
			else
			{
				// Mark the time when we last knew that the FIFO wasn't full
				// The theory here is that this will mostly be happening
				// when the MQTT task is active and we are missing interrupts
				// The event loop will timeout regularly and we will check the
				// FIFO buffer and see that it's not full.  We mark the time
				// when we checked so that when the FIFO becomes full, we can
				// use this timestamp to establish a lower bound
				user_lower_AXL_poll_detect_RTC_clock = RTC_GET_COUNTER();
			}
		} // event wait timeout
	} // while (quit == FALSE)

	/* Un-initialize resources */
	user_deinit();

	/* Delete task */
	xTask = NULL;
	vTaskDelete(NULL);
}

#endif // (__TCP_CLIENT_SLEEP2_SAMPLE__)
