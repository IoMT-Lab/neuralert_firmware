/**
 ****************************************************************************************
 *
 * @file common.h
 *
 * @brief Common constants, enums, structures externs, etc.
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
#ifndef APPS_INCLUDE_COMMON_H_
#define APPS_INCLUDE_COMMON_H_



#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "da16x_types.h"
//#include "spi_flash/spi_flash.h"
#ifndef __time64_t
#include "da16x_time.h"
#endif


// SPI bus parameters for external data flash
// Note this seems to only work when it's 5.  10 was an issue 7/22/22 FRS
#define SPI_MASTER_CLK 	5 	// 5Mhz
#define SPI_MASTER_CS 		0	// CS0



#define NUMBER_LEDS 4

/*
 * Legacy definition for Tim Mulroney timed transmit cycle
 */
/* samples/seconds * seconds * minute  120 * 60 * 5 = 6000*/
//#define NUMBER_SAMPLES_SECOND 14
//#define NUMBER_OF_SECONDS_PER_MINUTE 60
//#define NUMBER_OF_MINUTES_SAVED 5
//#define NUMBER_OF_MINUTES_SAVED 2
//#define NUMBER_SAMPLES_TOTAL (NUMBER_SAMPLES_SECOND * NUMBER_OF_SECONDS_PER_MINUTE * NUMBER_OF_MINUTES_SAVED)
//#define SECONDS_PER_SEND 20

#define NUMBER_TAP_WIDTH_MIN 2
#define NUMBER_TAP_WIDTH_MAX 3
#define NUMBER_TAP_WAIT 5
#define NUMBER_TAP_UPPER 70
#define NUMBER_TAP_SAMPLES (NUMBER_SAMPLES_SECOND * 10)

#define LEDS_AS_COLOR 1

/*
 * Legacy (Tim Mulroney) of the transmit interval
 */
//#define AWS_NVRAM_CONFIG_DUTY_CYCLE "MQTT_DUTY_CYCLE"
//#define AWS_NVRAM_CONFIG_DEFAULT_DUTY_CYCLE 60

enum LEDCOLOR
{
	LED_RED,
	LED_GREEN,
	LED_BLUE,
	LED_YELLOW,
};

enum LEDCOLOR_NUMBER
{
	BLACK,
	BLUE,
	GREEN,
	CYAN,
	RED,
	PURPLE,
	YELLOW,
	WHITE,
};

enum LEDSTATE
{
	LED_OFFX,
	LED_ONX,
	LED_FAST,
	LED_SLOW,
};


#define	FC9050_ADC_NO_TIMESTAMP		(0)
#define	FC9050_ADC_TIMEOUT_DMA		(2000)
#define FC9050_ADC_SEL_ADC_12		(12)
#define FC9050_ADC_DIVIDER_12		(4)
#define FC9050_ADC_NUM_READ		(16)

// Revised VREF, VREF_LOW, AND VREF_EXHAUSTED specified by Nicholas Joseph
// 11/16/22 after reviewing the circuit and ADC spec

// To get actual voltage reading, use the ration 54/25, which is the
// ratio of the voltage divider going into the ADC
#define VREF 1.4
#define VREF_LOW 1.29
#define VREF_EXHAUSTED 1.17
//#define VREF_EXHAUSTED 1.24
// Older values originally from Tim
//#define VREF 3.3
//#define VREF_LOW 3.0
// Exhaused specified by Nicholas Joseph 11/9/22 after testing actual performance
//#define VREF_EXHAUSTED 2.65
//#define VREF_EXHAUSTED 2.8


/*
 * Console display attributes
 * Based on VT-100
 * Set Display Attributes
 * https://www2.ccs.neu.edu/research/gpc/VonaUtils/vona/terminal/vtansi.htm
 *
 * Set Attribute Mode	<ESC>[{attr1};...;{attrn}m

    Sets multiple display attribute settings. The following lists standard attributes:

    0	Reset all attributes
    1	Bright
    2	Dim
    4	Underscore
    5	Blink
    7	Reverse
    8	Hidden

    	Foreground Colours
    30	Black
    31	Red
    32	Green
    33	Yellow
    34	Blue
    35	Magenta
    36	Cyan
    37	White

    	Background Colours
    40	Black
    41	Red
    42	Green
    43	Yellow
    44	Blue
    45	Magenta
    46	Cyan
    47	White
 *
 */
#define PRINTF_RED(...)	\
{\
	PRINTF("\33[1;31m");\
	PRINTF(__VA_ARGS__);\
	PRINTF("\33[0m");	\
}
#define PRINTF_GREEN(...)	\
{\
	PRINTF("\33[1;31m");\
	PRINTF(__VA_ARGS__);\
	PRINTF("\33[0m");	\
}
#define PRINTF_CYAN(...)	\
{\
	PRINTF("\33[1;31m");\
	PRINTF(__VA_ARGS__);\
	PRINTF("\33[0m");	\
}



/* prototypes */
int i2cWriteStop(int addr, uint8_t *data, int length);
int i2cWriteNoStop(int addr, uint8_t *data, int length);
int i2cRead(int addr, uint8_t *data, int length);
void setLEDState(uint8_t number1, uint8_t state1, uint16_t count1, uint8_t number2,  uint8_t state2, uint16_t count2, uint16_t secondsTotal);

#define PRINT_I2C 	DRV_PRINTF
#define	AT_I2C_LENGTH_FOR_WORD_ADDRESS		(2)
#define	AT_I2C_DATA_LENGTH			(32)
#define	AT_I2C_FIRST_WORD_ADDRESS		(0x00)
#define	AT_I2C_SECOND_WORD_ADDRESS		(0x00)
#define	AT_I2C_ERROR_DATA_CHECK			(0x8)
#define NUMBER_I2C_RETRYS 1

#define MC3672_ADDR     0x98

/* structs */
typedef struct
{
	uint8_t state1;					//!< LED state1
	int16_t count1;					//!< counts for led in state1
	uint8_t state2;					//!< LED state2
	int16_t count2;					//!< counts for led to reload state2
	int16_t countReload1;			//!< reload counts for led in state
	int16_t countReload2;			//!< reload counts for led to reload state
	uint8_t inProgress;				//!< 0 state1/count1 1 state2/count2
	int16_t secondsTotal;			//!< total time
} ledStruct;

typedef struct
{
	uint8_t color1;					//!< LEDs color
	uint8_t state1;					//!< LED state1
	int16_t count1;					//!< counts for led in state1
	uint8_t color2;					//!< LEDs color
	uint8_t state2;					//!< LED state2
	int16_t count2;					//!< counts for led to reload state2
	int16_t countReload1;			//!< reload counts for led in state
	int16_t countReload2;			//!< reload counts for led to reload state
	uint8_t inProgress;				//!< 0 state1/count1 1 state2/count2
	int16_t secondsTotal;			//!< total time
} ledColorStruct;



// ***********************************************************
// Error and event logging defines & struct
// ***********************************************************
// Set the following to enable or disable logging altogether
#define USERLOG_ENABLED pdTRUE
//#define USERLOG_ENABLED pdFALSE

// The system log starts in the upper part of the flash
// Because of the write characteristics of the flash, each log entry
// is one page or 256 bytes.
// The highest memory address is 0x7FFFFF
#define USERLOG_FLASH_BEGIN_ADDRESS 0x200000
#define USERLOG_FLASH_MAX_PAGES 22000

// Types of log entries
#define USERLOG_TYPE_INFORMATION 1
#define USERLOG_TYPE_ERROR 2
// Unique signature that identifies a log entry
#define USERLOG_SIGNATURE 0xB04ABCDE
#define USERLOG_STRING_MAX_LEN 128
 typedef struct
 	{
 		__time64_t 	user_log_timestamp;	// assigned relative time of log
 		ULONG		user_log_signature;	// special value that identifies this as a log entry
 		UCHAR		user_log_type;		// information/error/etc.
 		UCHAR		user_log_text[USERLOG_STRING_MAX_LEN]; // how many samples were in the FIFO
 	} USERLOG_ENTRY;




/*
 * Data structure for a single accelerometer reading
 * as of 7/6/22 used by MQTT transmission for staging data to send
 * NOTE - this is NOT the structure stored in external data flash.
 * For that, see accelBufferStruct below.
 */
typedef struct
{
	int16_t Xvalue;					//!< X-Value * 1000
	int16_t Yvalue;					//!< Y-Value * 1000
	int16_t Zvalue;					//!< Z-Value * 1000
#ifdef SAVEMAGNITUDE
	int16_t Magvalue;				//!< magnitude (sqrt(x^2 + y^2 + z^2) * 1000
#endif
#ifdef __TIME64__
	__time64_t accelTime;               //!< time when accelerometer data taken
#else
	time_t accelTime;
#endif /* __TIME64__ */
} accelDataStruct;


/*
 * Defines that characterize the accelerometer sampling rate and FIFO threshold
 *
 * Note that the slowest rate in the current hardware is 14, even though
 * the requirements say 10 per second.
 * See function mc36xx_init() for the actual setting of the accelerometer.
 *
 * The FIFO threshold determines the frequency of interrupts.  Because of the
 * time it takes to wake from low-power sleep, initialize the RTOS and SDK
 * and arrive at the user function that actually reads the contents of the
 * FIFO, it should be set below the max of 32 to give time for all that to
 * occur.  At 14 samples/second, each new sample arrives approximately 71.4 msec
 * after the interrupt occurs.  As of 9/2/22, without fast init turned on and
 * WIFI automatically starting, it takes about 110 msec to finish reading
 * the FIFO.  With a threshold of 30, we find that two more samples are
 * in the buffer by the time we read it and re-enable the interrupt.
 *
 * See function mc3672Init() for the actual initialization of the
 * accelerometer hardware.
 */
#define AXL_SAMPLES_PER_SECOND 14
#define AXL_FIFO_INTERRUPT_THRESHOLD 28


// ***********************************************************
// Accelerometer sample buffering in flash
// ***********************************************************
/*
 * Data structure for storing the entire FIFO contents read from the accelerometer
 * This is the structure that is populated during the accelerometer interrupt and
 * then passed to the buffering interface.  It is subsequently read from the
 * buffer by the MQTT transmission thread.
 *
 * The number of actual samples is stored because it varies depending on the
 * delay before we're able to read the FIFO contents.
 */
#define MAX_ACCEL_FIFO_SIZE 32
typedef struct
{
	// The following data sequence number is intended as a sanity check
	// as we store data in external flash.  It allows the MQTT reader to
	// double-check that data is being retrieved in the proper sequence.
	// It increases with each FIFO read sequence.
	// It's size allows 4,294,967,295 FIFO reads or several hundred years
	// of data capture
	ULONG  data_sequence;					// sequence number increases each read

	// The timestamp is intended to be the time at which the actual
	// accelerometer interrupt occurred, as best as we can capture.
	// The interrupt should occur when the accelerometer reaches the
	// FIFO threshold set at initialization, so we have to compute
	// the timestamps of the other samples backward and sometimes forward
	// from that sample.
	// (We sometimes have more than the FIFO threshold number of samples
	// because we were delayed in reading the FIFO data and in the meantime
	// another sample was taken.
	//
#ifdef __TIME64__
	__time64_t accelTime;               	//!< time when accelerometer data taken
#else
	time_t accelTime;
#endif /* __TIME64__ */

	//JW: The timestamp_sample was only used for trying to identify when the interrupt
	// was called.  This is no longer needed as we log the buffer read time now.
#if 0
	int8_t timestamp_sample;				// The INDEX of the sample to which
											// the timestamp applies
#endif // TO BE REMOVED -- DEPRECATED

	//JW: Need this sample, easier to keep it in the FIFO block rather than interpolate between
	// entries.
	__time64_t accelTime_prev;				// time of previous AXL reading JW: added this

	int8_t num_samples;						// actual # of samples stored
	int8_t Xvalue[MAX_ACCEL_FIFO_SIZE];		//!< X-Value
	int8_t Yvalue[MAX_ACCEL_FIFO_SIZE];		//!< Y-Value
	int8_t Zvalue[MAX_ACCEL_FIFO_SIZE];		//!< Z-Value
} accelBufferStruct;

/*
 * Data structure where assembled packet information is stored.
 * This structure is used to capture the stats of each packet
 */
typedef struct packetDataStruct
{
	int num_samples;
	int num_blocks;
	int start_block;
	int next_start_block;
	int end_block;
	int nvram_error;
	int flash_error;
	int done_flag;
} packetDataStruct;


/*
 * Data structure where the buffer management pointers are stored.
 * This structure could reside in RETMEM or in Flash
 * "pointers" are actually indexes as offsets, similar to a C array
 * A pointer of zero is the first location
 * Addresses are of FIFO storage blocks in Flash.  See the buffering
 * design documentation for layout, but essentially the buffer area is
 * an array of 256-byte Flash pages, with each FIFO struct stored at the start
 * of a page address.
 * As of this writing (6/22/22 FRS) the struct fits on one Flash page.
 */
//#define MAX_ACCEL_BLOCKS	3152
// "Invalid" AB address is used to indicate that there are no stored
// FIFO samples to transmit.  This occurs at power-on reset when the
// AB region is initialized and also if and when the MQTT transmit task
// catches up to the accelerometer read task.  This is highly unlikely
// since the accelerometer should be collecting data while the MQTT transmit
// task is operating
#define INVALID_AB_ADDRESS -1
// The "initialized" flag
#define AB_MANAGEMENT_INITIALIZED -5972
#define AB_INDEX_TYPE int32_t
#define ADDR_LEN 4
//#define FIRST_WRITE_POSITION  0x1000
//#define LAST_WRITE_POSITION  0xCB600

// The accelerometer buffer region starts after a 4096 byte reserved
// sector
#define AB_FLASH_BEGIN_ADDRESS 0x1000

// Pages are 256 bytes long and so addresses are multiples of 0x100
#define AB_FLASH_PAGE_SIZE 256
// Sectors are 4K byes long.  We must erase each sector before we use it.
// Sector addressers are multiples of 0x1000
#define AB_FLASH_SECTOR_SIZE 4096
// So, every 16 pages we have to pause and erase the next sector where
// we will write.
#define AB_PAGES_PER_SECTOR 16
// When transmitting, we don't want to run into the erase procedure.
// So we have a safety gap -- set to twice the erase size (in 10.3, about 4 seconds).
#define AB_TRANSMIT_SAFETY_GAP (2*AB_PAGES_PER_SECTOR)

// The accelerometer buffer region is designed to hold 2 hours of
// data, with one FIFO buffer structure per sector.
// So, 14 samples/second x 2 hours = 100,800 samples / 28 samples per FIFO
// results in 3600 pages.
// (We use 28 as a notional minimum, depending on the dynamic behavior as
//  the software evolves.  As of this writing, 7/19/22, we are experiencing
//  about 30 samples per read cycle.)
// ***NOTE*** this number must be a multiple of 16 so that we have an
//  whole number of sectors to support the erase function.
//  There are 16 pages per sector.
//
// NOTE we need to add the guard zone size because it is unused area
// Note that our 8 mbyte (64kbit) flash contains 2,048 4k sectors
// and 32,768 256-byte pages.  Note that there is a log region planned in
// upper flash memory
//
// 3600 pages / 16 = 225 4k sectors
// 3600 plus 288 guard zone
#define AB_FLASH_MAX_PAGES 3888 //MUST BE LESS THAN 65535 (see definition: uint16_t AB_transmit_stack[AB_FLASH_MAX_PAGES])
// 640 pages = 40 sectors @ 4K
// 640 pages = ~1472 seconds = ~24.5 minutes
// So, for development, this will let us observe all behaviors
//#define AB_FLASH_MAX_PAGES 640

typedef uint32_t _AB_transmit_map_t;

#define AB_TRANSMIT_MAP_SIZE (AB_FLASH_MAX_PAGES / sizeof(_AB_transmit_map_t) + 1)


// Macros for manipulating buffer bits
#define POS_TO_BIT(pos)						(1 << (pos % sizeof(_AB_transmit_map_t)))
#define SET_AB_POS(src, pos)				(src[pos / sizeof(_AB_transmit_map_t)] |= POS_TO_BIT(pos))
#define CLR_AB_POS(src, pos)				(src[pos / sizeof(_AB_transmit_map_t)] &= (~POS_TO_BIT(pos)))
#define POS_AB_SET(src, pos)				((src[pos / sizeof(_AB_transmit_map_t)] & POS_TO_BIT(pos)) == POS_TO_BIT(pos))


#define FLASH_NO_ERROR 0
#define FLASH_OPEN_ERROR -1
#define FLASH_WRITE_ERROR -2
#define FLASH_READ_ERROR -3
#define FLASH_DATA_ERROR -4





// An empty guard zone is needed between the accelerometer writing location and the
// MQTT transmit location.  If the MQTT is unable to transmit, the AXL will
// start to catch up to the transmit location.  This constant defines the size
// of the unused area in the AXL storage area that the MQTT task must maintain,
// even if it has to drop data.  As of 9/12/22 the checking is done in the MQTT task.
// We try to keep it a multiple of sector size (16).  It must cover at least one
// MQTT transmit cycle to give the AXL breathing room to continue to accumulate
// data while the MQTT trask is transmitting
//
// Notionally, a 5 minute transmit cycle would result in:
// 5 minutes = 300 seconds / ~2.2 seconds/AXL interrupt = ~136 blocks or pages
// to achieve ~5 minute transmit cycle
// For code drop 1 on 9/14/22, MQTT transmit is set to 144 FIFO blocks
// or about 5 minutes.
// So two cycles for safety would be 288 blocks.  Note that this time
// is taken from the complete flash buffering area, so if we want 2 hours
// to be stored, we need to add this many blocks to the flash buffering
// area since this is unused space.
// This is also how many FIFO buffers are discarded when the AXL catches up.
#define AB_FLASH_OVERLAP_LIMIT 288 //JW: Need to check if this is still used in LIMO.

// Define a threshold when we will start to warn the user that
// storage is running low and they should get to a WIFI hot spot if
// possible
// If we discard data when we reach about 2 hours, give about a
// 15-minute warning or 3 x 144
#define AB_FLASH_WARNING_THRESHOLD 432

//typedef struct
//{
//	int8_t AB_initialized_flag;			// if buffer mgt has been initialized
//	AB_INDEX_TYPE next_AB_write_position;			// index of next flash block to write
//	AB_INDEX_TYPE next_AB_transmit_position;		// index of next block to transmit
//                                            // should be the oldest data in the
//										 	// buffer
//} accelBufferManagementStruct;





enum SENSORTYPE
{
	SENSORTYPEMC3672,
};

/* prototypes */
extern int i2cWriteStop(int addr, uint8_t *data, int length);
extern int i2cWriteNoStop(int addr, uint8_t *data, int length);
extern int i2cRead(int addr, uint8_t *data, int length);
//void run_i2c_sample(ULONG arg);
void mc3672Init(void);
//void accel_callback();
void clear_intstate(uint8_t* state);
void time64_string (UCHAR *timestamp_str, __time64_t *timestamp);

/*
 * Accelerometer flash buffer API functions
 */
//extern int AB_initialize();
//extern AB_INDEX_TYPE AB_write_next_block(accelBufferStruct *AB_FIFO_block);
//extern int AB_read_block(AB_INDEX_TYPE);
//extern int update_next_AB_write_position(AB_INDEX_TYPE);


/* externals */
extern uint32_t duty_cycle_seconds;
//extern accelDataStruct accelData[];
extern accelBufferStruct receivedFIFO;					//New structure for FIFO data NJ 06/24/2022
extern accelBufferStruct checkFIFO;						//New structure for FIFO data NJ 07/06/2022
//extern accelBufferManagementStruct bufferManagement;	//New structure for SFLASH buffer pointer management NJ 06/24/2022
//extern uint32_t ptrAccelDataWr;
extern HANDLE gpioa;
extern HANDLE gpioc;
extern HANDLE hadc;
extern HANDLE I2C;
//extern HANDLE SPI;
//extern HANDLE MQTT_SPI_handle;						// SPI handle for the MQTT task to use
extern UINT8 i2c_data_read[];
extern UINT8 i2c_data[];
extern UINT8 sensorTypePresentAll;						//!< available sensors
extern int16_t lastXvalue;
extern int16_t lastYvalue;
extern int16_t lastZvalue;
extern TaskHandle_t handleAccelTask;					//!< accelerometer task handle
extern uint8_t accelIntFlag;

extern char macstr[];
extern char MACaddr[];

extern ledStruct ledControl[];
extern ledColorStruct ledColor;
extern uint8_t doubleTap;
extern uint8_t runFlag;
//extern uint8_t pubAckSeen;
extern UCHAR	runtime_cal_flag;		// system flag to enable runtime timing

extern long long user_rtc_wakeup_time;    		// RTC clock when wakeup happened
extern __time64_t user_rtc_wakeup_time_msec;  	// da16xx time when wakeup happened
extern __time64_t user_raw_rtc_wakeup_time_msec;  // relative msec time when wakeup happened
extern __time64_t user_raw_launch_time_msec;  // relative time msec since boot (RTC clock based)
//extern UINT32 user_how_wakened;					// da16x_boot_get_wakeupmode() for how we woke up
//extern int accelerometer_polled_timeout;			// true if event loop detected FIFO full

#endif /* APPS_INCLUDE_COMMON_H_ */

