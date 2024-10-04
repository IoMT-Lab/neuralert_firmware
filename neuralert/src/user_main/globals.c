/**
 ****************************************************************************************
 *
 * @file globals.c
 *
 * @brief Global variables and buffers
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
#include <stdint.h>
#include "common.h"
#include "da16x_time.h"

#define NUMBER_LEDS 4

HANDLE gpioa, gpioc, hadc;							//!< GPIO and ADC device handles
HANDLE I2C = NULL;
//HANDLE SPI = NULL;									// Serial flash SPI bus handle
//HANDLE MQTT_SPI_handle = NULL;						// SPI handle for the MQTT task to use

uint32_t duty_cycle_seconds;						//!< current remaining duty cycle seconds
//accelDataStruct accelData[NUMBER_SAMPLES_TOTAL];	//!< accelerometer data buffer
/*
 * Data structure where one FIFO full of data is stored by the Accelerometer
 * when a FIFO threshold interrupt occurs
 */
accelBufferStruct receivedFIFO;					//!< accelerometer data buffer for FIFO dumping
//accelBufferStruct checkFIFO;					//!< FIFO read back after writing

UINT8 i2c_data_read[AT_I2C_DATA_LENGTH];
UINT8 i2c_data[AT_I2C_DATA_LENGTH + AT_I2C_LENGTH_FOR_WORD_ADDRESS] = {0,};
UINT8 sensorTypePresentAll = 0;						//!< available sensors
int16_t lastXvalue;
int16_t lastYvalue;
int16_t lastZvalue;

//Added LED - NJ 06/14/2022
ledStruct ledControl[NUMBER_LEDS];					//!< led control as individual LEDS
ledColorStruct ledColor;							//!< led control as single RGB color

//TaskHandle_t handleAccelTask;						//!< accelerometer task handle
//uint8_t accelIntFlag =0;

// *************************************
// Timestamp management
// *************************************

// Timestamp when accelerometer interrupt occurred (RTC counter @ 32.768 kHz)
// Note some of these were experiments while working out which clock
// time to use.  They aren't all necessary
//long long user_rtc_wakeup_time;    // RTC clock when wakeup happened
//__time64_t user_rtc_wakeup_time_msec;  // da16xx msec time when wakeup happened
__time64_t user_current_wakeup_time_int;   // da16xx_time64() when wakeup happened

// This is the time relative to being wakened from sleep as close as possible
// (as of 9/1/22 done in user_init().
__time64_t user_raw_rtc_wakeup_time_msec;  // relative msec time when wakeup happened
int user_raw_rtc_wakeup_time_source;		// code for where this time came from
__time64_t user_raw_launch_time_msec;		// early timestamp

// When an accelerometer interrupt occurs and we're still awake,
// the interrupt takes this snapshot of the RTC clock
// so we know when it happened.  Processing will happen later,
// after the event works its way through the event processor
// and whatever other delays occur
//unsigned long long user_accelerometer_interrupt_time;	// RTC clock when wakeup happened


//uint8_t pubAckSeen = 0;								//!< =1 reply from AWS that publish was seen, =0 publish not seen

// Runflag that reflects the state of the NVRAM runflag set by
// a user command.  1 means go and 0 means just hang around near
// the start of the application to give the user time to
// do any necessary setup
uint8_t runFlag = 0;								//!< =1 start message processing, =0 do not start task

// MAC address of this particular DA16200 used for a unique
// device identifier when communicating with the cloud
char macstr[18];				//!< complete MAC address ASCII string
char MACaddr[7];				//!< last 6 characters of MAC address


