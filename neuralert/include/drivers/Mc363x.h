/**
 ****************************************************************************************
 *
 * @file Mc363x.h
 *
 * @brief Driver for MC363x accelerometer
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

#ifndef APPS_INCLUDE_MC363X_H_
#define APPS_INCLUDE_MC363X_H_

#include <stdint.h>
#include <stddef.h>
#include "user_dpm.h"

#define USING_IIC_36XX   1 // 36xx iic  1, spi 0

#if (!USING_IIC_36XX)
#define MC36XX_SPI_8M	0  // 36xx spi speed >2M  ,4M 8M  ,set 1
#else
#define MC36XX_SPI_8M	0  // 36xx spi speed >2M  ,4M 8M  ,set 1

#endif

// 3-axis-data disable
#define XDISABLE 0
#define YDISABLE 0
#define ZDISABLE 0


#define MC36XX_INTR_C_IAH_ACTIVE_LOW     (0x00)
#define MC36XX_INTR_C_IAH_ACTIVE_HIGH    (0x02)
#define MC36XX_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC36XX_INTR_C_IPP_MODE_PUSH_PULL     (0x01)



typedef enum
{
	MC36XX_AXIS_X=0,
	MC36XX_AXIS_Y,
	MC36XX_AXIS_Z,
	MC36XX_AXIS_END,
}MC36XX_AXIS;
typedef enum
{
	MC36XX_WAKE_POWER_LOWPOWER= 0,//000
    MC36XX_WAKE_POWER_ULTRA_LOWPOWER=3,
    MC36XX_WAKE_POWER_PRECISION,
    MC36XX_WAKE_POWER_PRE,
    MC36XX_WAKE_POWER_END,
}   MC36XX_WAKE_POWER;
typedef enum
{
	MC36XX_SNIFF_POWER_LOWPOWER= 0,
    MC36XX_SNIFF_POWER_ULTRA_LOWPOWER=3,
    MC36XX_SNIFF_POWER_PRECISION,
    MC36XX_SNIFF_POWER_PRE,
    MC36XX_SNIFF_POWER_END,
}   MC36XX_SNIFF_POWER;


typedef enum
{
    MC36XX_MODE_SLEEP = 0,
    MC36XX_MODE_STANDBY,
    MC36XX_MODE_SNIFF,
    MC36XX_MODE_FSNIFF,
    MC36XX_MODE_PWAKE,
    MC36XX_MODE_CWAKE,
    MC36XX_MODE_SWAKE,
    MC36XX_MODE_TRIG,
    MC36XX_MODE_END
}   MC36XX_MODE;
typedef enum
{
	MC36XX_WAKE_GAIN_HIGH= 0,
    MC36XX_WAKE_GAIN_MED,
    MC36XX_WAKE_GAIN_LOW,
    MC36XX_WAKE_GAIN_END,
}   MC36XX_WAKE_GAIN;

typedef enum
{
    MC36XX_SNIFF_GAIN_HIGH = 0,
    MC36XX_SNIFF_GAIN_MED,
    MC36XX_SNIFF_GAIN_LOW,
    MC36XX_SNIFF_GAIN_END,
}   MC36XX_SNIFF_GAIN;
typedef enum
{
    MC36XX_RANGE_2G = 0,
    MC36XX_RANGE_4G,
    MC36XX_RANGE_8G,
    MC36XX_RANGE_16G,
    MC36XX_RANGE_12G,
    MC36XX_RANGE_24G,
    MC36XX_RANGE_END
}   MC36XX_RANGE;
//uint8_t rangelist[6]={2,4,8,16,12,24};


typedef enum
{
    MC36XX_RESOLUTION_6BIT = 0,
    MC36XX_RESOLUTION_7BIT,
    MC36XX_RESOLUTION_8BIT,
    MC36XX_RESOLUTION_10BIT,
    MC36XX_RESOLUTION_12BIT,
    MC36XX_RESOLUTION_14BIT,
    MC36XX_RESOLUTION_END,
}   MC36XX_RESOLUTION;
//uint8_t resolutionlist[6]={6,7,8,10,12,14};

typedef enum
{
	MC36XX_SNIFF_DETECT_MODE_C2P=0,
	MC36XX_SNIFF_DETECT_MODE_C2B=1,
	MC36XX_SNIFF_DETECT_MODE_END,
}MC36XX_SNIFF_DETECT_MODE;


typedef enum
{
	MC36XX_CWAKE_SR_UL_DUMMY_BASE = 0,                                       // DO NOT USE (mark)
    MC36XX_CWAKE_SR_UL_0p4Hz,
    MC36XX_CWAKE_SR_UL_0p8Hz,
    MC36XX_CWAKE_SR_UL_1p6Hz,
    MC36XX_CWAKE_SR_UL_6p5Hz,
    MC36XX_CWAKE_SR_UL_13Hz,
    MC36XX_CWAKE_SR_UL_26Hz,
    MC36XX_CWAKE_SR_UL_51Hz,
    MC36XX_CWAKE_SR_UL_100Hz,
    MC36XX_CWAKE_SR_UL_197Hz,
    MC36XX_CWAKE_SR_UL_389Hz,
    MC36XX_CWAKE_SR_UL_761Hz,
    MC36XX_CWAKE_SR_UL_1122Hz,
    MC36XX_CWAKE_SR_UL_DUMMY_END,//13

    MC36XX_CWAKE_SR_LP_DUMMY_BASE = MC36XX_CWAKE_SR_UL_DUMMY_END,    // DO NOT USE (mark)
    MC36XX_CWAKE_SR_LP_0p4Hz,
    MC36XX_CWAKE_SR_LP_0p8Hz,
    MC36XX_CWAKE_SR_LP_1p7Hz,
    MC36XX_CWAKE_SR_LP_7Hz,
    MC36XX_CWAKE_SR_LP_14Hz,
    MC36XX_CWAKE_SR_LP_27Hz,
    MC36XX_CWAKE_SR_LP_54Hz,
    MC36XX_CWAKE_SR_LP_106Hz,
    MC36XX_CWAKE_SR_LP_210Hz,
    MC36XX_CWAKE_SR_LP_411Hz,
    MC36XX_CWAKE_SR_LP_606Hz,
    MC36XX_CWAKE_SR_LP_DUMMY_END,//25


 	MC36XX_CWAKE_SR_HIGH_PR_DUMMY_BASE = MC36XX_CWAKE_SR_LP_DUMMY_END,   // DO NOT USE (mark)
    MC36XX_CWAKE_SR_HIGH_PR_0p2Hz,
    MC36XX_CWAKE_SR_HIGH_PR_0p4Hz,
    MC36XX_CWAKE_SR_HIGH_PR_0p9Hz,
    MC36XX_CWAKE_SR_HIGH_PR_7p2Hz,
    MC36XX_CWAKE_SR_HIGH_PR_14Hz,
    MC36XX_CWAKE_SR_HIGH_PR_28Hz,
    MC36XX_CWAKE_SR_HIGH_PR_55Hz,
    MC36XX_CWAKE_SR_HIGH_PR_81Hz,
    MC36XX_CWAKE_SR_HIGH_PR_100Hz=(MC36XX_CWAKE_SR_LP_DUMMY_END+0x0F),
    MC36XX_CWAKE_SR_HIGH_PR_DUMMY_END,//34
}MC36XX_CWAKE_SR;
typedef enum
{
    MC36XX_SNIFF_SR_LP_DEFAULT_6Hz = 0,
    MC36XX_SNIFF_SR_LP_0p4Hz,
    MC36XX_SNIFF_SR_LP_0p8Hz,
    MC36XX_SNIFF_SR_LP_2Hz,
    MC36XX_SNIFF_SR_LP_6Hz,
    MC36XX_SNIFF_SR_LP_13Hz,
    MC36XX_SNIFF_SR_LP_26Hz,
    MC36XX_SNIFF_SR_LP_50Hz,
    MC36XX_SNIFF_SR_LP_100Hz,
    MC36XX_SNIFF_SR_LP_200Hz,
    MC36XX_SNIFF_SR_LP_400Hz,
    MC36XX_SNIFF_SR_END,//12
}   MC36XX_SNIFF_SR;

typedef struct{
	uint8_t filen ;//= 0; //  fifo length you want to set
	uint8_t fion ;//= 0;  //  fifo  ON or OFF
	uint8_t intr_type ;// = 0;
	uint8_t intr_status ;//= 0;
	uint8_t work_mode ;//= 0;
	uint8_t read_style ;//= 0;
	uint8_t x_axis_disable ;//= 0;
	uint8_t y_axis_disable ;//= 0;
	uint8_t z_axis_disable ;//= 0;
	MC36XX_WAKE_POWER wakemode ;//= MC36XX_WAKE_POWER_LOWPOWER;
	MC36XX_SNIFF_POWER sniffmode ;//= MC36XX_SNIFF_POWER_PRECISION;
	MC36XX_WAKE_GAIN wakegain ;//= MC36XX_WAKE_GAIN_LOW;
	MC36XX_SNIFF_GAIN sniffgain ;//= MC36XX_SNIFF_GAIN_HIGH;
	uint16_t datagain;
	uint8_t range ;
	uint8_t resolution;
}Mc363X_All_Status;



#define MC36XX_REG_EXT_STAT_1       0x00
#define MC36XX_REG_EXT_STAT_2       0x01
#define MC36XX_REG_XOUT_LSB         0x02
#define MC36XX_REG_XOUT_MSB         0x03
#define MC36XX_REG_YOUT_LSB         0x04
#define MC36XX_REG_YOUT_MSB         0x05
#define MC36XX_REG_ZOUT_LSB         0x06
#define MC36XX_REG_ZOUT_MSB         0x07
#define MC36XX_REG_STATUS_1         0x08
#define MC36XX_REG_INTR_S           0x09
#define MC36XX_REG_FEATURE_C_1      0x0D
#define MC36XX_REG_FEATURE_C_2      0x0E
#define MC36XX_REG_PWR_C      	    0x0F
#define MC36XX_REG_MODE_C           0x10
#define MC36XX_REG_WAKE_C           0x11
#define MC36XX_REG_SNIFF_C          0x12
#define MC36XX_REG_SNIFFTH_C        0x13
#define MC36XX_REG_SNIFF_CFG        0x14
#define MC36XX_REG_RANGE_C          0x15
#define MC36XX_REG_FIFO_C           0x16
#define MC36XX_REG_INTR_C           0x17
#define MC36XX_REG_CHIP_ID          0x18
#define MC36XX_REG_TRIM_C			0x1A
#define MC36XX_REG_OSR_C			0x1C
#define MC36XX_REG_DMX              0x20
#define MC36XX_REG_DMY              0x21
#define MC36XX_REG_DMZ              0x22
#define MC36XX_REG_RESET            0x24
#define MC36XX_REG_SECR			    0X25
#define MC36XX_REG_DCM_C			0X28
#define MC36XX_REG_XOFFL            0x2A
#define MC36XX_REG_XOFFH            0x2B
#define MC36XX_REG_YOFFL            0x2C
#define MC36XX_REG_YOFFH            0x2D
#define MC36XX_REG_ZOFFL            0x2E
#define MC36XX_REG_ZOFFH            0x2F
#define MC36XX_REG_XGAIN            0x30
#define MC36XX_REG_YGAIN            0x31
#define MC36XX_REG_ZGAIN            0x32
#define MC36XX_REG_OPT              0x3B
#define MC36XX_REG_LOC_X            0x3C
#define MC36XX_REG_LOC_Y            0x3D
#define MC36XX_REG_LOT_dAOFSZ       0x3E
#define MC36XX_REG_WAF_LOT          0x3F

#define MC3672_TEST		0x71










#endif /* APPS_INCLUDE_MC363X_H_ */
