/**
 ****************************************************************************************
 *
 * @file Mc363x.c
 *
 * @brief Accelerometer driver
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

/*
 * I2C ADDRESS [7bit] 0x4C or 0x6C\n
 * when chip is in sleep mode ,standby mode , you can set most of the registers except protected registers.\n
 * if chip is in cwake ,swake ,sniff mode, you can only set mode(0x10) register .\n

 * if you just want to read acc data at any time , use Cwake mode .\n
 * If you just want to  use the motion interrupt of Mc36xx ,Use Sniff mode .\n
 * if you want to read acc data at any time and use the motion interrupt source, use Swake mode .\n
 */


#include "sdk_type.h"
#include "da16x_types.h"
#include "common.h"
#include "Mc363x.h"
#include "da16x_system.h"
#include "app_common_util.h"
//#include "sys_i2c.h" - Commented out NJ 05/19/2022
/* prototypes */
void set_interface(void);

#define SIXBYTESONETIME  0
#define SIXTIMEBYTESONETIME 1

uint8_t resolutionlist[6]={6,7,8,10,12,14};
uint8_t rangelist[6]={2,4,8,16,12,24};


//Mc363X_All_Status mc363X_All_Status;


Mc363X_All_Status mc363X_All_Status ={
		.filen = 1,
		.fion = 0,
		.intr_type  = 0,
		.intr_status = 0,
		.work_mode = 0,
		.read_style = 0,
		.x_axis_disable = 0,
		.y_axis_disable = 0,
		.z_axis_disable = 0,
		.wakemode = MC36XX_WAKE_POWER_LOWPOWER,
		.sniffmode = MC36XX_SNIFF_POWER_PRECISION,
		.wakegain = MC36XX_WAKE_GAIN_LOW,
		.sniffgain = MC36XX_SNIFF_GAIN_HIGH,
	    .datagain = 1024,
	    .range = 0,//2g
	    .resolution=4,//12bit
};

//Brought in from app_command.c - NJ 05/19/2022

int i2cWriteStop(int addr, uint8_t *data, int length)
{
	int address = addr, trys;
	int status;

	// Set Address for Atmel eeprom AT24C512
	DRV_I2C_IOCTL(I2C, I2C_SET_CHIPADDR, &address);

	// Handle, buffer, length, stop enable, dummy
	for(trys = 0; trys < NUMBER_I2C_RETRYS; trys++)
	{
		status = DRV_I2C_WRITE(I2C, data,length, 1, 0);
		if(status)
			break;
	}
	return status;
}

int i2cWriteNoStop(int addr, uint8_t *data, int length)
{
	int address = addr, trys;
	int status,i;

	// Set Address for Atmel eeprom AT24C512
	DRV_I2C_IOCTL(I2C, I2C_SET_CHIPADDR, &address);

	// Handle, buffer, length, address length, dummy
	for(trys = 0; trys < 1; trys++)
	{
		status = DRV_I2C_WRITE(I2C, data,length, 1, 0);
		if(status)
			break;
	}
	return status;
}

int i2cRead(int addr, uint8_t *data, int length)
{
	int address = addr, trys;
	int status, i;

	// Set Address for Atmel eeprom AT24C512
	DRV_I2C_IOCTL(I2C, I2C_SET_CHIPADDR, &address);

	// Handle, buffer, length, address length, dummy
	for(trys = 0; trys < NUMBER_I2C_RETRYS; trys++)
	{
		status = DRV_I2C_READ(I2C, data, length, 1, 0);
		if(status)
			break;
	}
	return status;
}


//extern unsigned char mc_write_regs(unsigned char register_address, unsigned char *value, unsigned char number_of_bytes);
//extern unsigned char mc_read_regs(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes);

static void	MSLEEP(unsigned int	msec)
{
	portTickType xFlashRate, xLastFlashTime;

	if((msec/portTICK_RATE_MS) == 0)
	{
		xFlashRate = 1;
	}
	else
	{
		xFlashRate = msec/portTICK_RATE_MS;
	}
	xLastFlashTime = xTaskGetTickCount();
	vTaskDelayUntil( &xLastFlashTime, xFlashRate );
}



// call it  when chip is power on , or  you want to reset all the registers .
void reset_chip()
{
	unsigned char _bRegData = 0x01, buf[2];
	unsigned i = 0;
	int status;

//	mc_write_regs(MC36XX_REG_MODE_C, &_bRegData, 1);
	buf[0] = MC36XX_REG_MODE_C;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
//	mc_delay_ms(10);
//	vTaskDelay(3);
	MSLEEP(30);
	_bRegData = 0x40; // reset chip & reload registers
//	mc_write_regs(MC36XX_REG_RESET, &_bRegData, 1);
	buf[0] = MC36XX_REG_RESET;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
//	mc_delay_ms(50);
	MSLEEP(60);
	do{
		i++;
		if(i>=10)
			break;
//		mc_read_regs(MC36XX_REG_FEATURE_C_1, &_bRegData,1);
		buf[0] = MC36XX_REG_FEATURE_C_1;
		status = i2cRead(MC3672_ADDR, buf, 1);
		set_interface();
//		mc_delay_ms(1);
		MSLEEP(1);
	}
	#if !USING_IIC_36XX
	while((buf[0] & 0x80)!= 0x80);
	#else
	while((buf[0] & 0x40)!= 0x40);
	#endif

	_bRegData = 0x00;
//	mc_read_regs(MC36XX_REG_FEATURE_C_1, &_bRegData, 1);
	buf[0] = MC36XX_REG_FEATURE_C_1;
	status = i2cRead(MC3672_ADDR, buf, 1);
//	mc_printf("0x0d=%x\n",_bRegData);
//	APRINTF_Y("0x0d = 0x%02X %d\n",buf[0], i);

	_bRegData = 0x42;
//	mc_write_regs(MC36XX_REG_PWR_C, &_bRegData, 1);
	buf[0] = MC36XX_REG_PWR_C;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
	_bRegData = 0x01;
//	mc_write_regs(MC36XX_REG_DMX, &_bRegData, 1);
	buf[0] = MC36XX_REG_DMX;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
	_bRegData = 0x80;
//	mc_write_regs(MC36XX_REG_DMY, &_bRegData, 1);
	buf[0] = MC36XX_REG_DMY;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
	_bRegData = 0x00;
//	mc_write_regs(MC36XX_REG_DCM_C, &_bRegData, 1);
	buf[0] = MC36XX_REG_DCM_C;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
	_bRegData = 0x00;
//	mc_write_regs(MC36XX_REG_TRIM_C, &_bRegData, 1);
	buf[0] = MC36XX_REG_TRIM_C;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
	_bRegData = 0x01;
//	mc_write_regs(MC36XX_REG_MODE_C, &_bRegData, 1); //standby mode
	buf[0] = MC36XX_REG_MODE_C;
	buf[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, buf, 2);
#if 0
#endif
}

/////
//choose interface   spi or iic
////
void set_interface(void)
{
   unsigned char _bRegData = 0x00;
   int status ;

   #if USING_IIC_36XX
//	_bRegData = 0x50;
	_bRegData = 0x40;
   #else
	_bRegData = 0x90; 	//spi , iic 50
   #endif
//	mc_write_regs(MC36XX_REG_FEATURE_C_1, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_FEATURE_C_1; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
}

////
//  set chip mode , sleep , standby  , sniff  ,cwake ,swake
////
void set_mode(MC36XX_MODE mode)
{
	 unsigned char _bRegData = 0x00;
	 int status;

	 if(mode>MC36XX_MODE_PWAKE && mode<MC36XX_MODE_END)
	 {
	 	mc363X_All_Status.work_mode = mode;
		mode=(mode|(mc363X_All_Status.x_axis_disable<<4)|(mc363X_All_Status.y_axis_disable<<5)|((mc363X_All_Status.z_axis_disable<<6)));
//		mc_printf("CLOSE-x-y-z\r\n");
//		APRINTF_Y("CLOSE-x-y-z\r\n");
	 }
	  _bRegData = mode;
//	mc_write_regs(MC36XX_REG_MODE_C, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_MODE_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);


}

void set_wakegain(MC36XX_WAKE_GAIN wake_gain)
{
	unsigned char _bRegData = 0x01;
	 int status;

	 _bRegData=0x01;
//	 mc_write_regs(0x20, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_DMX; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	_bRegData = 0x00;
//	mc_read_regs(0x21, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_DMY; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	_bRegData &= 0x3F;
	_bRegData |= (wake_gain << 6);
//	mc_write_regs(0x21, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_DMY; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
	mc363X_All_Status.wakegain = wake_gain;
}

void set_sniffgain(MC36XX_SNIFF_GAIN sniff_gain)
{
	unsigned char _bRegData = 0x00;
	 int status;

	 _bRegData=0;
//	mc_write_regs(0x20, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_DMX; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
	_bRegData = 0x00;
//	mc_read_regs(0x21, &_bRegData, 1);
	_bRegData &= 0x3F;
	_bRegData |= (sniff_gain << 6 );
//	mc_write_regs(0x21, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_DMY; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
}

void set_range_resolution(MC36XX_RANGE range,MC36XX_RESOLUTION resolution)
{
	unsigned char _bRegData = 0x00;
	int status;

	 _bRegData = ((range<<4)|resolution);
//	mc_write_regs(0x15, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_RANGE_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
	mc363X_All_Status.range = rangelist[range];
	mc363X_All_Status.resolution = resolutionlist[resolution];
	mc363X_All_Status.datagain = (1<<resolutionlist[resolution])/(2*rangelist[range]);
	APRINTF_Y("range: %d g Resolution: %d bits gain: 0x%04X\r\n",mc363X_All_Status.range, mc363X_All_Status.resolution, mc363X_All_Status.datagain);
}

void set_power_mode(MC36XX_WAKE_POWER wakepowermode,MC36XX_SNIFF_POWER sniffpowermode)
{
	unsigned char _bRegData;//,premode;//= 0x00;
	int status;
	//mc_read_regs(MC36XX_REG_OSR_C, &premode, 1);
	//mc_printf("set_power_mode premode=%x \r\n", premode);

	//if(wakepowermode<=4)
	_bRegData = wakepowermode;

	_bRegData |= (sniffpowermode<<4);
	mc363X_All_Status.wakemode = wakepowermode;
	mc363X_All_Status.sniffmode = sniffpowermode;
	#if MC36XX_SPI_8M
	_bRegData |= 0x80;//0x80
	#endif
//	mc_printf("set_power_mode sniffpowermode=%x 0x1c=%x\r\n", sniffpowermode,_bRegData);
	APRINTF_Y("set_power_mode sniffpowermode=0x%x 0x1c=0x%x\r\n", sniffpowermode,_bRegData);

//	mc_write_regs(MC36XX_REG_OSR_C, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_OSR_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

}

void set_sniff_rate(MC36XX_SNIFF_SR rate)
{
    	unsigned char _bRegData ,_bRegPowerMode= 0x00;
    	int status;

		_bRegData = rate;
//		mc_write_regs(MC36XX_REG_SNIFF_C, &_bRegData, 1);
		i2c_data[0] = MC36XX_REG_SNIFF_C; 	//Word Address to Write Data. 2 Bytes.
		i2c_data[1] = _bRegData; 			// Data..
		status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
}

void set_fifo_Len(uint8_t len,uint8_t onoff,uint8_t readfeature)
{
	unsigned char _bRegData, buf[2] ;
	int status;

	if(onoff==1)
	{
		if(len!=0)
		//_bRegData=((onoff<<6)|len|0x20);//
		_bRegData=((onoff<<6)|len);//   				NJ 06/23/2022 - FIFO Threshold set to 30, FIFO continues to accept new data as long as there's space remaining in FIFO
		else
		_bRegData=((onoff<<6));
	}
	else
	_bRegData|=0x00;//0x80   if write 0x80 , you read all data '0'!

	if(resolutionlist[mc363X_All_Status.resolution]>12)
	{
		mc363X_All_Status.resolution = 4;
		mc363X_All_Status.datagain = (1<<resolutionlist[mc363X_All_Status.resolution])/(2*rangelist[mc363X_All_Status.range]);
	}

//	mc_write_regs(MC36XX_REG_FIFO_C, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_FIFO_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
//	mc_printf("0x16=%x\r\n", _bRegData);
	PRINTF("0x16=0x%x\r\n", _bRegData);
//	mc_read_regs(MC36XX_REG_FEATURE_C_2, &_bRegData, 1);
	buf[0] = MC36XX_REG_FEATURE_C_2;
	status = i2cRead(MC3672_ADDR, &_bRegData, 1);


	/* 																									NJ 04/26/2022 - Commented out lines 336-346 - Writing to 0x0E - already executed in set_clear_IntMethod()

	_bRegData = 0x00; //stream mode |0x20
	_bRegData |=(readfeature<<1);
//	mc_printf("0x0E=%x\r\n", _bRegData);
	PRINTF("0x0E=0x%x\r\n", _bRegData);
//	mc_write_regs(MC36XX_REG_FEATURE_C_2, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_FEATURE_C_2; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);
	*/
}

void set_wake_rate(MC36XX_CWAKE_SR rate)
{
	unsigned char _bRegData=0x00;
	int status;

	//  suppose using LOWPOWERMODE  !!!!!!!!
	rate-=MC36XX_CWAKE_SR_LP_DUMMY_BASE;
//	set_power_mode(MC36XX_WAKE_POWER_LOWPOWER,MC36XX_SNIFF_POWER_PRE);
	//mc_printf("wake rate=%d\r\n",rate);
//	mc_write_regs(MC36XX_REG_WAKE_C, &rate, 1);
	i2c_data[0] = MC36XX_REG_WAKE_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = rate; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

}


void set_sniff_motion_threshold(MC36XX_AXIS axis,unsigned char threshold,MC36XX_SNIFF_DETECT_MODE mode,uint8_t cnt )
{
	unsigned char _bRegData = 0x00;
	unsigned char sniff14 = 0x00, buf[2];
	int status;

//	mc_read_regs(MC36XX_REG_SNIFF_CFG, &sniff14, 1);
	buf[0] = MC36XX_REG_SNIFF_CFG;
	status = i2cRead(MC3672_ADDR, &sniff14, 1);
	switch(axis)
	{
	case MC36XX_AXIS_X:
		_bRegData = 0x01; //Put X-axis to active
		break;
	case MC36XX_AXIS_Y: //Put Y-axis to active
		_bRegData = 0x02;
		break;
	case MC36XX_AXIS_Z: //Put Z-axis to active
		_bRegData = 0x03;
		break;
	default:
		break;
	}
//	mc_write_regs(MC36XX_REG_SNIFF_CFG, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_SNIFF_CFG; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	_bRegData = threshold;
	_bRegData = (_bRegData|(mode<<7));
//	mc_write_regs(MC36XX_REG_SNIFFTH_C, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_SNIFFTH_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	if(cnt==0)
		return;
	switch(axis)
	{
	case MC36XX_AXIS_X:	//Put X-axis to active
		_bRegData = 0x05;
		break;
	case MC36XX_AXIS_Y: //Put Y-axis to active
		_bRegData = 0x06;
		break;
	case MC36XX_AXIS_Z: //Put Z-axis to active
		_bRegData = 0x07;
		break;
	default:
		break;
	}
	//sniff14 |= _bRegData ;
//	mc_write_regs(MC36XX_REG_SNIFF_CFG, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_SNIFF_CFG; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);


//	mc_read_regs(MC36XX_REG_SNIFFTH_C, &_bRegData, 1);
	buf[0] = MC36XX_REG_SNIFFTH_C;
	status = i2cRead(MC3672_ADDR, &_bRegData, 1);

	_bRegData |= cnt;
	_bRegData = (_bRegData|(mode<<7));
//	mc_write_regs(MC36XX_REG_SNIFFTH_C, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_SNIFFTH_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	sniff14 |= 0x08;
//	mc_write_regs(MC36XX_REG_SNIFF_CFG, &sniff14, 1);
	i2c_data[0] = MC36XX_REG_SNIFF_CFG; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

}

/////
// choose mode (c2p,c2b)  choose sniff rate  set threshold for three axis
/////
void set_sniff_motion_parameters(MC36XX_SNIFF_DETECT_MODE mode,unsigned char threshold,uint8_t cnt)
{
	set_sniff_motion_threshold(MC36XX_AXIS_X,threshold,mode,cnt);
	set_sniff_motion_threshold(MC36XX_AXIS_Y,threshold,mode,cnt);
	set_sniff_motion_threshold(MC36XX_AXIS_Z,threshold,mode,cnt);
}

/////
//set int type
////
void set_int_type(
        unsigned char bSwakeEnable,
		unsigned char bFifoThreshEnable,
		unsigned char bFifoFullEnable,
		unsigned char bFifoEmptyEnable,
		unsigned char bACQEnable,
		unsigned char bWakeEnable)
{

	unsigned char _bPreMode ,_bRegData= 0;
	int status;

	//mc_read_regs(M_DRV_MC36XX_REG_MODE_C, &_bPreMode, 1);
	//set_mode(MC36XX_MODE_STANDBY);

#if 0
	_bRegData =  (((bSwakeEnable & 0x01) << 7)
		      | ((bFifoThreshEnable & 0x01) << 6)
			  | ((bFifoFullEnable & 0x01) << 5)
			  | ((bFifoEmptyEnable & 0x01) << 4)
			  | ((bACQEnable & 0x01) << 3)
			  | ((bWakeEnable & 0x01) << 2)
			  | MC36XX_INTR_C_IAH_ACTIVE_LOW
			  | MC36XX_INTR_C_IPP_MODE_PUSH_PULL);
#else
	_bRegData =  (((bSwakeEnable & 0x01) << 7)
		      | ((bFifoThreshEnable & 0x01) << 6)
			  | ((bFifoFullEnable & 0x01) << 5)
			  | ((bFifoEmptyEnable & 0x01) << 4)
			  | ((bACQEnable & 0x01) << 3)
			  | ((bWakeEnable & 0x01) << 2)
			  | MC36XX_INTR_C_IAH_ACTIVE_HIGH
			  | MC36XX_INTR_C_IPP_MODE_PUSH_PULL);
#endif

//	mc_write_regs(MC36XX_REG_INTR_C, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_INTR_C; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	mc363X_All_Status.intr_type = _bRegData;
	//set_mode(_bPreMode&0x07);
}


void set_clear_Intmethod()
{
	unsigned char _bRegData , buf[2];
	int status;

// Edited to enable interrupt clearance only by reading register 0x09   -  NJ 06/21/2022
//	mc_read_regs(MC36XX_REG_FEATURE_C_2, &_bRegData, 1);
//	buf[0] = MC36XX_REG_FEATURE_C_2;
//	status = i2cRead(MC3672_ADDR, &_bRegData, 1);
//	_bRegData =(_bRegData); //stream mode |0x20

	_bRegData = 0;
//	mc_printf("clear-int-method 0x0E=%x\r\n", _bRegData);
//	APRINTF_Y("clear-int-method 0x0E=0x%x\r\n", _bRegData);
//	mc_write_regs(MC36XX_REG_FEATURE_C_2, &_bRegData, 1);
	i2c_data[0] = MC36XX_REG_FEATURE_C_2; 	//Word Address to Write Data. 2 Bytes.
	i2c_data[1] = _bRegData; 			// Data..
	status = i2cRead(MC3672_ADDR, i2c_data, 2);
}



/*
 * clear_intstate: Clear the MC3632 interrupt
 *                  Returns the contents of the interrupt status register (0x09)
 */
void clear_intstate(uint8_t* state)
{
	int status;
	unsigned char _bRegData, buf[2];

 //   mc_read_regs(MC36XX_REG_INTR_S, state,1);
	buf[0] = MC36XX_REG_INTR_S;
	status = i2cRead(MC3672_ADDR, buf, 1);
	*state = buf[0];

	// Commented out. Read register 0x09 will clear interrupt. No writes needed.
//	mc_write_regs(MC36XX_REG_INTR_S, state,1);
//	buf[0] = MC36XX_REG_INTR_S; 	//Word Address to Write Data. 2 Bytes.
//	buf[1] = *state; 			// Data..
//	status = i2cWriteStop(MC3672_ADDR, buf, 2);
//	mc_printf("clear int\r\n");
//	PRINTF("clear int 0x%02X\r\n",buf[1]);

	//Clear FIFO																		// NJ - 04/27/2022 - Added lines 537-544 to clear and reset contents of FIFO buffer each time interrupt is cleared - Interrupt is cleared after reading all 32 samples in buffer
//	_bRegData = 0x80;
//	i2c_data[0] = MC36XX_REG_FIFO_C; 	//Word Address to Write Data. 2 Bytes.
//	i2c_data[1] = _bRegData; 			// Data..
//	status = i2cWriteStop(MC3672_ADDR, i2c_data, 2);

	// Read status register 0x08 to see state of FIFO buffer
	// Commented out. Only needed for debug purposes -   			NJ 06/21/2022
//	i2c_data[0] = MC36XX_REG_STATUS_1; 	//Word Address to Write Data. 2 Bytes.
//	status = i2cRead(MC3672_ADDR, i2c_data, 1);
	//Printf("RESET FIFO\r\n");
}

int mc36xx_init(void)
{
	unsigned char buf[2]={0};
	uint8_t i,err=0;
	int status;

//	mc_read_regs(MC36XX_REG_CHIP_ID, buf,1);
	buf[0] = MC36XX_REG_CHIP_ID; 	//Word Address to Write Data. 2 Bytes.
	status = i2cRead(MC3672_ADDR, buf, 1);
//	mc_printf("chip id 0x18 = %x\r\n",buf[0]);
//	APRINTF_Y("chip id 0x18 = 0x%x status: %d\r\n",buf[0], status);
	if(buf[0] == MC3672_TEST)
	{
		sensorTypePresentAll = sensorTypePresentAll | (1 << SENSORTYPEMC3672);
	}
	set_interface();
	reset_chip();
	set_mode(MC36XX_MODE_STANDBY);
//	set_range_resolution(MC36XX_RANGE_4G,MC36XX_RESOLUTION_12BIT);// 2^12bit / (2*4G) = 2^9 =512 . Here ,512 = 1G .
	set_range_resolution(MC36XX_RANGE_4G,MC36XX_RESOLUTION_8BIT);// 2^8bit / (2*4G) = 2^5 = 32 . Here ,32 = 1G .				 			NJ 04/26/2022 - SET NEW RESOLUTION TO 8BITS
//	set_range_resolution(MC36XX_RANGE_2G,MC36XX_RESOLUTION_10BIT);// 2^10bit / (2*2G) = 2^7 =128 . Here ,128 = 1G .
	/*now using lowpower mode in this function*/
//	set_wake_rate(MC36XX_CWAKE_SR_LP_54Hz);
	set_wake_rate(MC36XX_CWAKE_SR_LP_14Hz);
	set_sniff_rate(MC36XX_SNIFF_SR_LP_13Hz);
//	set_sniff_rate(MC36XX_SNIFF_SR_LP_50Hz);

	set_clear_Intmethod(); // write 0x09 to clear intr state

	clear_intstate(&i);
#if 0
#endif
	return err;
}

void mc_timer_handle(void )
{

	 uint8_t i,fifo_length;
	 unsigned char rawdata[192]; //6*32
	 unsigned char buf[2], buf1[10];
	 int status;
	 int16_t Xvalue;
	 int16_t Yvalue;
	 int16_t Zvalue;


	if(mc363X_All_Status.fion==1)
	{
//		mc_read_regs(0x0A, rawdata,1);
		buf[0] = 0x0A; 	//Word Address to Write Data. 2 Bytes.
		status = i2cRead(MC3672_ADDR, rawdata, 1);
//		mc_printf(" 0x0a=%d\r\n",rawdata[0]);
		PRINTF(" 0x0a=%d\r\n",rawdata[0]);
		fifo_length = (rawdata[0]&0x3f);
//		mc_printf(" fifo_length=%d\n",fifo_length);
		PRINTF(" fifo_length=%d\n",fifo_length);

		if(fifo_length>30)
			fifo_length=30;

		for(i=0;i<fifo_length;i++){
//			mc_read_regs(MC36XX_REG_XOUT_LSB, rawdata,6);
//			buf[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
//			status = i2cRead(MC3672_ADDR, rawdata, 6);
//			Xvalue = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0]);
//			Yvalue = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2]);
//			Zvalue = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4]);
//			/*Xvalue = (short)(((unsigned short)rawdata[1+6*i] << 8) + (unsigned short)rawdata[0+6*i]);
//			Yvalue = (short)(((unsigned short)rawdata[3+6*i] << 8) + (unsigned short)rawdata[2+6*i]);
//			Zvalue = (short)(((unsigned short)rawdata[5+6*i] << 8) + (unsigned short)rawdata[4+6*i]);	*/
//			mc_printf("fifo Data %d, %d, %d\r\n",Xvalue,Yvalue,Zvalue);
//			PRINTF("fifo Data 0x%04X %d, 0x%04X %d, 0x%04X %d\r\n",Xvalue,Xvalue,Yvalue,Yvalue,Zvalue,Zvalue);
//			Xvalue = (Xvalue * 1000)/511;
//			Yvalue = (Yvalue * 1000)/511;
//			Zvalue = (Zvalue * 1000)/511;
			PRINTF("X: %d Y: %d Z: %d\r\n",lastXvalue,lastYvalue,lastZvalue);
			}

	}
	else
	{

//		mc_read_regs(MC36XX_REG_XOUT_LSB, rawdata,6);
//#if 0
//		for(i=0;i< 6;i++)
//		{
//			buf[0] = (MC36XX_REG_XOUT_LSB) + i; 	//Word Address to Write Data. 2 Bytes.
//			status = i2cRead(MC3672_ADDR, buf, 1);
//			PRINTF("i: %d 0x%02X\r\n",MC36XX_REG_XOUT_LSB + i,buf[0]);
//			rawdata[i] = buf[0];
//		}
//#else
//		rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
//		status = i2cRead(MC3672_ADDR, rawdata, 6);
//		PRINTF("status: %d\r\n",status);
//		for(i=0;i< 6;i++)
//		{
//			PRINTF("i: %d 0x%02X\r\n",MC36XX_REG_XOUT_LSB + i,rawdata[i]);
//		}
//#endif
#if 0
		Xvalue = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0]);
		Yvalue = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2]);
		Zvalue = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4]);
//		mc_printf("no fifo=%d,Data %d, %d, %d\r\n",Xvalue,Yvalue,Zvalue);
		PRINTF("no fifo Data 0x%04X %d, 0x%04X %d, 0x%04X %d\r\n",Xvalue,Xvalue,Yvalue,Yvalue,Zvalue,Zvalue);
		Xvalue = (Xvalue * 1000)/511;
		Yvalue = (Yvalue * 1000)/511;
		Zvalue = (Zvalue * 1000)/511;
#endif
		PRINTF("X: %d Y: %d Z: %d\r\n",lastXvalue,lastYvalue,lastZvalue);
	}

	return ;


}

#if 0
void mc_interrupt_handler(void)
{

	unsigned char rawdata[192];	//6*32
	unsigned char buf[0];
	uint8_t intrtype,i ;
	short rdata[32][3];
	int status;

//	mc_read_regs(MC36XX_REG_STATUS_1, rawdata,1);
	buf[0] = MC36XX_REG_STATUS_1; 	//Word Address to Write Data. 2 Bytes.
	status = i2cRead(MC3672_ADDR, rawdata, 1);
//	mc_printf("Int FIFO State0x08= %x \r\n",rawdata[0]);
	PRINTF("Int FIFO State0x08= 0x%x \r\n",rawdata[0]);

	//clear intr states      must do this      pay attention to 0xE bit4
	//if want to ignore 0x0e  ,  read  0x09 ,then write 0x09  .

	clear_intstate(&intrtype);

	if(( (intrtype&0x04) == 0x04 )) // sniff-intr INT_Wake
	{
		if( (rawdata[0]&0x04)==0x04 )
		{
//			mc_printf("sniff mode trigger to cwake \r\n");
			PRINTF("sniff mode trigger to cwake \r\n");
		}
//		 mc_delay_ms(300);
		MSLEEP(300);
		 set_mode(MC36XX_MODE_STANDBY);
		 //  delay time is needed ,See Datasheet 5.6 Mode State Machine Flow.
//		 mc_delay_ms(4);
		MSLEEP(4);
		 set_mode(MC36XX_MODE_SNIFF);

	}

	if(( (intrtype&0x80) == 0x80 )) //swake-intr
	{

	}

	if( (intrtype&0x40) == 0x40 ) //fito-threshold-intr
	{

		for(i=0;i<mc363X_All_Status.filen;i++)// filen = watermark
		{
//			mc_read_regs(MC36XX_REG_XOUT_LSB,rawdata,6);
			buf[0] = 0x0A; 	//Word Address to Write Data. 2 Bytes.
			status = i2cRead(0x0A, rawdata, 6);
			rdata[i][0] = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0]);
			rdata[i][1] = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2]);
			rdata[i][2] = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4]);
		}
//		mc_printf("Int Rawdata:  %d, %d, %d\r\n",rdata[0][0],rdata[0][1],rdata[0][2]);
		PRINTF("Int Rawdata:  %d, %d, %d\r\n",rdata[0][0],rdata[0][1],rdata[0][2]);

	}
	return ;

}
#endif

//void main(void)
//void mc3672Init(int FIFO_threshold)
void mc3672Init(void)
{
	uint32_t pin;

	// ************************************************
	// Accelerometer FIFO threshold that determines
	// the frequency of interrupts
	// ************************************************
	int FIFO_threshold = AXL_FIFO_INTERRUPT_THRESHOLD;

	PRINTF("----->mc3672Init called.  FIFO threshold %d\r\n", FIFO_threshold);

	APRINTF_Y("mc3672Init called\r\n");
	mc36xx_init();

	mc363X_All_Status.wakemode = MC36XX_WAKE_POWER_LOWPOWER;
	mc363X_All_Status.sniffmode =MC36XX_SNIFF_POWER_PRECISION;
	set_power_mode(mc363X_All_Status.wakemode,mc363X_All_Status.sniffmode);
	//Set range and resolution


#if 0
//  1 : use sniff mode and cwake mode  ,
	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	mc363X_All_Status.sniffgain = MC36XX_SNIFF_GAIN_HIGH;
	mc363X_All_Status.work_mode = MC36XX_MODE_SNIFF;
	set_wakegain(mc363X_All_Status.wakegain);
	set_sniffgain(mc363X_All_Status.sniffgain);
	set_sniff_motion_parameters(MC36XX_SNIFF_DETECT_MODE_C2B,4,0);
	// set interrupt source
	set_int_type(0,0,0,0,0,1); //sniff-intr
	set_mode(mc363X_All_Status.work_mode);
#endif

#if 0
//  2:  use swake mode with no fifo
	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	mc363X_All_Status.work_mode = MC36XX_MODE_SWAKE;

	set_sniff_motion_parameters(MC36XX_SNIFF_DETECT_MODE_C2P,4,0);
	set_int_type(1,0,0,0,0,0); //swake-intr
	set_wakegain(mc363X_All_Status.wakegain);
	set_mode(mc363X_All_Status.work_mode);
#endif

#if 0

//  3:  use swake mode with  fifo and [no fifo-intr no swake-intr]
	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	mc363X_All_Status.work_mode = MC36XX_MODE_SWAKE;
	mc363X_All_Status.filen = 10;// 0 stream mode , >0 using watermark mode  !!!
	mc363X_All_Status.fion = 1;
	mc363X_All_Status.read_style = SIXBYTESONETIME;
	set_fifo_Len(mc363X_All_Status.filen,mc363X_All_Status.fion,mc363X_All_Status.read_style);
	set_sniff_motion_parameters(MC36XX_SNIFF_DETECT_MODE_C2P,4,0);
	set_int_type(0,0,0,0,0,0); //fifo-thr-int
	set_wakegain(mc363X_All_Status.wakegain);
	set_mode(mc363X_All_Status.work_mode);
#endif

#if 0
//  4:  use swake mode with  fifo and  [ fifo-intr , swake-intr]
	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	mc363X_All_Status.work_mode = MC36XX_MODE_SWAKE;
	mc363X_All_Status.filen = 10;// 0 stream mode , >0 using watermark mode  !!!
	mc363X_All_Status.fion = 1;
	mc363X_All_Status.read_style = SIXBYTESONETIME;
	set_fifo_Len(mc363X_All_Status.filen,mc363X_All_Status.fion,mc363X_All_Status.read_style);
	set_sniff_motion_parameters(MC36XX_SNIFF_DETECT_MODE_C2P,4,0);
	set_int_type(1,1,0,0,0,0); //fifo-thr-int
	set_wakegain(mc363X_All_Status.wakegain);
	set_mode(mc363X_All_Status.work_mode);
#endif

#if 1 																									//NJ 04/26/2022 - Changed to enable this accelerometer configuration

//  5:  use cwake mode with fifo and fifo-intr

	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	//mc363X_All_Status.filen = /*10*/ 0;// 0 stream mode , >0 using watermark mode  !!!    				NJ 04/26/2022 - FIFO length set to 0 samples
//	mc363X_All_Status.filen = /*10*/ 30;// 0 stream mode , >0 using watermark mode  !!!    				NJ 06/23/2022 - FIFO length set to 30 samples
	// Set the FIFO threshold to the specified value
	mc363X_All_Status.filen = FIFO_threshold;// 0 stream mode , >0 using watermark mode  !!!    				NJ 06/23/2022 - FIFO length set to 30 samples
	mc363X_All_Status.fion = 1;		// FIFO threshold enabled
	mc363X_All_Status.read_style = SIXTIMEBYTESONETIME;													// NJ 04/26/2022 - Changed for burst (FIFO full read)
	mc363X_All_Status.work_mode = MC36XX_MODE_CWAKE;
	set_wakegain(mc363X_All_Status.wakegain);
	set_fifo_Len(mc363X_All_Status.filen,mc363X_All_Status.fion,mc363X_All_Status.read_style);
	//set_int_type(0,0,1,0,0,0); // fifo-full-intr  													    NJ 04/26/2022 - FIFO-FULL INTERRUPT
	set_int_type(0,1,0,0,0,0); // fifo-thr-intr  													    NJ 06/23/2022 - FIFO-FULL INTERRUPT
	set_mode(mc363X_All_Status.work_mode);
#endif

#if 0																									// NJ 04/26/2022 - Changed to disable this accelerometer configuration

//  5:  use cwake mode without fifo and fifo-intr

	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	mc363X_All_Status.work_mode = MC36XX_MODE_CWAKE;
	set_wakegain(mc363X_All_Status.wakegain);
	set_int_type(0,0,0,0,0,0);
	set_mode(mc363X_All_Status.work_mode);
#endif

#if 0 //Change modes to cwake with fifo - NJ 04/26/2022

//  6:  use cwake mode without fifo and acquition intr

	mc363X_All_Status.wakegain = MC36XX_WAKE_GAIN_LOW;
	mc363X_All_Status.work_mode = MC36XX_MODE_CWAKE;
	set_wakegain(mc363X_All_Status.wakegain);
	set_int_type(0,0,0,0,1,0);
	set_mode(mc363X_All_Status.work_mode);
#endif

	if(sensorTypePresentAll & (1 << SENSORTYPEMC3672))
	{
		APRINTF_Y("MC3672 Accelerometer present!\r\n");
	}
	else
	{
		APRINTF_Y("MC3672 Accelerometer not present!\r\n");
	}

	/*
	 * enable GPIOA[11] interrupt
	 */
//	pin = GPIO_PIN11;
//	GPIO_IOCTL(gpioa, GPIO_SET_INTR_ENABLE, &pin);
}

void accel_callback()
{

#if 1
	BaseType_t xHigherPriorityTaskWoken;

   /* xHigherPriorityTaskWoken must be initialised to pdFALSE.  If calling
	vTaskNotifyGiveFromISR() unblocks the handling task, and the priority of
	the handling task is higher than the priority of the currently running task,
	then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
	xHigherPriorityTaskWoken = pdFALSE;

	/* Unblock the handling task so the task can perform any processing necessitated
	    by the interrupt.  xHandlingTask is the task's handle, which was obtained
	    when the task was created. */
//	vTaskNotifyGiveIndexedFromISR( handleAccelTask, 0, &xHigherPriorityTaskWoken );
	vTaskNotifyGiveFromISR( handleAccelTask, &xHigherPriorityTaskWoken );

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
    The macro used to do this is dependent on the port and may be called
    portEND_SWITCHING_ISR. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


#else
	int status;
	uint8_t state;
	int16_t Xvalue;
	int16_t Yvalue;
	int16_t Zvalue;
	unsigned char rawdata[8];
#ifdef __TIME64__
	__time64_t now;
#else
	time_t now;
#endif /* __TIME64__ */
	rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
//	status = i2cRead(MC3672_ADDR, rawdata, 6);
	Xvalue = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0]);
	Yvalue = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2]);
	Zvalue = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4]);
//	da16x_time64_msec(NULL, &now);
	accelData[ptrAccelDataWr].Xvalue = (Xvalue * 1000)/511;
	accelData[ptrAccelDataWr].Yvalue = (Yvalue * 1000)/511;
	accelData[ptrAccelDataWr].Zvalue = (Zvalue * 1000)/511;
	accelData[ptrAccelDataWr].accelTime = now;
	ptrAccelDataWr++;
	if(ptrAccelDataWr >= NUMBER_SAMPLES_TOTAL)
		ptrAccelDataWr = 0;
//	clear_intstate(&state);
#endif
}


