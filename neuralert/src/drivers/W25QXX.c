/**
 ****************************************************************************************
 *
 * @file W25QXX.c
 *
 * @brief External flash driver
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

#include "da16x_system.h"
//#include "spi_flash/spi_flash.h"
#include "W25QXX.h"


#define SPI_SFLASH_ADDRESS 0x1000
#define	TEST_DBG_DUMP(tag, buf, siz)	thread_peripheral_dump( # buf , buf , siz)
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

#define GET_BIT(reg, loc)	((reg) & ((0x1) << (loc)))
//#define SPI_FLASH_PRINT(...)	PRINTF(__VA_ARGS__)
#define SPI_FLASH_PRINT(...)

#undef  SPI_FLASH_PECYCLE_GUARD

#define SPI_FLASH_LOCK(handler, ioctldata, boolval)      {                      \
                ioctldata[0] = boolval;                                         \
                ioctldata[1] = portMAX_DELAY;                                   \
                ioctldata[2] = handler->spi_cs;                                 \
                SPI_IOCTL(handler->spi,SPI_SET_LOCK, ioctldata);                \
        }


static  UINT32 _spi_instance = 0;
SemaphoreHandle_t _spi_flash_semaphore = NULL;

//PATCHED
HANDLE flash_open(UINT32 spi_clock, UINT32 spi_cs) {
	UINT32 ioctldata[4];
	spi_flash_t *spi_flash;
	INT32 status;

	spi_flash = (spi_flash_t*) pvPortMalloc(sizeof(spi_flash_t));

	if (spi_flash == NULL) {
		PRINTF("spi_flash_open: unable to allocate handle\n");
		return NULL;
	}

	if (_spi_flash_semaphore == NULL) {
		 vSemaphoreCreateBinary(_spi_flash_semaphore);
	}
	configASSERT(_spi_flash_semaphore);
//	configASSERT(xSemaphoreTake(_spi_flash_semaphore, 10000 / portTICK_PERIOD_MS) == pdTRUE);
	xSemaphoreTake(_spi_flash_semaphore, portMAX_DELAY);

	spi_flash->spi = SPI_CREATE(SPI_UNIT_0);

	if (spi_flash->spi == NULL) {
		PRINTF("spi_flash_open: unable to create handle\n");
		vPortFree(spi_flash);
		xSemaphoreGive(_spi_flash_semaphore);
		return NULL;
	}

	spi_flash->spi_clock = spi_clock;
	spi_flash->spi_cs = spi_cs;

	if( _spi_instance == 0){
			ioctldata[0] = (1 * MBYTE);
			SPI_IOCTL(spi_flash->spi, SPI_SET_MAX_LENGTH, ioctldata);

			SPI_FLASH_PRINT("spi_cs: %d\n", spi_flash->spi_cs);

			_sys_clock_read(ioctldata, sizeof(UINT32));
			SPI_FLASH_PRINT("Core Clk - %d Hz\n", ioctldata[0]);

			SPI_IOCTL(spi_flash->spi, SPI_SET_CORECLOCK, ioctldata);

			SPI_FLASH_PRINT("spi_clock: %d\n", spi_flash->spi_clock);
			ioctldata[0] = spi_flash->spi_clock * MHz;

			SPI_FLASH_PRINT("Set SPI Clk - %d Hz\n", ioctldata[0]);
			SPI_IOCTL(spi_flash->spi, SPI_SET_SPEED, ioctldata);

			ioctldata[0] = SPI_TYPE_MOTOROLA_O0H0;
			SPI_IOCTL(spi_flash->spi, SPI_SET_FORMAT, ioctldata);

			ioctldata[0] = SPI_DMA_MP0_BST(8) | SPI_DMA_MP0_IDLE(1)
			| SPI_DMA_MP0_HSIZE(XHSIZE_DWORD)
			| SPI_DMA_MP0_AI(SPI_ADDR_INCR)
			;
			SPI_IOCTL(spi_flash->spi, SPI_SET_DMA_CFG, ioctldata);
			SPI_IOCTL(spi_flash->spi, SPI_SET_DMAMODE, NULL);

			ioctldata[0] = spi_flash->spi_cs;
			ioctldata[1] = 4; // 4-wire
			SPI_IOCTL(spi_flash->spi, SPI_SET_WIRE, (VOID*) ioctldata);

			ioctldata[0] = SPI_DELAY_INDEX_LOW;
			SPI_IOCTL(spi_flash->spi, SPI_SET_DELAY_INDEX, ioctldata);

			status = SPI_INIT(spi_flash->spi);


			if (status == TRUE) {
		//			PRINTF("SPI initialization succeeded.\n");
				} else {
					PRINTF("SPI failed to initialize.\n");
					xSemaphoreGive(_spi_flash_semaphore);
					return NULL;
			}

			if (SPI_IOCTL(spi_flash->spi, SPI_GET_SPEED, ioctldata) == TRUE) {
				SPI_FLASH_PRINT("SPI - %d Hz\n", ioctldata[0]);
				if ((spi_flash->spi_clock * MHz) < ioctldata[0]) {
					SPI_FLASH_PRINT("SPI - %d Hz (wrong)!!\n", ioctldata[0]);
					xSemaphoreGive(_spi_flash_semaphore);
					return NULL;
				} else {
					SPI_FLASH_PRINT("SPI - %d Hz\n", ioctldata[0]);
				}
			}
	}

	_spi_instance++;

	xSemaphoreGive(_spi_flash_semaphore);

	return spi_flash;
}

//PATCHED
int flash_close(HANDLE handler) {
	spi_flash_t *spi_flash;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;

	configASSERT(_spi_flash_semaphore);
	xSemaphoreTake(_spi_flash_semaphore, portMAX_DELAY);

	if (spi_flash->spi != NULL) {
		SPI_CLOSE(spi_flash->spi);
		spi_flash->spi = NULL;
	}

	if(_spi_instance != 0){
		_spi_instance--;
	}

	xSemaphoreGive(_spi_flash_semaphore);

	vPortFree(spi_flash);
	return TRUE;
}


//PATCHED
int readJEDEC(HANDLE handler, UINT8 *rx_buf) {
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	int status = 0;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;

	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);

	/*
	 * READ PRODUCT IDENTIFICATION BY JEDEC ID OPERATION (RDJDID, 9FH)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 9Fh\n");
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
			| SPI_SPI_PHASE_ADDR_3BYTE | SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
			);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);
	status =  SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_JEDEC_ID, 0x00, 0x00, NULL, 0,
			rx_buf, 3);

	SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);

	return status;
}

//PATCHED
int readStatReg1(HANDLE handler, UINT8 *rx_buf) {
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	INT32 status;
	UINT32 addrbyte = 3;
	UINT16 rsdr = 0;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;

	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);
	/*
	 * READ STATUS REGISTER1 (RSR1, 05h)
	 */
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
					| ((addrbyte == 3) ?
							SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
					| SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
					);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, rx_buf,
					1);

	SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);

	SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);

	return TRUE;
}

//PATCHED
int readStatReg2(HANDLE handler, UINT8 *rx_buf) {
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	INT32 status;
	UINT32 addrbyte = 3;
	UINT16 rsdr = 0;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;

	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);

	/*
	 * READ STATUS REGISTER2 (RSR2, 35h)
	 */
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
					| ((addrbyte == 3) ?
							SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
					| SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
					);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG2, 0x00, 0x00, NULL, 0, rx_buf,
					1);

	SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);

	SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);

	return TRUE;
}

//PATCHED
int readStatReg3(HANDLE handler, UINT8 *rx_buf) {
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	INT32 status;
	UINT32 addrbyte = 3;
	UINT16 rsdr = 0;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;
	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);

	/*
	 * READ STATUS REGISTER3 (RSR3, 15h)
	 */
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
					| ((addrbyte == 3) ?
							SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
					| SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
					);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG3, 0x00, 0x00, NULL, 0, rx_buf,
					1);

	SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);

	SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);

	return TRUE;
}

//PATCHED
int eraseSector_4K(HANDLE handler, UINT32 address) {
#if 1
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	INT32 status;
	UINT8 counter = 0;
	UINT32 addrbyte = 3;
	UINT16 rsdr = 0;
	UINT8 timeout;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;

	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);

	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 05h, RDSR\n");

	//Code will hang if there is a SPI flash action already processing a previous command
	//Exits while loop when device is no longer busy
	timeout = 0;
	counter = 100;

	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 0) == 0x0)
			break;

//		counter++;
//		if(counter > 15)
//		{
//			timeout = 1;
//			break;
//		}
		SYSUSLEEP(1000);
	};

	if(counter == 0)
	{
		Printf("eraseSector_4K: chip busy pre-check timeout\n");
	}

	/*
	 * WRITE ENABLE OPERATION (WREN, 06H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-0] 06h, WREN\n");
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
			| SPI_SPI_PHASE_ADDR_3BYTE | SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(0,0,SPI_BUS_SPI) // DATA
			);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);
	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_WRITE_ENABLE, 0x00, 0x00, NULL, 0,
			NULL, 0);
	SPI_FLASH_PRINT("status: %d\n", status);

	/*
	 * Make sure that the Write Enable has taken effect
	 * READ STATUS REGISTER OPERATION (RDSR, 03H)
	 */
	//Exits while loop when device is completes write enable operation
//	timeout = 0;
	counter = 100;
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 03h, RDSR\n");
	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 1) == 0x2)
			break;

//		counter++;
//		if(counter > 15)
//		{
//			timeout = 1;
//			break;
//		}
		SYSUSLEEP(1000);
	};

	if(counter == 0)
	{
		Printf("eraseSector_4K: pre-erase read status register WREN timeout\n");
	}

	/*
	 * SECTOR ERASE OPERATION (SER, D7H/20H)  - Erases 4K
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-1-0-0] 20h, SER\n");
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
			| SPI_SPI_PHASE_ADDR_3BYTE | SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(1,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(0,0,SPI_BUS_SPI) // DATA
			);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);
	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_SECTOR_ERASE_4K, address, 0x00, NULL, 0,
			NULL, 0);
	SPI_FLASH_PRINT("status: %d\n", status);


        // Waiting until ERASE-op is complete.
	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 03h, RDSR\n");
	counter = 100;
	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 0) == 0x0)
			break;

		SYSUSLEEP(10000);
	}

	if(counter == 0)
		{
			Printf("eraseSector_4K: busy post-erase read status register\n");
		}



	SPI_FLASH_PRINT("4K Sector Erase Complete\n");

	SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);
	return 1;
#endif
}


//PATCHED
int pageWrite(HANDLE handler, UINT32 address, UINT8 *tx_buf,
		UINT32 tx_len) {
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	INT32 status;
	UINT8 counter = 0;
	UINT32 addrbyte = 3;
	UINT16 rsdr = 0;

	if (handler == NULL) {
		return -1;
	}

	spi_flash = (spi_flash_t*) handler;

	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);

	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 05h, RDSR\n");
	counter = 100;
	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 0) == 0x0)
			break;
//		Printf("pageWrite: Busy 1  Statreg 1: %02x\n", rsdr);
//		counter++;
//		if(counter > 15)
//			break;
		 SYSUSLEEP(1000);
	}
	configASSERT(counter);
	counter = 100;

	/*
	 * WRITE ENABLE OPERATION (WREN, 06H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-0] 06h, WREN\n");
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
			| SPI_SPI_PHASE_ADDR_3BYTE | SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(0,0,SPI_BUS_SPI) // DATA
			);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);
	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_WRITE_ENABLE, 0x00, 0x00, NULL, 0,
			NULL, 0);
	SPI_FLASH_PRINT("status: %d\n", status);

	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 05h, RDSR\n");
	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 1) == 0x2)
			break;
//		Printf("pageWrite: Busy 2  Statreg 1: %02x\n", rsdr);
//		counter++;
//		if(counter > 15)
//		{
//			Printf("pageWrite: Timeout on Write Enable\n");
//			break;
//		}
		 SYSUSLEEP(1000);
	}

	configASSERT(counter);
	counter = 100;

	/*
	 * PAGE PROGRAM OPERATION (PP, 02H)
	 */

	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-1-0-1] 02h, addr %dB\n", addrbyte);
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
			| ((addrbyte == 3) ?
					SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
			| SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(1,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
			);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

//	vTaskDelay(1);

	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_PAGE_PROGRAM, address, 0x00, tx_buf,
			tx_len, NULL, 0);


        // Waiting until PROGRAM-op is complete.
	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 03h, RDSR\n");
	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 0) == 0x0)
			break;

		SYSUSLEEP(1000);
	}

	configASSERT(counter);
	counter = 0;

	SPI_FLASH_PRINT("Page write complete\n");
    SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);

	return status;
}

//PATCH
int pageRead(HANDLE handler, UINT32 address, UINT8 *rx_buf, UINT32 rx_len) {
	spi_flash_t *spi_flash;
	UINT32 busctrl[3];
	INT32 status;
	UINT8 counter = 0;
	UINT8 timeout = 0;
	UINT32 addrbyte = 3;
	UINT16 rsdr = 0;

	if (handler == NULL) {
		return -1;
	}

	spi_flash = (spi_flash_t*) handler;

	SPI_FLASH_LOCK(spi_flash, busctrl, TRUE);

	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 05h, RDSR\n");
	counter = 100;
//	timeout = 0;
	while (counter--) {
		busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
				| ((addrbyte == 3) ?
						SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
				| SPI_SET_SPI_DUMMY_CYCLE(0);
		busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
				SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
				);
		SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

		SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
				1);
		SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
		if (GET_BIT(rsdr, 0) == 0x0)
			break;
//		counter++;
//		if(counter > 15)
//		{
//			timeout = 1;
//			break;
//		}
		SYSUSLEEP(1000);
	};

	if(counter == 0)
	{
		SPI_FLASH_PRINT("pageRead: Pre-read busy check timeout\n");
	}

	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-1-0-1] 03h, addr %dB\n", addrbyte);
	busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
			| ((addrbyte == 3) ?
					SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
			| SPI_SET_SPI_DUMMY_CYCLE(0);
	busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(1,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
			SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
			);
	SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);
	status = SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_DATA, address, 0x00, NULL, 0,
			rx_buf, rx_len);

	/*
	 * READ STATUS REGISTER OPERATION (RDSR, 05H)
	 */
	SPI_FLASH_PRINT(">>>>>>\n[SPI  1-0-0-1] 05h, RDSR\n");
	counter = 100;
		while (counter--) {
			busctrl[0] = SPI_SPI_TIMEOUT_EN | SPI_SPI_PHASE_CMD_1BYTE
					| ((addrbyte == 3) ?
							SPI_SPI_PHASE_ADDR_3BYTE : SPI_SPI_PHASE_ADDR_4BYTE)
					| SPI_SET_SPI_DUMMY_CYCLE(0);
			busctrl[1] = SPI_SET_SPI_BUS_TYPE(SPI_BUS_TYPE(1,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(0,0,SPI_BUS_SPI), SPI_BUS_TYPE(0,0,SPI_BUS_SPI),
					SPI_BUS_TYPE(1,0,SPI_BUS_SPI) // DATA
					);
			SPI_IOCTL(spi_flash->spi, SPI_SET_BUSCONTROL, busctrl);

			SPI_SFLASH_TRANSMIT(spi_flash->spi, W25QXX_COMMAND_READ_STATUS_REG1, 0x00, 0x00, NULL, 0, &rsdr,
					1);
			SPI_FLASH_PRINT("RDSR: 0x%x\n", rsdr);
			if (GET_BIT(rsdr, 0) == 0x0)
				break;
			SYSUSLEEP(1);
		}
		if(counter == 0)
		{
			SPI_FLASH_PRINT("pageRead: Post-read busy check timeout\n");
		}

	SPI_FLASH_PRINT("Page read complete\n");

	SPI_FLASH_LOCK(spi_flash, busctrl, FALSE);
	return status;
}


//PATCH
int w25q64Init(HANDLE handler, UINT8 *rx_buf){

	spi_flash_t *spi_flash;

	if (handler == NULL) {
		return FALSE;
	}

	spi_flash = (spi_flash_t*) handler;

	readJEDEC(spi_flash, rx_buf);
//	TEST_DBG_DUMP(0, rx_buf, 3);

	/* Read Status Register 1 */
	readStatReg1(spi_flash, rx_buf);
//	TEST_DBG_DUMP(0, rx_buf, 1);

	/* Read Status Register 2 */
	readStatReg2(spi_flash, rx_buf);
//	TEST_DBG_DUMP(0, rx_buf, 1);

	/* Read Status Register 3 */
	readStatReg3(spi_flash, rx_buf);
//	TEST_DBG_DUMP(0, rx_buf, 1);

//	eraseChip(spi_flash);
//	eraseSector_4K(spi_flash, SPI_SFLASH_ADDRESS);
	return TRUE;

}
