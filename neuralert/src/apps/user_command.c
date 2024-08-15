/**
 ****************************************************************************************
 *
 * @file user_command.c
 *
 * @brief Console command specified by customer
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

#include "user_command.h"

//JW: EVERYTHING IN THIS File that is wrapped in if defined(__TCP_CLIENT_SLEEP2_SAMPLE__) was added
#include "app_common_util.h"
#include "app_provision.h"
// #include "app_aws_user_conf.h" - Not needed - NJ - 05/19/2022
#include "util_api.h"
#include "common.h"
#include "adc.h"
#include "Mc363x.h" //JW: The x was the wrong case previously.
#include "user_nvram_cmd_table.h"
#include "W25QXX.h"

/* globals */
// Timers for controlling the LED blink
	HANDLE	dtimer0, dtimer1;

// Helper function(s)
void user_text_copy(UCHAR *to_string, UCHAR *from_string, int max_len);
void time64_msec_string (UCHAR *time_str, __time64_t *time_msec);



#include "assert.h"
#include "clib.h"
struct phy_chn_info
{
    uint32_t info1;
    uint32_t info2;
};

UCHAR user_power_level[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
UCHAR user_power_level_dsss[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };




/* External global functions */
extern int fc80211_set_tx_power_table(int ifindex, unsigned int *one_regcode_start_addr, unsigned int *one_regcode_start_addr_dsss);
extern int fc80211_set_tx_power_grade_idx(int ifindex, int grade_idx , int grade_idx_dsss );
extern void phy_get_channel(struct phy_chn_info *info, uint8_t index);


// Added entire command list from previous software version for debug purposes using the command-line - NJ 05/19/2022
//-----------------------------------------------------------------------
// Command User-List
//-----------------------------------------------------------------------
////////////////////  APPS Test command  ////////////////////////////////
//void cmd_app_ap_mode_reset(int argc, char *argv[]);
//void cmd_app_ap_mode_prov_start(int argc, char *argv[]);
//void cmd_app_sta_mode_reset(int argc, char *argv[]);
//extern void cmd_app_1N_provisioning_start(int argc, char *argv[]);
void cmd_app_test(int argc, char *argv[]);
void cmd_accel(int argc, char *argv[]);
void cmd_adc(int argc, char *argv[]);
//void cmd_duty_cycle(int argc, char *argv[]);
void cmd_i2cread(int argc, char *argv[]);
void cmd_i2cwrite(int argc, char *argv[]);
void cmd_led(int argc, char *argv[]);
void cmd_led_state(int argc, char *argv[]);
void cmd_run(int argc, char *argv[]);
void cmd_log(int argc, char *argv[]);
void cmd_flash(int argc, char *argv[]);

void cmd_rf_ctl(int argc, char *argv[]); //Added command function for RF control - NJ 05/19/2022


typedef	struct	{
	HANDLE			timer;
//	OAL_EVENT_GROUP	*event;
	UINT32			cmd;
} TIMER_INFO_TYPE;

#define	DTIMER_CMD_RUN			0x0
#define	DTIMER_CMD_STOP			0x1

#define DTIMER_FLAG_0			0x4
#define DTIMER_FLAG_1			0x8



/////////////////////////////////////////////////////////////////////////
#if defined (__SUPPROT_ATCMD__) // aws + atcmd
extern void aws_iot_set_feature(int argc, char *argv[]);
extern void aws_iot_config_thing(int argc, char *argv[]);
extern void aws_iot_at_push_com(int argc, char *argv[]);
extern void aws_iot_at_response_ok(int argc, char *argv[]);


const COMMAND_TREE_TYPE	cmd_user_list[] = {
	////////////////////  APPS Test command  ////////////////////////////////

	{ "AWSIOT",			CMD_MY_NODE,	cmd_user_list,	NULL,							"aws iot at command"	},	// Head

	{ "-------",		CMD_FUNC_NODE,	NULL,			NULL,							"--------------------------------"	},
	{ "SET",			CMD_FUNC_NODE,	NULL,			&aws_iot_set_feature,			"set feature to run aws-iot thing"	},
	{ "CFG",			CMD_FUNC_NODE,	NULL,			&aws_iot_config_thing,			"config  attribute about Thing"	},
	{ "CMD",			CMD_FUNC_NODE,	NULL,			&aws_iot_at_push_com,			"save command to NVRAM"	},
	{ "OK",				CMD_FUNC_NODE,	NULL,			&aws_iot_at_response_ok,		"OK response" }, // AT+AWS OK ... from MCU

	/////////////////////////////////////////////////////////////////////////
	{ NULL, 			CMD_NULL_NODE,	NULL,			NULL,							NULL	}	// Tail
};
#else   // aws - Added rfctl to command list NJ 05/11/2022
const COMMAND_TREE_TYPE	cmd_user_list[] = {
	{ "user",			CMD_MY_NODE,	cmd_user_list,	NULL,							"User cmd "	},
	{ "-------",		CMD_FUNC_NODE,	NULL,			NULL,							"--------------------------------"	},
	{ "rfctl",			CMD_FUNC_NODE,	NULL,			&cmd_rf_ctl,					"rfctl [option] on off ex) rfctl off"		},
	{ "accel",			CMD_FUNC_NODE,	NULL,			&cmd_accel,						"accel [x]"		},
	{ "adc",			CMD_FUNC_NODE,	NULL,			&cmd_adc,						"adc"		},
	{ "flash",			CMD_FUNC_NODE,	NULL,			&cmd_flash,						"flash read [address] or flash info or flash help"		},
	{ "i2cread",		CMD_FUNC_NODE,	NULL,			&cmd_i2cread,					"i2cread [address] [length]"		},
	{ "i2cwrite",		CMD_FUNC_NODE,	NULL,			&cmd_i2cwrite,					"i2cwrite address data"		},
	{ "led",			CMD_FUNC_NODE,	NULL,			&cmd_led,						"led l s"	},
	{ "ledstate",		CMD_FUNC_NODE,	NULL,			&cmd_led_state,					"ledstate l s"	},
	{ "log",			CMD_FUNC_NODE,	NULL,			&cmd_log,						"log read [entry #] or log info or log help"	},
	{ "run",			CMD_FUNC_NODE,	NULL,			&cmd_run,						"run [0/1]"					},
    { "-------",     	CMD_FUNC_NODE,  NULL,          	NULL,             				"--------------------------------" },
    { "testcmd",     	CMD_FUNC_NODE,  NULL,           &cmd_test,        				"testcmd [option]"                 },
#if defined(__COAP_CLIENT_SAMPLE__)
    { "coap_client", CMD_FUNC_NODE,  NULL,          &cmd_coap_client, "CoAP Client"                      },
#endif /* __COAP_CLIENT_SAMPLE__ */
	{ "utxpwr",      CMD_FUNC_NODE,  NULL,          &cmd_txpwr,       "testcmd [option]"                 },

#if !defined (__BLE_COMBO_REF__)
	////////////////////  APPS Test command  ////////////////////////////////
#if defined(BUILD_APPS_CMD_APMODE)
	{ "apreboot",		CMD_FUNC_NODE,	NULL,			&cmd_app_ap_mode_reset, 		"ap mode reset"	},
	{ "apstart",		CMD_FUNC_NODE,	NULL,			&cmd_app_ap_mode_prov_start, 	"ap mode provisioning start"	},
#endif
#if defined(BUILD_APPS_CMD_1N)
	{ "nreboot",		CMD_FUNC_NODE,	NULL,			&cmd_app_sta_mode_reset, 		"1:n provisioning reset"	},
	{ "nstart",			CMD_FUNC_NODE,	NULL,			&cmd_app_1N_provisioning_start, "1:n provisioning start"	},
#endif
	/////////////////////////////////////////////////////////////////////////
#endif

	{ NULL, 			CMD_NULL_NODE,	NULL,			NULL,							NULL	}	// Tail
};
#endif  // (__SUPPROT_ATCMD__)



int freq_to_channel(unsigned char band, unsigned short freq)
{
    int channel = 0;

    // 2.4.GHz
    if (band == 0) {
        // Check if frequency is in the expected range
        if ((freq < 2412) || (freq > 2484)) {
            PRINTF("Wrong channel\n");
            return (channel);
        }

        // Compute the channel number
        if (freq == 2484) {
            channel = 14;
        } else {
            channel = (freq - 2407) / 5;
        }
    }
    // 5 GHz
    else if (band == 1) {
        assert(0);

        // Check if frequency is in the expected range
        if ((freq < 5005) || (freq > 5825)) {
            PRINTF("Wrong channel\n");
            return (channel);
        }

        // Compute the channel number
        channel = (freq - 5000) / 5;
    }

    return (channel);
}

int get_current_freq_ch(void)
{
    struct phy_chn_info phy_info;
    unsigned char band;
    unsigned short freq;
    int chan;
#define    PHY_PRIM    0

    phy_get_channel(&phy_info, PHY_PRIM);
    band = phy_info.info1 & 0xFF;
    freq = phy_info.info2 & 0xffff;

    chan = freq_to_channel(band, freq);

    PRINTF("BAND:            %d\n"
           "center_freq:    %d MHz\n"
           "chan:    %d \n",    band, freq, chan);
    return chan;
}





//
//-----------------------------------------------------------------------
// Internal Functions
//-----------------------------------------------------------------------

void cmd_test(int argc, char *argv[])
{    
    if (argc < 2) {
        PRINTF("Usage: testcmd [option]\n   ex) testcmd test\n\n");
        return;
    }

    PRINTF("\n### TEST CMD : %s ###\n\n", argv[1]);
}

void cmd_txpwr(int argc, char *argv[])
{    
    int i;
    int channel, pwr_mode, power, power_dsss;

    if (argc != 30) {
        goto error;
    }

    pwr_mode = (int)ctoi(argv[1]);
    PRINTF("TX PWR : [%s] ", (pwr_mode)?"AP":"STA");
    PRINTF("\n OFDM \n");
    for (i = 0; i < 14; i++) {
        user_power_level[i] = (UCHAR)htoi(argv[i + 2]);

        if (user_power_level[i] > 0x0f) {
            PRINTF("..................\n");
            goto error;
        } else {
            PRINTF("CH%02d[0x%x] ", i + 1, user_power_level[i]);
        }

        if (i == 6)
            PRINTF("\n");
    }

    PRINTF("\n DSSS \n");
    for (i = 14; i < 28; i++) {
        user_power_level_dsss[i - 14] = (UCHAR)htoi(argv[i + 2]);

        if (user_power_level_dsss[i - 14] > 0x0f) {
            PRINTF("..................\n");
            goto error;
        } else {
            PRINTF("CH%02d[0x%x] ", i + 1, user_power_level_dsss[i - 14]);
        }

        if (i == 20)
            PRINTF("\n");
    }

    PRINTF("\n");
    channel = get_current_freq_ch();
    power = user_power_level[channel-1];
    power_dsss = user_power_level_dsss[channel-1];

    fc80211_set_tx_power_table(pwr_mode, (unsigned int *)&user_power_level, (unsigned int *)&user_power_level_dsss);
    fc80211_set_tx_power_grade_idx(pwr_mode, power, power_dsss);

    PRINTF("TX PWR set channel[%d], pwr [0x%x]\n", channel, power);

    return;


error:
    PRINTF("\n\tUsage : utxpwr mode [CH1 CH2 ... CH13 CH14] [CH1 CH2 ... CH13 CH14 [DSSS]]\n");
    PRINTF("\t\tCH    0~f\n");
    PRINTF("\t\tex) utxpwr 0 1 1 1 1 1 1 1 1 1 1 1 f f f 2 3 3 3 3 3 3 3 3 3 1 f f f\n");

    return;
}

#if defined(__COAP_CLIENT_SAMPLE__)
void cmd_coap_client(int argc, char *argv[])
{
    extern void coap_client_sample_cmd(int argc, char *argv[]);

    coap_client_sample_cmd(argc, argv);

    return;
}
#endif /* __COAP_CLIENT_SAMPLE__ */



//
//-----------------------------------------------------------------------
// Internal Functions
//-----------------------------------------------------------------------

void cmd_rf_ctl(int argc, char *argv[])
{
	extern void wifi_cs_rf_cntrl(int flag);

	if (argc < 2)
	{
		PRINTF("Usage: rfctl [option] on off ex) rfctl off\n\n");
		return;
	}

	if (strcasecmp(argv[1], "on") == 0){
		wifi_cs_rf_cntrl(0);
		PRINTF("\n### Wi-Fi RF is On ###\n\n");
	}
	else if (strcasecmp(argv[1], "off") == 0){
		wifi_cs_rf_cntrl(1);
		PRINTF("\n### Wi-Fi RF is Off ###\n\n");
	}
	else{
		PRINTF("Usage: rfctl [option] on off ex) rfctl off\n\n");
		return;
	}
}

/*
 * Set one LED to a state
 */
void setLEDState(uint8_t number1, 	// color A
				uint8_t state1, 	//
				uint16_t count1, 	// probably 1/8ths duration in state A
				uint8_t number2,  	// color B
				uint8_t state2, 	//
				uint16_t count2, 	// probably 1/8ths duration in state B
				uint16_t secondsTotal)  // length of time to do this
{
	UINT32	ioctldata[3];

	/* ledColor is color
	 * 000-black
	 * 001-blue
	 * 010-green
	 * 011-cyan
	 * 100-red
	 * 101-purple
	 * 110-yellow
	 * 111-white
	 *
	 * State
	 * 0 - Off
	 * 1 - On
	 * 2 - Fast
	 * 3 - Slow
	 */
#ifdef LEDS_AS_COLOR
	/* set combine 3 leds for color */
	ledColor.color1 = number1;
	ledColor.state1 = state1;
	ledColor.count1 = count1;
	ledColor.countReload1 = count1;
	ledColor.color2 = number2;
	ledColor.state2 = state2;
	ledColor.count2 = count2;
	ledColor.countReload2 = count2;
	ledColor.inProgress = 0;
	ledColor.secondsTotal = secondsTotal;
#else
	/* set individual led */
	ledControl[number1].state1 = state1;
	ledControl[number1].count1 = count1;
	ledControl[number1].countReload1 = count1;
	ledControl[number1].state2 = state2;
	ledControl[number1].count2 = count2;
	ledControl[number1].countReload2 = count2;
	ledControl[number1].inProgress = 0;
	ledControl[number1].secondsTotal = secondsTotals;
#endif

	/* stop LED timer count finished */
//	DTIMER_IOCTL(dtimer0, DTIMER_SET_ACTIVE, ioctldata );
}




void cmd_app_test(int argc, char *argv[])
{
	if (argc < 2)
	{
		APRINTF("Usage: testcmd [option]\n   ex) testcmd test\n\n");
		return;
	}

	APRINTF("\n### TEST CMD : %s ###\n\n", argv[1]);
}

void cmd_accel(int argc, char *argv[])
{
	APRINTF(" ACCELEROMETER command disabled (see Fred)\n");

#if 0
	int i, count = 10;
	unsigned char str[50],str2[20];
	char nowStr[20];
	uint32_t data;
#ifdef __TIME64__
	__time64_t now;
#else
	time_t now;
#endif /* __TIME64__ */
	int status;
	unsigned char buf[2];
	uint32_t pin;
	uint8_t state;
	int16_t Xvalue;
	int16_t Yvalue;
	int16_t Zvalue;
	unsigned char rawdata[8];
	uint16_t write_data;
//	float mag;

	if (argc > 2)
	{
		PRINTF("Usage: accel [x]\n\n");
		return;
	}
	if (argc == 2)
	{
		count = strtol(argv[1], NULL, 10);
	}
	rawdata[0] = MC36XX_REG_XOUT_LSB; 	//Word Address to Write Data. 2 Bytes.
	status = i2cRead(MC3672_ADDR, rawdata, 6);
	Xvalue = (short)(((unsigned short)rawdata[1] << 8) + (unsigned short)rawdata[0]);
	Yvalue = (short)(((unsigned short)rawdata[3] << 8) + (unsigned short)rawdata[2]);
	Zvalue = (short)(((unsigned short)rawdata[5] << 8) + (unsigned short)rawdata[4]);
//				PRINTF("Data 0x%04X %d, 0x%04X %d, 0x%04X %d\r\n",Xvalue,Xvalue,Yvalue,Yvalue,Zvalue,Zvalue);
	da16x_time64_msec(NULL, &now);
	uint64_t num1 = ((now/1000000) * 1000000);
	uint64_t num2 = now - num1;
	uint32_t num3 = num2;
	sprintf(nowStr,"%ld",now/1000000);
	sprintf(str2,"%06ld",num3);
	strcat(nowStr,str2);
	Xvalue = (Xvalue * 1000)/511;
	Yvalue = (Yvalue * 1000)/511;
	Zvalue = (Zvalue * 1000)/511;
//	mag = (Xvalue * Xvalue);
//	mag += (Yvalue * Yvalue);
//	mag += (Zvalue * Zvalue);
//	mag = sqrt(mag);
	PRINTF("X: %d Y: %d x: %d %s\r\n",Xvalue, Yvalue, Zvalue, nowStr);
	PRINTF("i\tX\tY\tZ\tts\r\n");
	for(i=0; i < count;i++)
	{
		now = accelData[i].accelTime;
		uint64_t num1 = ((now/1000000) * 1000000);
		uint64_t num2 = now - num1;
		uint32_t num3 = num2;
		sprintf(nowStr,"%ld",now/1000000);
		sprintf(str2,"%06ld",num3);
		strcat(nowStr,str2);
#ifdef SAVEMAGNITUDE
		PRINTF("%d\t%d\t%d\t%d\t%d\t%s\r\n",i,accelData[i].Xvalue,accelData[i].Yvalue,accelData[i].Zvalue,accelData[i].Magvalue,nowStr);
#else
		PRINTF("%d\t%d\t%d\t%d\t%s\r\n",i,accelData[i].Xvalue,accelData[i].Yvalue,accelData[i].Zvalue,nowStr);
#endif
		}
#endif
}

void cmd_adc(int argc, char *argv[])
{
    int status;
	uint32_t data;
	uint16_t adcData;
	float adcDataFloat;
	uint16_t write_data;

	// Read Current ADC_0 Value. Caution!! When read current adc value consequently, need delay at each read function bigger than Sampling Frequency
	write_data = GPIO_PIN10;
	GPIO_WRITE(gpioa, GPIO_PIN10, &write_data, sizeof(uint16_t));   /* enable battery input */
	vTaskDelay(5);
	DRV_ADC_READ(hadc, DA16200_ADC_CH_0, (UINT32 *)&data, 0);
	adcData = (data >> 4) & 0xFFF;
	adcDataFloat = (adcData * VREF)/4095.0;
    PRINTF("Current ADC Value = 0x%x 0x%04X %d\r\n",data&0xffff,adcData, (uint16_t)(adcDataFloat * 100));
	write_data = 0;
	GPIO_WRITE(gpioa, GPIO_PIN10, &write_data, sizeof(uint16_t));   /* disable battery input */
}
#if 0
void cmd_duty_cycle(int argc, char *argv[])
{
	PRINTF("** DUTY CYCLE command disabled (see Fred)\n");
#if 0
	int status, temp;
	char str[20];

	if (argc > 2)
	{
		PRINTF("Usage: dutycycle  data\n   ex) dutycycle 60\n\n");
		return;
	}
	if (argc == 2)
	{
//		duty_cycle = strtol(argv[1], NULL, 10) * 100;
		temp = strtol(argv[1], NULL, 10);
		if(temp >= (NUMBER_OF_SECONDS_PER_MINUTE * NUMBER_OF_MINUTES_SAVED))
			temp = (NUMBER_OF_SECONDS_PER_MINUTE * NUMBER_OF_MINUTES_SAVED);
		duty_cycle_seconds = 0;		/* start new cycle*/
		sprintf(str,"dutycycle=%d",temp);
		status = write_nvram_int(AWS_NVRAM_CONFIG_DUTY_CYCLE, temp);
	}
	PRINTF("\n### dutycycle CMD : %d %d seconds ###\n\n", temp,duty_cycle_seconds);
	read_nvram_int(AWS_NVRAM_CONFIG_DUTY_CYCLE, &temp);
	PRINTF("NVRam dutycycle: %d\r\n",temp);
#endif
}
#endif


int flash_erase_sector(ULONG EraseAddr)
{

	HANDLE SPI;
	int spi_status;
	int erase_status;
	UINT8 rx_data[3];

	/*
	 * Initialize the SPI bus and the Winbond external flash
	 */
//	spi_flash_config_pin();	// Hack to reconfigure the pin multiplexing

	// Get handle for the SPI bus
	SPI = spi_flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\n**** spi_flash_open() returned NULL ****\n");
		return pdFALSE;
	}
//	PRINTF(" SPI handle acquired\n");

	// Do device initialization
	spi_status = w25q64Init(SPI, rx_data);
	if (!spi_status)
	{
		PRINTF("\n**** w25q64Init() returned FALSE ****\n");
		return pdFALSE;
	}

	// Erase the sector specified
	Printf("  Erasing Sector Location: %x \n", EraseAddr);
//	Printf("  Erasing Chip  \n");

	erase_status = eraseSector_4K(SPI, EraseAddr);
//	erase_status = eraseChip(SPI);
	if(!erase_status)
	{
		PRINTF("\n\n**** eraseSector_4K() returned FALSE ****\n");
		return pdFALSE;
	}

	spi_status = flash_close(SPI);
	return pdTRUE;

}  // flash_erase_sector


int flash_read_page(ULONG ReadAddr, UCHAR *PageData)
{

	HANDLE SPI;
	int spi_status;
	UINT8 rx_data[3];

	/*
	 * Initialize the SPI bus and the Winbond external flash
	 */
//	spi_flash_config_pin();	// Hack to reconfigure the pin multiplexing

	// Get handle for the SPI bus
	SPI = flash_open(SPI_MASTER_CLK, SPI_MASTER_CS);
	if (SPI == NULL)
	{
		PRINTF("\n**** spi_flash_open() returned NULL ****\n");
		return pdFALSE;
	}
//	PRINTF(" SPI handle acquired\n");

	// Do device initialization
	spi_status = w25q64Init(SPI, rx_data);
	if (!spi_status)
	{
		PRINTF("\n**** w25q64Init() returned FALSE ****\n");
		spi_status = flash_close(SPI);
		return pdFALSE;
	}

	// Now read the block
	spi_status = pageRead(SPI, ReadAddr, (UINT8 *)PageData, 256);

	if(spi_status < 0)
	{
		Printf("  ***** flash_read_page error reading block 0x%x\n", ReadAddr);
		spi_status = flash_close(SPI);
		return pdFALSE;
	}
	else
	{
		spi_status = flash_close(SPI);
		return pdTRUE;
	}

}  // flash_read_page

/// Hexa Dump API //////////////////////////////////////////////////////////
/*
direction: 0: UART0, 1: ATCMD_INTERFACE
output_fmt: 0: ascii only, 1 hexa only, 2 hexa with ascii
*/
#if 0
void internal_hex_dump(u8 *title, const void *buf, size_t len, char direction, char output_fmt)
{
	size_t i, llen;
	const u8 *pos = buf;
	const size_t line_len = 16;
	int hex_index = 0;
	char* buf_prt;

	buf_prt = pvPortMalloc(64);

	if (output_fmt)
	{
		PRINTF(">>> %s \n", title);
	}

	if (buf == NULL)
	{
		PRINTF(" - hexdump%s(len=%lu): [NULL]\n",
			output_fmt == OUTPUT_HEXA_ONLY ? "":"_ascii",
		       	(unsigned long) len);
		return;
	}

	if (output_fmt)
	{
		PRINTF("- (len=%lu):\n", (unsigned long) len);
	}

	while (len)
	{
		char tmp_str[4];

		llen = len > line_len ? line_len : len;

		memset(buf_prt, 0, 64);

		if (output_fmt)
		{
			sprintf(buf_prt, "[%08x] ", hex_index);

			for (i = 0; i < llen; i++)
			{
				sprintf(tmp_str, " %02x", pos[i]);
				strcat(buf_prt, tmp_str);
			}

			hex_index = hex_index + i;

			for (i = llen; i < line_len; i++)
			{
				strcat(buf_prt, "   ");  /* _xx */
			}

			if (direction)
			{
#ifdef  __TEST_USER_AT_CMD__
				PRINTF_ATCMD("%s  ", buf_prt);
#endif
			}
			else
			{
				PRINTF("%s  ", buf_prt);
			}

			memset(buf_prt, 0, 64);
		}

		if (output_fmt == OUTPUT_HEXA_ASCII || output_fmt == OUTPUT_ASCII_ONLY)
		{
			for (i = 0; i < llen; i++)
			{
				if (   (pos[i] >= 0x20 && pos[i] < 0x7f)
					|| (output_fmt == OUTPUT_ASCII_ONLY && (pos[i]  == 0x0d
					|| pos[i]  == 0x0a
					|| pos[i]  == 0x0c)))
				{
					sprintf(tmp_str, "%c", pos[i]);
					strcat(buf_prt, tmp_str);
				}
				else if (output_fmt)
				{
					strcat(buf_prt, ".");
				}
			}
		}

		if (output_fmt)
		{
			for (i = llen; i < line_len; i++)
			{
				strcat(buf_prt, " ");
			}

			if (direction)
			{
				strcat(buf_prt, "\n\r");	/* ATCMD */
			}
			else
			{
				strcat(buf_prt, "\n");		/* Normal */
			}
		}

		if (direction)
		{
#ifdef  __TEST_USER_AT_CMD__
			PRINTF_ATCMD(buf_prt);
#endif
		}
		else
		{
			PRINTF(buf_prt);
		}

		pos += llen;
		len -= llen;
	}

	vPortFree(buf_prt);

}
#endif

void user_hex_dump(UCHAR *data, UINT length)
{
	hexa_dump_print("", data, length, 0, OUTPUT_HEXA_ASCII);
}

char *user_log_type_string(UCHAR TypeCode)
{
	switch(TypeCode)
	{
	case USERLOG_TYPE_INFORMATION:
		return "Info";
	case USERLOG_TYPE_ERROR:
		return "Error";
	default:
		return "Unknown";
	}
}

void user_log_display(USERLOG_ENTRY *LogEntry)
{
	UCHAR time_string[20];
	UCHAR readable_string[40];

	if(LogEntry->user_log_signature != USERLOG_SIGNATURE)
	{
		PRINTF(" --- Log entry is missing signature ---\n");
		user_hex_dump((UCHAR *)LogEntry, 256);
	}
	else
	{
		PRINTF(" Entry type: %d (%s)\n", LogEntry->user_log_type,
				user_log_type_string(LogEntry->user_log_type));
		time64_string (time_string, &LogEntry->user_log_timestamp);
		time64_msec_string (readable_string, &LogEntry->user_log_timestamp);
		PRINTF(" Timestamp : %s %s\n", time_string, readable_string);

//		PRINTF(" Timestamp : %s\n", time_string);

		PRINTF(" Text      : %s\n", LogEntry->user_log_text);
	}

}

void user_log_list(int lognum, USERLOG_ENTRY *LogEntry)
{
	UCHAR type_string[10];
	UCHAR time_string[20];
	ULONG time_since_boot_milliseconds;
	ULONG time_since_boot_seconds;
	ULONG time_since_boot_minutes;
	ULONG time_since_boot_hours;
	ULONG time_since_boot_days;
	__time64_t stamp;
	int is_error_entry;

	if(LogEntry->user_log_signature != USERLOG_SIGNATURE)
	{
		PRINTF(" --- Log entry is missing signature ---\n");
		user_hex_dump((UCHAR *)LogEntry, 256);
	}
	else
	{
//		PRINTF(" Entry type: %d (%s)\n", LogEntry->user_log_type,
//				user_log_type_string(LogEntry->user_log_type));
		switch(LogEntry->user_log_type)
		{
		case USERLOG_TYPE_INFORMATION:
			strcpy(type_string, "Inf");
			is_error_entry = pdFALSE;
			break;
		case USERLOG_TYPE_ERROR:
			strcpy(type_string, "Err");
			is_error_entry = pdTRUE;
			break;
		default:
			strcpy(type_string, "Unk");
			is_error_entry = pdTRUE;
		}
		stamp = LogEntry->user_log_timestamp;

		time_since_boot_seconds = (ULONG)(stamp / (__time64_t)1000);
		time_since_boot_milliseconds = (ULONG)(stamp
						  - ((__time64_t)time_since_boot_seconds * (__time64_t)1000));
		time_since_boot_minutes = (ULONG)(time_since_boot_seconds / (ULONG)60);
		time_since_boot_hours = (ULONG)(time_since_boot_minutes / (ULONG)60);
		time_since_boot_days = (ULONG)(time_since_boot_hours / (ULONG)24);

		time_since_boot_seconds = time_since_boot_seconds % (ULONG)60;
		time_since_boot_minutes = time_since_boot_minutes % (ULONG)60;
		time_since_boot_hours = time_since_boot_hours % (ULONG)24;

		time64_string (time_string, &LogEntry->user_log_timestamp);
#if 0
		PRINTF(" Timestamp : %s %u Days %02u:%02u:%02u.%03u\n",
				time_string,
				time_since_boot_days,
				time_since_boot_hours,
				time_since_boot_minutes,
				time_since_boot_seconds,
				time_since_boot_milliseconds);
#endif
//		PRINTF(" Timestamp : %s\n", time_string);

//		PRINTF(" Text      : %s\n", LogEntry->user_log_text);


		if(is_error_entry)
		{
			PRINTF_RED(" %d, %s, %u Days %02u:%02u:%02u.%03u, \"%s\"\n",
					lognum,
					type_string,
					time_since_boot_days,
					time_since_boot_hours,
					time_since_boot_minutes,
					time_since_boot_seconds,
					time_since_boot_milliseconds,
					LogEntry->user_log_text);

		}
		else
		{
			PRINTF(" %d, %s, %u Days %02u:%02u:%02u.%03u, \"%s\"\n",
					lognum,
					type_string,
					time_since_boot_days,
					time_since_boot_hours,
					time_since_boot_minutes,
					time_since_boot_seconds,
					time_since_boot_milliseconds,
					LogEntry->user_log_text);
		}

	}

}


#if 0
// Display some flash data
static void user_hex_dump(UCHAR *DumpData, int DumpLen)
{
	int mylen;

	UINT8 bytes[256];
	mylen = DumpLen;
	if (DumpLen > 1024)
	{
		Printf(" user_hex_dump() max length 1024 exceeded\n");
		mylen = 1024;
	}
	for (int j=0; j < _length; j++)
	{
		Printf("0x%02x,", _data[j] & 0xFF);
	}
	printf("\n");


}
#endif


void display_FIFO_data(UCHAR *FIFO_buffer)
{
	accelBufferStruct *fifo;	// working point
	char time_string[20];
	int i;

	fifo = (accelBufferStruct *)FIFO_buffer;

	PRINTF(" FIFO structure contents:\n");
	PRINTF("  Data sequence  : %d\n", fifo->data_sequence);
	time64_string (time_string, &fifo->accelTime);
	PRINTF("  Timestamp      : %s\n", time_string);
	//PRINTF("  Timestamp index: %d\n", fifo->timestamp_sample); \\JW: deprecated.
	PRINTF("  Num of samples : %d\n", fifo->num_samples);
	if (fifo->num_samples > 0 && fifo->num_samples < MAX_ACCEL_FIFO_SIZE)
	{
		PRINTF("  #    X    Y    Z\n");
		for (i=0; i<fifo->num_samples; i++)
		{
			PRINTF(" %2d  %4d %4d %4d\n", i,
					fifo->Xvalue[i],
					fifo->Yvalue[i],
					fifo->Zvalue[i]);
		}
	}
}



/**
 *******************************************************************************
 * @brief Flash read/write command
 *
 * Syntax:
 *
 *    flash read <address>
  *******************************************************************************
 */
void cmd_flash(int argc, char *argv[])
{
	int read_status;
	int erase_status;
//	int unlock_status;
	int init_status = TRUE;
	UINT8 rx_data[3];
	ULONG ReadAddr = 0;
	ULONG EraseAddr;
	ULONG StartAddr;
	ULONG EndAddr;
	int NumBlocks;
	UCHAR PageData[256];
	int i;

	if (argc < 2)
	{
		PRINTF(" Usage:  flash info\n");
		PRINTF("     or  flash read <address>  {hex dump}\n");
		PRINTF("     or  flash read <page address> <num pages>  {hex dump}\n");
		PRINTF("     or  flash aread <address>  {accelerometer data}\n");
		PRINTF("     or  flash aread <page address> <num pages>  {accelerometer data}\n");
		PRINTF("     or  flash erase <sector address> [num sectors]\n");
		return;
	}
	if (strcasecmp(argv[1], "info") == 0)
	{
		EndAddr = (ULONG)AB_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(AB_FLASH_MAX_PAGES - 1));

		PRINTF("Accelerometer data storage:\n");
		PRINTF(" Start address    : 0x%0x  (%d)\n", AB_FLASH_BEGIN_ADDRESS, AB_FLASH_BEGIN_ADDRESS);
		PRINTF(" Number of pages  : 0x%0x  (%d)\n", AB_FLASH_MAX_PAGES, AB_FLASH_MAX_PAGES);
		PRINTF(" Page size (bytes): 0x%0x  (%d)\n", AB_FLASH_PAGE_SIZE, AB_FLASH_PAGE_SIZE);
		PRINTF(" Last page address: 0x%0x  (%d)\n", EndAddr, EndAddr);
		PRINTF("\n");
		EndAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(USERLOG_FLASH_MAX_PAGES - 1));
		PRINTF("Log area:\n");
		PRINTF(" Start address    : 0x%0x  (%d)\n", USERLOG_FLASH_BEGIN_ADDRESS, USERLOG_FLASH_BEGIN_ADDRESS);
		PRINTF(" Number of pages  : 0x%0x  (%d)\n", USERLOG_FLASH_MAX_PAGES, USERLOG_FLASH_MAX_PAGES);
		PRINTF(" Last page address: 0x%0x  (%d)\n", EndAddr, EndAddr);
	}
	else if (strcasecmp(argv[1], "erase") == 0)
	{
		if (argc == 3)
		{
			// Base of zero allows decimal or hex with "0x..."
			EraseAddr = strtol(argv[2], NULL, 0);
			PRINTF(" Erase address: %ld  0x%x\n", EraseAddr, EraseAddr);
			if ((EraseAddr % AB_FLASH_SECTOR_SIZE) != 0)
			{
				PRINTF(" *** WARNING - not a sector address\n");
			}
			read_status =  flash_erase_sector(EraseAddr);
			if(!read_status)
			{
				PRINTF(" Unable to erase sector\n");
			}
			else
			{
				PRINTF(" Sector erased\n");
			}
		}
		else if (argc == 4)
		{
			// Note there are 2,000 sectors in the entire 8MB flash
			// Base of zero allows decimal or hex with "0x..."
			StartAddr = strtol(argv[2], NULL, 0);
			NumBlocks = strtol(argv[3], NULL, 0);
			PRINTF(" Start address: %ld  0x%x - %d sectors\n", StartAddr, StartAddr,
					NumBlocks);
			if ((StartAddr % AB_FLASH_SECTOR_SIZE) != 0)
			{
				PRINTF(" *** WARNING - not a sector address\n");
			}
			EraseAddr = StartAddr;
			for (i = 0; i < NumBlocks; i++)
			{
				read_status =  flash_erase_sector(EraseAddr);
				if(!read_status)
				{
					PRINTF(" Unable to erase sector %d: 0x%x\n", i, EraseAddr);
				}
				else
				{
					PRINTF(" Sector %d erased: 0x%x\n", i, EraseAddr);
				}
				EraseAddr += AB_FLASH_SECTOR_SIZE;

//				vTaskDelay(pdMS_TO_TICKS(100));
			}

		}

		else
		{
			PRINTF(" Usage:  flash info\n");
			PRINTF("     or  flash read <address>  {hex dump}\n");
			PRINTF("     or  flash read <page address> <num pages>  {hex dump}\n");
			PRINTF("     or  flash aread <address>  {accelerometer data}\n");
			PRINTF("     or  flash aread <page address> <num pages>  {accelerometer data}\n");
			PRINTF("     or  flash erase <sector address> [num sectors]\n");
		}
	}
	else if (strcasecmp(argv[1], "read") == 0)
	{
		if (argc == 3)
		{
			// Base of zero allows decimal or hex with "0x..."
			ReadAddr = strtol(argv[2], NULL, 0);
			PRINTF(" Read address: %ld  0x%x\n", ReadAddr, ReadAddr);
			read_status =  flash_read_page(ReadAddr, PageData);
			if(!read_status)
			{
				PRINTF(" Unable to read page\n");
			}
			else
			{
				PRINTF(" Page data:\n");
				user_hex_dump(PageData, 256);
			}
		}
		else if (argc == 4)
		{
			// Base of zero allows decimal or hex with "0x..."
			StartAddr = strtol(argv[2], NULL, 0);
			NumBlocks = strtol(argv[3], NULL, 0);
			PRINTF(" Start address: %ld  0x%x - %d pages\n", ReadAddr, ReadAddr,
					NumBlocks);
			ReadAddr = StartAddr;
			for (i = 0; i < NumBlocks; i++)
			{
				read_status =  flash_read_page(ReadAddr, PageData);
				if(!read_status)
				{
					PRINTF(" Unable to read page\n");
				}
				else
				{
					PRINTF(" Page data: 0x%0x\n", ReadAddr);
					user_hex_dump(PageData, 256);
				}
				ReadAddr += 256;
				// reading zeroes delay
//				vTaskDelay(pdMS_TO_TICKS(750));
			}

		}
	}
	else if (strcasecmp(argv[1], "aread") == 0)
	{
		if (argc == 3)
		{
			// Base of zero allows decimal or hex with "0x..."
			ReadAddr = strtol(argv[2], NULL, 0);
			PRINTF(" Read address: %ld  0x%x\n", ReadAddr, ReadAddr);
			read_status =  flash_read_page(ReadAddr, PageData);
			if(!read_status)
			{
				PRINTF(" Unable to read page\n");
			}
			else
			{
				PRINTF(" Page data:\n");
				display_FIFO_data(PageData);
			}
		}
		else if (argc == 4)
		{
			// Base of zero allows decimal or hex with "0x..."
			StartAddr = strtol(argv[2], NULL, 0);
			NumBlocks = strtol(argv[3], NULL, 0);
			PRINTF(" Start address: %ld  0x%x - %d pages\n", ReadAddr, ReadAddr,
					NumBlocks);
			ReadAddr = StartAddr;
			for (i = 0; i < NumBlocks; i++)
			{
				read_status =  flash_read_page(ReadAddr, PageData);
				if(!read_status)
				{
					PRINTF(" Unable to read page\n");
				}
				else
				{
					PRINTF(" Page data: 0x%0x\n", ReadAddr);
					display_FIFO_data(PageData);
				}
				ReadAddr += 256;
				// reading zeroes delay
				vTaskDelay(pdMS_TO_TICKS(750));
			}

		}
	}
	else
	{
		PRINTF(" Usage:  flash info\n");
		PRINTF("     or  flash read <address>  {hex dump}\n");
		PRINTF("     or  flash read <page address> <num pages>  {hex dump}\n");
		PRINTF("     or  flash aread <address>  {accelerometer data}\n");
		PRINTF("     or  flash aread <page address> <num pages>  {accelerometer data}\n");
		PRINTF("     or  flash erase <sector address> [num sectors]\n");
		return;
	}


}




/*
 * LED timer callback function
 */
static 	void dtimer_callback_0(void *param)
{

	#ifdef LEDS_AS_COLOR
	/* ledColor is color
	 * 000-black
	 * 001-blue
	 * 010-green
	 * 011-cyan
	 * 100-red
	 * 101-purple
	 * 110-yellow
	 * 111-white
	 *
	 * actual LED 0 = On and 1 = Off
	 */
	TIMER_INFO_TYPE	*tinfo;
	UINT32	ioctldata[1];
	uint16_t write_data;
	uint16_t write_data_red;
	uint16_t write_data_green;
	uint16_t write_data_blue;
	static int cnt;
	uint8_t color;
	uint8_t state;

	if( param == NULL)
	{
		return ;
	}
	tinfo = (TIMER_INFO_TYPE*) param;

	if( tinfo->cmd == DTIMER_CMD_STOP )
	{
		ioctldata[0] = DTIMER_DEV_INTR_DISABLE ;
		DTIMER_IOCTL(tinfo->timer, DTIMER_SET_MODE, ioctldata );
	}

	cnt++;
	if(ledColor.inProgress == 0)
	{
		color = ledColor.color1 & 0x7;
		state = ledColor.state1 & 0x3;
	}
	else
	{
		color = ledColor.color2 & 0x7;
		state = ledColor.state2 & 0x3;
	}

	/* assume all leds are off */
	write_data = GPIO_PIN10;
	write_data_red = GPIO_PIN6;
	write_data_green = GPIO_PIN7;
	write_data_blue = GPIO_PIN8;

	/* set up the state */
	switch (state)
	{
		case LED_OFFX:
			break;

		case LED_ONX:
			if(color & 0x1)
				write_data_blue = 0;
			if(color & 0x2)
				write_data_green = 0;
			if(color & 0x4)
				write_data_red = 0;
			write_data  = 0;
			break;

		case LED_FAST:
			if(cnt & 0x02)
			{
				if(color & 0x1)
					write_data_blue = 0;
				if(color & 0x2)
					write_data_green = 0;
				if(color & 0x4)
					write_data_red = 0;
				write_data  = 0;
			}
			break;

		case LED_SLOW:
			if(cnt & 0x08)
			{
				if(color & 0x1)
					write_data_blue = 0;
				if(color & 0x2)
					write_data_green = 0;
				if(color & 0x4)
					write_data_red = 0;
				write_data  = 0;
			}
			break;
	}
	if(ledColor.inProgress == 0)
	{
		if(ledColor.count1 == 0)
		{
			write_data = GPIO_PIN10;
			write_data_red = GPIO_PIN6;
			write_data_green = GPIO_PIN7;
			write_data_blue = GPIO_PIN8;
		}
		GPIO_WRITE(gpioc, GPIO_PIN6, &write_data_red, sizeof(uint16_t));
		GPIO_WRITE(gpioc, GPIO_PIN7, &write_data_green, sizeof(uint16_t));
		GPIO_WRITE(gpioc, GPIO_PIN8, &write_data_blue, sizeof(uint16_t));

		if(ledColor.count1 != -1)			/* is it hold */
		{
			if((cnt & 0x7)	== 0)						/* count seconds */
			{
				if(ledColor.count1 > 0)
					ledColor.count1--;			/* count down timer interval */
				if( (ledColor.count1 == 0) && (ledColor.count2 != 0) )  /* if a second count loaded start second interval */
					ledColor.inProgress = 1;
			}
		}
	}
	else
	{
		if(ledColor.count2 == 0)
		{
			write_data = 0;
			write_data_red = 0;
			write_data_green = 0;
			write_data_blue = 0;
		}
		GPIO_WRITE(gpioc, GPIO_PIN6, &write_data_red, sizeof(uint16_t));
		GPIO_WRITE(gpioc, GPIO_PIN7, &write_data_green, sizeof(uint16_t));
		GPIO_WRITE(gpioc, GPIO_PIN8, &write_data_blue, sizeof(uint16_t));

		if(ledColor.count2 != -1)			/* is it hold */
		{
			if((cnt & 0x7)	== 0)						/* count seconds */
			{
				if(ledColor.count2 > 0)
					ledColor.count2--;			/* count down timer interval */
				if( ledColor.count2 == 0)  /* if a second count loaded start second interval */
				{
					ledColor.inProgress = 0;
					ledColor.count1 = ledColor.countReload1;
					ledColor.count2 = ledColor.countReload2;
				}
			}
		}
	}

#if 1
	if((cnt & 0x7)	== 0)						/* count seconds */
	{
		if(ledColor.secondsTotal > 0)
			ledColor.secondsTotal--;			/* count down timer interval */
		if(ledColor.secondsTotal == 0)
		{
			ledColor.count1 = 0;
			ledColor.countReload1 = 0;
			ledColor.count2 = 0;
			ledColor.countReload2 = 0;
			write_data = GPIO_PIN10;
			write_data_red = GPIO_PIN6;
			write_data_green = GPIO_PIN7;
			write_data_blue = GPIO_PIN8;
			GPIO_WRITE(gpioc, GPIO_PIN6, &write_data_red, sizeof(uint16_t));
			GPIO_WRITE(gpioc, GPIO_PIN7, &write_data_green, sizeof(uint16_t));
			GPIO_WRITE(gpioc, GPIO_PIN8, &write_data_blue, sizeof(uint16_t));

			/* stop LED timer count finished */
//			DTIMER_IOCTL(tinfo->timer, DTIMER_SET_DEACTIVE, ioctldata );
		}
	}
#endif
#endif

}
#if 0
static 	void dtimer_callback_1(void *param)
{
	TIMER_INFO_TYPE	*tinfo;
	UINT32	ioctldata[1];
	uint16_t write_data;
	uint16_t read_data;

#if 1
	if( param == NULL)
	{
		return ;
	}
	tinfo = (TIMER_INFO_TYPE*) param;

	if( tinfo->cmd == DTIMER_CMD_STOP )
	{
		ioctldata[0] = DTIMER_DEV_INTR_DISABLE ;
		DTIMER_IOCTL(tinfo->timer, DTIMER_SET_MODE, ioctldata );
	}
#endif
	if(duty_cycle_seconds != 0)			/* count down duty_cycle */
		duty_cycle_seconds--;
}
#endif

/*
 * Start the ARM dual-timer mechanism for LED flashing
 */
void start_LED_timer()
{
//	OAL_EVENT_GROUP	*event;
	UINT32	ioctldata[3], tickvalue[2], starttick;
	TIMER_INFO_TYPE tinfo[2];
	UNSIGNED masked_evt, checkflag, stackflag;
	UINT32 	cursysclock;

	checkflag = 0;

//	starttick = OAL_RETRIEVE_CLOCK();

	_sys_clock_read( &cursysclock, sizeof(UINT32));

	//======================================================
	// Timer Creation
	//
//	event = (OAL_EVENT_GROUP *)APP_MALLOC(sizeof(OAL_EVENT_GROUP));

//	OAL_CREATE_EVENT_GROUP(event, "dt.test");
//	OAL_SET_EVENTS(event, 0, OAL_AND );

	dtimer0 = DTIMER_CREATE(DTIMER_UNIT_00);
//	dtimer1 = DTIMER_CREATE(DTIMER_UNIT_01);

	tinfo[0].timer = dtimer0;
//	tinfo[0].event = event;
	tinfo[0].cmd   = DTIMER_CMD_RUN;

//	tinfo[1].timer = dtimer1;
//	tinfo[1].event = event;
//	tinfo[1].cmd   = DTIMER_CMD_RUN;

	//======================================================
	// Timer Initialization
	//
	if( dtimer0 != NULL )
	{
		DTIMER_INIT(dtimer0);

		ioctldata[0] = DTIMER_DEV_INTR_DISABLE ;
		DTIMER_IOCTL(dtimer0, DTIMER_SET_MODE, ioctldata );

		// Timer Callback
		ioctldata[0] = 0;
		ioctldata[1] = (UINT32)dtimer_callback_0;
		ioctldata[2] = (UINT32)&(tinfo[0]);
		DTIMER_IOCTL(dtimer0, DTIMER_SET_CALLACK, ioctldata );

		// Timer Period
		ioctldata[0] = cursysclock ;	// Clock 120 MHz
//		ioctldata[1] = (cursysclock/(20*MHz)) ;	// Divider
		ioctldata[1] = (cursysclock/(15*MHz)) ;	// Divider
		DTIMER_IOCTL(dtimer0, DTIMER_SET_LOAD, ioctldata );
		PRINTF("%ld %ld\r\n",cursysclock,ioctldata[1]);  /* ioctldata[0]/ioctldata[1] = period Hz */

		// Timer Configuration
		ioctldata[0] = DTIMER_DEV_INTR_ENABLE
				| DTIMER_DEV_PERIODIC_MODE
				| DTIMER_DEV_PRESCALE_1
				| DTIMER_DEV_32BIT_SIZE
				| DTIMER_DEV_WRAPPING ;
		DTIMER_IOCTL(dtimer0, DTIMER_SET_MODE, ioctldata );

		DTIMER_IOCTL(dtimer0, DTIMER_SET_ACTIVE, ioctldata );

		checkflag |= 0x0004;
		PRINTF("Start Timer 0\n");
	}
#if 0
	if( dtimer1 != NULL ){
		DTIMER_INIT(dtimer1);

		ioctldata[0] = DTIMER_DEV_INTR_DISABLE ;
		DTIMER_IOCTL(dtimer1, DTIMER_SET_MODE, ioctldata );

		// Timer Callback
		ioctldata[0] = 0;
		ioctldata[1] = (UINT32)dtimer_callback_1;
		ioctldata[2] = (UINT32)&(tinfo[0]);
		DTIMER_IOCTL(dtimer1, DTIMER_SET_CALLACK, ioctldata );

		// Timer Period
		ioctldata[0] = cursysclock ;	// Clock 120 MHz
//		ioctldata[1] = (cursysclock/(20*MHz)) ;	// Divider
		ioctldata[1] = (cursysclock/(cursysclock)) ;	// Divider
		DTIMER_IOCTL(dtimer1, DTIMER_SET_LOAD, ioctldata );
		PRINTF("%ld %ld\r\n",cursysclock,ioctldata[1]);  /* ioctldata[0]/ioctldata[1] = period Hz */

		// Timer Configuration
		ioctldata[0] = DTIMER_DEV_INTR_ENABLE
				| DTIMER_DEV_PERIODIC_MODE
				| DTIMER_DEV_PRESCALE_1
				| DTIMER_DEV_32BIT_SIZE
				| DTIMER_DEV_WRAPPING ;
		DTIMER_IOCTL(dtimer1, DTIMER_SET_MODE, ioctldata );

		DTIMER_IOCTL(dtimer1, DTIMER_SET_ACTIVE, ioctldata );

		checkflag |= 0x0004;
		PRINTF("Start Timer 1\n");
	}
#endif

}


// Tim's LED function
void cmd_led_state(int argc, char *argv[])
{

	if (argc == 4)
	{
		/* led number, led state, led count */
		// if LEDS_AS_COLOR
		//  color1, state1, count1
		setLEDState(strtol(argv[1], NULL, 10),strtol(argv[2], NULL, 10),strtol(argv[3], NULL, 10),0 ,0, 0, strtol(argv[3], NULL, 10));
	}
	else if (argc == 7)
	{
		/* led number1, led state1, led count1, led number2, led state2, led count2, count1 + count2 */
		// if LEDS_AS_COLOR
		//  color1, state1, count1, color2, state2, count2
		setLEDState(strtol(argv[1], NULL, 10),strtol(argv[2], NULL, 10),strtol(argv[3], NULL, 10),strtol(argv[4], NULL, 10),strtol(argv[5], NULL, 10),strtol(argv[6], NULL, 10),strtol(argv[3], NULL, 10) + strtol(argv[6], NULL, 10));
	}
	else if (argc == 8)
	{
		/* led number1, led state1, led count1, led number2, led state2, led count2, secondsTotal */
		// if LEDS_AS_COLOR
		//  color1, state1, count1, color2, state2, count2
		setLEDState(strtol(argv[1], NULL, 10),strtol(argv[2], NULL, 10),strtol(argv[3], NULL, 10),strtol(argv[4], NULL, 10),strtol(argv[5], NULL, 10),strtol(argv[6], NULL, 10),strtol(argv[7], NULL, 10));
	}

#ifdef LEDS_AS_COLOR
	PRINTF("Color1\tState1\tCount1\tColor2\tState2\tCount2\tReload1\tReload2\tSecs\tinProgress\r\n");
	PRINTF("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", ledColor.color1, ledColor.state1, ledColor.count1, \
			ledColor.color2, ledColor.state2, ledColor.count2, \
			ledColor.countReload1, ledColor.countReload2, ledColor.secondsTotal, ledColor.inProgress);
#else
	PRINTF("LED\tState1\tCount1\tState2\tCount2\tReload1\tReload2\tSecs\tinProgress\r\n");
	for(int i=0; i < NUMBER_LEDS; i++)
		PRINTF("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", i, ledControl[i].state1, ledControl[i].count1, ledControl[i].state2, ledControl[i].count2, ledControl[i].countReload1, ledControl[i].countReload2, ledControl[i].secondsTotal, ledControl[i].inProgress);
#endif
	if ((argc == 1) || (argc == 4))
		return;

	PRINTF("Usage: ledstate color state count \n   ex) ledstate 1 3 5\n\n");
}


void cmd_led(int argc, char *argv[])
{
	int status, led, state;
	char str[20];
    uint16_t write_data_red;
    uint16_t write_data_green;
    uint16_t write_data_blue;
    uint16_t leds[3] = {GPIO_PIN6, GPIO_PIN7, GPIO_PIN8};
    uint16_t dummy = 0;

	if (argc != 3)
	{
		PRINTF("Usage: led  led[0,1,2] state[0,1]\n \n\n");
		return;
	}
	led = strtol(argv[1], NULL, 10) & 0x3;
	state = strtol(argv[2], NULL, 10) & 0x1;
	if(state)
	{
		dummy = leds[led];
		GPIO_WRITE(gpioc, leds[led], &leds[led], sizeof(uint16_t));
		dummy = GPIO_PIN6;
		GPIO_WRITE(gpioc, GPIO_PIN6, &dummy, sizeof(uint16_t));
	}
	else
	{
		dummy = 0;
	    GPIO_WRITE(gpioc, leds[led], &dummy, sizeof(uint16_t));
		GPIO_WRITE(gpioc, GPIO_PIN6, &dummy, sizeof(uint16_t));
	}

	PRINTF("led %d state %d\r\n",led,state);
}


void cmd_log_help(void)
{
	PRINTF(" Usage:  log info\n");
//	PRINTF("         log list <entry #>\n");
	PRINTF("         log list <entry #> [num entries] with <entry> from 1 to %d\n");
//	PRINTF("                     (<entry> is from 1 to %d)\n", USERLOG_FLASH_MAX_PAGES);
//	PRINTF("         log read <entry #>\n");
	PRINTF("         log read <entry #> [num entries]\n");
//	PRINTF(" or use flash command for hex dump\n");
	PRINTF("         log search <string> enclosed in quotes, case-sensitive\n");
	PRINTF("      or log search #errors    to list all error entries");
	//	PRINTF("                     (<string> is case sensitive)\n");
//	PRINTF("         log erase <sector address> in hex \"0x\" form or decimal\n");
	PRINTF("\n");
}
/*
 * Log command:
 *
 * syntax is:
 *   log        - shows current log stats
 *   log help   - shows command help
 *
 *  Note - user commands are normally issued after a boot when the
 *  RUN flag isn't set.  So the whole log mechanism isn't set up
 *  as it is for regular operation.
 *  So we need to figure out what log entries might exist in
 *  flash.
 *
 */
void cmd_log(int argc, char *argv[])
{
	int status;
	int deviceAddress = 0, length = 1, regAddress = 0;

	int read_status;
	int erase_status;
//	int unlock_status;
	int init_status = TRUE;
	UINT8 rx_data[3];
	int StartEntry;
	int ReadEntry;
	ULONG ReadAddr;
	ULONG EraseAddr;
	ULONG StartAddr;
	ULONG EndAddr;
	int NumBlocks;
	UCHAR PageData[256];
	USERLOG_ENTRY *LogEntry;
	// stuff for analyzing the log
	__time64_t OldestTimestamp;
	__time64_t NewestTimestamp;
	ULONG OldestAddr = 0;
	ULONG NewestAddr = 0;
	LONG FirstInactiveAddr;
	LONG LastInactiveAddr;
	int num_entries;
	int num_inactive;
	int num_error_entries;
	int num_info_entries;
	int num_unknown_entries;
	int total_flash_logs_stored;
	int first_found;
	int i;
	int list_errors_only = pdFALSE;
	UCHAR time_string[20];
	ULONG time_since_boot_milliseconds;
	ULONG time_since_boot_seconds;
	ULONG time_since_boot_minutes;
	ULONG time_since_boot_hours;
	ULONG time_since_boot_days;
	__time64_t stamp;
	// stuff for search
	UCHAR search_string[200];
	int num_found;


	if (argc < 2)
	{
		cmd_log_help();
		return;
	}
	if (strcasecmp(argv[1], "info") == 0)
	{
		EndAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
				((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(USERLOG_FLASH_MAX_PAGES - 1));
		PRINTF("\n");
		PRINTF("Log area:\n");
		PRINTF(" Start address         : 0x%0x  (%d)\n", USERLOG_FLASH_BEGIN_ADDRESS, USERLOG_FLASH_BEGIN_ADDRESS);
		PRINTF(" Last page address     : 0x%0x  (%d)\n", EndAddr, EndAddr);
		PRINTF(" Flash page size       : %d\n", AB_FLASH_PAGE_SIZE);
		PRINTF(" Max number of entries : %d\n", USERLOG_FLASH_MAX_PAGES);
		PRINTF(" Log entry struct size : %d\n", sizeof(USERLOG_ENTRY));
		PRINTF(" Log entry max text len: %d\n", USERLOG_STRING_MAX_LEN);
		PRINTF("... Analyzing ...\n");
		// Note - there will probably be at least a gap where the unused
		// entries from the last erase operation occurred.
		first_found = pdFALSE;
		num_entries = 0;
		num_inactive = 0;
		num_error_entries = 0;
		num_info_entries = 0;
		num_unknown_entries = 0;
		FirstInactiveAddr = -1;
		LastInactiveAddr = 0;
		for (i = 0; i < USERLOG_FLASH_MAX_PAGES; i++)
		{
			if((i % 100) == 0)
			{
				PRINTF("%5d\r", i);
			}
			ReadAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(i));
//			PRINTF("\n Entry %d address: %ld  0x%x\n",
//					ReadEntry, ReadAddr, ReadAddr);
			read_status =  flash_read_page(ReadAddr, PageData);
			if(!read_status)
			{
				PRINTF(" Unable to read page %ld\n", ReadAddr);
			}
			else
			{
				LogEntry = (USERLOG_ENTRY *)PageData;
				if(LogEntry->user_log_signature == USERLOG_SIGNATURE)
				{
					num_entries++;
					switch(LogEntry->user_log_type)
					{
					case USERLOG_TYPE_INFORMATION:
						num_info_entries++;
						break;
					case USERLOG_TYPE_ERROR:
						num_error_entries++;
						break;
					default:
						num_unknown_entries++;
					}

					if (!first_found)
					{
						// Initialize search
						OldestTimestamp = LogEntry->user_log_timestamp;
						NewestTimestamp = LogEntry->user_log_timestamp;
						OldestAddr = i;
						NewestAddr = i;
						first_found = pdTRUE;
//						PRINTF("First active log entry is %d 0x%0x\n", (i+1), ReadAddr);
					}
					else
					{
						// Timestamps are in milliseconds from boot time
						// so the lowest number is the oldest
						// But note that there are potentially discontinuities
						// since there is an entire erased sector (16 pages)
						// at the location of the next log write.
						// Plus - if we're testing, there may be old
						// log entries floating around
						if(LogEntry->user_log_timestamp < OldestTimestamp)
						{
							OldestTimestamp = LogEntry->user_log_timestamp;
							OldestAddr = i;
						}
						if(LogEntry->user_log_timestamp > NewestTimestamp)
						{
							NewestTimestamp = LogEntry->user_log_timestamp;
							NewestAddr = i;
						}

					} // not first log entry
				} // if has log signature
				else
				{
					num_inactive++;
					if(FirstInactiveAddr == -1)
					{
						FirstInactiveAddr = i;
					}
					LastInactiveAddr = i;
				}
			} // if read status
		} // for i
		// Show results
		PRINTF("          \r");  // Erase the progress number
		PRINTF("\n");
		PRINTF(" Number of active log entries found      : %d\n", num_entries);
		PRINTF(" Number of inactive log entries found    : %d\n", num_inactive);
		PRINTF(" Number of error  log entries found      : %d\n", num_error_entries);
		PRINTF(" Number of information log entries found : %d\n", num_info_entries);
		if(num_unknown_entries > 0)
		{
			PRINTF(" Number of unknown log entries found     : %d\n", num_unknown_entries);
		}

		if (num_entries > 0)
		{
			stamp = OldestTimestamp;
			time_since_boot_seconds = (ULONG)(stamp / (__time64_t)1000);
			time_since_boot_milliseconds = (ULONG)(stamp
							  - ((__time64_t)time_since_boot_seconds * (__time64_t)1000));
			time_since_boot_minutes = (ULONG)(time_since_boot_seconds / (ULONG)60);
			time_since_boot_hours = (ULONG)(time_since_boot_minutes / (ULONG)60);
			time_since_boot_days = (ULONG)(time_since_boot_hours / (ULONG)24);

			time_since_boot_seconds = time_since_boot_seconds % (ULONG)60;
			time_since_boot_minutes = time_since_boot_minutes % (ULONG)60;
			time_since_boot_hours = time_since_boot_hours % (ULONG)24;

			time64_string (time_string, &OldestTimestamp);

			PRINTF("Oldest active log entry     : %d [%s] %u Days %02u:%02u:%02u.%03u\n",
					(OldestAddr+1),
					time_string,
					time_since_boot_days,
					time_since_boot_hours,
					time_since_boot_minutes,
					time_since_boot_seconds,
					time_since_boot_milliseconds);

			stamp = NewestTimestamp;
			time_since_boot_seconds = (ULONG)(stamp / (__time64_t)1000);
			time_since_boot_milliseconds = (ULONG)(stamp
							  - ((__time64_t)time_since_boot_seconds * (__time64_t)1000));
			time_since_boot_minutes = (ULONG)(time_since_boot_seconds / (ULONG)60);
			time_since_boot_hours = (ULONG)(time_since_boot_minutes / (ULONG)60);
			time_since_boot_days = (ULONG)(time_since_boot_hours / (ULONG)24);

			time_since_boot_seconds = time_since_boot_seconds % (ULONG)60;
			time_since_boot_minutes = time_since_boot_minutes % (ULONG)60;
			time_since_boot_hours = time_since_boot_hours % (ULONG)24;

			time64_string (time_string, &NewestTimestamp);

			PRINTF("Newest active log entry     : %d [%s] %u Days %02u:%02u:%02u.%03u\n",
					(NewestAddr+1),
					time_string,
					time_since_boot_days,
					time_since_boot_hours,
					time_since_boot_minutes,
					time_since_boot_seconds,
					time_since_boot_milliseconds);
		} // if there are active log entries


		PRINTF(" First inactive log entry                : %u\n", (FirstInactiveAddr+1));
		PRINTF(" Last inactive log entry                 : %u\n", (LastInactiveAddr+1));

		total_flash_logs_stored = (NewestAddr - OldestAddr) + 1;
		if (total_flash_logs_stored < 0)
		{
			// This involves a wraparound
			total_flash_logs_stored += USERLOG_FLASH_MAX_PAGES;
		}

//		PRINTF(" Total flash log entries (computed)      : %d\n", total_flash_logs_stored);
		PRINTF("\n");
	} // info command
	else if (strcasecmp(argv[1], "search") == 0)
	{
		if (argc != 3)
		{
			cmd_log_help();
			return;
		}
		user_text_copy(search_string, argv[2], 199);
		if (strlen (search_string) <= 0)
		{
			cmd_log_help();
			return;
		}
		// asking for errors only?
		if(strstr(search_string, "#errors") != NULL)
		{
			list_errors_only = pdTRUE;
			PRINTF("... Searching log for error entries ...\n");
		}
		else
		{
			list_errors_only = pdFALSE;
			PRINTF("... Searching log for [%s] ...\n", search_string);
		}


		// Note - there will probably be at least a gap where the unused
		// entries from the last erase operation occurred.
		first_found = pdFALSE;
		num_entries = 0;
		num_inactive = 0;
		num_found = 0;
		for (i = 0; i < USERLOG_FLASH_MAX_PAGES; i++)
		{
			ReadAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(i));
//			PRINTF("\n Entry %d address: %ld  0x%x\n",
//					ReadEntry, ReadAddr, ReadAddr);
			read_status =  flash_read_page(ReadAddr, PageData);
			if(!read_status)
			{
				PRINTF(" Unable to read page %ld\n", ReadAddr);
			}
			else
			{
				LogEntry = (USERLOG_ENTRY *)PageData;
				if(LogEntry->user_log_signature == USERLOG_SIGNATURE)
				{
					num_entries++;
					if(list_errors_only)
					{
						if(USERLOG_TYPE_ERROR == LogEntry->user_log_type)
						{
							num_found++;
							user_log_list((i+1), LogEntry);
						}
					}
					else if(strstr(LogEntry->user_log_text, search_string) != NULL)
					{
						num_found++;
						user_log_list((i+1), LogEntry);
					}
				} // if has log signature
				else
				{
					num_inactive++;
				}
			} // if read status
		} // for i
		// Show results
		PRINTF("          \r");  // Erase the progress number
		PRINTF("\n");
		PRINTF(" Number of active log entries searched : %d\n", num_entries);
		PRINTF(" Number of inactive log entries skipped: %d\n", num_inactive);
		PRINTF(" Number of matching entries found      : %d\n", num_found);

		PRINTF("\n");
	} // info command
	else if (strcasecmp(argv[1], "read") == 0)
	{
		if (argc == 3)
		{
			// Base of zero argument allows decimal or hex with "0x..."
			StartEntry = strtol(argv[2], NULL, 0);
			ReadAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(StartEntry - 1));
			PRINTF("\n Entry %d address: %ld  0x%x\n",
					StartEntry, ReadAddr, ReadAddr);
			read_status =  flash_read_page(ReadAddr, PageData);
			if(!read_status)
			{
				PRINTF(" Unable to read page\n");
			}
			else
			{
				PRINTF(" Page data:\n");
//				user_hex_dump(PageData, 256);
				user_log_display((USERLOG_ENTRY *)PageData);
			}
		}
		else if (argc == 4)
		{
			// Base of zero allows decimal or hex with "0x..."
			StartEntry = strtol(argv[2], NULL, 0);
			NumBlocks = strtol(argv[3], NULL, 0);
			PRINTF(" Start entry: %d - %d entries\n",
					StartEntry, NumBlocks);
			ReadEntry = StartEntry;
			for (i = 0; i < NumBlocks; i++)
			{
				ReadAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
						((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(ReadEntry - 1));
				PRINTF("\n Entry %d address: %ld  0x%x\n",
						ReadEntry, ReadAddr, ReadAddr);
				read_status =  flash_read_page(ReadAddr, PageData);
				if(!read_status)
				{
					PRINTF(" Unable to read page\n");
				}
				else
				{
//					PRINTF(" Page data: 0x%0x\n", ReadAddr);
//					user_hex_dump(PageData, 256);
					user_log_display((USERLOG_ENTRY *)PageData);
				}
				ReadEntry++;
				// reading zeroes delay
//				vTaskDelay(pdMS_TO_TICKS(10));
			}
		}
		else
		{
			cmd_log_help();
		}
	}
	else if (strcasecmp(argv[1], "list") == 0)
	{
		if (argc == 3)
		{
			// Base of zero argument allows decimal or hex with "0x..."
			StartEntry = strtol(argv[2], NULL, 0);
			ReadAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
					((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(StartEntry - 1));
//			PRINTF("\n Entry %d address: %ld  0x%x\n",
//					StartEntry, ReadAddr, ReadAddr);
			read_status =  flash_read_page(ReadAddr, PageData);
			if(!read_status)
			{
				PRINTF(" Unable to read page\n");
			}
			else
			{
				PRINTF("\n");
//				user_hex_dump(PageData, 256);
				user_log_list(StartEntry, (USERLOG_ENTRY *)PageData);
			}
		}
		else if (argc == 4)
		{
			// Base of zero allows decimal or hex with "0x..."
			StartEntry = strtol(argv[2], NULL, 0);
			NumBlocks = strtol(argv[3], NULL, 0);
			PRINTF(" Start entry: %d - %d entries\n",
					StartEntry, NumBlocks);
			ReadEntry = StartEntry;
			for (i = 0; i < NumBlocks; i++)
			{
				ReadAddr = (ULONG)USERLOG_FLASH_BEGIN_ADDRESS +
						((ULONG)AB_FLASH_PAGE_SIZE * (ULONG)(ReadEntry - 1));
//				PRINTF("\n Entry %d address: %ld  0x%x\n",
//						ReadEntry, ReadAddr, ReadAddr);
				read_status =  flash_read_page(ReadAddr, PageData);
				if(!read_status)
				{
					PRINTF(" Unable to read page\n");
				}
				else
				{
//					PRINTF(" Page data: 0x%0x\n", ReadAddr);
//					user_hex_dump(PageData, 256);
					user_log_list(ReadEntry, (USERLOG_ENTRY *)PageData);
				}
				ReadEntry++;
				// reading zeroes delay
//				vTaskDelay(pdMS_TO_TICKS(10));
			}

		}
		else
		{
			cmd_log_help();
		}
	}
	else if (strcasecmp(argv[1], "erase") == 0)
	{
		if (argc == 3)
		{
			// Base of zero allows decimal or hex with "0x..."
			EraseAddr = strtol(argv[2], NULL, 0);
			PRINTF(" Erase address: %ld  0x%x\n", EraseAddr, EraseAddr);
			if ((EraseAddr % AB_FLASH_SECTOR_SIZE) != 0)
			{
				PRINTF(" *** WARNING - not a sector address\n");
			}
			read_status =  flash_erase_sector(EraseAddr);
			if(!read_status)
			{
				PRINTF(" Unable to erase sector\n");
			}
			else
			{
				PRINTF(" Sector erased\n");
			}
		}
		else
		{
			cmd_log_help();
		}
	}
	else
	{
		cmd_log_help();
		return;
	}





#if 0
	if (argc < 4)
	{
		PRINTF("Usage: cmd_i2cwrite deviceAddress regAddress length\n   ex) cmd_i2cwrite f ab\n\n");
		return;
	}
	deviceAddress = strtol(argv[1], NULL, 16);
	regAddress = strtol(argv[2], NULL, 16);
	length = strtol(argv[3], NULL, 10);
	PRINTF("I2READ CMD : 0x%04X 0x%04X %d\r\n", deviceAddress, regAddress, length);
	i2c_data[0] = regAddress; //Word Address to Write Data. 2 Bytes. refer at24c512 DataSheet
	// Handle, buffer, length, stop enable, dummy
	status = i2cRead(deviceAddress, i2c_data, length);
	PRINTF("Status: %d\r\n",status);

	for(i=0; i< length; i++)
	{
		PRINTF("add: 0x%02X 0x%02X\r\n",i+regAddress,i2c_data[i]);
	}
#endif

}


void cmd_i2cread(int argc, char *argv[])
{
	int status, i;
	int deviceAddress = 0, length = 1, regAddress = 0;

	if (argc < 4)
	{
		PRINTF("Usage: cmd_i2cwrite deviceAddress regAddress length\n   ex) cmd_i2cwrite f ab\n\n");
		return;
	}
	deviceAddress = strtol(argv[1], NULL, 16);
	regAddress = strtol(argv[2], NULL, 16);
	length = strtol(argv[3], NULL, 10);
	PRINTF("I2READ CMD : 0x%04X 0x%04X %d\r\n", deviceAddress, regAddress, length);
	i2c_data[0] = regAddress; //Word Address to Write Data. 2 Bytes. refer at24c512 DataSheet
	// Handle, buffer, length, stop enable, dummy
	status = i2cRead(deviceAddress, i2c_data, length);
	PRINTF("Status: %d\r\n",status);

	for(i=0; i< length; i++)
	{
		PRINTF("add: 0x%02X 0x%02X\r\n",i+regAddress,i2c_data[i]);
	}

}


void cmd_i2cwrite(int argc, char *argv[])
{
	int status;
	int deviceAddress = 0, data = 1, regAddress = 0;

	if (argc < 4)
	{
		PRINTF("Usage: cmd_i2cwrite deviceAddress regAddress data\n   ex) cmd_i2cwrite f ab\n\n");
		return;
	}
	deviceAddress = strtol(argv[1], NULL, 16);
	regAddress = strtol(argv[2], NULL, 16);
	data = strtol(argv[3], NULL, 16);
	i2c_data[0] = regAddress; //Word Address to Write Data. 2 Bytes. refer at24c512 DataSheet
	i2c_data[1] = data; //Word Address to Write Data. 2 Bytes. refer at24c512 DataSheet

	PRINTF("I2CWRITE CMD : 0x%02X 0x%03X 0x%02X\r\n", deviceAddress, regAddress, data);
	// Handle, buffer, length, stop enable, dummy
//	status = DRV_I2C_WRITE(I2C, i2c_data,1, 1, 0);
	status = i2cWriteStop(deviceAddress, i2c_data, 2);
//	status = i2cWriteNoStop(address, i2c_data, 1);
	PRINTF("Status: %d\r\n",status);
}
#if 0
// Note NJ moved to M363x.c module
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
#endif

void cmd_run(int argc, char *argv[])
{
	int storedRunFlag;

	if (argc > 2)
	{
		PRINTF("Usage: run  [0/1]\n   ex) run 1\n\n");
		return;
	}
	if (argc == 2)
	{
		runFlag = strtol(argv[1], NULL, 10) & 0x1;
		user_set_int(DA16X_CONF_INT_RUN_FLAG, runFlag, 0);
	}
	PRINTF("\n### run CMD : %d ###\n\n", runFlag);
	user_get_int(DA16X_CONF_INT_RUN_FLAG, &storedRunFlag);
	PRINTF("NVRam runFlag: %i\r\n",storedRunFlag);
}


#if 0 //!defined (__BLE_COMBO_REF__)
/**
 ****************************************************************************************
 * @brief function for command test
 * @param[in] argc  not yet used
 * @param[in] argv[]  not yet used
 * @return  void
 ****************************************************************************************
 */
void cmd_app_ap_mode_prov_start(int argc, char *argv[])
{
        app_start_provisioning(AWS_MODE_GEN);
}

void cmd_app_ap_mode_reset(int argc, char *argv[])
{
	APRINTF("\n[CMD APP] ap_mode_reset....");
	app_reboot_ap_mode(SYSMODE_AP_ONLY, AP_OPEN_MODE);       //  1: remove NVRAM
}

#define TEST_HOTSPOT_NAME	"TestHotspot"
#define TEST_HOTSPOT_PW		"12345678"
void cmd_app_sta_mode_reset(int argc, char *argv[])
{
	app_prov_config_t config;
	APRINTF("\n[CMD APP] STA_mode_reset....");

	memset(&config, 0, sizeof(app_prov_config_t));
	strcpy(config.ssid, TEST_HOTSPOT_NAME);
	config.auth_type = 3; //WPA2-PSK
	strcpy(config.psk, TEST_HOTSPOT_PW);
	config.auto_restart_flag = 1; //restart
	config.dpm_mode = 0; //disable
	app_reset_to_station_mode(1, &config);
}

#endif          // !defined (__BLE_COMBO_REF__)














//Commented out - NJ 05/19/2022
#if 0
const COMMAND_TREE_TYPE	cmd_user_list[] = {
	{ "user",			CMD_MY_NODE,	cmd_user_list,	NULL,		"User cmd "				},	// Head

	{ "-------",		CMD_FUNC_NODE,	NULL,	NULL,						"--------------------------------"	},
	{ "testcmd",		CMD_FUNC_NODE,	NULL,	&cmd_test,					"testcmd [option]"					},
#if defined(__COAP_CLIENT_SAMPLE__)
	{ "coap_client",	CMD_FUNC_NODE,	NULL,	&cmd_coap_client,			"CoAP Client"						},
#endif /* __COAP_CLIENT_SAMPLE__ */
    { NULL, 			CMD_NULL_NODE,	NULL,	NULL,	NULL 		}	// Tail
};

//
//-----------------------------------------------------------------------
// Internal Functions
//-----------------------------------------------------------------------

void cmd_test(int argc, char *argv[])
{
	if (argc < 2)
	{
		PRINTF("Usage: testcmd [option]\n   ex) testcmd test\n\n");
		return;
	}

	PRINTF("\n### TEST CMD : %s ###\n\n", argv[1]);
}

#if defined(__COAP_CLIENT_SAMPLE__)
void cmd_coap_client(int argc, char *argv[])
{
	extern void coap_client_sample_cmd(int argc, char *argv[]);

	coap_client_sample_cmd(argc, argv);

	return ;
}
#endif /* __COAP_CLIENT_SAMPLE__ */


#endif



/* EOF */
