/**
 ****************************************************************************************
 *
 * @file user_command.c
 *
 * @brief Console command specified by customer
 *
 *
 * Modified from Renesas Electronics SDK example code with the same name
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


void cmd_run(int argc, char *argv[]);



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
	{ "testcmd",		CMD_FUNC_NODE,  NULL,          &cmd_test,						"testcmd [option]"                 },
	{ "run",			CMD_FUNC_NODE,	NULL,			&cmd_run,						"run [0/1/2]"					},
    { "-------",     	CMD_FUNC_NODE,  NULL,          	NULL,             				"--------------------------------" },
#if defined(__COAP_CLIENT_SAMPLE__)
    { "coap_client", CMD_FUNC_NODE,  NULL,          &cmd_coap_client, "CoAP Client"                      },
#endif /* __COAP_CLIENT_SAMPLE__ */

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
	ledColor.count1 = (int16_t)count1;
	ledColor.countReload1 = (int16_t)count1;
	ledColor.color2 = number2;
	ledColor.state2 = state2;
	ledColor.count2 = (int16_t)count2;
	ledColor.countReload2 = (int16_t)count2;
	ledColor.inProgress = 0;
	ledColor.secondsTotal = (int16_t)secondsTotal;
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

			}
			break;

		default:
			break;
	}
	if(ledColor.inProgress == 0)
	{
		if(ledColor.count1 == 0)
		{
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
	UINT32	ioctldata[3];
	TIMER_INFO_TYPE tinfo[2];
	UINT32 	cursysclock;



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

		//checkflag |= 0x0004;
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


void cmd_run(int argc, char *argv[])
{
	int storedRunFlag;

	if (argc > 2)
	{
		PRINTF("Usage: run  [0/1/2]\n   ex) run 1\n\n");
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

}

#if defined(__COAP_CLIENT_SAMPLE__)
void cmd_coap_client(int argc, char *argv[])
{
    extern void coap_client_sample_cmd(int argc, char *argv[]);

    coap_client_sample_cmd(argc, argv);

    return;
}
#endif /* __COAP_CLIENT_SAMPLE__ */

/* EOF */


/* EOF */
