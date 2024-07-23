/*
 * wpa_supplicant/hostapd / common helper functions, etc.
 * Copyright (c) 2002-2007, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"

#ifdef CONFIG_STA_COUNTRY_CODE
#include "supp_config.h"
#include "da16x_dpm_regs.h"
#include "da16x_dpm.h"
#endif /* CONFIG_STA_COUNTRY_CODE */
#include "common_utils.h"

extern int country_channel_range (char* country, int *startCH, int *endCH);
extern void reboot_func(UINT flag);

#ifdef CONFIG_LOG_MASK
static unsigned long supp_log_mask = SUPP_LOG_DEFAULT;

void set_log_mask(unsigned long mask, unsigned long enable)
{
	if (!mask) {
		if (enable == 0) {
			supp_log_mask = SUPP_LOG_DEFAULT;
		} else {
			supp_log_mask = 0;
		}
	} else if (enable) {
		supp_log_mask |= mask;
	} else {
		supp_log_mask &= ~mask;
	}
 
//	da16x_notice_prt("[%s] supp_log_mask = %d\n", __func__, supp_log_mask);
}

void set_log_mask_whole(unsigned long mask)
{
    supp_log_mask = mask;
}

void set_log_mask_whole_wpa(int mask)
{
    wpa_debug_level = mask;
}

int get_log_mask(void)
{
	return supp_log_mask;
}
#endif /* CONFIG_LOG_MASK */

static int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}


int hex2byte(const char *hex)
{
	int a, b;
	a = hex2num(*hex++);
	if (a < 0)
		return -1;
	b = hex2num(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}


static const char * hwaddr_parse(const char *txt, u8 *addr)
{
	size_t i;

	for (i = 0; i < ETH_ALEN; i++) {
		int a;

		a = hex2byte(txt);
		if (a < 0)
			return NULL;
		txt += 2;
		addr[i] = a;
		if (i < ETH_ALEN - 1 && *txt++ != ':')
			return NULL;
	}
	return txt;
}


/**
 * hwaddr_aton - Convert ASCII string to MAC address (colon-delimited format)
 * @txt: MAC address as a string (e.g., "00:11:22:33:44:55")
 * @addr: Buffer for the MAC address (ETH_ALEN = 6 bytes)
 * Returns: 0 on success, -1 on failure (e.g., string not a MAC address)
 */
int hwaddr_aton(const char *txt, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++) {
		int a, b;

		a = hex2num(*txt++);
		if (a < 0)
			return -1;
		b = hex2num(*txt++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
		if (i < 5 && *txt++ != ':')
			return -1;
	}

	return 0;
}

/*
 * hwaddr_masked_aton - Convert ASCII string with optional mask to MAC address (colon-delimited format)
 * @txt: MAC address with optional mask as a string (e.g., "00:11:22:33:44:55/ff:ff:ff:ff:00:00")
 * @addr: Buffer for the MAC address (ETH_ALEN = 6 bytes)
 * @mask: Buffer for the MAC address mask (ETH_ALEN = 6 bytes)
 * @maskable: Flag to indicate whether a mask is allowed
 * Returns: 0 on success, -1 on failure (e.g., string not a MAC address)
 */
int hwaddr_masked_aton(const char *txt, u8 *addr, u8 *mask, u8 maskable)
{
	const char *r;

#ifdef CHECK_SUPPLICANT_ERR
	extern int isspace (int __c);
#endif /*CHECK_SUPPLICANT_ERR*/

	/* parse address part */
	r = hwaddr_parse(txt, addr);
	if (!r)
		return -1;

	/* check for optional mask */
	if (*r == '\0' || isspace((unsigned char) *r)) {
		/* no mask specified, assume default */
		os_memset(mask, 0xff, ETH_ALEN);
	} else if (maskable && *r == '/') {
		/* mask specified and allowed */
		r = hwaddr_parse(r + 1, mask);
		/* parser error? */
		if (!r)
			return -1;
	} else {
		/* mask specified but not allowed or trailing garbage */
		return -1;
	}

	return 0;
}

/**
 * hwaddr_compact_aton - Convert ASCII string to MAC address (no colon delimitors format)
 * @txt: MAC address as a string (e.g., "001122334455")
 * @addr: Buffer for the MAC address (ETH_ALEN = 6 bytes)
 * Returns: 0 on success, -1 on failure (e.g., string not a MAC address)
 */
int hwaddr_compact_aton(const char *txt, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++) {
		int a, b;

		a = hex2num(*txt++);
		if (a < 0)
			return -1;
		b = hex2num(*txt++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
	}

	return 0;
}

/**
 * hwaddr_aton2 - Convert ASCII string to MAC address (in any known format)
 * @txt: MAC address as a string (e.g., 00:11:22:33:44:55 or 0011.2233.4455)
 * @addr: Buffer for the MAC address (ETH_ALEN = 6 bytes)
 * Returns: Characters used (> 0) on success, -1 on failure
 */
int hwaddr_aton2(const char *txt, u8 *addr)
{
	int i;
	const char *pos = txt;

	for (i = 0; i < 6; i++) {
		int a, b;

		while (*pos == ':' || *pos == '.' || *pos == '-')
			pos++;

		a = hex2num(*pos++);
		if (a < 0)
			return -1;
		b = hex2num(*pos++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
	}

	return pos - txt;
}


/**
 * hexstr2bin - Convert ASCII hex string into binary data
 * @hex: ASCII hex string (e.g., "01ab")
 * @buf: Buffer for the binary data
 * @len: Length of the text to convert in bytes (of buf); hex will be double
 * this size
 * Returns: 0 on success, -1 on failure (invalid hex string)
 */
int hexstr2bin(const char *hex, u8 *buf, size_t len)
{
	size_t i;
	int a;
	const char *ipos = hex;
	u8 *opos = buf;

	for (i = 0; i < len; i++) {
		a = hex2byte(ipos);
		if (a < 0)
			return -1;
		*opos++ = a;
		ipos += 2;
	}
	return 0;
}

#ifndef	NO_CONFIG_WRITE
int hwaddr_mask_txt(char *buf, size_t len, const u8 *addr, const u8 *mask)
{
	size_t i;
	int print_mask = 0;
	int res;

	for (i = 0; i < ETH_ALEN; i++) {
		if (mask[i] != 0xff) {
			print_mask = 1;
			break;
		}
	}

	if (print_mask)
		res = os_snprintf(buf, len, MACSTR "/" MACSTR,
				  MAC2STR(addr), MAC2STR(mask));
	else
		res = os_snprintf(buf, len, MACSTR, MAC2STR(addr));
	if (os_snprintf_error(len, res))
		return -1;
	return res;
}
#endif	/* ! NO_CONFIG_WRITE */

/**
 * inc_byte_array - Increment arbitrary length byte array by one
 * @counter: Pointer to byte array
 * @len: Length of the counter in bytes
 *
 * This function increments the last byte of the counter by one and continues
 * rolling over to more significant bytes if the byte was incremented from
 * 0xff to 0x00.
 */
void inc_byte_array(u8 *counter, size_t len)
{
	int pos = len - 1;
	while (pos >= 0) {
		counter[pos]++;
		if (counter[pos] != 0)
			break;
		pos--;
	}
}

#ifdef CONFIG_OWE_TRANS
void disc_byte_array(u8 *counter, size_t len)
{
	int pos = len - 1;
	while (pos >= 0) {
		counter[pos]--;
		if (counter[pos] != 0)
			break;
		pos--;
	}
}
#endif // CONFIG_OWE_TRANS

void wpa_get_ntp_timestamp(u8 *buf)
{
	struct os_time now;
	u32 sec, usec;
	be32 tmp;

	/* 64-bit NTP timestamp (time from 1900-01-01 00:00:00) */
	os_get_time(&now);
	sec = now.sec + 2208988800U; /* Epoch to 1900 */
	/* Estimate 2^32/10^6 = 4295 - 1/32 - 1/512 */
	usec = now.usec;
	usec = 4295 * usec - (usec >> 5) - (usec >> 9);
	tmp = host_to_be32(sec);
	os_memcpy(buf, (u8 *) &tmp, 4);
	tmp = host_to_be32(usec);
	os_memcpy(buf + 4, (u8 *) &tmp, 4);
}


static inline int _wpa_snprintf_hex(char *buf, size_t buf_size, const u8 *data,
				    size_t len, int uppercase)
{
	size_t i;
	char *pos = buf, *end = buf + buf_size;
	int ret;
	if (buf_size == 0)
		return 0;
	for (i = 0; i < len; i++) {
		ret = os_snprintf(pos, end - pos, uppercase ? "%02X" : "%02x",
				  data[i]);
		if (ret < 0 || ret >= end - pos) {
			end[-1] = '\0';
			return pos - buf;
		}
		pos += ret;
	}
	end[-1] = '\0';
	return pos - buf;
}

/**
 * wpa_snprintf_hex - Print data as a hex string into a buffer
 * @buf: Memory area to use as the output buffer
 * @buf_size: Maximum buffer size in bytes (should be at least 2 * len + 1)
 * @data: Data to be printed
 * @len: Length of data in bytes
 * Returns: Number of bytes written
 */
int wpa_snprintf_hex(char *buf, size_t buf_size, const u8 *data, size_t len)
{
	return _wpa_snprintf_hex(buf, buf_size, data, len, 0);
}


/**
 * wpa_snprintf_hex_uppercase - Print data as a upper case hex string into buf
 * @buf: Memory area to use as the output buffer
 * @buf_size: Maximum buffer size in bytes (should be at least 2 * len + 1)
 * @data: Data to be printed
 * @len: Length of data in bytes
 * Returns: Number of bytes written
 */
int wpa_snprintf_hex_uppercase(char *buf, size_t buf_size, const u8 *data,
			       size_t len)
{
	return _wpa_snprintf_hex(buf, buf_size, data, len, 1);
}


#if 1//def CONFIG_ANSI_C_EXTRA

#ifdef _WIN32_WCE
void perror(const char *s)
{
	wpa_printf(MSG_ERROR, "%s: GetLastError: %d",
		   s, (int) GetLastError());
}
#endif /* _WIN32_WCE */

#ifndef SUPPORT_FREERTOS
int optind = 1;
#endif	/*SUPPORT_FREERTOS*/
int optopt;
char *optarg;


#ifndef SUPPORT_FREERTOS
int getopt(int argc, char *const argv[], const char *optstring)
{
	static int optchr = 1;
	char *cp;

	if (optchr == 1) {
		if (optind >= argc) {
			/* all arguments processed */
			return EOF;
		}

		if (argv[optind][0] != '-' || argv[optind][1] == '\0') {
			/* no option characters */
			return EOF;
		}
	}

	if (os_strcmp(argv[optind], "--") == 0) {
		/* no more options */
		optind++;
		return EOF;
	}

	optopt = argv[optind][optchr];
	cp = os_strchr(optstring, optopt);
	if (cp == NULL || optopt == ':') {
		if (argv[optind][++optchr] == '\0') {
			optchr = 1;
			optind++;
		}
		return '?';
	}

	if (cp[1] == ':') {
		/* Argument required */
		optchr = 1;
		if (argv[optind][optchr + 1]) {
			/* No space between option and argument */
			optarg = &argv[optind++][optchr + 1];
		} else if (++optind >= argc) {
			/* option requires an argument */
			return '?';
		} else {
			/* Argument in the next argv */
			optarg = argv[optind++];
		}
	} else {
		/* No argument */
		if (argv[optind][++optchr] == '\0') {
			optchr = 1;
			optind++;
		}
		optarg = NULL;
	}
	return *cp;
}
#endif	/*SUPPORT_FREERTOS*/
#endif /* CONFIG_ANSI_C_EXTRA */


#ifdef CONFIG_NATIVE_WINDOWS
/**
 * wpa_unicode2ascii_inplace - Convert unicode string into ASCII
 * @str: Pointer to string to convert
 *
 * This function converts a unicode string to ASCII using the same
 * buffer for output. If UNICODE is not set, the buffer is not
 * modified.
 */
void wpa_unicode2ascii_inplace(TCHAR *str)
{
#ifdef UNICODE
	char *dst = (char *) str;
	while (*str)
		*dst++ = (char) *str++;
	*dst = '\0';
#endif /* UNICODE */
}


TCHAR * wpa_strdup_tchar(const char *str)
{
#ifdef UNICODE
	TCHAR *buf;
	buf = os_malloc((strlen(str) + 1) * sizeof(TCHAR));
	if (buf == NULL)
		return NULL;
	wsprintf(buf, L"%S", str);
	return buf;
#else /* UNICODE */
	return os_strdup(str);
#endif /* UNICODE */
}
#endif /* CONFIG_NATIVE_WINDOWS */


void printf_encode(char *txt, size_t maxlen, const u8 *data, size_t len)
{
	char *end = txt + maxlen;
	size_t i;

	for (i = 0; i < len; i++) {
		if (txt + 4 >= end)
			break;

		switch (data[i]) {
		case '\"':
			*txt++ = '\\';
			*txt++ = '\"';
			break;
		case '\\':
			*txt++ = '\\';
			*txt++ = '\\';
			break;
#if 0	/* by Bright */
		case '\e':
			*txt++ = '\\';
			*txt++ = 'e';
			break;
#endif	/* 0 */

		case '\n':
			*txt++ = '\\';
			*txt++ = 'n';
			break;
		case '\r':
			*txt++ = '\\';
			*txt++ = 'r';
			break;
		case '\t':
			*txt++ = '\\';
			*txt++ = 't';
			break;
		default:
			if (data[i] >= 32 && data[i] <= 127) {
				*txt++ = data[i];
			} else {
				txt += os_snprintf(txt, end - txt, "\\x%02x",
						   data[i]);
			}
			break;
		}
	}

	*txt = '\0';
}


size_t printf_decode(u8 *buf, size_t maxlen, const char *str)
{
	const char *pos = str;
	size_t len = 0;
	int val;

	while (*pos)
	{
		if (len + 1 >= maxlen)
			break;

		switch (*pos)
		{
			case '\\':
				pos++;
				switch (*pos)
				{
#if 0	/* Does not supported */
					case '\\':
						buf[len++] = '\\';
						pos++;
						break;
#endif	/* 0 */

					case '"':
						buf[len++] = '"';
						pos++;
						break;
#if 1 // ORG String(No conversion.)
					case 'n':
						buf[len++] = '\\';
						buf[len++] = 'n';
						pos++;
						break;

					case 'r':
						buf[len++] = '\\';
						buf[len++] = 'r';
						pos++;
						break;

					case 't':
						buf[len++] = '\\';
						buf[len++] = 't';
						pos++;
						break;
#else
					case 'n':
						buf[len++] = '\n';
						pos++;
						break;

					case 'r':
						buf[len++] = '\r';
						pos++;
						break;

					case 't':
						buf[len++] = '\t';
						pos++;
						break;
#endif /* 0 */

#if 0	/* Does not supported */
				case '\e':
					case 'e':
						buf[len++] = '\e';
						pos++;
						break;
#endif	/* 0 */
					case 'x':
						pos++;
						val = hex2byte(pos);
						if (val < 0) {
							val = hex2num(*pos);
							if (val < 0)
								break;
							buf[len++] = val;
							pos++;
						} else {
							buf[len++] = val;
							pos += 2;
						}
						break;

#if 0 // Doesn't support the Octal-Number cases ...
					case '0':
					case '1':
					case '2':
					case '3':
					case '4':
					case '5':
					case '6':
					case '7':
						val = *pos++ - '0';
						if (*pos >= '0' && *pos <= '7')
							val = val * 8 + (*pos++ - '0');
						if (*pos >= '0' && *pos <= '7')
							val = val * 8 + (*pos++ - '0');
						buf[len++] = val;
						break;
#endif // 0

					default:
                        buf[len++] = *(pos-1);
						break;
				}
				break;

			default:
				buf[len++] = *pos++;
				break;
		}
	}

	if (maxlen > len)
		buf[len] = '\0';

	return len;
}


/**
 * wpa_ssid_txt - Convert SSID to a printable string
 * @ssid: SSID (32-octet string)
 * @ssid_len: Length of ssid in octets
 * Returns: Pointer to a printable string
 *
 * This function can be used to convert SSIDs into printable form. In most
 * cases, SSIDs do not use unprintable characters, but IEEE 802.11 standard
 * does not limit the used character set, so anything could be used in an SSID.
 *
 * This function uses a static buffer, so only one call can be used at the
 * time, i.e., this is not re-entrant and the returned buffer must be used
 * before calling this again.
 */
const char * wpa_ssid_txt(const u8 *ssid, size_t ssid_len)
{
	static char ssid_txt[32 * 4 + 1];

	if (ssid == NULL) { /* hidden ssid */
		ssid_txt[0] = '\0';
		return ssid_txt;
	}

	printf_encode(ssid_txt, sizeof(ssid_txt), ssid, ssid_len);
	return ssid_txt;
}


#ifdef SUPPLICANT_PLAIN_TEXT_SSID
const char * wpa_ssid_plain_txt(const u8 *ssid, size_t ssid_len)
{
#if 1	/* Need to assign single variable "ssid_txt[]" */
	static char ssid_txt[32 * 4 + 1];
#else
	static char ssid_txt[32 + 1];
#endif	/* 0 */

	if (ssid == NULL) { /* hidden ssid */
		ssid_txt[0] = '\0';
		return ssid_txt;
	}

	memset(ssid_txt, 0, sizeof(ssid_txt));
	strncpy(ssid_txt, (char*)ssid, ssid_len);

	return ssid_txt;
}
#endif /* SUPPLICANT_PLAIN_TEXT_SSID */


void * __hide_aliasing_typecast(void *foo)
{
	return foo;
}


char * wpa_config_parse_string(const char *value, size_t *len)
{
	if (*value == '\'') {
		const char *pos;
		char *str;

		value++;
		pos = os_strrchr(value, '\'');
		
		if (pos == NULL || pos[1] != '\0') {
			da16x_cli_prt("[%s] value= [ %s ]\n",
					__func__, value);
			return NULL;
		}

		*len = pos - value;
		str = dup_binstr(value, *len);

		if (str == NULL) {
			da16x_cli_prt("[%s] value= [ %s ]\n",
					__func__, value);
			return NULL;
		}

		return str;
	} else if (*value == '\"') {
		const char *pos;
		char *str;
		
		value++;
		pos = os_strrchr(value, '\"');

		if (pos == NULL || pos[1] != '\0') {
			return NULL;
		}

		*len = pos - value;
		str = dup_binstr(value, *len);

		if (str == NULL) {
			return NULL;
		}
		return str;
	} else if (*value == 'P' && value[1] == '"') {
		const char *pos;
		char *tstr, *str;
		size_t tlen;
		value += 2;
		pos = os_strrchr(value, '"');
		if (pos == NULL || pos[1] != '\0')
			return NULL;
		tlen = pos - value;
		tstr = dup_binstr(value, tlen);
		if (tstr == NULL)
			return NULL;

		str = os_malloc(tlen + 1);
		if (str == NULL) {
			os_free(tstr);
			return NULL;
		}

		*len = printf_decode((u8 *) str, tlen + 1, tstr);
		os_free(tstr);

		return str;
	} else {
		u8 *str;
		size_t tlen, hlen = os_strlen(value);
		if (hlen & 1)
			return NULL;
		tlen = hlen / 2;
		str = os_malloc(tlen + 1);
		if (str == NULL)
			return NULL;
		if (hexstr2bin(value, str, tlen)) {
			os_free(str);
			return NULL;
		}
		str[tlen] = '\0';
		*len = tlen;
		return (char *) str;
	}
}


int is_hex(const u8 *data, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (data[i] < 32 || data[i] >= 127)
			return 1;
	}
	return 0;
}


int find_first_bit2(u32 value)
{
	int pos = 0;

	while (value) {
		if (value & 0x1)
			return pos;
		value >>= 1;
		pos++;
	}

	return -1;
}


int has_ctrl_char(const u8 *data, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (data[i] < 32 || data[i] == 127)
			return 1;
	}
	return 0;
}


int has_newline(const char *str)
{
	while (*str) {
		if (*str == '\n' || *str == '\r')
			return 1;
		str++;
	}
	return 0;
}


size_t merge_byte_arrays(u8 *res, size_t res_len,
			 const u8 *src1, size_t src1_len,
			 const u8 *src2, size_t src2_len)
{
	size_t len = 0;

	os_memset(res, 0, res_len);

	if (src1) {
		if (src1_len >= res_len) {
			os_memcpy(res, src1, res_len);
			return res_len;
		}

		os_memcpy(res, src1, src1_len);
		len += src1_len;
	}

	if (src2) {
		if (len + src2_len >= res_len) {
			os_memcpy(res + len, src2, res_len - len);
			return res_len;
		}

		os_memcpy(res + len, src2, src2_len);
		len += src2_len;
	}

	return len;
}


char * dup_binstr(const void *src, size_t len)
{
	char *res;

	if (src == NULL)
		return NULL;
	res = os_malloc(len + 1);
	if (res == NULL)
		return NULL;
	os_memcpy(res, src, len);
	res[len] = '\0';

	return res;
}


int freq_range_list_parse(struct wpa_freq_range_list *res, const char *value)
{
	struct wpa_freq_range *freq = NULL, *n;
	unsigned int count = 0;
	const char *pos, *pos2, *pos3;

	/*
	 * Comma separated list of frequency ranges.
	 * For example: 2412-2432,2462,5000-6000
	 */
	pos = value;
	while (pos && pos[0]) {
		n = os_realloc_array(freq, count + 1,
				     sizeof(struct wpa_freq_range));
		if (n == NULL) {
			os_free(freq);
			return -1;
		}
		freq = n;
		freq[count].min = atoi(pos);
		pos2 = os_strchr(pos, '-');
		pos3 = os_strchr(pos, ',');
		if (pos2 && (!pos3 || pos2 < pos3)) {
			pos2++;
			freq[count].max = atoi(pos2);
		} else
			freq[count].max = freq[count].min;
		pos = pos3;
		if (pos)
			pos++;
		count++;
	}
	os_free(res->range);
	res->range = freq;
	res->num = count;

	return 0;
}


int freq_range_list_includes(const struct wpa_freq_range_list *list,
			     unsigned int freq)
{
	unsigned int i;

	if (list == NULL)
		return 0;

	for (i = 0; i < list->num; i++) {
		if (freq >= list->range[i].min && freq <= list->range[i].max)
			return 1;
	}

	return 0;
}


char * freq_range_list_str(const struct wpa_freq_range_list *list)
{
	char *buf, *pos, *end;
	size_t maxlen;
	unsigned int i;
	int res;

	if (list->num == 0)
		return NULL;

	maxlen = list->num * 30;
	buf = os_malloc(maxlen);
	if (buf == NULL)
		return NULL;
	pos = buf;
	end = buf + maxlen;

	for (i = 0; i < list->num; i++) {
		struct wpa_freq_range *range = &list->range[i];

		if (range->min == range->max)
			res = os_snprintf(pos, end - pos, "%s%u",
					  i == 0 ? "" : ",", range->min);
		else
			res = os_snprintf(pos, end - pos, "%s%u-%u",
					  i == 0 ? "" : ",",
					  range->min, range->max);
		if (res < 0 || res > end - pos) {
			os_free(buf);
			return NULL;
		}
		pos += res;
	}

	return buf;
}

#ifdef CONFIG_STA_COUNTRY_CODE
extern int i3ed11_ch_to_freq(int chan, int band);
int country_to_freq_range_list(struct wpa_freq_range * freq_range, const char *country)
{
	unsigned int i;

	for (i = 0; i < NUM_SUPPORTS_COUNTRY; i++) {
		const struct country_ch_power_level *field = (void*)cc_power_table(i);
		int min_chan, max_chan;
		
		country_channel_range(field->country, &min_chan, &max_chan);

		if (os_strncasecmp(country, field->country, 3) == 0) {
			freq_range->min = i3ed11_ch_to_freq(min_chan, 0);
			freq_range->max = i3ed11_ch_to_freq(max_chan, 0);
			da16x_scan_prt(" min_chan = %d, max_chan = %d\n", freq_range->min, freq_range->max);
			dpm_save_country_code(country);

			return 0;
		}
	}

	da16x_err_prt("\nNot Support Country Code (%s)\n",country);
	return -1;
}

#endif /* CONFIG_STA_COUNTRY_CODE */

int int_array_len(const int *a)
{
	int i;
	for (i = 0; a && a[i]; i++)
		;
	return i;
}


void int_array_concat(int **res, const int *a)
{
	int reslen, alen, i;
	int *n;

	reslen = int_array_len(*res);
	alen = int_array_len(a);

	n = os_realloc_array(*res, reslen + alen + 1, sizeof(int));
	if (n == NULL) {
		os_free(*res);
		*res = NULL;
		return;
	}
	for (i = 0; i <= alen; i++)
		n[reslen + i] = a[i];
	*res = n;
}


static int freq_cmp(const void *a, const void *b)
{
	int _a = *(int *) a;
	int _b = *(int *) b;

	if (_a == 0)
		return 1;
	if (_b == 0)
		return -1;
	return _a - _b;
}


void int_array_sort_unique(int *a)
{
	int alen;
	int i, j;

	if (a == NULL)
		return;

	alen = int_array_len(a);
	qsort(a, alen, sizeof(int), freq_cmp);

	i = 0;
	j = 1;
	while (a[i] && a[j]) {
		if (a[i] == a[j]) {
			j++;
			continue;
		}
		a[++i] = a[j++];
	}
	if (a[i])
		i++;
	a[i] = 0;
}

void swap_buf(const u8 *orig, u8 *swap, int len)
{
	int     i, j;

	os_memset(swap, 0, len);
	for (i = len - 1, j = 0; i >= 0; i--, j++) {
		swap[j] = orig[i];
	}

	return ;
}

void int_array_add_unique(int **res, int a)
{
	int reslen;
	int *n;

	for (reslen = 0; *res && (*res)[reslen]; reslen++) {
		if ((*res)[reslen] == a)
			return; /* already in the list */
	}

	n = os_realloc_array(*res, reslen + 2, sizeof(int));
	if (n == NULL) {
		os_free(*res);
		*res = NULL;
		return;
	}

	n[reslen] = a;
	n[reslen + 1] = 0;

	*res = n;
}

#ifdef	CONFIG_SUPP27_STR_CLR_FREE
void str_clear_free(char *str)
{
	if (str) {
		size_t len = os_strlen(str);
		os_memset(str, 0, len);
		os_free(str);
	}
}
#endif	/* CONFIG_SUPP27_STR_CLR_FREE */

#ifdef	CONFIG_SUPP27_BIN_CLR_FREE
void bin_clear_free(void *bin, size_t len)
{
	if (bin) {
		os_memset(bin, 0, len);
		os_free(bin);
	}
}
#endif	/* CONFIG_SUPP27_BIN_CLR_FREE */


void heap_bin_clear_free(void *bin, size_t len)
{
	if (bin) {
		memset(bin, 0, len);
		os_free(bin);
	}
}


#ifdef	__SUPP_27_SUPPORT__
int random_mac_addr(u8 *addr)
{
	if (os_get_random(addr, ETH_ALEN) < 0)
		return -1;
	addr[0] &= 0xfe; /* unicast */
	addr[0] |= 0x02; /* locally administered */
	return 0;
}


int random_mac_addr_keep_oui(u8 *addr)
{
	if (os_get_random(addr + 3, 3) < 0)
		return -1;
	addr[0] &= 0xfe; /* unicast */
	addr[0] |= 0x02; /* locally administered */
	return 0;
}
#endif	/* __SUPP_27_SUPPORT__ */


/**
 * cstr_token - Get next token from const char string
 * @str: a constant string to tokenize
 * @delim: a string of delimiters
 * @last: a pointer to a character following the returned token
 *      It has to be set to NULL for the first call and passed for any
 *      further call.
 * Returns: a pointer to token position in str or NULL
 *
 * This function is similar to str_token, but it can be used with both
 * char and const char strings. Differences:
 * - The str buffer remains unmodified
 * - The returned token is not a NULL terminated string, but a token
 *   position in str buffer. If a return value is not NULL a size
 *   of the returned token could be calculated as (last - token).
 */
const char * cstr_token(const char *str, const char *delim, const char **last)
{
	const char *end, *token = str;

	if (!str || !delim || !last)
		return NULL;

	if (*last)
		token = *last;

	while (*token && os_strchr(delim, *token))
		token++;

	if (!*token)
		return NULL;

	end = token + 1;

	while (*end && !os_strchr(delim, *end))
		end++;

	*last = end;
	return token;
}


/**
 * str_token - Get next token from a string
 * @buf: String to tokenize. Note that the string might be modified.
 * @delim: String of delimiters
 * @context: Pointer to save our context. Should be initialized with
 *	NULL on the first call, and passed for any further call.
 * Returns: The next token, NULL if there are no more valid tokens.
 */
char * str_token(char *str, const char *delim, char **context)
{
	char *token = (char *) cstr_token(str, delim, (const char **) context);

	if (token && **context)
		*(*context)++ = '\0';

	return token;
}

#ifdef ENABLE_BUF_DUMP_DBG

void da16x_dump(u8 *title, const u8 *buf, size_t len)
{
        int i;
        
	da16x_notice_prt("%s - hexdump(len=%lu):\n\t", title, (unsigned long) len);

        if (buf == NULL) {
		da16x_notice_prt(" [NULL]");
        } else {
		for (i = 0; i < (int)len; i++) {
			if ((i > 0) && (i % 16) == 0)
				da16x_notice_prt("\n\t");

			da16x_notice_prt("%02x ", (unsigned char)buf[i]);
		}
        }
        da16x_notice_prt("\n");
}

void da16x_ascii_dump(u8 *title, const void *buf, size_t len)
{
	size_t i, llen;
	const u8 *pos = buf;
	const size_t line_len = 16;

	da16x_notice_prt("%s - hexdump_ascii(len=%lu):\n\t", title, (unsigned long) len);

	if (buf == NULL) {
		da16x_notice_prt(" - hexdump_ascii(len=%lu): [NULL]\n",
		       			(unsigned long) len);
		return;
	}

	da16x_notice_prt("- (len=%lu):\n", (unsigned long) len);

	while (len) {
		llen = len > line_len ? line_len : len;
		da16x_notice_prt("    ");

		for (i = 0; i < llen; i++)
			da16x_notice_prt(" %02x", pos[i]);
		for (i = llen; i < line_len; i++)
			da16x_notice_prt("  ");

		da16x_notice_prt("  ");

		for (i = 0; i < llen; i++) {
			if (pos[i] >= 0x20 && pos[i] <= 0x7f)
				da16x_notice_prt("%c", pos[i]);
			else
				da16x_notice_prt("_");
		}

		for (i = llen; i < line_len; i++)
			da16x_notice_prt(" ");

		da16x_notice_prt("\n");

		pos += llen;
		len -= llen;
	}
}

void da16x_buf_dump(u8 *title, const struct wpabuf *buf)
{
	da16x_dump(title, buf ? buf->buf : NULL, buf ? buf->used : 0);
}

#endif	/* ENABLE_BUF_DUMP_DBG */

#ifdef CONFIG_LOG_MASK
extern void trc_que_proc_vprint(u16 tag, const char *format, va_list arg);

void da16x_info_prt(const char *fmt,...)
{
	va_list ap;

	if (supp_log_mask & SUPP_LOG_INFO) {
		va_start(ap,fmt);
		trc_que_proc_vprint(0, fmt, ap);
		va_end(ap);		   
	}
}

void da16x_notice_prt(const char *fmt,...)
{
	va_list ap;

	if(supp_log_mask & SUPP_LOG_NOTICE) {
		va_start(ap,fmt);
		trc_que_proc_vprint(0, fmt, ap);
		va_end(ap);		   
	}
}

void da16x_warn_prt(const char *fmt,...)
{
	va_list ap;

	if(supp_log_mask & SUPP_LOG_WARN) {
		va_start(ap,fmt);
		trc_que_proc_vprint(0, fmt, ap);
		va_end(ap);		   
	}
}

void da16x_err_prt(const char *fmt,...)
{
	va_list ap;

	if(supp_log_mask & SUPP_LOG_ERR) {
		va_start(ap,fmt);
		trc_que_proc_vprint(0, fmt, ap);
		va_end(ap);		   
	}
}

void da16x_fatal_prt(const char *fmt,...)
{
	va_list ap;

	if(supp_log_mask & SUPP_LOG_FATAL) {
		va_start(ap,fmt);
		trc_que_proc_vprint(0, fmt, ap);
		va_end(ap);		   
	}
}

void da16x_debug_prt(const char *fmt,...)
{
	va_list ap;

	if(supp_log_mask & SUPP_LOG_DEBUG) {
		va_start(ap,fmt);
		trc_que_proc_vprint(0, fmt, ap);
		va_end(ap);		   
	}
}
#else
void da16x_info_prt(const char *fmt,...)
{
}
#endif /* CONFIG_LOG_MASK */


// supplicant 2.7 ===========================================

#define LOG_HEAP_MALLOC

#ifdef	UNUSED_CODE
void wpa_debug_print_timestamp(void)
{
#ifndef CONFIG_ANDROID_LOG
	struct os_time tv;

	if (!wpa_debug_timestamp)
		return;

	os_get_time(&tv);
#ifdef CONFIG_DEBUG_FILE
	if (out_file) {
		fprintf(out_file, "%ld.%06u: ", (long) tv.sec,
			(unsigned int) tv.usec);
	} else
#endif /* CONFIG_DEBUG_FILE */
	PRINTF("%ld.%06u: ", (long) tv.sec, (unsigned int) tv.usec);
#endif /* CONFIG_ANDROID_LOG */
}
#endif	/* UNUSED_CODE */

/**
 * wpa_printf - conditional printf
 * @level: priority level (MSG_*) of the message
 * @fmt: printf format string, followed by optional arguments
 *
 * This function is used to print conditional debugging and error messages. The
 * output may be directed to stdout, stderr, and/or syslog based on
 * configuration.
 *
 * Note: New line '\n' is added to the end of the text when printing to stdout.
 */
#ifdef CONFIG_WPA_PRINTF
void wpa_printf(int level, const char *fmt, ...)
#else
void _wpa_printf(int level, const char *fmt, ...)
#endif /* CONFIG_WPA_PRINTF */
{
#if	1	// def	CONFIG_DBG_LOG_MSG
	va_list ap;

	va_start(ap, fmt);
	if (level >= wpa_debug_level) {
#if 0 //def CONFIG_ANDROID_LOG
		__android_log_vprint(wpa_to_android_level(level),
				     ANDROID_LOG_NAME, fmt, ap);
#else /* CONFIG_ANDROID_LOG */
#if 0 //def CONFIG_DEBUG_SYSLOG
		if (wpa_debug_syslog) {
			vsyslog(syslog_priority(level), fmt, ap);
		} else {
#endif /* CONFIG_DEBUG_SYSLOG */
#ifdef	UNUSED_CODE
		wpa_debug_print_timestamp();
#endif	/* UNUSED_CODE */
#if 0 //def CONFIG_DEBUG_FILE
		if (out_file) {
			vfprintf(out_file, fmt, ap);
			fprintf(out_file, "\n");
		} else {
#endif /* CONFIG_DEBUG_FILE */

#if 1 // FC9000 ANSI COLOR
		if (level == MSG_USER_DEBUG) {
			PRINTF(ANSI_COLOR_LIGHT_GREEN);
		} else if (level == MSG_WARNING) {
			PRINTF(ANSI_COLOR_LIGHT_YELLOW);
		} else if (level >= MSG_ERROR) {
			PRINTF(ANSI_COLOR_LIGHT_RED);
		} else if (level == MSG_DEBUG) {
			PRINTF(ANSI_COLOR_CYAN);
		}

		trc_que_proc_vprint(0, fmt, ap);

		if (level >= MSG_WARNING || level == MSG_DEBUG || level == MSG_EXCESSIVE || level == MSG_USER_DEBUG) {
			PRINTF(ANSI_COLOR_DEFULT"\n");	
		} else
			PRINTF("\n");
#else
		vprintf(fmt, ap);
		PRINTF("\n");
#endif /* 1 */


#if 0 //def CONFIG_DEBUG_FILE
		}
#endif /* CONFIG_DEBUG_FILE */
#if 0 //def CONFIG_DEBUG_SYSLOG
		}
#endif /* CONFIG_DEBUG_SYSLOG */
#endif /* CONFIG_ANDROID_LOG */
	}
	va_end(ap);

#if 0 //def CONFIG_DEBUG_LINUX_TRACING
	if (wpa_debug_tracing_file != NULL) {
		va_start(ap, fmt);
		fprintf(wpa_debug_tracing_file, WPAS_TRACE_PFX, level);
		vfprintf(wpa_debug_tracing_file, fmt, ap);
		fprintf(wpa_debug_tracing_file, "\n");
		fflush(wpa_debug_tracing_file);
		va_end(ap);
	}
#endif /* CONFIG_DEBUG_LINUX_TRACING */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}


#ifdef CONFIG_WPA_PRINTF

#ifdef ENABLE_HOSTAPD_LOGGER_DBG
void hostapd_logger(void *ctx, const u8 *addr, unsigned int module, int level,
		    const char *fmt, ...)
{
	va_list ap;
	char *buf;
	int buflen;

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
		wpa_printf(MSG_ERROR, "hostapd_logger: Failed to allocate "
			   "message buffer");
		return;
	}
	va_start(ap, fmt);
    vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);
#if 0
	if (hostapd_logger_cb)
		hostapd_logger_cb(ctx, addr, module, level, buf, len);
	else 
#endif /* 0 */
	if (addr)
		wpa_printf_dbg(MSG_DEBUG, "hostapd_logger: STA " MACSTR " - %s",
			   MAC2STR(addr), buf);
	else
		wpa_printf_dbg(MSG_DEBUG, "hostapd_logger: %s", buf);
#ifdef LOG_HEAP_MALLOC
	heap_bin_clear_free(buf, buflen);
#else
	bin_clear_free(buf, buflen);
#endif /* LOG_HEAP_MALLOC */
}
#endif /* ENABLE_HOSTAPD_LOGGER_DBG */


#ifdef ENABLE_HEXDUMP_DBG
static void _wpa_hexdump(int level, const char *title, const u8 *buf,
			 size_t len, int show)
{
	size_t i;

#if 0 //def CONFIG_DEBUG_LINUX_TRACING
	if (wpa_debug_tracing_file != NULL) {
		fprintf(wpa_debug_tracing_file,
			WPAS_TRACE_PFX "%s - hexdump(len=%lu):",
			level, title, (unsigned long) len);
		if (buf == NULL) {
			fprintf(wpa_debug_tracing_file, " [NULL]\n");
		} else if (!show) {
			fprintf(wpa_debug_tracing_file, " [REMOVED]\n");
		} else {
			for (i = 0; i < len; i++)
				fprintf(wpa_debug_tracing_file,
					" %02x", buf[i]);
		}
		fflush(wpa_debug_tracing_file);
	}
#endif /* CONFIG_DEBUG_LINUX_TRACING */

	if (level < wpa_debug_level)
		return;
#if 0 //def CONFIG_ANDROID_LOG
	{
		const char *display;
		char *strbuf = NULL;
		size_t slen = len;
		if (buf == NULL) {
			display = " [NULL]";
		} else if (len == 0) {
			display = "";
		} else if (show && len) {
			/* Limit debug message length for Android log */
			if (slen > 32)
				slen = 32;
#ifdef LOG_HEAP_MALLOC
			strbuf = os_malloc(1 + 3 * slen);
#else
			strbuf = os_malloc(1 + 3 * slen);
#endif /* LOG_HEAP_MALLOC */
			if (strbuf == NULL) {
				wpa_printf(MSG_ERROR, "wpa_hexdump: Failed to "
					   "allocate message buffer");
				return;
			}

			for (i = 0; i < slen; i++)
				os_snprintf(&strbuf[i * 3], 4, " %02x",
					    buf[i]);

			display = strbuf;
		} else {
			display = " [REMOVED]";
		}

		__android_log_print(wpa_to_android_level(level),
				    ANDROID_LOG_NAME,
				    "%s - hexdump(len=%lu):%s%s",
				    title, (long unsigned int) len, display,
				    len > slen ? " ..." : "");
		bin_clear_free(strbuf, 1 + 3 * slen);
		return;
	}
#else /* CONFIG_ANDROID_LOG */
#if 0 //def CONFIG_DEBUG_SYSLOG
	if (wpa_debug_syslog) {
		const char *display;
		char *strbuf = NULL;

		if (buf == NULL) {
			display = " [NULL]";
		} else if (len == 0) {
			display = "";
		} else if (show && len) {
#ifdef LOG_HEAP_MALLOC
			strbuf = os_malloc(1 + 3 * len);
#else
			strbuf = os_malloc(1 + 3 * len);
#endif /* LOG_HEAP_MALLOC */
			if (strbuf == NULL) {
				wpa_printf(MSG_ERROR, "wpa_hexdump: Failed to "
					   "allocate message buffer");
				return;
			}

			for (i = 0; i < len; i++)
				os_snprintf(&strbuf[i * 3], 4, " %02x",
					    buf[i]);

			display = strbuf;
		} else {
			display = " [REMOVED]";
		}

		syslog(syslog_priority(level), "%s - hexdump(len=%lu):%s",
		       title, (unsigned long) len, display);
#ifdef LOG_HEAP_MALLOC
		heap_bin_clear_free(strbuf, 1 + 3 * len);
#else
		bin_clear_free(strbuf, 1 + 3 * len);
#endif /* LOG_HEAP_MALLOC */
		return;
	}
#endif /* CONFIG_DEBUG_SYSLOG */
#ifdef	UNUSED_CODE
	wpa_debug_print_timestamp();
#endif	/* UNUSED_CODE */
#if 0 //def CONFIG_DEBUG_FILE
	if (out_file) {
		fprintf(out_file, "%s - hexdump(len=%lu):",
			title, (unsigned long) len);
		if (buf == NULL) {
			fprintf(out_file, " [NULL]");
		} else if (show) {
			for (i = 0; i < len; i++)
				fprintf(out_file, " %02x", buf[i]);
		} else {
			fprintf(out_file, " [REMOVED]");
		}
		fprintf(out_file, "\n");
	} else {
#endif /* CONFIG_DEBUG_FILE */
	PRINTF("%s - hexdump(len=%lu):\n\t", title, (unsigned long) len);
	if (buf == NULL) {
		PRINTF(" [NULL]");
	} else if (show) {
#if 1 // NOSHOW
		for (i = 0; i < len; i++) {
			if ((i > 0) && (i % 16) == 0)
				PRINTF("\n\t");

			PRINTF(" %02x", buf[i]);
		}
	} else {
#endif /* 0 */
		PRINTF(" [REMOVED]");
	}
	PRINTF("\n");
#if 0 //def CONFIG_DEBUG_FILE
	}
#endif /* CONFIG_DEBUG_FILE */
#endif /* CONFIG_ANDROID_LOG */
}


void wpa_hexdump(int level, const char *title, const void *buf, size_t len)
{
	_wpa_hexdump(level, title, buf, len, wpa_debug_show_keys);
}


#ifdef ENABLE_HEXDUMP_KEY_DBG
void wpa_hexdump_key(int level, const char *title, const void *buf, size_t len)
{
	_wpa_hexdump(level, title, buf, len, wpa_debug_show_keys);
}
#endif /* ENABLE_HEXDUMP_KEY_DBG */


#ifdef ENABLE_HEXDUMP_ASCII_DBG
static void _wpa_hexdump_ascii(int level, const char *title, const void *buf,
			       size_t len, int show)
{
	size_t i, llen;
	const u8 *pos = buf;
	const size_t line_len = 16;

#if 0 //def CONFIG_DEBUG_LINUX_TRACING
	if (wpa_debug_tracing_file != NULL) {
		fprintf(wpa_debug_tracing_file,
			WPAS_TRACE_PFX "%s - hexdump_ascii(len=%lu):",
			level, title, (unsigned long) len);
		if (buf == NULL) {
			fprintf(wpa_debug_tracing_file, " [NULL]\n");
		} else if (!show) {
			fprintf(wpa_debug_tracing_file, " [REMOVED]\n");
		} else {
			/* can do ascii processing in userspace */
			for (i = 0; i < len; i++)
				fprintf(wpa_debug_tracing_file,
					" %02x", pos[i]);
		}
		fflush(wpa_debug_tracing_file);
	}
#endif /* CONFIG_DEBUG_LINUX_TRACING */

	if (level < wpa_debug_level)
		return;
#if 0 //def CONFIG_ANDROID_LOG
	_wpa_hexdump(level, title, buf, len, show);
#else /* CONFIG_ANDROID_LOG */
#ifdef	UNUSED_CODE
	wpa_debug_print_timestamp();
#endif	/* UNUSED_CODE */
#if 0 //def CONFIG_DEBUG_FILE
	if (out_file) {
		if (!show) {
			fprintf(out_file,
				"%s - hexdump_ascii(len=%lu): [REMOVED]\n",
				title, (unsigned long) len);
			return;
		}
		if (buf == NULL) {
			fprintf(out_file,
				"%s - hexdump_ascii(len=%lu): [NULL]\n",
				title, (unsigned long) len);
			return;
		}
		fprintf(out_file, "%s - hexdump_ascii(len=%lu):\n",
			title, (unsigned long) len);
		while (len) {
			llen = len > line_len ? line_len : len;
			fprintf(out_file, "    ");
			for (i = 0; i < llen; i++)
				fprintf(out_file, " %02x", pos[i]);
			for (i = llen; i < line_len; i++)
				fprintf(out_file, "   ");
			fprintf(out_file, "   ");
			for (i = 0; i < llen; i++) {
				if (isprint(pos[i]))
					fprintf(out_file, "%c", pos[i]);
				else
					fprintf(out_file, "_");
			}
			for (i = llen; i < line_len; i++)
				fprintf(out_file, " ");
			fprintf(out_file, "\n");
			pos += llen;
			len -= llen;
		}
	} else {
#endif /* CONFIG_DEBUG_FILE */
	if (!show) {
		PRINTF("%s - hexdump_ascii(len=%lu): [REMOVED]\n",
		       title, (unsigned long) len);
		return;
	}
	if (buf == NULL) {
		PRINTF("%s - hexdump_ascii(len=%lu): [NULL]\n",
		       title, (unsigned long) len);
		return;
	}
	PRINTF("%s - hexdump_ascii(len=%lu):\n", title, (unsigned long) len);
	while (len) {
		llen = len > line_len ? line_len : len;
		PRINTF("    ");
		for (i = 0; i < llen; i++)
			PRINTF(" %02x", pos[i]);
		for (i = llen; i < line_len; i++)
			PRINTF("   ");
		PRINTF("   ");
		for (i = 0; i < llen; i++) {
			if (isprint(pos[i]))
				PRINTF("%c", pos[i]);
			else
				PRINTF("_");
		}
		for (i = llen; i < line_len; i++)
			PRINTF(" ");
		PRINTF("\n");
		pos += llen;
		len -= llen;
	}
#ifdef CONFIG_DEBUG_FILE
	}
#endif /* CONFIG_DEBUG_FILE */
#endif /* CONFIG_ANDROID_LOG */
}

void wpa_hexdump_ascii(int level, const char *title, const void *buf,
		       size_t len)
{
	_wpa_hexdump_ascii(level, title, buf, len, wpa_debug_show_keys);
}



void wpa_hexdump_ascii_key(int level, const char *title, const void *buf,
			   size_t len)
{
	_wpa_hexdump_ascii(level, title, buf, len, wpa_debug_show_keys);
}
#endif /* ENABLE_HEXDUMP_ASCII_DBG */


#include "wpabuf.h"


/**
 * wpa_hexdump - conditional hex dump
 * @level: priority level (MSG_*) of the message
 * @title: title of for the message
 * @buf: data buffer to be dumped
 * @len: length of the buf
 *
 * This function is used to print conditional debugging and error messages. The
 * output may be directed to stdout, stderr, and/or syslog based on
 * configuration. The contents of buf is printed out has hex dump.
 */
//void wpa_hexdump(int level, const char *title, const void *buf, size_t len);

#ifdef ENABLE_HEXDUMP_BUF_DBG
/*static inline */ void wpa_hexdump_buf(int level, const char *title,
				   const struct wpabuf *buf)
{
#if 0
	wpa_hexdump(level, title, buf ? wpabuf_head(buf) : NULL,
		    buf ? wpabuf_len(buf) : 0);
#else
	wpa_hexdump(level, title, buf ? buf->buf : NULL,
		    buf ? (buf->used) : 0);
#endif /* 0 */
}
#endif /* ENABLE_HEXDUMP_BUF_DBG */

#ifdef ENABLE_HEXDUMP_KEY_DBG
#ifdef ENABLE_HEXDUMP_BUF_KEY_DBG
/**
 * wpa_hexdump_key - conditional hex dump, hide keys
 * @level: priority level (MSG_*) of the message
 * @title: title of for the message
 * @buf: data buffer to be dumped
 * @len: length of the buf
 *
 * This function is used to print conditional debugging and error messages. The
 * output may be directed to stdout, stderr, and/or syslog based on
 * configuration. The contents of buf is printed out has hex dump. This works
 * like wpa_hexdump(), but by default, does not include secret keys (passwords,
 * etc.) in debug output.
 */

//void wpa_hexdump_key(int level, const char *title, const void *buf, size_t len);

/*static inline */ void wpa_hexdump_buf_key(int level, const char *title,
				       const struct wpabuf *buf)
{
#if 0
	wpa_hexdump_key(level, title, buf ? wpabuf_head(buf) : NULL,
			buf ? wpabuf_len(buf) : 0);
#else
	wpa_hexdump_key(level, title, buf ? buf->buf : NULL,
			buf ? (buf->used) : 0);
#endif /* 0 */
}
#endif /* ENABLE_HEXDUMP_BUF_KEY_DBG */
#endif /* ENABLE_HEXDUMP_KEY_DBG */
#endif /* ENABLE_HEXDUMP_DBG */

#endif	/* CONFIG_WPA_PRINTF */

#ifdef CONFIG_WPA_MSG
void wpa_msg(void *ctx, int level, const char *fmt, ...)
{
#if	1	// def	CONFIG_DBG_LOG_MSG
	va_list ap;
	char *buf;
	int buflen;
#ifdef MSG_PREFIX
	char prefix[130];
#endif /* MSG_PREFIX */

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
#ifdef CONFIG_WPA_PRINTF
		wpa_printf(MSG_ERROR, "wpa_msg: Failed to allocate message "
			   "buffer");
#else
		_wpa_printf(MSG_ERROR, "wpa_msg: Failed to allocate message "
			   "buffer");
#endif /* CONFIG_WPA_PRINTF */
		return;
	}
	
	va_start(ap, fmt);

#ifdef MSG_PREFIX
	prefix[0] = '\0';
#endif /* MSG_PREFIX */
#if 0
	if (wpa_msg_ifname_cb) {
		const char *ifname = wpa_msg_ifname_cb(ctx);
		if (ifname) {
			int res = os_snprintf(prefix, sizeof(prefix), "%s: ",
					      ifname);
			if (os_snprintf_error(sizeof(prefix), res))
				prefix[0] = '\0';
		}
	}
#endif /* 0 */

    vsnprintf(buf, buflen, fmt, ap);

	va_end(ap);


#ifdef CONFIG_WPA_PRINTF
#ifdef MSG_PREFIX
	wpa_printf(level, "%s%s", prefix, buf);
#else
	wpa_printf(level, "%s", buf);
#endif /* MSG_PREFIX */

#else
#ifdef MSG_PREFIX
	_wpa_printf(level, "%s%s", prefix, buf);
#else
	_wpa_printf(level, "%s", buf);
#endif /* MSG_PREFIX */
#endif /* CONFIG_WPA_PRINTF */

#if 0
	if (wpa_msg_cb)
		wpa_msg_cb(ctx, level, WPA_MSG_PER_INTERFACE, buf, len);
#endif /* 1 */
#ifdef LOG_HEAP_MALLOC
	heap_bin_clear_free(buf, buflen);
#else
	bin_clear_free(buf, buflen);
#endif /* LOG_HEAP_MALLOC */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}
#endif /* CONFIG_WPA_MSG */


#ifdef CONFIG_WPA_MSG_CTRL
void wpa_msg_ctrl(void *ctx, int level, const char *fmt, ...)
{
#ifdef	CONFIG_DBG_LOG_MSG
	va_list ap;
	char *buf;
	int buflen;
	int len;

	if (!wpa_msg_cb)
		return;

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
		wpa_printf(MSG_ERROR, "wpa_msg_ctrl: Failed to allocate "
			   "message buffer");
		return;
	}
	va_start(ap, fmt);
	len = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);
#if 0
	wpa_msg_cb(ctx, level, WPA_MSG_PER_INTERFACE, buf, len);
#else
	wpa_printf(level, "%s", buf);
#endif /* 0 */
#ifdef LOG_HEAP_MALLOC
	heap_bin_clear_free(buf, buflen);
#else
	bin_clear_free(buf, buflen);
#endif /* LOG_HEAP_MALLOC */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}
#endif /* CONFIG_WPA_MSG_CTRL */


#ifdef CONFIG_WPA_MSG_GLOBAL
void wpa_msg_global(void *ctx, int level, const char *fmt, ...)
{
#ifdef	CONFIG_DBG_LOG_MSG
	va_list ap;
	char *buf;
	int buflen;
	int len;

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
		wpa_printf(MSG_ERROR, "wpa_msg_global: Failed to allocate "
			   "message buffer");
		return;
	}
	va_start(ap, fmt);
	len = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);

	wpa_printf(level, "%s", buf);

#if 0
	if (wpa_msg_cb)
		wpa_msg_cb(ctx, level, WPA_MSG_GLOBAL, buf, len);
#endif /* 0 */
#ifdef LOG_HEAP_MALLOC
	heap_bin_clear_free(buf, buflen);
#else
	bin_clear_free(buf, buflen);
#endif /* LOG_HEAP_MALLOC */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}
#endif /* CONFIG_WPA_MSG_GLOBAL */


#ifdef CONFIG_WPA_MSG_GLOBAL_CTRL
void wpa_msg_global_ctrl(void *ctx, int level, const char *fmt, ...)
{
#ifdef	CONFIG_DBG_LOG_MSG
	va_list ap;
	char *buf;
	int buflen;
	int len;

	if (!wpa_msg_cb)
		return;

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
		wpa_printf(MSG_ERROR,
			   "wpa_msg_global_ctrl: Failed to allocate message buffer");
		return;
	}
	va_start(ap, fmt);
	len = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);
#if 0
	wpa_msg_cb(ctx, level, WPA_MSG_GLOBAL, buf, len);
#else
	wpa_printf(level, "%s", buf);
#endif /* 0 */
#ifdef LOG_HEAP_MALLOC
	heap_bin_clear_free(buf, buflen);
#else
	bin_clear_free(buf, buflen);
#endif /* LOG_HEAP_MALLOC */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}
#endif /* CONFIG_WPA_MSG_GLOBAL_CTRL */


#ifdef CONFIG_WPA_MSG_NO_GLOBAL
void wpa_msg_no_global(void *ctx, int level, const char *fmt, ...)
{
#ifdef	CONFIG_DBG_LOG_MSG
	va_list ap;
	char *buf;
	int buflen;
	int len;

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
		wpa_printf(MSG_ERROR, "wpa_msg_no_global: Failed to allocate "
			   "message buffer");
		return;
	}
	va_start(ap, fmt);
	len = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);

	wpa_printf(level, "%s", buf);

#if 0
	if (wpa_msg_cb)
		wpa_msg_cb(ctx, level, WPA_MSG_NO_GLOBAL, buf, len);
#endif /* 0 */
#ifdef LOG_HEAP_MALLOC
	heap_bin_clear_free(buf, buflen);
#else
	bin_clear_free(buf, buflen);
#endif /* LOG_HEAP_MALLOC */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}
#endif /* CONFIG_WPA_MSG_NO_GLOBAL */


#ifdef CONFIG_WPA_MSG_GLOBAL_ONLY
void wpa_msg_global_only(void *ctx, int level, const char *fmt, ...)
{
#ifdef	CONFIG_DBG_LOG_MSG
	va_list ap;
	char *buf;
	int buflen;
	int len;

	va_start(ap, fmt);
	buflen = vsnprintf(NULL, 0, fmt, ap) + 1;
	va_end(ap);

#ifdef LOG_HEAP_MALLOC
	buf = os_malloc(buflen);
#else
	buf = os_malloc(buflen);
#endif /* LOG_HEAP_MALLOC */
	if (buf == NULL) {
		wpa_printf(MSG_ERROR, "%s: Failed to allocate message buffer",
			   __func__);
		return;
	}
	va_start(ap, fmt);
	len = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);

	wpa_printf(level, "%s", buf);

#if 0
	if (wpa_msg_cb)
		wpa_msg_cb(ctx, level, WPA_MSG_ONLY_GLOBAL, buf, len);
#endif /* 1 */
#ifdef LOG_HEAP_MALLOC
	os_free(buf);
#else
	os_free(buf);
#endif /* LOG_HEAP_MALLOC */

#else
	da16x_prt(fmt);
#endif	/* CONFIG_DBG_LOG_MSG */
}
#endif /* CONFIG_WPA_MSG_GLOBAL_ONLY */


#ifdef	CONFIG_SUPP27_EVENTS
const char * debug_level_str(int level)
{
	switch (level) {
	case MSG_EXCESSIVE:
		return "EXCESSIVE";
	case MSG_MSGDUMP:
		return "MSGDUMP";
	case MSG_DEBUG:
		return "DEBUG";
	case MSG_INFO:
		return "INFO";
	case MSG_WARNING:
		return "WARNING";
	case MSG_ERROR:
		return "ERROR";
	default:
		return "?";
	}
}


int str_to_debug_level(const char *s)
{
	if (os_strcasecmp(s, "EXCESSIVE") == 0)
		return MSG_EXCESSIVE;
	if (os_strcasecmp(s, "MSGDUMP") == 0)
		return MSG_MSGDUMP;
	if (os_strcasecmp(s, "DEBUG") == 0)
		return MSG_DEBUG;
	if (os_strcasecmp(s, "INFO") == 0)
		return MSG_INFO;
	if (os_strcasecmp(s, "WARNING") == 0)
		return MSG_WARNING;
	if (os_strcasecmp(s, "ERROR") == 0)
		return MSG_ERROR;
	return -1;
}
#endif	/* CONFIG_SUPP27_EVENTS */

#include "supp_driver.h"

const char * event_to_string(enum wpa_event_type event)
{
#define E2S(n) case EVENT_ ## n: return #n
	switch (event) {
		E2S(ASSOC);
		E2S(DISASSOC);
		E2S(MICHAEL_MIC_FAILURE);
		E2S(SCAN_RESULTS);
		E2S(ASSOCINFO);
		E2S(INTERFACE_STATUS);
		E2S(PMKID_CANDIDATE);
		E2S(TDLS);
		E2S(FT_RESPONSE);
		E2S(IBSS_RSN_START);
		E2S(AUTH);
		E2S(DEAUTH);
		E2S(ASSOC_REJECT);
		E2S(AUTH_TIMED_OUT);
		E2S(ASSOC_TIMED_OUT);
		E2S(WPS_BUTTON_PUSHED);
		E2S(TX_STATUS);
		E2S(RX_FROM_UNKNOWN);
		E2S(RX_MGMT);
		E2S(REMAIN_ON_CHANNEL);
		E2S(CANCEL_REMAIN_ON_CHANNEL);
		E2S(RX_PROBE_REQ);
		E2S(NEW_STA);
		E2S(EAPOL_RX);
		E2S(SIGNAL_CHANGE);
		E2S(INTERFACE_ENABLED);
		E2S(INTERFACE_DISABLED);
		E2S(CHANNEL_LIST_CHANGED);
		E2S(INTERFACE_UNAVAILABLE);
		E2S(BEST_CHANNEL);
		E2S(UNPROT_DEAUTH);
		E2S(UNPROT_DISASSOC);
		E2S(STATION_LOW_ACK);
		E2S(IBSS_PEER_LOST);
		E2S(DRIVER_GTK_REKEY);
		E2S(SCHED_SCAN_STOPPED);
		E2S(DRIVER_CLIENT_POLL_OK);
		E2S(EAPOL_TX_STATUS);
		E2S(CH_SWITCH);
		E2S(WNM);
		E2S(CONNECT_FAILED_REASON);
		E2S(DFS_RADAR_DETECTED);
		E2S(DFS_CAC_FINISHED);
		E2S(DFS_CAC_ABORTED);
		E2S(DFS_NOP_FINISHED);
		E2S(SURVEY);
		E2S(SCAN_STARTED);
		E2S(AVOID_FREQUENCIES);
		E2S(NEW_PEER_CANDIDATE);
		E2S(ACS_CHANNEL_SELECTED);
		E2S(DFS_CAC_STARTED);
		E2S(P2P_LO_STOP);
		E2S(BEACON_LOSS);
		E2S(DFS_PRE_CAC_EXPIRED);
		E2S(EXTERNAL_AUTH);
		E2S(PORT_AUTHORIZED);
		E2S(STATION_OPMODE_CHANGED);
		E2S(INTERFACE_MAC_CHANGED);
		E2S(WDS_STA_INTERFACE_STATUS);
		E2S(DHCP_NO_RESPONSE);
		E2S(DHCP_ACK_OK);
	}

	return "UNKNOWN";
#undef E2S
}

void reboot_func_cb(void *eloop_ctx, void *timeout_ctx)
{
	da16x_err_prt("[%s] Reboot\n", __func__);
	reboot_func(99 /* SYS_REBOOT_NONACTION */);
}

/* EOF */
