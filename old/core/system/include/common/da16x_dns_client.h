/**
 ****************************************************************************************
 *
 * @file da16x_dns_client.h
 *
 * @brief Define for DNS Client Module
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


#ifndef	__DA16X_DNS_CLIENT_H__
#define	__DA16X_DNS_CLIENT_H__

#include "sdk_type.h"

#include "sys_feature.h"
#include "da16x_system.h"
#include "iface_defs.h"
#include "common_def.h"

#include "da16x_network_common.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/dns.h"
#include "lwip/err.h"


/* Do not commit if log's define is changed. */
#undef	DNS_DEBUG_INFO
#undef	DNS_DEBUG_TEMP
#define	DNS_DEBUG_ERROR
#define	DNS_DEBUG_PRINT

#ifdef	DNS_DEBUG_INFO
#define DNS_DBG_INFO		PRINTF
#else
#define DNS_DBG_INFO(...)	do { } while (0);
#endif

#ifdef  DNS_DEBUG_ERROR
#define DNS_DBG_ERR		PRINTF
#else
#define DNS_DBG_ERR(...)	do { } while (0);
#endif

#ifdef  DNS_DEBUG_TEMP
#define DNS_DBG_TEMP		PRINTF
#else
#define DNS_DBG_TEMP(...)	do { } while (0);
#endif

#ifdef  DNS_DEBUG_PRINT
#define DNS_DBG_PRINT		PRINTF
#else
#define DNS_DBG_PRINT(...)	do { } while (0);
#endif


#define NX_DNS_ENABLE_EXTENDED_RR_TYPES

#define DNS_WAIT				500
#define DNS_LOCAL_CACHE_SIZE	128
#define DNS_BUFFER_SIZE			200
#define DNS_RECORD_COUNT		10

#define DNS_QUERY_AAAA			"aaaa"	/* ipv6 network address */
#define DNS_QUERY_A				"a"		/* network address */
#define DNS_QUERY_SRV			"srv"	/* Location of services */
#define DNS_QUERY_ALL			"all"	/* all query */
#define DNS_QUERY_MX			"mx"	/* mail exchanger */
#define DNS_QUERY_NS			"ns"	/* name server */
#define DNS_QUERY_SOA			"soa"	/* Start of authority */
#define DNS_QUERY_HINFO			"hinfo"	/* host info */
#define DNS_QUERY_TXT			"txt"	/* txt 값 */
#define DNS_QUERY_PTR			"ptr"	/* Reverse DNS query */
#define DNS_QUERY_CNAME			"cname"	/* Canonical name */

#if defined ( __DNS_2ND_CACHE_SUPPORT__ )
#define	MAX_URL_STRING_LEN		128
#define	MAX_IPADDR_LEN			16
#define	MAX_URL_TABLE_CNT		25

/// URL to IP_Address matching table
typedef struct {
	char	url_str[MAX_URL_STRING_LEN];
	char	ipaddr_str[MAX_IPADDR_LEN];
} url_to_ip_addr_table_t;

typedef struct {
	char	saved_cnt;

	url_to_ip_addr_table_t table[MAX_URL_TABLE_CNT];
} url_to_ip_addr_t;
#endif	// __DNS_2ND_CACHE_SUPPORT__


int get_dns_addr(int iface, char *result_str);
int set_dns_addr(int iface, char *ip_addr);
int get_dns_addr_2nd(int iface, char *result_str);
int set_dns_addr_2nd(int iface, char *ip_addr);

void dns_req_resolved(const char* hostname, const ip_addr_t *ipaddr, void *arg);
int da16x_dns_2nd_cache_add(char* url, char* ip_addr, unsigned long ip_addr_long);
UINT da16x_dns_2nd_cache_is_enabled(void);
UINT da16x_dns_2nd_cache_find_answer(UCHAR* host_name, unsigned long* ip_addr);

char *dns_A_Query(char *domain_name, unsigned long wait_option);

unsigned int dns_ALL_Query(unsigned char *domain_name,			\
			unsigned char *record_buffer,			\
			unsigned int record_buffer_size,		\
			unsigned int *record_count,			\
			unsigned long wait_option);


#endif	/* __DA16X_DNS_CLIENT_H__ */

/* EOF */
