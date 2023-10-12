/*
 * WPA Supplicant / Configuration file structures
 * Copyright (c) 2003-2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef SUPP_CONFIG_H
#define SUPP_CONFIG_H

#include "sdk_type.h"
#include "common_def.h"


#define DEFAULT_EAPOL_VERSION				1
#ifdef CONFIG_NO_SCAN_PROCESSING
  #define DEFAULT_AP_SCAN					2
#else /* CONFIG_NO_SCAN_PROCESSING */
  #define DEFAULT_AP_SCAN					1
#endif /* CONFIG_NO_SCAN_PROCESSING */

#define DEFAULT_USER_MPM					1
#define DEFAULT_MAX_PEER_LINKS				4	//99
#define DEFAULT_POINT_MAX_PEER_LINKS		4	//99
#define DEFAULT_POTAL_MAX_PEER_LINKS		1	//99

#define DEFAULT_MESH_MAX_INACTIVITY			300
#define DEFAULT_FAST_REAUTH					1

#define DEFAULT_P2P_GO_INTENT				3
#define DEFAULT_P2P_INTRA_BSS				1
#define DEFAULT_P2P_LISTEN_REG_CLASS		81
#define DEFAULT_P2P_OPER_REG_CLASS			81
#define DEFAULT_P2P_LISTEN_CHANNEL			0
#define DEFAULT_P2P_OPER_CHANNEL			0
#define DEFAULT_P2P_FIND_TIMEOUT			20		/* s */
#define DEFAULT_P2P_LISTEN_TIME				1000	/* ms */
#define DEFAULT_P2P_NEGO_WAIT_TIME			1000 	/* ms */ /* 30000 */

/*
 * The default dot11RSNASAERetransPeriod is defined as 40 ms in the standard,
 * but use 1000 ms in practice to avoid issues on low power CPUs.
 */
#define DEFAULT_DOT11_RSNA_SAE_RETRANS_PERIOD	1000

#define DEFAULT_P2P_GO_MAX_INACTIVITY		(5 * 60)
#define DEFAULT_P2P_OPTIMIZE_LISTEN_CHAN	0
#define DEFAULT_AP_ISOLATE					0
#define DEFAULT_ACCESS_NETWORK_TYPE			15
#define DEFAULT_SCAN_CUR_FREQ				0
#define DEFAULT_P2P_SEARCH_DELAY			500
#define DEFAULT_RAND_ADDR_LIFETIME			60
#define DEFAULT_KEY_MGMT_OFFLOAD			1
#define DEFAULT_CERT_IN_CB					1
#define DEFAULT_P2P_GO_CTWINDOW				0
#define DEFAULT_WPA_RSC_RELAXATION			1

#define DEFAULT_MBO_CELL_CAPA				MBO_CELL_CAPA_NOT_SUPPORTED
#define DEFAULT_DISASSOC_IMMINENT_RSSI_THRESHOLD	-75
#define DEFAULT_OCE_SUPPORT					OCE_STA

#define DEFAULT_AP_MAX_INACTIVITY			(5 * 60)
#define DEFAULT_WPS_DEVICE_TYPE				"0-00000000-0"
#define DEFAULT_BSS_EXPIRATION_AGE			180
#define DEFAULT_BSS_EXPIRATION_SCAN_COUNT	2

#if defined ( __SUPPORT_CON_EASY__ )
  #define DEFAULT_MAX_NUM_STA				1	/* In small system, AP mode has 1 STA */
#elif defined ( __SMALL_SYSTEM__ )
  #define DEFAULT_MAX_NUM_STA				1	/* In small system, AP mode has 1 STA */
#elif defined (__WPA3_WIFI_CERTFIED__)
  #define DEFAULT_MAX_NUM_STA				7	/* for Anti-clogging test */
#else
  #define DEFAULT_MAX_NUM_STA				6	/* 128 */
#endif /* ... */

#ifdef	CONFIG_CONCURRENT
  #define DEFAULT_CONCURRENT_MAX_NUM_STA	1
#endif	/* CONFIG_CONCURRENT */

#define DEFAULT_ACCESS_NETWORK_TYPE			15
#define DEFAULT_SCAN_CUR_FREQ				0

#ifdef CONFIG_ACL
#define DEFAULT_ACL_CLEAR					9
#endif /* CONFIG_ACL */

#define DEFAULT_AP_SSID						"DA16200"

#define DEFAULT_AP_COUNTRY					"KR"	/* ASCII code : 55 53 */
#define DEFAULT_AP_CHANNEL					i3ed11_freq_to_ch(FREQUENCE_DEFAULT)

#define MAX_SAE_GROUPS						6

#define DEFAULT_PMK_LIFE_TIME               31536000	/* 1 year in sec */

#include "config_ssid.h"
#include "wps/wps.h"
#include "common/ieee802_11_defs.h"
#include "common/ieee802_11_common.h"

#include "supp_def.h"
#include "ap/ap_config.h"

/* wpa_config_global */

#ifdef CONFIG_WPS
#define ENV_DEVICE_NAME				"dev_name"
#define ENV_MANUFACTURER			"manuf"
#define ENV_MODEL_NAME				"m_name"
#define ENV_MODEL_NUMBER			"m_num"
#define ENV_SERIAL_NUMBER			"srl_num"
#define ENV_DEVICE_TYPE				"dev_type"
#define ENV_CONFIG_METHODS			"cfg_methods"
#endif /* CONFIG_WPS */

#define ENV_DTIM_PERIOD				"dtim_period"
#define ENV_BEACON_INT				"beacon_int"
#define ENV_ISOLATE					"isolate"
#define ENV_RTS_THRESHOLD			"rts_threshold"

#define ENV_COUNTRY					"country_code"

#define ENV_P2P_SSID_POSTFIX		"p2p_ssid_postfix"
#define ENV_P2P_LISTEN_CHANNEL		"p2p_listen_channel"
#define ENV_P2P_OPER_CHANNEL		"p2p_oper_channel"
#define ENV_P2P_FIND_TIMEOUT		"p2p_find_timeout"
#define ENV_P2P_GO_INTENT			"p2p_go_intent"

#ifdef CONFIG_AP_WMM
#define ENV_WMM_ENABLED				"wmm_enabled"
#define ENV_WMM_PS_ENABLED			"wmm_ps_enabled"
#define ENV_WMM_PRAMS_AP			"wmm_AP_%d_%s"
#define ENV_WMM_PRAMS_STA			"wmm_STA_%d_%s"
#define ENV_WMM_PS_CWMIN			"cwmin"
#define ENV_WMM_PS_CWMAX			"cwmax"
#define ENV_WMM_PS_AIFS				"aifs"
#define ENV_WMM_PS_BURST			"burst"
#define ENV_WMM_PS_TXOP				"txop"
#endif /* CONFIG_AP_WMM */

#define ENV_AP_MAX_INACTIVITY		"ap_max_inactivity"
#define ENV_AP_SEND_KA				"ap_send_ka"

#define ENV_BSSID					"bssid"
#define ENV_BSSID_HINT				"bssid_hint"
#define ENV_PSK						"psk"
#define ENV_PROTO					"proto"
#define ENV_KEY_MGMT				"key_mgmt"
#define ENV_PAIRWISE				"pairwise"
#define ENV_GROUP					"group"
#define ENV_GROUP_MGMT				"group_mgmt"
#define ENV_AUTH_ALG				"auth_alg"
#define ENV_EAP						"eap"
#define ENV_WEP_KEY_X				"wep_key%d"
#define	ENV_PROACTIVE_KEY_CACHING	"proActKeyCaching"

#ifdef CONFIG_IEEE80211W
#define ENV_IEEE80211W				"ieee80211w"
#endif /* CONFIG_IEEE80211W */

#ifdef CONFIG_AUTOSCAN
#define ENV_AUTOSCAN				"autocan"
#endif

#define ENV_SYS_MODE				"SYSMODE"

#ifdef CONFIG_ACL
#define ENV_ACL_MAC					"acl_mac"
#define ENV_ACL_CMD					"acl_cmd"
#endif /* CONFIG_ACL */

#define ENV_TLS_VER					"tls_ver"
#define ENV_PEAP_VER				"peap_ver"
#define ENV_CLIENT_CERT				"client_cert"
#define ENV_CLIENT_KEY				"client_key"

#ifdef EAP_FAST
#define ENV_FAST_PAC				"fast_pac"
#define ENV_FAST_PAC_LEN			"fast_pac_len"
#endif /* EAP_FAST */

#ifdef CONFIG_ALLOW_ANY_OPEN_AP
#define ENV_ALLOW_ANY_OPEN_AP		"allow_any_open"
#endif

#ifdef CONFIG_LOG_MASK
#define ENV_SUPP_LOG_MASK			"supp_log_mask"
#define ENV_SUPP_WPA_LOG_MASK       "supp_wpa_log_mask"
#endif /* CONFIG_LOG_MASK */

#ifdef	CONFIG_SIMPLE_ROAMING
#define ENV_ROAM					"STA_roam"
#define ENV_ROAM_THRESHOLD		 	"STA_roam_thold"
#define DEFAULT_ROAM_DISABLE		0
#define ROAM_SCAN_INT				1
#define ROAM_SUSPEND_CNT		 	2
#define ROAM_RSSI_THRESHOLD 		-65

enum simple_roam_state {
	ROAM_DISABLE = 0,
	ROAM_ENABLE = 1,
	ROAM_SCANNING = 2
} ;
#endif /* CONFIG_SIMPLE_ROAMING */

#define ENV_AP_SET_GREENFIELD		"greenfield" //"AP_set_greenfield"
#define ENV_AP_SET_HT_PROTECTION	"ht_protection"

#define MAX_WPA_PROFILE				4 // 3
#define PREFIX_NETWORK_PROFILE		"N%d_%s"


#define FIXED_NETWORK_ID_STA		0
#define FIXED_NETWORK_ID_AP			1
#define FIXED_NETWORK_ID_P2P		2
#define FIXED_NETWORK_ID_MESH_POINT	3
#define FIXED_NETWORK_ID_NONE		9

//struct country_ch_power_level {
//	char*	country;
//	UCHAR	ch[14];
//};

#define ENV_PMK_LIFE_TIME           "pmk_lifetime"

extern ULONG cc_power_table(UINT index);
extern UINT cc_power_table_size(void);

#define NOTAPPLICABLE_CHANNEL		-1
#define NOTINCLUDED_CHANNEL			-2

#define NUM_SUPPORTS_COUNTRY		cc_power_table_size() //49

#define MAX_ROAMING_CONS			36
#define MAX_ROAMING_CONS_OI_LEN		15

#pragma pack(1)
typedef struct network_field {
	const char *name;
	u8 set;
} network_field_t;
#pragma pack()

extern network_field_t network_fields[];

#ifdef CONFIG_WPA_CRED
struct wpa_cred {
	/**
	 * next - Next credential in the list
	 *
	 * This pointer can be used to iterate over all credentials. The head
	 * of this list is stored in the cred field of struct wpa_config.
	 */
	struct wpa_cred *next;

	/**
	 * id - Unique id for the credential
	 *
	 * This identifier is used as a unique identifier for each credential
	 * block when using the control interface. Each credential is allocated
	 * an id when it is being created, either when reading the
	 * configuration file or when a new credential is added through the
	 * control interface.
	 */
	int id;

	/**
	 * temporary - Whether this credential is temporary and not to be saved
	 */
	int temporary;

	/**
	 * priority - Priority group
	 *
	 * By default, all networks and credentials get the same priority group
	 * (0). This field can be used to give higher priority for credentials
	 * (and similarly in struct wpa_ssid for network blocks) to change the
	 * Interworking automatic networking selection behavior. The matching
	 * network (based on either an enabled network block or a credential)
	 * with the highest priority value will be selected.
	 */
	int priority;

#ifdef	CONFIG_PCSC_FUNCS
	/**
	 * pcsc - Use PC/SC and SIM/USIM card
	 */
	int pcsc;
#endif	/* CONFIG_PCSC_FUNCS */

#ifdef	CONFIG_INTERWORKING
	/**
	 * realm - Home Realm for Interworking
	 */
	char *realm;

	/**
	 * username - Username for Interworking network selection
	 */
	char *username;

	/**
	 * password - Password for Interworking network selection
	 */
	char *password;

	/**
	 * ext_password - Whether password is a name for external storage
	 */
	int ext_password;
#endif	/* CONFIG_INTERWORKING */

	/**
	 * ca_cert - CA certificate for Interworking network selection
	 */
	char *ca_cert;

	/**
	 * client_cert - File path to client certificate file (PEM/DER)
	 *
	 * This field is used with Interworking networking selection for a case
	 * where client certificate/private key is used for authentication
	 * (EAP-TLS). Full path to the file should be used since working
	 * directory may change when wpa_supplicant is run in the background.
	 *
	 * Alternatively, a named configuration blob can be used by setting
	 * this to blob://blob_name.
	 */
	char *client_cert;

	/**
	 * private_key - File path to client private key file (PEM/DER/PFX)
	 *
	 * When PKCS#12/PFX file (.p12/.pfx) is used, client_cert should be
	 * commented out. Both the private key and certificate will be read
	 * from the PKCS#12 file in this case. Full path to the file should be
	 * used since working directory may change when wpa_supplicant is run
	 * in the background.
	 *
	 * Windows certificate store can be used by leaving client_cert out and
	 * configuring private_key in one of the following formats:
	 *
	 * cert://substring_to_match
	 *
	 * hash://certificate_thumbprint_in_hex
	 *
	 * For example: private_key="hash://63093aa9c47f56ae88334c7b65a4"
	 *
	 * Note that when running wpa_supplicant as an application, the user
	 * certificate store (My user account) is used, whereas computer store
	 * (Computer account) is used when running wpasvc as a service.
	 *
	 * Alternatively, a named configuration blob can be used by setting
	 * this to blob://blob_name.
	 */
	char *private_key;

	/**
	 * private_key_passwd - Password for private key file
	 */
	char *private_key_passwd;

#ifdef CONFIG_PCSC_FUNCS
	/**
	 * imsi - IMSI in <MCC> | <MNC> | '-' | <MSIN> format
	 */
	char *imsi;

	/**
	 * milenage - Milenage parameters for SIM/USIM simulator in
	 *	<Ki>:<OPc>:<SQN> format
	 */
	char *milenage;
#endif /* CONFIG_PCSC_FUNCS */

	/**
	 * domain_suffix_match - Constraint for server domain name
	 *
	 * If set, this FQDN is used as a suffix match requirement for the AAA
	 * server certificate in SubjectAltName dNSName element(s). If a
	 * matching dNSName is found, this constraint is met. If no dNSName
	 * values are present, this constraint is matched against SubjectName CN
	 * using same suffix match comparison. Suffix match here means that the
	 * host/domain name is compared one label at a time starting from the
	 * top-level domain and all the labels in @domain_suffix_match shall be
	 * included in the certificate. The certificate may include additional
	 * sub-level labels in addition to the required labels.
	 *
	 * For example, domain_suffix_match=example.com would match
	 * test.example.com but would not match test-example.com.
	 */
	char *domain_suffix_match;

	/**
	 * domain - Home service provider FQDN(s)
	 *
	 * This is used to compare against the Domain Name List to figure out
	 * whether the AP is operated by the Home SP. Multiple domain entries
	 * can be used to configure alternative FQDNs that will be considered
	 * home networks.
	 */
	char **domain;

	/**
	 * num_domain - Number of FQDNs in the domain array
	 */
	size_t num_domain;

#ifdef	CONFIG_ROAM_CONSORTIUM
	/**
	 * roaming_consortium - Roaming Consortium OI
	 *
	 * If roaming_consortium_len is non-zero, this field contains the
	 * Roaming Consortium OI that can be used to determine which access
	 * points support authentication with this credential. This is an
	 * alternative to the use of the realm parameter. When using Roaming
	 * Consortium to match the network, the EAP parameters need to be
	 * pre-configured with the credential since the NAI Realm information
	 * may not be available or fetched.
	 */
	u8 roaming_consortium[15];

	/**
	 * roaming_consortium_len - Length of roaming_consortium
	 */
	size_t roaming_consortium_len;

	/**
	 * required_roaming_consortium - Required Roaming Consortium OI
	 *
	 * If required_roaming_consortium_len is non-zero, this field contains
	 * the Roaming Consortium OI that is required to be advertised by the AP
	 * for the credential to be considered matching.
	 */
	u8 required_roaming_consortium[15];

	/**
	 * required_roaming_consortium_len - Length of required_roaming_consortium
	 */
	size_t required_roaming_consortium_len;
#endif	/* CONFIG_ROAM_CONSORTIUM */

	/**
	 * roaming_consortiums - Roaming Consortium OI(s) memberships
	 *
	 * This field contains one or more OIs identifying the roaming
	 * consortiums of which the provider is a member. The list is sorted
	 * from the most preferred one to the least preferred one. A match
	 * between the Roaming Consortium OIs advertised by an AP and the OIs
	 * in this list indicates that successful authentication is possible.
	 * (Hotspot 2.0 PerProviderSubscription/<X+>/HomeSP/RoamingConsortiumOI)
	 */
	u8 roaming_consortiums[MAX_ROAMING_CONS][MAX_ROAMING_CONS_OI_LEN];

	/**
	 * roaming_consortiums_len - Length on roaming_consortiums[i]
	 */
	size_t roaming_consortiums_len[MAX_ROAMING_CONS];

	/**
	 * num_roaming_consortiums - Number of entries in roaming_consortiums
	 */
	unsigned int num_roaming_consortiums;

	/**
	 * eap_method - EAP method to use
	 *
	 * Pre-configured EAP method to use with this credential or %NULL to
	 * indicate no EAP method is selected, i.e., the method will be
	 * selected automatically based on ANQP information.
	 */
	struct eap_method_type *eap_method;

	/**
	 * phase1 - Phase 1 (outer authentication) parameters
	 *
	 * Pre-configured EAP parameters or %NULL.
	 */
	char *phase1;

	/**
	 * phase2 - Phase 2 (inner authentication) parameters
	 *
	 * Pre-configured EAP parameters or %NULL.
	 */
	char *phase2;

#ifdef	CONFIG_INTERWORKING
	struct excluded_ssid {
		u8 ssid[SSID_MAX_LEN];
		size_t ssid_len;
	} *excluded_ssid;
	size_t num_excluded_ssid;
#endif	/* CONFIG_INTERWORKING */

#ifdef	CONFIG_ROAMING_PARTNER
	struct roaming_partner {
		char fqdn[128];
		int exact_match;
		u8 priority;
		char country[3];
	} *roaming_partner;
	size_t num_roaming_partner;
#endif	/* CONFIG_ROAMING_PARTNER */

	int update_identifier;

	/**
	 * provisioning_sp - FQDN of the SP that provisioned the credential
	 */
	char *provisioning_sp;

	/**
	 * sp_priority - Credential priority within a provisioning SP
	 *
	 * This is the priority of the credential among all credentials
	 * provisionined by the same SP (i.e., for entries that have identical
	 * provisioning_sp value). The range of this priority is 0-255 with 0
	 * being the highest and 255 the lower priority.
	 */
	int sp_priority;

	unsigned int min_dl_bandwidth_home;
	unsigned int min_ul_bandwidth_home;
	unsigned int min_dl_bandwidth_roaming;
	unsigned int min_ul_bandwidth_roaming;

	/**
	 * max_bss_load - Maximum BSS Load Channel Utilization (1..255)
	 * This value is used as the maximum channel utilization for network
	 * selection purposes for home networks. If the AP does not advertise
	 * BSS Load or if the limit would prevent any connection, this
	 * constraint will be ignored.
	 */
	unsigned int max_bss_load;

	unsigned int num_req_conn_capab;
	u8 *req_conn_capab_proto;
	int **req_conn_capab_port;

	/**
	 * ocsp - Whether to use/require OCSP to check server certificate
	 *
	 * 0 = do not use OCSP stapling (TLS certificate status extension)
	 * 1 = try to use OCSP stapling, but not require response
	 * 2 = require valid OCSP stapling response
	 */
	int ocsp;

	/**
	 * sim_num - User selected SIM identifier
	 *
	 * This variable is used for identifying which SIM is used if the system
	 * has more than one.
	 */
	int sim_num;
};
#endif /* CONFIG_WPA_CRED */


#define CFG_CHANGED_DEVICE_NAME BIT(0)
#define CFG_CHANGED_CONFIG_METHODS BIT(1)
#define CFG_CHANGED_DEVICE_TYPE BIT(2)
#define CFG_CHANGED_OS_VERSION BIT(3)
#define CFG_CHANGED_UUID BIT(4)
#define CFG_CHANGED_COUNTRY BIT(5)
#define CFG_CHANGED_SEC_DEVICE_TYPE BIT(6)
#define CFG_CHANGED_P2P_SSID_POSTFIX BIT(7)
#define CFG_CHANGED_WPS_STRING BIT(8)
#ifdef	CONFIG_P2P_UNUSED_CMD
#define CFG_CHANGED_P2P_INTRA_BSS BIT(9)
#endif	/* CONFIG_P2P_UNUSED_CMD */
#define CFG_CHANGED_VENDOR_EXTENSION BIT(10)
#ifdef	CONFIG_P2P_UNUSED_CMD
#define CFG_CHANGED_P2P_LISTEN_CHANNEL BIT(11)
#define CFG_CHANGED_P2P_OPER_CHANNEL BIT(12)
#define CFG_CHANGED_P2P_PREF_CHAN BIT(13)
#define CFG_CHANGED_EXT_PW_BACKEND BIT(14)
#define CFG_CHANGED_NFC_PASSWORD_TOKEN BIT(15)
#define CFG_CHANGED_P2P_PASSPHRASE_LEN BIT(16)
#endif	/* CONFIG_P2P_UNUSED_CMD */
#define CFG_CHANGED_SCHED_SCAN_PLANS BIT(17)
#define CFG_CHANGED_WOWLAN_TRIGGERS BIT(18)

/**
 * struct wpa_config - wpa_supplicant configuration data
 *
 * This data structure is presents the per-interface (radio) configuration
 * data. In many cases, there is only one struct wpa_config instance, but if
 * more than one network interface is being controlled, one instance is used
 * for each.
 */
struct wpa_config {
	/**
	 * ssid - Head of the global network list
	 *
	 * This is the head for the list of all the configured networks.
	 */
	struct wpa_ssid *ssid;

#ifdef CONFIG_PRIO_GROUP
	/**
	 * pssid - Per-priority network lists (in priority order)
	 */
	struct wpa_ssid **pssid;

	/**
	 * num_prio - Number of different priorities used in the pssid lists
	 *
	 * This indicates how many per-priority network lists are included in
	 * pssid.
	 */
	int num_prio;
#endif /* CONFIG_PRIO_GROUP */

#ifdef CONFIG_WPA_CRED
	/**
	 * cred - Head of the credential list
	 *
	 * This is the head for the list of all the configured credentials.
	 */
	struct wpa_cred *cred;
#endif /* CONFIG_WPA_CRED */

	/**
	 * eapol_version - IEEE 802.1X/EAPOL version number
	 *
	 * wpa_supplicant is implemented based on IEEE Std 802.1X-2004 which
	 * defines EAPOL version 2. However, there are many APs that do not
	 * handle the new version number correctly (they seem to drop the
	 * frames completely). In order to make wpa_supplicant interoperate
	 * with these APs, the version number is set to 1 by default. This
	 * configuration value can be used to set it to the new version (2).
	 */
	int eapol_version;

	/**
	 * ap_scan - AP scanning/selection
	 *
	 * By default, wpa_supplicant requests driver to perform AP
	 * scanning and then uses the scan results to select a
	 * suitable AP. Another alternative is to allow the driver to
	 * take care of AP scanning and selection and use
	 * wpa_supplicant just to process EAPOL frames based on IEEE
	 * 802.11 association information from the driver.
	 *
	 * 1: wpa_supplicant initiates scanning and AP selection (default).
	 *
	 * 0: Driver takes care of scanning, AP selection, and IEEE 802.11
	 * association parameters (e.g., WPA IE generation); this mode can
	 * also be used with non-WPA drivers when using IEEE 802.1X mode;
	 * do not try to associate with APs (i.e., external program needs
	 * to control association). This mode must also be used when using
	 * wired Ethernet drivers.
	 *
	 * 2: like 0, but associate with APs using security policy and SSID
	 * (but not BSSID); this can be used, e.g., with ndiswrapper and NDIS
	 * drivers to enable operation with hidden SSIDs and optimized roaming;
	 * in this mode, the network blocks in the configuration are tried
	 * one by one until the driver reports successful association; each
	 * network block should have explicit security policy (i.e., only one
	 * option in the lists) for key_mgmt, pairwise, group, proto variables.
	 *
	 * Note: ap_scan=2 should not be used with the nld11 driver interface
	 * (the current Linux interface). ap_scan=1 is optimized work working
	 * with nld11. For finding networks using hidden SSID, scan_ssid=1 in
	 * the network block can be used with nld11.
	 */
#ifndef FEATURE_USE_DEFAULT_AP_SCAN
	int ap_scan;
#endif /* FEATURE_USE_DEFAULT_AP_SCAN */

#ifdef	CONFIG_BGSCAN
	/**
	 * bgscan - Background scan and roaming parameters or %NULL if none
	 *
	 * This is an optional set of parameters for background scanning and
	 * roaming within a network (ESS). For more detailed information see
	 * ssid block documentation.
	 *
	 * The variable defines default bgscan behavior for all BSS station
	 * networks except for those which have their own bgscan configuration.
	 */
	 char *bgscan;
#endif	/* CONFIG_BGSCAN */

#ifdef CONFIG_SCAN_OFFLOAD
	/**
	 * disable_scan_offload - Disable automatic offloading of scan requests
	 *
	 * By default, %wpa_supplicant tries to offload scanning if the driver
	 * indicates support for this (sched_scan). This configuration
	 * parameter can be used to disable this offloading mechanism.
	 */
	int disable_scan_offload;
#endif /* CONFIG_SCAN_OFFLOAD */

	/**
	 * ctrl_interface - Parameters for the control interface
	 *
	 * If this is specified, %wpa_supplicant will open a control interface
	 * that is available for external programs to manage %wpa_supplicant.
	 * The meaning of this string depends on which control interface
	 * mechanism is used. For all cases, the existence of this parameter
	 * in configuration is used to determine whether the control interface
	 * is enabled.
	 *
	 * For UNIX domain sockets (default on Linux and BSD): This is a
	 * directory that will be created for UNIX domain sockets for listening
	 * to requests from external programs (CLI/GUI, etc.) for status
	 * information and configuration. The socket file will be named based
	 * on the interface name, so multiple %wpa_supplicant processes can be
	 * run at the same time if more than one interface is used.
	 * /var/run/wpa_supplicant is the recommended directory for sockets and
	 * by default, wpa_cli will use it when trying to connect with
	 * %wpa_supplicant.
	 *
	 * Access control for the control interface can be configured
	 * by setting the directory to allow only members of a group
	 * to use sockets. This way, it is possible to run
	 * %wpa_supplicant as root (since it needs to change network
	 * configuration and open raw sockets) and still allow GUI/CLI
	 * components to be run as non-root users. However, since the
	 * control interface can be used to change the network
	 * configuration, this access needs to be protected in many
	 * cases. By default, %wpa_supplicant is configured to use gid
	 * 0 (root). If you want to allow non-root users to use the
	 * control interface, add a new group and change this value to
	 * match with that group. Add users that should have control
	 * interface access to this group.
	 *
	 * When configuring both the directory and group, use following format:
	 * DIR=/var/run/wpa_supplicant GROUP=wheel
	 * DIR=/var/run/wpa_supplicant GROUP=0
	 * (group can be either group name or gid)
	 *
	 * For UDP connections (default on Windows): The value will be ignored.
	 * This variable is just used to select that the control interface is
	 * to be created. The value can be set to, e.g., udp
	 * (ctrl_interface=udp).
	 *
	 * For Windows Named Pipe: This value can be used to set the security
	 * descriptor for controlling access to the control interface. Security
	 * descriptor can be set using Security Descriptor String Format (see
	 * http://msdn.microsoft.com/library/default.asp?url=/library/en-us/secauthz/security/security_descriptor_string_format.asp).
	 * The descriptor string needs to be prefixed with SDDL=. For example,
	 * ctrl_interface=SDDL=D: would set an empty DACL (which will reject
	 * all connections).
	 */
	char *ctrl_interface;

#ifdef CONFIG_SUPP27_IFACE
	/**
	 * ctrl_interface_group - Control interface group (DEPRECATED)
	 *
	 * This variable is only used for backwards compatibility. Group for
	 * UNIX domain sockets should now be specified using GROUP=group in
	 * ctrl_interface variable.
	 */
	char *ctrl_interface_group;
#endif /* CONFIG_SUPP27_IFACE */

#ifdef	CONFIG_FAST_REAUTH
	/**
	 * fast_reauth - EAP fast re-authentication (session resumption)
	 *
	 * By default, fast re-authentication is enabled for all EAP methods
	 * that support it. This variable can be used to disable fast
	 * re-authentication (by setting fast_reauth=0). Normally, there is no
	 * need to disable fast re-authentication.
	 */
	int fast_reauth;
#endif	/* CONFIG_FAST_REAUTH */

#ifdef CONFIG_OPENSSL_MOD
	/**
	 * opensc_engine_path - Path to the OpenSSL engine for opensc
	 *
	 * This is an OpenSSL specific configuration option for loading OpenSC
	 * engine (engine_opensc.so); if %NULL, this engine is not loaded.
	 */
	char *opensc_engine_path;

	/**
	 * pkcs11_engine_path - Path to the OpenSSL engine for PKCS#11
	 *
	 * This is an OpenSSL specific configuration option for loading PKCS#11
	 * engine (engine_pkcs11.so); if %NULL, this engine is not loaded.
	 */
	char *pkcs11_engine_path;

	/**
	 * pkcs11_module_path - Path to the OpenSSL OpenSC/PKCS#11 module
	 *
	 * This is an OpenSSL specific configuration option for configuring
	 * path to OpenSC/PKCS#11 engine (opensc-pkcs11.so); if %NULL, this
	 * module is not loaded.
	 */
	char *pkcs11_module_path;

	/**
	 * openssl_ciphers - OpenSSL cipher string
	 *
	 * This is an OpenSSL specific configuration option for configuring the
	 * default ciphers. If not set, "DEFAULT:!EXP:!LOW" is used as the
	 * default.
	 */
	char *openssl_ciphers;
#endif /* CONFIG_OPENSSL_MOD */

#ifdef	CONFIG_PCSC_FUNCS
	/**
	 * pcsc_reader - PC/SC reader name prefix
	 *
	 * If not %NULL, PC/SC reader with a name that matches this prefix is
	 * initialized for SIM/USIM access. Empty string can be used to match
	 * the first available reader.
	 */
	char *pcsc_reader;

	/**
	 * pcsc_pin - PIN for USIM, GSM SIM, and smartcards
	 *
	 * This field is used to configure PIN for SIM/USIM for EAP-SIM and
	 * EAP-AKA. If left out, this will be asked through control interface.
	 */
	char *pcsc_pin;
#endif	/* CONFIG_PCSC_FUNCS */

#ifdef	CONFIG_EXT_SIM
	/**
	 * external_sim - Use external processing for SIM/USIM operations
	 */
	int external_sim;
#endif /* CONFIG_EXT_SIM */

	/**
	 * driver_param - Driver interface parameters
	 *
	 * This text string is passed to the selected driver interface with the
	 * optional struct wpa_driver_ops::set_param() handler. This can be
	 * used to configure driver specific options without having to add new
	 * driver interface functionality.
	 */
	char *driver_param;

	/**
	 * dot11RSNAConfigPMKLifetime - Maximum lifetime of a PMK
	 *
	 * dot11 MIB variable for the maximum lifetime of a PMK in the PMK
	 * cache (unit: seconds).
	 */
	unsigned int dot11RSNAConfigPMKLifetime;

	/**
	 * dot11RSNAConfigPMKReauthThreshold - PMK re-authentication threshold
	 *
	 * dot11 MIB variable for the percentage of the PMK lifetime
	 * that should expire before an IEEE 802.1X reauthentication occurs.
	 */
	unsigned int dot11RSNAConfigPMKReauthThreshold;

	/**
	 * dot11RSNAConfigSATimeout - Security association timeout
	 *
	 * dot11 MIB variable for the maximum time a security association
	 * shall take to set up (unit: seconds).
	 */
	unsigned int dot11RSNAConfigSATimeout;

	/**
	 * update_config - Is wpa_supplicant allowed to update configuration
	 *
	 * This variable control whether wpa_supplicant is allow to re-write
	 * its configuration with wpa_config_write(). If this is zero,
	 * configuration data is only changed in memory and the external data
	 * is not overriden. If this is non-zero, wpa_supplicant will update
	 * the configuration data (e.g., a file) whenever configuration is
	 * changed. This update may replace the old configuration which can
	 * remove comments from it in case of a text file configuration.
	 */
	int update_config;

#ifndef CONFIG_NO_CONFIG_BLOBS
	/**
	 * blobs - Configuration blobs
	 */
	struct wpa_config_blob *blobs;
#endif /* CONFIG_NO_CONFIG_BLOBS */

	/**
	 * uuid - Universally Unique IDentifier (UUID; see RFC 4122) for WPS
	 */
	u8 uuid[16];

	/**
	 * auto_uuid - Automatic UUID behavior
	 * 0 = generate static value based on the local MAC address (default)
	 * 1 = generate a random UUID every time wpa_supplicant starts
	 */
	int auto_uuid;

	/**
	 * device_name - Device Name (WPS)
	 * User-friendly description of device; up to 32 octets encoded in
	 * UTF-8
	 */
	char *device_name;

	/**
	 * manufacturer - Manufacturer (WPS)
	 * The manufacturer of the device (up to 64 ASCII characters)
	 */
	char *manufacturer;

	/**
	 * model_name - Model Name (WPS)
	 * Model of the device (up to 32 ASCII characters)
	 */
	char *model_name;

	/**
	 * model_number - Model Number (WPS)
	 * Additional device description (up to 32 ASCII characters)
	 */
	char *model_number;

	/**
	 * serial_number - Serial Number (WPS)
	 * Serial number of the device (up to 32 characters)
	 */
	char *serial_number;

	/**
	 * device_type - Primary Device Type (WPS)
	 */
	u8 device_type[WPS_DEV_TYPE_LEN];

	/**
	 * config_methods - Config Methods
	 *
	 * This is a space-separated list of supported WPS configuration
	 * methods. For example, "label virtual_display virtual_push_button
	 * keypad".
	 * Available methods: usba ethernet label display ext_nfc_token
	 * int_nfc_token nfc_interface push_button keypad
	 * virtual_display physical_display
	 * virtual_push_button physical_push_button.
	 */
	char *config_methods;

	/**
	 * os_version - OS Version (WPS)
	 * 4-octet operating system version number
	 */
	u8 os_version[4];

	/**
	 * country - Country code
	 *
	 * This is the ISO/IEC alpha2 country code for which we are operating
	 * in
	 */
	char country[3];

#ifdef CONFIG_STA_COUNTRY_CODE
	struct wpa_freq_range country_range;
#endif /* CONFIG_STA_COUNTRY_CODE */

	/**
	 * wps_cred_processing - Credential processing
	 *
	 *   0 = process received credentials internally
	 *   1 = do not process received credentials; just pass them over
	 *	ctrl_iface to external program(s)
	 *   2 = process received credentials internally and pass them over
	 *	ctrl_iface to external program(s)
	 */
	int wps_cred_processing;

#define MAX_SEC_DEVICE_TYPES 5
	/**
	 * sec_device_types - Secondary Device Types (P2P)
	 */
	u8 sec_device_type[MAX_SEC_DEVICE_TYPES][WPS_DEV_TYPE_LEN];
	int num_sec_device_types;

#ifdef	CONFIG_P2P
#if 1	/* by Shingu 20160726 (P2P Config) */
	u8	*p2p_ssid;
	size_t	p2p_ssid_len;
	char	*p2p_passphrase;
#endif	/* 1 */

	int p2p_listen_reg_class;
	int p2p_listen_channel;
	int p2p_oper_reg_class;
	int p2p_oper_channel;
	int p2p_go_intent;
	char p2p_ssid_postfix[32];

#ifdef	CONFIG_P2P_OPTION
	int persistent_reconnect;
#endif	/* CONFIG_P2P_OPTION */

	int p2p_intra_bss;

#ifdef	CONFIG_P2P_UNUSED_CMD
	unsigned int num_p2p_pref_chan;
	struct p2p_channel *p2p_pref_chan;
	struct wpa_freq_range_list p2p_no_go_freq;
	int p2p_add_cli_chan;
#endif	/* CONFIG_P2P_UNUSED_CMD */

#ifdef	CONFIG_P2P_OPTION
	int p2p_ignore_shared_freq;
#endif	/* CONFIG_P2P_OPTION */

	int	p2p_find_timeout;	// Unit: 1 second /* FC9000 Only ?? */
	int p2p_optimize_listen_chan;
#endif	/* CONFIG_P2P */

#ifdef	CONFIG_WPS
	struct wpabuf *wps_vendor_ext_m1;

#define MAX_WPS_VENDOR_EXT 10

	/**
	 * wps_vendor_ext - Vendor extension attributes in WPS
	 */
	struct wpabuf *wps_vendor_ext[MAX_WPS_VENDOR_EXT];
#endif	/* CONFIG_WPS */

#ifdef	CONFIG_P2P
	/**
	 * p2p_group_idle - Maximum idle time in seconds for P2P group
	 *
	 * This value controls how long a P2P group is maintained after there
	 * is no other members in the group. As a GO, this means no associated
	 * stations in the group. As a P2P client, this means no GO seen in
	 * scan results. The maximum idle time is specified in seconds with 0
	 * indicating no time limit, i.e., the P2P group remains in active
	 * state indefinitely until explicitly removed. As a P2P client, the
	 * maximum idle time of P2P_MAX_CLIENT_IDLE seconds is enforced, i.e.,
	 * this parameter is mainly meant for GO use and for P2P client, it can
	 * only be used to reduce the default timeout to smaller value. A
	 * special value -1 can be used to configure immediate removal of the
	 * group for P2P client role on any disconnection after the data
	 * connection has been established.
	 */
	int p2p_group_idle;

#ifdef	CONFIG_P2P_UNUSED_CMD
	/**
	 * p2p_go_freq_change_policy - The GO frequency change policy
	 *
	 * This controls the behavior of the GO when there is a change in the
	 * map of the currently used frequencies in case more than one channel
	 * is supported.
	 *
	 * @P2P_GO_FREQ_MOVE_SCM: Prefer working in a single channel mode if
	 * possible. In case the GO is the only interface using its frequency
	 * and there are other station interfaces on other frequencies, the GO
	 * will migrate to one of these frequencies.
	 *
	 * @P2P_GO_FREQ_MOVE_SCM_PEER_SUPPORTS: Same as P2P_GO_FREQ_MOVE_SCM,
	 * but a transition is possible only in case one of the other used
	 * frequencies is one of the frequencies in the intersection of the
	 * frequency list of the local device and the peer device.
	 *
	 * @P2P_GO_FREQ_MOVE_STAY: Prefer to stay on the current frequency.
	 *
	 * @P2P_GO_FREQ_MOVE_SCM_ECSA: Same as
	 * P2P_GO_FREQ_MOVE_SCM_PEER_SUPPORTS but a transition is possible only
	 * if all the group members advertise eCSA support.
	 */
	enum {
		P2P_GO_FREQ_MOVE_SCM = 0,
		P2P_GO_FREQ_MOVE_SCM_PEER_SUPPORTS = 1,
		P2P_GO_FREQ_MOVE_STAY = 2,
		P2P_GO_FREQ_MOVE_SCM_ECSA = 3,
		P2P_GO_FREQ_MOVE_MAX = P2P_GO_FREQ_MOVE_SCM_ECSA,
	} p2p_go_freq_change_policy;
#endif	/* CONFIG_P2P_UNUSED_CMD */

#define DEFAULT_P2P_GO_FREQ_MOVE P2P_GO_FREQ_MOVE_STAY

	/**
	 * p2p_passphrase_len - Passphrase length (8..63) for P2P GO
	 *
	 * This parameter controls the length of the random passphrase that is
	 * generated at the GO.
	 */
	unsigned int p2p_passphrase_len;
#endif	/* CONFIG_P2P */

	/**
	 * bss_max_count - Maximum number of BSS entries to keep in memory
	 */
	unsigned int bss_max_count;

#ifndef CONFIG_IMMEDIATE_SCAN
	/**
	 * bss_expiration_age - BSS entry age after which it can be expired
	 *
	 * This value controls the time in seconds after which a BSS entry
	 * gets removed if it has not been updated or is not in use.
	 */
	unsigned int bss_expiration_age;

	/**
	 * bss_expiration_scan_count - Expire BSS after number of scans
	 *
	 * If the BSS entry has not been seen in this many scans, it will be
	 * removed. A value of 1 means that entry is removed after the first
	 * scan in which the BSSID is not seen. Larger values can be used
	 * to avoid BSS entries disappearing if they are not visible in
	 * every scan (e.g., low signal quality or interference).
	 */
	unsigned int bss_expiration_scan_count;
#endif /* CONFIG_IMMEDIATE_SCAN */

	/**
	 * filter_ssids - SSID-based scan result filtering
	 *
	 *   0 = do not filter scan results
	 *   1 = only include configured SSIDs in scan results/BSS table
	 */
	int filter_ssids;

#ifdef CONFIG_SCAN_FILTER_RSSI
	/**
	 * filter_rssi - RSSI-based scan result filtering
	 *
	 * 0 = do not filter scan results
	 * -n = filter scan results below -n dBm
	 */
	int filter_rssi;
#endif /* CONFIG_SCAN_FILTER_RSSI */

	/**
	 * max_num_sta - Maximum number of STAs in an AP/P2P GO
	 */
	unsigned int max_num_sta;

	/**
	 * ap_isolate - Whether to use client isolation feature
	 *
	 * Client isolation can be used to prevent low-level bridging of
	 * frames between associated stations in the BSS. By default,
	 * this bridging is allowed (ap_isolate=0); except in P2P GO case,
	 * where p2p_intra_bss parameter is used to determine whether to allow
	 * intra-BSS forwarding (ap_isolate = !p2p_intra_bss).
	 *
	 * 0 = do not enable AP isolation
	 * 1 = enable AP isolation
	 */
	int ap_isolate;

	/**
	 * freq_list - Array of allowed scan frequencies or %NULL for all
	 *
	 * This is an optional zero-terminated array of frequencies in
	 * megahertz (MHz) to allow for narrowing scanning range.
	 */
	int *freq_list;

	/**
	 * scan_cur_freq - Whether to scan only the current channel
	 *
	 * If true, attempt to scan only the current channel if any other
	 * VIFs on this radio are already associated on a particular channel.
	 */
	int scan_cur_freq;

	/**
	 * changed_parameters - Bitmap of changed parameters since last update
	 */
	unsigned int changed_parameters;

	/**
	 * disassoc_low_ack - Disassociate stations with massive packet loss
	 */
	int disassoc_low_ack;

#ifdef	CONFIG_INTERWORKING
	/**
	 * interworking - Whether Interworking (IEEE 802.11u) is enabled
	 */
	int interworking;
#endif	/* CONFIG_INTERWORKING */

	/**
	 * access_network_type - Access Network Type
	 *
	 * When Interworking is enabled, scans will be limited to APs that
	 * advertise the specified Access Network Type (0..15; with 15
	 * indicating wildcard match).
	 */
	int access_network_type;

#ifdef	CONFIG_P2P_UNUSED_CMD
	 /**
	  * go_interworking - Whether Interworking for P2P GO is enabled
	  */
	int go_interworking;

	/**
	 * go_access_network_type - P2P GO Access Network Type
	 *
	 * This indicates which access network type to advertise if Interworking
	 * is enabled for P2P GO.
	 */
	int go_access_network_type;

	/**
	 * go_internet - Interworking: Internet connectivity (0 or 1)
	 */
	int go_internet;

	/**
	 * go_venue_group - Interworking: Venue group
	 */
	int go_venue_group;

	/**
	 * go_venue_type: Interworking: Venue type
	 */
	int go_venue_type;
#endif	/* CONFIG_P2P_UNUSED_CMD */

	/**
	 * hessid - Homogenous ESS identifier
	 *
	 * If this is set (any octet is non-zero), scans will be used to
	 * request response only from BSSes belonging to the specified
	 * Homogeneous ESS. This is used only if interworking is enabled.
	 */
	u8 hessid[ETH_ALEN];

#ifdef CONFIG_HS20
	/**
	 * hs20 - Hotspot 2.0
	 */
	int hs20;
#endif /* CONFIG_HS20 */

	/**
	 * pbc_in_m1 - AP mode WPS probing workaround for PBC with Windows 7
	 *
	 * Windows 7 uses incorrect way of figuring out AP's WPS capabilities
	 * by acting as a Registrar and using M1 from the AP. The config
	 * methods attribute in that message is supposed to indicate only the
	 * configuration method supported by the AP in Enrollee role, i.e., to
	 * add an external Registrar. For that case, PBC shall not be used and
	 * as such, the PushButton config method is removed from M1 by default.
	 * If pbc_in_m1=1 is included in the configuration file, the PushButton
	 * config method is left in M1 (if included in config_methods
	 * parameter) to allow Windows 7 to use PBC instead of PIN (e.g., from
	 * a label in the AP).
	 */
	int pbc_in_m1;

	/**
	 * autoscan - Automatic scan parameters or %NULL if none
	 *
	 * This is an optional set of parameters for automatic scanning
	 * within an interface in following format:
	 * <autoscan module name>:<module parameters>
	 */
	char *autoscan;

#ifdef	CONFIG_AP
	/*
	 * ap_max_inactivity - Timeout in seconds to detect STA inactivity
	 *
	 * inactive stations.
	 * By default: 300 seconds.
	 */
	int ap_max_inactivity;

	/*
	 * ap_send_ka - Enable to send keep-alive (NULL Packet) periodically
	 */
	int ap_send_ka;
#endif	/* CONFIG_AP */

#ifdef CONFIG_WPS_NFC
	/**
	 * wps_nfc_pw_from_config - NFC Device Password was read from config
	 *
	 * This parameter can be determined whether the NFC Device Password was
	 * included in the configuration (1) or generated dynamically (0). Only
	 * the former case is re-written back to the configuration file.
	 */
	int wps_nfc_pw_from_config;

	/**
	 * wps_nfc_dev_pw_id - NFC Device Password ID for password token
	 */
	int wps_nfc_dev_pw_id;

	/**
	 * wps_nfc_dh_pubkey - NFC DH Public Key for password token
	 */
	struct wpabuf *wps_nfc_dh_pubkey;

	/**
	 * wps_nfc_dh_privkey - NFC DH Private Key for password token
	 */
	struct wpabuf *wps_nfc_dh_privkey;

	/**
	 * wps_nfc_dev_pw - NFC Device Password for password token
	 */
	struct wpabuf *wps_nfc_dev_pw;
#endif /* CONFIG_WPS_NFC */

#ifdef CONFIG_EXT_PASSWORD
	/**
	 * ext_password_backend - External password backend or %NULL if none
	 *
	 * format: <backend name>[:<optional backend parameters>]
	 */
	char *ext_password_backend;
#endif /* CONFIG_EXT_PASSWORD */

#ifdef CONFIG_P2P
	/*
	 * p2p_go_max_inactivity - Timeout in seconds to detect STA inactivity
	 *
	 * This timeout value is used in P2P GO mode to clean up
	 * inactive stations.
	 * By default: 300 seconds.
	 */
	int p2p_go_max_inactivity;
#endif /* CONFIG_P2P */

#ifdef	CONFIG_AP_WMM
	int hostapd_wmm_enabled;	// QoS (STA)
	int hostapd_wmm_ps_enabled;	// UAPSD

	struct hostapd_wmm_ac_params wmm_ac_params[4];

	/**
	 * auto_interworking - Whether to use network selection automatically
	 *
	 * 0 = do not automatically go through Interworking network selection
	 *     (i.e., require explicit interworking_select command for this)
	 * 1 = perform Interworking network selection if one or more
	 *     credentials have been configured and scan did not find a
	 *     matching network block
	 */
	int auto_interworking;
#endif	/* CONFIG_AP_WMM */

#ifdef	CONFIG_P2P
#ifdef	CONFIG_P2P_UNUSED_CMD
	/**
	 * p2p_go_ht40 - Default mode for HT40 enable when operating as GO.
	 *
	 * This will take effect for p2p_group_add, p2p_connect, and p2p_invite.
	 * Note that regulatory constraints and driver capabilities are
	 * consulted anyway, so setting it to 1 can't do real harm.
	 * By default: 0 (disabled)
	 */
	int p2p_go_ht40;

	/**
	 * p2p_go_vht - Default mode for VHT enable when operating as GO
	 *
	 * This will take effect for p2p_group_add, p2p_connect, and p2p_invite.
	 * Note that regulatory constraints and driver capabilities are
	 * consulted anyway, so setting it to 1 can't do real harm.
	 * By default: 0 (disabled)
	 */
	int p2p_go_vht;

	/**
	 * p2p_go_ctwindow - CTWindow to use when operating as GO
	 *
	 * By default: 0 (no CTWindow). Values 0-127 can be used to indicate
	 * the length of the CTWindow in TUs.
	 */
	int p2p_go_ctwindow;
#endif	/* CONFIG_P2P_UNUSED_CMD */

	/**
	 * p2p_disabled - Whether P2P operations are disabled for this interface
	 */
	int p2p_disabled;

	/**
	 * p2p_no_group_iface - Whether group interfaces can be used
	 *
	 * By default, wpa_supplicant will create a separate interface for P2P
	 * group operations if the driver supports this. This functionality can
	 * be disabled by setting this parameter to 1. In that case, the same
	 * interface that was used for the P2P management operations is used
	 * also for the group operation.
	 */
	int p2p_no_group_iface;

#ifdef	CONFIG_P2P_UNUSED_CMD
	/**
	 * p2p_cli_probe - Enable/disable P2P CLI probe request handling
	 *
	 * If this parameter is set to 1, a connected P2P Client will receive
	 * and handle Probe Request frames. Setting this parameter to 0
	 * disables this option. Default value: 0.
	 *
	 * Note: Setting this property at run time takes effect on the following
	 * interface state transition to/from the WPA_COMPLETED state.
	 */
	int p2p_cli_probe;
#endif	/* CONFIG_P2P_UNUSED_CMD */
#endif	/* CONFIG_P2P */

	/**
	 * okc - Whether to enable opportunistic key caching by default
	 *
	 * By default, OKC is disabled unless enabled by the per-network
	 * proactive_key_caching=1 parameter. okc=1 can be used to change this
	 * default behavior.
	 */
	int okc;

#ifdef CONFIG_IEEE80211W
	/**
	 * pmf - Whether to enable/require PMF by default
	 *
	 * By default, PMF is disabled unless enabled by the per-network
	 * ieee80211w=1 or ieee80211w=2 parameter. pmf=1/2 can be used to change
	 * this default behavior for RSN network (this is not applicable for
	 * non-RSN cases).
	 */
	enum mfp_options pmf;
#endif /* CONFIG_IEEE80211W */

#ifdef CONFIG_SAE
	/**
	 * sae_groups - Preference list of enabled groups for SAE
	 *
	 * By default (if this parameter is not set), the mandatory group 19
	 * (ECC group defined over a 256-bit prime order field) is preferred,
	 * but other groups are also enabled. If this parameter is set, the
	 * groups will be tried in the indicated order.
	 */
	int *sae_groups;
#endif /* CONFIG_SAE */

	/**
	 * dtim_period - Default DTIM period in Beacon intervals
	 *
	 * This parameter can be used to set the default value for network
	 * blocks that do not specify dtim_period.
	 */
	int dtim_period;

	/**
	 * beacon_int - Default Beacon interval in TU
	 *
	 * This parameter can be used to set the default value for network
	 * blocks that do not specify beacon_int.
	 */
	int beacon_int;

	/**
	 * ap_vendor_elements: Vendor specific elements for Beacon/ProbeResp
	 *
	 * This parameter can be used to define additional vendor specific
	 * elements for Beacon and Probe Response frames in AP/P2P GO mode. The
	 * format for these element(s) is a hexdump of the raw information
	 * elements (id+len+payload for one or more elements).
	 */
	struct wpabuf *ap_vendor_elements;

#ifdef CONFIG_IGNOR_OLD_SCAN
	/**
	 * ignore_old_scan_res - Ignore scan results older than request
	 *
	 * The driver may have a cache of scan results that makes it return
	 * information that is older than our scan trigger. This parameter can
	 * be used to configure such old information to be ignored instead of
	 * allowing it to update the internal BSS table.
	 */
	int ignore_old_scan_res;
#endif /* CONFIG_IGNOR_OLD_SCAN */

#ifdef CONFIG_SCHED_SCAN
	/**
	 * sched_scan_interval -  schedule scan interval
	 */
	unsigned int sched_scan_interval;

	/**
	 * sched_scan_start_delay - Schedule scan start delay before first scan
	 *
	 * Delay (in seconds) before scheduling first scan plan cycle. The
	 * driver may ignore this parameter and start immediately (or at any
	 * other time), if this feature is not supported.
	 */
	unsigned int sched_scan_start_delay;
#endif /* CONFIG_SCHED_SCAN */

	/**
	 * tdls_external_control - External control for TDLS setup requests
	 *
	 * Enable TDLS mode where external programs are given the control
	 * to specify the TDLS link to get established to the driver. The
	 * driver requests the TDLS setup to the supplicant only for the
	 * specified TDLS peers.
	 */
	int tdls_external_control;

#ifdef CONFIG_P2P
	u8 ip_addr_go[4];
	u8 ip_addr_mask[4];
	u8 ip_addr_start[4];
	u8 ip_addr_end[4];
#endif /* CONFIG_P2P */

#if 1 /* FC9000 Only ?? */
	enum wpas_mode mode;

#ifdef CONFIG_ACL
	/**
	 * ACL (Access Control List)
	 */

/*	enum { ACCEPT_UNLESS_DENIED = 0,
		DENY_UNLESS_ACCEPTED = 1,
		USE_EXTERNAL_RADIUS_AUTH = 2
	} macaddr_acl;   */
	int macaddr_acl;

	struct mac_acl_entry acl_mac[NUM_MAX_ACL];
	int acl_num;
#endif /* CONFIG_ACL */

#ifdef CONFIG_AP_POWER
	char *ap_power;
#endif /* CONFIG_AP_POWER */

#ifdef CONFIG_ALLOW_ANY_OPEN_AP
	int allow_any_open_ap;
#endif

#ifdef CONFIG_SIMPLE_ROAMING
	enum simple_roam_state roam_state;
	int roam_cnt;
	int roam_threshold;
	u8 candi_bssid[ETH_ALEN];
#endif /* CONFIG_SIMPLE_ROAMING */
	int greenfield;
	int rts_threshold;
	u8 ht_protection;

#ifdef CONFIG_AP_WIFI_MODE
	int wifi_mode;
#endif /* CONFIG_AP_WIFI_MODE */
#endif /* FC9000 Only ?? */

	/**
	 * osu_dir - OSU provider information directory
	 *
	 * If set, allow FETCH_OSU control interface command to be used to fetch
	 * OSU provider information into all APs and store the results in this
	 * directory.
	 */
	char *osu_dir;

#ifdef CONFIG_WOWLAN
	/**
	 * wowlan_triggers - Wake-on-WLAN triggers
	 *
	 * If set, these wowlan triggers will be configured.
	 */
	char *wowlan_triggers;
#endif /* CONFIG_WOWLAN */

	/**
	 * p2p_search_delay - Extra delay between concurrent search iterations
	 *
	 * Add extra delay (in milliseconds) between search iterations when
	 * there is a concurrent operation to make p2p_find friendlier to
	 * concurrent operations by avoiding it from taking 100% of radio
	 * resources.
	 */
	unsigned int p2p_search_delay;

	/**
	 * mac_addr - MAC address policy default
	 *
	 * 0 = use permanent MAC address
	 * 1 = use random MAC address for each ESS connection
	 * 2 = like 1, but maintain OUI (with local admin bit set)
	 *
	 * By default, permanent MAC address is used unless policy is changed by
	 * the per-network mac_addr parameter. Global mac_addr=1 can be used to
	 * change this default behavior.
	 */
	int mac_addr;

#ifdef CONFIG_RANDOM_ADDR
	/**
	 * rand_addr_lifetime - Lifetime of random MAC address in seconds
	 */
	unsigned int rand_addr_lifetime;

	/**
	 * preassoc_mac_addr - Pre-association MAC address policy
	 *
	 * 0 = use permanent MAC address
	 * 1 = use random MAC address
	 * 2 = like 1, but maintain OUI (with local admin bit set)
	 */
	int preassoc_mac_addr;
#endif /* CONFIG_RANDOM_ADDR */


	/**
	 * key_mgmt_offload - Use key management offload
	 *
	 * Key management offload should be used if the device supports it.
	 * Key management offload is the capability of a device operating as
	 * a station to do the exchange necessary to establish temporal keys
	 * during initial RSN connection, after roaming, or during a PTK
	 * rekeying operation.
	 */
	int key_mgmt_offload;

#ifdef CONFIG_MESH
	/**
	 * user_mpm - MPM residency
	 *
	 * 0: MPM lives in driver.
	 * 1: wpa_supplicant handles peering and station allocation.
	 *
	 * If AMPE or SAE is enabled, the MPM is always in userspace.
	 */
	int user_mpm;

	/**
	 * max_peer_links - Maximum number of peer links
	 *
	 * Maximum number of mesh peering currently maintained by the STA.
	 */
	int max_peer_links;
#endif /* CONFIG_MESH */

	/**
	 * cert_in_cb - Whether to include a peer certificate dump in events
	 *
	 * This controls whether peer certificates for authentication server and
	 * its certificate chain are included in EAP peer certificate events.
	 */
	int cert_in_cb;

#ifdef CONFIG_MESH
	/**
	 * mesh_max_inactivity - Timeout in seconds to detect STA inactivity
	 *
	 * This timeout value is used in mesh STA to clean up inactive stations.
	 * By default: 300 seconds.
	 */
	int mesh_max_inactivity;
#endif /* CONFIG_MESH */

#ifdef CONFIG_SAE
	/**
	 * dot11RSNASAERetransPeriod - Timeout to retransmit SAE Auth frame
	 *
	 * This timeout value is used in mesh STA to retransmit
	 * SAE Authentication frame.
	 * By default: 1000 milliseconds.
	 */
	int dot11RSNASAERetransPeriod;
#endif /* CONFIG_SAE */

	/**
	 * passive_scan - Whether to force passive scan for network connection
	 *
	 * This parameter can be used to force only passive scanning to be used
	 * for network connection cases. It should be noted that this will slow
	 * down scan operations and reduce likelihood of finding the AP. In
	 * addition, some use cases will override this due to functional
	 * requirements, e.g., for finding an AP that uses hidden SSID
	 * (scan_ssid=1) or P2P device discovery.
	 */
	int passive_scan;

	/**
	 * reassoc_same_bss_optim - Whether to optimize reassoc-to-same-BSS
	 */
	int reassoc_same_bss_optim;

	/**
	 * wps_priority - Priority for the networks added through WPS
	 *
	 * This priority value will be set to each network profile that is added
	 * by executing the WPS protocol.
	 */
	int wps_priority;

#ifdef CONFIG_FST
	/**
	 * fst_group_id - FST group ID
	 */
	char *fst_group_id;

	/**
	 * fst_priority - priority of the interface within the FST group
	 */
	int fst_priority;

	/**
	 * fst_llt - default FST LLT (Link-Lost Timeout) to be used for the
	 * interface.
	 */
	int fst_llt;
#endif /* CONFIG_FST */

	 /**
	  * wpa_rsc_relaxation - RSC relaxation on GTK installation
	  *
	  * Values:
	  * 0 - use the EAPOL-Key RSC value on GTK installation
	  * 1 - use the null RSC if a bogus RSC value is detected in message 3
	  * of 4-Way Handshake or message 1 of Group Key Handshake.
	  */
	 int wpa_rsc_relaxation;

#ifdef CONFIG_SCHED_SCAN
	/**
	 * sched_scan_plans - Scan plans for scheduled scan
	 *
	 * Each scan plan specifies the interval between scans and the number of
	 * iterations. The last scan plan only specifies the scan interval and
	 * will be run infinitely.
	 *
	 * format: <interval:iterations> <interval2:iterations2> ... <interval>
	 */
	 char *sched_scan_plans;
#endif /* CONFIG_SCHED_SCAN */

#ifdef CONFIG_MBO
	/**
	 * non_pref_chan - Non-preferred channels list, separated by spaces.
	 *
	 * format: op_class:chan:preference:reason<:detail>
	 * Detail is optional.
	 */
	char *non_pref_chan;

	/**
	 * mbo_cell_capa - Cellular capabilities for MBO
	 */
	enum mbo_cellular_capa mbo_cell_capa;

	/**
	 * disassoc_imminent_rssi_threshold - RSSI threshold of candidate AP
	 * when disassociation imminent is set.
	 */
	int disassoc_imminent_rssi_threshold;

	/**
	 * oce - Enable OCE in STA and/or STA-CFON mode
	 *  - Set BIT(0) to enable OCE in non-AP STA mode
	 *  - Set BIT(1) to enable OCE in STA-CFON mode
	 */
	unsigned int oce;
#endif /* CONFIG_MBO */

#ifdef CONFIG_GAS
	/**
	 * gas_address3 - GAS Address3 field behavior
	 *
	 * Values:
	 * 0 - P2P specification (Address3 = AP BSSID)
	 * 1 = IEEE 802.11 standard compliant (Address3 = Wildcard BSSID when
	 *	sent to not-associated AP; if associated, AP BSSID)
	 */
	int gas_address3;
#endif /* CONFIG_GAS */

	/**
	 * ftm_responder - Publish FTM (fine timing measurement)
	 * responder functionality
	 *
	 * Values:
	 * 0 - do not publish FTM responder functionality (Default)
	 * 1 - publish FTM responder functionality in
	 *	bit 70 of Extended Capabilities element
	 * Note, actual FTM responder operation is managed outside
	 * wpa_supplicant.
	 */
	int ftm_responder;

	/**
	 * ftm_initiator - Publish FTM (fine timing measurement)
	 * initiator functionality
	 *
	 * Values:
	 * 0 - do not publish FTM initiator functionality (Default)
	 * 1 - publish FTM initiator functionality in
	 *	bit 71 of Extended Capabilities element
	 * Note, actual FTM initiator operation is managed outside
	 * wpa_supplicant.
	 */
	int ftm_initiator;

#ifdef CONFIG_GAS
	/**
	 * gas_rand_addr_lifetime - Lifetime of random MAC address for ANQP in
	 *	seconds
	 */
	unsigned int gas_rand_addr_lifetime;

	/**
	 * gas_rand_mac_addr - GAS MAC address policy
	 *
	 * 0 = use permanent MAC address
	 * 1 = use random MAC address
	 * 2 = like 1, but maintain OUI (with local admin bit set)
	 */
	int gas_rand_mac_addr;
#endif /* CONFIG_GAS */


#ifdef CONFIG_DPP
	/**
	 * dpp_config_processing - How to process DPP configuration
	 *
	 * 0 = report received configuration to an external program for
	 *	processing; do not generate any network profile internally
	 * 1 = report received configuration to an external program and generate
	 *	a network profile internally, but do not automatically connect
	 *	to the created (disabled) profile; the network profile id is
	 *	reported to external programs
	 * 2 = report received configuration to an external program, generate
	 *	a network profile internally, try to connect to the created
	 *	profile automatically
	 */
	int dpp_config_processing;
#endif /* CONFIG_DPP */
	
#ifdef CONFIG_OWE_TRANS
	int owe_transition_mode;
#endif // CONFIG_OWE_TRANS
};


/* Prototypes for common functions from config.c */

void wpa_config_free(struct wpa_config *ssid);
void wpa_config_free_ssid(struct wpa_ssid *ssid);
void wpa_config_foreach_network(struct wpa_config *config,
				void (*func)(void *, struct wpa_ssid *),
				void *arg);
struct wpa_ssid * wpa_config_get_network(struct wpa_config *config, int id);
struct wpa_ssid * wpa_config_add_network(struct wpa_config *config,
				char *ifname, int index);
int wpa_config_remove_network(struct wpa_config *config, int id);
void wpa_config_set_global_defaults(struct wpa_config *config);
void wpa_config_set_network_defaults(struct wpa_ssid *ssid);
void wpa_config_set_config_clean(struct wpa_config *config);
int wpa_config_set(struct wpa_ssid *ssid, const char *var, const char *value);
int wpa_config_set_quoted(struct wpa_ssid *ssid, const char *var,
			  const char *value);
//char * nvram_config_get(int i);
#ifdef	UNUSED_CODE
int wpa_config_dump_values(struct wpa_config *config, char *buf,
			   size_t buflen);
int wpa_config_get_value(const char *name, struct wpa_config *config,
			 char *buf, size_t buflen);
#endif	/* UNUSED_CODE */

char ** wpa_config_get_all(struct wpa_ssid *ssid, int get_keys);
char * wpa_config_get(struct wpa_ssid *ssid, const char *var);
char * wpa_config_get_no_key(struct wpa_ssid *ssid, const char *var);
void wpa_config_update_psk(struct wpa_ssid *ssid);
#ifdef CONFIG_PRIO_GROUP
int wpa_config_add_prio_network(struct wpa_config *config,
				struct wpa_ssid *ssid);
int wpa_config_update_prio_list(struct wpa_config *config);
#endif /* CONFIG_PRIO_GROUP */


#ifndef CONFIG_NO_CONFIG_BLOBS
const struct wpa_config_blob * wpa_config_get_blob(struct wpa_config *config,
						   const char *name);
void wpa_config_set_blob(struct wpa_config *config,
			 struct wpa_config_blob *blob);
void wpa_config_free_blob(struct wpa_config_blob *blob);
int wpa_config_remove_blob(struct wpa_config *config, const char *name);
void wpa_config_flush_blobs(struct wpa_config *config);
#endif /* CONFIG_NO_CONFIG_BLOBS */


#ifdef CONFIG_CRED
struct wpa_cred * wpa_config_get_cred(struct wpa_config *config, int id);
struct wpa_cred * wpa_config_add_cred(struct wpa_config *config);
int wpa_config_remove_cred(struct wpa_config *config, int id);
void wpa_config_free_cred(struct wpa_cred *cred);
int wpa_config_set_cred(struct wpa_cred *cred, const char *var,
			const char *value);
char * wpa_config_get_cred_no_key(struct wpa_cred *cred, const char *var);
#endif /* CONFIG_CRED */

struct wpa_config * wpa_config_alloc_empty(const char *ctrl_interface,
					   const char *driver_param);
#if 1 /* FC9000 supplicant 2.4 ?? */
int hostapd_iconf_write(struct hostapd_config *iconf);
int hostapd_conf_write(struct hostapd_bss_config *conf);
int hostapd_iconf_read(struct hostapd_config *iconf);
#endif /* FC9000 supplicant 2.4 ?? */

#ifndef CONFIG_NO_STDOUT_DEBUG
void wpa_config_debug_dump_networks(struct wpa_config *config);
#else /* CONFIG_NO_STDOUT_DEBUG */
#define wpa_config_debug_dump_networks(c) do { } while (0)
#endif /* CONFIG_NO_STDOUT_DEBUG */


/* Prototypes for common functions from config.c */
int wpa_config_process_global(struct wpa_config *config, char *pos);


#ifdef	UNUSED_CODE
int wpa_config_get_num_global_field_names(void);

const char * wpa_config_get_global_field_name(unsigned int i, int *no_var);
#endif	/* UNUSED_CODE */

/* Prototypes for backend specific functions from the selected config_*.c */

#if 0	/* for Concurrent */
/**
 * wpa_config_read - Read and parse configuration database
 * @name: Name of the configuration (e.g., path and file name for the
 * configuration file)
 * @cfgp: Pointer to previously allocated configuration data or %NULL if none
 * Returns: Pointer to allocated configuration data or %NULL on failure
 *
 * This function reads configuration data, parses its contents, and allocates
 * data structures needed for storing configuration information. The allocated
 * data can be freed with wpa_config_free().
 *
 * Each configuration backend needs to implement this function.
 */

// Supplicant 2.7
//struct wpa_config * wpa_config_read(const char *name, struct wpa_config *cfgp); 

struct wpa_config * wpa_config_read(struct wpa_config *cfgp);
#else
/**
 * wpa_config_read - Read and parse configuration database
 * @name: Name of the configuration (e.g., path and file name for the
 * configuration file)
 * @ifname: interface name
 * Returns: Pointer to allocated configuration data or %NULL on failure
 *
 * This function reads configuration data, parses its contents, and allocates
 * data structures needed for storing configuration information. The allocated
 * data can be freed with wpa_config_free().
 *
 * Each configuration backend needs to implement this function.
 */
#if defined ( __SUPP_27_SUPPORT__ )
struct wpa_config * wpa_config_read(const char *name, char *ifname);
#else
struct wpa_config * wpa_config_read(char *ifname);
#endif // __SUPP_27_SUPPORT__
int wpa_config_read_networks(struct wpa_config *config, char *ifname);
int wpa_config_read_global(struct wpa_config *config, char *ifname);
#endif	/* 0 */

/**
 * wpa_config_write - Write or update configuration data
 * @name: Name of the configuration (e.g., path and file name for the
 * configuration file)
 * @config: Configuration data from wpa_config_read()
 * @ifname: Interface device name (sta0, softap1, or p2p1)
 * Returns: 0 on success, -1 on failure
 *
 * This function write all configuration data into an external database (e.g.,
 * a text file) in a format that can be read with wpa_config_read(). This can
 * be used to allow wpa_supplicant to update its configuration, e.g., when a
 * new network is added or a password is changed.
 *
 * Each configuration backend needs to implement this function.
 */
// Supplicant 2.7
//int wpa_config_write(const char *name, struct wpa_config *config);

#if defined ( __SUPP_27_SUPPORT__ )
int wpa_config_write(char *confname, struct wpa_config *config, char *ifname);
#else
int wpa_config_write(struct wpa_config *config, char *ifname);
#endif // __SUPP_27_SUPPORT__

void dpm_disconn_reconf(struct wpa_config *config, char *ifname);

extern char * alloc_strdup(const char *str);

int wpa_config_read_nvram_int(const char *name, int *_val);
char * wpa_config_read_nvram_string(const char *name);

int wpa_config_write_nvram_int(const char *name, int val, int def);
int wpa_config_write_nvram_string(const char *name, const char *val);
void write_str(const char *field, struct wpa_ssid *ssid);
int chk_channel_by_country(char *country, int set_channel, int mode,
		 	    int *rec_channel);
#endif /* SUPP_CONFIG_H */

/* EOF */
