/*
 * WPA Supplicant - Basic AP mode support routines
 * Copyright (c) 2003-2009, Jouni Malinen <j@w1.fi>
 * Copyright (c) 2009, Atheros Communications
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"

#ifdef	CONFIG_AP

#include "supp_eloop.h"
#include "uuid.h"
#include "common/ieee802_11_defs.h"
#include "wpa_ctrl.h"
#include "eapol_supp/eapol_supp_sm.h"
#include "crypto/dh_group5.h"

#include "ap/hostapd.h"
#include "ap/ap_config.h"
#include "ap/ap_drv_ops.h"
#ifdef NEED_AP_MLME
#include "ap/ieee802_11.h"
#endif /* NEED_AP_MLME */
#include "ap/beacon.h"
#include "ap/ieee802_1x.h"
#include "ap/hostapd.h"
#include "ap/wps_hostapd.h"
#include "ap/ctrl_iface_ap.h"
#include "ap/dfs.h"
#include "wps/wps.h"
#include "common/ieee802_11_defs.h"
#include "config_ssid.h"
#include "supp_config.h"
#include "wpa_supplicant_i.h"
#include "driver_i.h"
#include "p2p_supplicant.h"
#include "ap.h"
#include "bss.h"
#include "ap/sta_info.h"
#include "driver_fc80211.h"

//#include "notify.h"

#pragma GCC diagnostic ignored "-Wimplicit-function-declaration"

#ifdef CONFIG_WPS
static void wpas_wps_ap_pin_timeout(void *eloop_data, void *user_ctx);
#endif /* CONFIG_WPS */


extern int i3ed11_freq_to_ch(int);
extern void supplicant_done(void);
extern int i3ed11_ch_to_freq(int chan, enum i3ed11_band band);
extern void hostapd_tx_queue_params(struct hostapd_iface *iface);
extern unsigned int ctoi(char *s);


#ifdef CONFIG_IEEE80211AC
static void wpas_conf_ap_vht(struct wpa_supplicant *wpa_s,
			     struct wpa_ssid *ssid,
			     struct hostapd_config *conf,
			     struct hostapd_hw_modes *mode)
{
#ifdef CONFIG_P2P
	u8 center_chan = 0;
	u8 channel = conf->channel;
#endif /* CONFIG_P2P */

	if (!conf->secondary_channel)
		goto no_vht;

#ifdef CONFIG_IEEE80211AC
	/* Use the maximum oper channel width if it's given. */
	if (ssid->max_oper_chwidth)
		conf->vht_oper_chwidth = ssid->max_oper_chwidth;

	ieee80211_freq_to_chan(ssid->vht_center_freq2,
			       &conf->vht_oper_centr_freq_seg1_idx);
#endif /* CONFIG_IEEE80211AC */

#ifdef CONFIG_P2P
	if (!ssid->p2p_group)
#endif	/* CONFIG_P2P */
	{
#ifdef CONFIG_IEEE80211AC
		if (!ssid->vht_center_freq1 ||
		    conf->vht_oper_chwidth == VHT_CHANWIDTH_USE_HT)
#endif /* CONFIG_IEEE80211AC */
			goto no_vht;
#ifdef CONFIG_IEEE80211AC
		ieee80211_freq_to_chan(ssid->vht_center_freq1,
				       &conf->vht_oper_centr_freq_seg0_idx);
		wpa_printf_dbg(MSG_DEBUG, "VHT seg0 index %d for AP",
			   conf->vht_oper_centr_freq_seg0_idx);
#endif /* CONFIG_IEEE80211AC */
		return;
	}

#ifdef CONFIG_P2P
	switch (conf->vht_oper_chwidth) {
	case VHT_CHANWIDTH_80MHZ:
	case VHT_CHANWIDTH_80P80MHZ:
		center_chan = wpas_p2p_get_vht80_center(wpa_s, mode, channel);
		wpa_printf_dbg(MSG_DEBUG,
			   "VHT center channel %u for 80 or 80+80 MHz bandwidth",
			   center_chan);
		break;
	case VHT_CHANWIDTH_160MHZ:
		center_chan = wpas_p2p_get_vht160_center(wpa_s, mode, channel);
		wpa_printf_dbg(MSG_DEBUG,
			   "VHT center channel %u for 160 MHz bandwidth",
			   center_chan);
		break;
	default:
		/*
		 * conf->vht_oper_chwidth might not be set for non-P2P GO cases,
		 * try oper_cwidth 160 MHz first then VHT 80 MHz, if 160 MHz is
		 * not supported.
		 */
		conf->vht_oper_chwidth = VHT_CHANWIDTH_160MHZ;
		center_chan = wpas_p2p_get_vht160_center(wpa_s, mode, channel);
		if (center_chan) {
			wpa_printf_dbg(MSG_DEBUG,
				   "VHT center channel %u for auto-selected 160 MHz bandwidth",
				   center_chan);
		} else {
			conf->vht_oper_chwidth = VHT_CHANWIDTH_80MHZ;
			center_chan = wpas_p2p_get_vht80_center(wpa_s, mode,
								channel);
			wpa_printf_dbg(MSG_DEBUG,
				   "VHT center channel %u for auto-selected 80 MHz bandwidth",
				   center_chan);
		}
		break;
	}
	if (!center_chan)
		goto no_vht;

	conf->vht_oper_centr_freq_seg0_idx = center_chan;
	wpa_printf_dbg(MSG_DEBUG, "VHT seg0 index %d for P2P GO",
		   conf->vht_oper_centr_freq_seg0_idx);
	return;
#endif /* CONFIG_P2P */


no_vht:

	wpa_printf_dbg(MSG_DEBUG,
		   "No VHT higher bandwidth support for the selected channel %d",
		   conf->channel);
	conf->vht_oper_centr_freq_seg0_idx =
		conf->channel + conf->secondary_channel * 2;

	conf->vht_oper_chwidth = VHT_CHANWIDTH_USE_HT;
}
#endif /* CONFIG_IEEE80211AC */


int wpa_supplicant_conf_ap_ht(struct wpa_supplicant *wpa_s,
			      struct wpa_ssid *ssid,
			      struct hostapd_config *conf)
{

#ifdef CONFIG_ACS /* DA16200 */
	if (ssid->acs == 1 || ssid->frequency == 0) {
		if (ssid->wifi_mode == DA16X_WIFI_MODE_B_ONLY) {
			ssid->disable_ht = 1;
			conf->hw_mode = HOSTAPD_MODE_IEEE80211B;
		} else {
			conf->hw_mode = HOSTAPD_MODE_IEEE80211G;
		}
	} else
#endif /* CONFIG_ACS */
	{
		conf->hw_mode = ieee80211_freq_to_chan(ssid->frequency,
						       &conf->channel);

#ifdef CONFIG_AP_WIFI_MODE
		conf->wifi_mode = ssid->wifi_mode;
	
		if (conf->wifi_mode == DA16X_WIFI_MODE_BG ||
			conf->wifi_mode == DA16X_WIFI_MODE_G_ONLY ||
			conf->wifi_mode == DA16X_WIFI_MODE_B_ONLY) {
			ssid->disable_ht = 1;
		} else {
			ssid->disable_ht = 0;
		}
	
		if (conf->hw_mode == HOSTAPD_MODE_IEEE80211G &&
			conf->wifi_mode == DA16X_WIFI_MODE_B_ONLY) {
			conf->hw_mode = HOSTAPD_MODE_IEEE80211B;
		}
#endif /* CONFIG_AP_WIFI_MODE */

		if (conf->hw_mode == NUM_HOSTAPD_MODES) {
			wpa_printf(MSG_ERROR, "Unsupported AP mode frequency: %d MHz",
				   ssid->frequency);
			return -1;
		}
	}

	/* TODO: enable HT40 if driver supports it;
	 * drop to 11b if driver does not support 11g */

#ifdef CONFIG_IEEE80211N
	/*
	 * Enable HT20 if the driver supports it, by setting conf->ieee80211n
	 * and a mask of allowed capabilities within conf->ht_capab.
	 * Using default config settings for: conf->ht_op_mode_fixed,
	 * conf->secondary_channel, conf->require_ht
	 */
	if (wpa_s->hw.modes) {
		struct hostapd_hw_modes *mode = NULL;
		int i, no_ht = 0;

		wpa_printf_dbg(MSG_DEBUG,
			   "Determining HT/VHT options based on driver capabilities (freq=%u chan=%u)",
			   ssid->frequency, conf->channel);

		for (i = 0; i < wpa_s->hw.num_modes; i++) {
			if (wpa_s->hw.modes[i].mode == conf->hw_mode) {
				mode = &wpa_s->hw.modes[i];
				break;
			}
		}

#ifdef CONFIG_HT_OVERRIDES
		if (ssid->disable_ht)
			ssid->ht = 0;
#endif /* CONFIG_HT_OVERRIDES */

		if (!ssid->ht) {
			wpa_printf_dbg(MSG_DEBUG,
				   "HT not enabled in network profile");
			conf->ieee80211n = 0;
			conf->ht_capab = 0;
			no_ht = 1;
		}

#ifdef CONFIG_AP_WIFI_MODE
		if(ssid->wifi_mode == DA16X_WIFI_MODE_N_ONLY) {
			conf->require_ht = 1;
		}
#endif /* CONFIG_AP_WIFI_MODE */

		if (!no_ht && mode && mode->ht_capab) {
			wpa_printf_dbg(MSG_DEBUG,
				   "Enable HT support (p2p_group=%d 11a=%d ht40_hw_capab=%d ssid->ht40=%d)",
#ifdef CONFIG_P2P
				   ssid->p2p_group,
#else
					0,
#endif /* CONFIG_P2P */
				   conf->hw_mode == HOSTAPD_MODE_IEEE80211A,
				   !!(mode->ht_capab &
				      HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET),
				   ssid->ht40);
			conf->ieee80211n = 1;
			
#ifdef CONFIG_P2P_UNUSED_CMD
			if (ssid->p2p_group &&
			    conf->hw_mode == HOSTAPD_MODE_IEEE80211A &&
			    (mode->ht_capab &
			     HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET) &&
			    ssid->ht40) {
				conf->secondary_channel =
					wpas_p2p_get_ht40_mode(wpa_s, mode,
							       conf->channel);
				wpa_printf_dbg(MSG_DEBUG,
					   "HT secondary channel offset %d for P2P group",
					   conf->secondary_channel);
			}
#endif /* CONFIG_P2P_UNUSED_CMD */

			if (
#ifdef CONFIG_P2P
				!ssid->p2p_group &&
#endif	/* CONFIG_P2P */
			    (mode->ht_capab &
			     HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET)) {
				conf->secondary_channel = ssid->ht40;
				wpa_printf_dbg(MSG_DEBUG,
					   "HT secondary channel offset %d for AP",
					   conf->secondary_channel);
			}

			if (conf->secondary_channel)
				conf->ht_capab |=
					HT_CAP_INFO_SUPP_CHANNEL_WIDTH_SET;

			/*
			 * white-list capabilities that won't cause issues
			 * to connecting stations, while leaving the current
			 * capabilities intact (currently disabled SMPS).
			 */
			conf->ht_capab |= mode->ht_capab &
				(HT_CAP_INFO_GREEN_FIELD |
				 HT_CAP_INFO_SHORT_GI20MHZ |
				 HT_CAP_INFO_SHORT_GI40MHZ |
				 HT_CAP_INFO_RX_STBC_MASK |
				 HT_CAP_INFO_TX_STBC |
				 HT_CAP_INFO_MAX_AMSDU_SIZE);

#ifdef CONFIG_IEEE80211AC
			if (mode->vht_capab && ssid->vht) {
				conf->ieee80211ac = 1;
				conf->vht_capab |= mode->vht_capab;
				wpas_conf_ap_vht(wpa_s, ssid, conf, mode);
			}
#endif /* CONFIG_IEEE80211AC */
		}
	}

	if (conf->secondary_channel) {
		struct wpa_supplicant *iface;

		for (iface = wpa_s->global->ifaces; iface; iface = iface->next)
		{
			if (iface == wpa_s ||
			    iface->wpa_state < WPA_AUTHENTICATING ||
			    (int) iface->assoc_freq != ssid->frequency)
				continue;

			/*
			 * Do not allow 40 MHz co-ex PRI/SEC switch to force us
			 * to change our PRI channel since we have an existing,
			 * concurrent connection on that channel and doing
			 * multi-channel concurrency is likely to cause more
			 * harm than using different PRI/SEC selection in
			 * environment with multiple BSSes on these two channels
			 * with mixed 20 MHz or PRI channel selection.
			 */
			conf->no_pri_sec_switch = 1;
		}
	}
#endif /* CONFIG_IEEE80211N */

	return 0;
}


static int wpa_supplicant_conf_ap(struct wpa_supplicant *wpa_s,
				  struct wpa_ssid *ssid,
				  struct hostapd_config *conf)
{
	struct hostapd_bss_config *bss = conf->bss[0];

	conf->driver = wpa_s->driver;

	os_strlcpy(bss->iface, wpa_s->ifname, sizeof(bss->iface));

	if (wpa_supplicant_conf_ap_ht(wpa_s, ssid, conf))
		return -1;

	if (ssid->pbss > 1) {
		wpa_printf(MSG_ERROR, "Invalid pbss value(%d) for AP mode",
			   ssid->pbss);
		return -1;
	}
	bss->pbss = ssid->pbss;

#ifdef CONFIG_ACS
	if (ssid->acs || ssid->frequency == 0) {
		/* Setting channel to 0 in order to enable ACS */
		conf->channel = 0;
		conf->acs = 1;

		ssid->acs = 1;
		wpa_s->scanning = 1;
		wpa_printf_dbg(MSG_DEBUG, "Use automatic channel selection");

	}
#endif /* CONFIG_ACS */

#ifdef CONFIG_IEEE80211H
	if (ieee80211_is_dfs(ssid->frequency, wpa_s->hw.modes,
			     wpa_s->hw.num_modes) && wpa_s->conf->country[0]) {
#ifdef CONFIG_IEEE80211H
		conf->ieee80211h = 1;
#endif /* CONFIG_IEEE80211H */
#ifdef CONFIG_IEEE80211D
		conf->ieee80211d = 1;
#endif /* CONFIG_IEEE80211D */
		conf->country[0] = wpa_s->conf->country[0];
		conf->country[1] = wpa_s->conf->country[1];
#if 1 /* FC9000 */
		conf->country[2] = wpa_s->conf->country[2];
#else
		conf->country[2] = ' ';
#endif /* FC9000 */
	}
#else
#ifdef CONFIG_IEEE80211D
	conf->ieee80211d = 1;
#endif /* CONFIG_IEEE80211D */

	conf->country[0] = wpa_s->conf->country[0];
	conf->country[1] = wpa_s->conf->country[1];
#if 1 /* FC9000 */
	conf->country[2] = wpa_s->conf->country[2];
#else
	conf->country[2] = ' ';
#endif /* FC9000 */

#endif /* CONFIG_IEEE80211H */

#ifdef CONFIG_P2P
	if (conf->hw_mode == HOSTAPD_MODE_IEEE80211G &&
	    (ssid->mode == WPAS_MODE_P2P_GO ||
	     ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)) {
		/* Remove 802.11b rates from supported and basic rate sets */
		int *list = os_malloc(4 * sizeof(int));
		if (list) {
			list[0] = 60;
			list[1] = 120;
			list[2] = 240;
			list[3] = -1;
		}
		conf->basic_rates = list;

		list = os_malloc(9 * sizeof(int));
		if (list) {
			list[0] = 60;
			list[1] = 90;
			list[2] = 120;
			list[3] = 180;
			list[4] = 240;
			list[5] = 360;
			list[6] = 480;
			list[7] = 540;
			list[8] = -1;
		}
		conf->supported_rates = list;
	}

	bss->isolate = !wpa_s->conf->p2p_intra_bss;
#ifdef	CONFIG_P2P_UNUSED_CMD
	bss->force_per_enrollee_psk = wpa_s->global->p2p_per_sta_psk;
#endif	/* CONFIG_P2P_UNUSED_CMD */

	if (ssid->p2p_group) {
		os_memcpy(bss->ip_addr_go, wpa_s->parent->conf->ip_addr_go, 4);
		os_memcpy(bss->ip_addr_mask, wpa_s->parent->conf->ip_addr_mask,
			  4);
		os_memcpy(bss->ip_addr_start,
			  wpa_s->parent->conf->ip_addr_start, 4);
		os_memcpy(bss->ip_addr_end, wpa_s->parent->conf->ip_addr_end,
			  4);
	}
#endif /* CONFIG_P2P */

#ifdef CONFIG_AP_ISOLATION
	bss->isolate = ssid->isolate;
#endif /* CONFIG_AP_ISOLATION */

	if (ssid->ssid_len == 0) {
		wpa_printf(MSG_ERROR, "No SSID configured for AP mode");
		return -1;
	}
	os_memcpy(bss->ssid.ssid, ssid->ssid, ssid->ssid_len);
	bss->ssid.ssid_len = ssid->ssid_len;
	bss->ssid.ssid_set = 1;

#ifdef CONFIG_OWE_AP
	if (ssid->key_mgmt == WPA_KEY_MGMT_OWE) {
		// check indicator
		os_strlcpy(bss->owe_transition_ifname, wpa_s->ifname, sizeof(bss->iface));

		os_memcpy(bss->owe_transition_ssid, ssid->ssid, ssid->ssid_len);
		bss->owe_transition_ssid_len = ssid->ssid_len;

		os_memcpy(bss->owe_transition_bssid, wpa_s->own_addr,ETH_ALEN);
#ifdef CONFIG_OWE_TRANS
		if(wpa_s->conf->owe_transition_mode == 1){	
			bss->owe_transition_mode = wpa_s->conf->owe_transition_mode;
			memset(bss->owe_transition_ifname,NULL,sizeof(bss->iface));
			os_strlcpy(bss->owe_transition_ifname, STA_DEVICE_NAME, sizeof(bss->iface));
			disc_byte_array(bss->owe_transition_bssid,ETH_ALEN);
		}	
#endif // CONFIG_OWE_TRANS
	}
#endif /* CONFIG_OWE_AP */

	bss->ignore_broadcast_ssid = ssid->ignore_broadcast_ssid;

	if (ssid->auth_alg)
		bss->auth_algs = ssid->auth_alg;

	if (wpa_key_mgmt_wpa_psk(ssid->key_mgmt))
		bss->wpa = ssid->proto;

#ifdef CONFIG_OWE_AP
	if (ssid->key_mgmt == WPA_KEY_MGMT_OWE) {
		bss->wpa = WPA_PROTO_RSN;
	}
#endif /* CONFIG_OWE_AP */
	bss->wpa_key_mgmt = ssid->key_mgmt;
	
	bss->wpa_pairwise = ssid->pairwise_cipher;

	if (ssid->psk_set) {
#ifdef	CONFIG_SUPP27_BIN_CLR_FREE
		bin_clear_free(bss->ssid.wpa_psk, sizeof(*bss->ssid.wpa_psk));
#else
		os_free(bss->ssid.wpa_psk);
#endif	/* CONFIG_SUPP27_BIN_CLR_FREE */
		bss->ssid.wpa_psk = os_zalloc(sizeof(struct hostapd_wpa_psk));
		if (bss->ssid.wpa_psk == NULL)
			return -1;
		os_memcpy(bss->ssid.wpa_psk->psk, ssid->psk, PMK_LEN);
		bss->ssid.wpa_psk->group = 1;
		bss->ssid.wpa_psk_set = 1;
	} else if (ssid->passphrase) {
		bss->ssid.wpa_passphrase = os_strdup(ssid->passphrase);

#ifdef CONFIG_AP_WEP_KEY
	} else if (ssid->wep_key_len[0] || ssid->wep_key_len[1] ||
		   ssid->wep_key_len[2] || ssid->wep_key_len[3]) {
		struct hostapd_wep_keys *wep = &bss->ssid.wep;
		int i;
		for (i = 0; i < NUM_WEP_KEYS; i++) {
			if (ssid->wep_key_len[i] == 0)
				continue;
			wep->key[i] = os_memdup(ssid->wep_key[i],
						ssid->wep_key_len[i]);
			if (wep->key[i] == NULL)
				return -1;
			wep->len[i] = ssid->wep_key_len[i];
		}
		wep->idx = ssid->wep_tx_keyidx;
		wep->keys_set = 1;
#endif /* CONFIG_AP_WEP_KEY */
	}

#ifdef CONFIG_SAE
	if ((ssid->key_mgmt | WPA_KEY_MGMT_SAE) && ssid->sae_password) {
		bss->ssid.wpa_passphrase = os_strdup(ssid->sae_password);
	} else if ((ssid->key_mgmt | WPA_KEY_MGMT_SAE) && ssid->passphrase) {
		bss->ssid.wpa_passphrase = os_strdup(ssid->passphrase);
	}

#if 0
	PRINTF("[%s] bss->ssid.wpa_passphrase = %s\n", __func__, bss->ssid.wpa_passphrase);
	PRINTF("[%s] ssid->sae_password  = %s\n", __func__, ssid->sae_password);
	PRINTF("[%s] ssid->passphrase    = %s\n", __func__, ssid->passphrase);
#endif /* 0 */
	
#endif /* CONFIG_SAE */

#ifdef CONFIG_INTERWORKING
	if (wpa_s->conf->go_interworking) {
		wpa_printf_dbg(MSG_DEBUG,
			   "P2P: Enable Interworking with access_network_type: %d",
			   wpa_s->conf->go_access_network_type);
#ifdef CONFIG_INTERWORKING
		bss->interworking = wpa_s->conf->go_interworking;
#endif /* CONFIG_INTERWORKING */
		bss->access_network_type = wpa_s->conf->go_access_network_type;
		bss->internet = wpa_s->conf->go_internet;
		if (wpa_s->conf->go_venue_group) {
			wpa_printf_dbg(MSG_DEBUG,
				   "P2P: Venue group: %d  Venue type: %d",
				   wpa_s->conf->go_venue_group,
				   wpa_s->conf->go_venue_type);
			bss->venue_group = wpa_s->conf->go_venue_group;
			bss->venue_type = wpa_s->conf->go_venue_type;
			bss->venue_info_set = 1;
		}
	}
#endif /* CONFIG_INTERWORKING */

#if 1
	if (ssid->ap_max_inactivity)
		bss->ap_max_inactivity = ssid->ap_max_inactivity;

#if 1	/* by Shingu 20161010 (Keep-alive) */
	if (wpa_s->conf->ap_send_ka)
		bss->ap_send_ka = wpa_s->conf->ap_send_ka;
#endif	/* 1 */

#else
	if (ssid->ap_max_inactivity)
		bss->ap_max_inactivity = ssid->ap_max_inactivity;
#endif /* 1 */

	if (ssid->dtim_period)
		bss->dtim_period = ssid->dtim_period;
	else if (wpa_s->conf->dtim_period)
		bss->dtim_period = wpa_s->conf->dtim_period;

	if (ssid->beacon_int)
		conf->beacon_int = ssid->beacon_int;
	else if (wpa_s->conf->beacon_int)
		conf->beacon_int = wpa_s->conf->beacon_int;

#ifdef CONFIG_P2P
#ifdef	CONFIG_P2P_UNUSED_CMD
	if (ssid->mode == WPAS_MODE_P2P_GO ||
	    ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION) {
		if (wpa_s->conf->p2p_go_ctwindow > conf->beacon_int) {
			wpa_printf(MSG_INFO,
				   "CTWindow (%d) is bigger than beacon interval (%d) - avoid configuring it",
				   wpa_s->conf->p2p_go_ctwindow,
				   conf->beacon_int);
			conf->p2p_go_ctwindow = 0;
		} else {
			conf->p2p_go_ctwindow = wpa_s->conf->p2p_go_ctwindow;
		}
	}
#endif	/* CONFIG_P2P_UNUSED_CMD */
#endif /* CONFIG_P2P */

	if ((bss->wpa & 2) && bss->rsn_pairwise == 0)
		bss->rsn_pairwise = bss->wpa_pairwise;
	bss->wpa_group = wpa_select_ap_group_cipher(bss->wpa, bss->wpa_pairwise,
						    bss->rsn_pairwise);

	if (bss->wpa && bss->ieee802_1x)
		bss->ssid.security_policy = SECURITY_WPA;
	else if (bss->wpa)
		bss->ssid.security_policy = SECURITY_WPA_PSK;
	else if (bss->ieee802_1x) {
		int cipher = WPA_CIPHER_NONE;
		bss->ssid.security_policy = SECURITY_IEEE_802_1X;
#ifdef CONFIG_AP_WEP_KEY
		bss->ssid.wep.default_len = bss->default_wep_key_len;
		if (bss->default_wep_key_len)
			cipher = bss->default_wep_key_len >= 13 ?
				WPA_CIPHER_WEP104 : WPA_CIPHER_WEP40;
#endif /* CONFIG_AP_WEP_KEY */
		bss->wpa_group = cipher;
		bss->wpa_pairwise = cipher;
		bss->rsn_pairwise = cipher;
#ifdef CONFIG_AP_WEP_KEY
	} else if (bss->ssid.wep.keys_set) {
		int cipher = WPA_CIPHER_WEP40;
		if (bss->ssid.wep.len[0] >= 13)
			cipher = WPA_CIPHER_WEP104;
		bss->ssid.security_policy = SECURITY_STATIC_WEP;
		bss->wpa_group = cipher;
		bss->wpa_pairwise = cipher;
		bss->rsn_pairwise = cipher;
#endif /* CONFIG_AP_WEP_KEY */
	} else {
		bss->ssid.security_policy = SECURITY_PLAINTEXT;
		bss->wpa_group = WPA_CIPHER_NONE;
		bss->wpa_pairwise = WPA_CIPHER_NONE;
		bss->rsn_pairwise = WPA_CIPHER_NONE;
	}

	if (bss->wpa_group_rekey < 86400 && (bss->wpa & 2) &&
	    (bss->wpa_group == WPA_CIPHER_CCMP ||
	     bss->wpa_group == WPA_CIPHER_GCMP ||
	     bss->wpa_group == WPA_CIPHER_CCMP_256 ||
	     bss->wpa_group == WPA_CIPHER_GCMP_256)) {
		/*
		 * Strong ciphers do not need frequent rekeying, so increase
		 * the default GTK rekeying period to 24 hours.
		 */
		bss->wpa_group_rekey = 86400;
	}

#ifdef CONFIG_IEEE80211W
	if (ssid->ieee80211w != MGMT_FRAME_PROTECTION_DEFAULT)
		bss->ieee80211w = ssid->ieee80211w;
#endif /* CONFIG_IEEE80211W */

#ifdef CONFIG_WPS
	/*
	 * Enable WPS by default for open and WPA/WPA2-Personal network, but
	 * require user interaction to actually use it. Only the internal
	 * Registrar is supported.
	 */
	if (bss->ssid.security_policy != SECURITY_WPA_PSK
#ifdef	CONFIG_AP_PLAIN_TEXT_SEC
	    && bss->ssid.security_policy != SECURITY_PLAINTEXT
#endif	/* CONFIG_AP_PLAIN_TEXT_SEC */
	   )
		goto no_wps;

	if (bss->ssid.security_policy == SECURITY_WPA_PSK &&
	    (!(bss->rsn_pairwise & (WPA_CIPHER_CCMP | WPA_CIPHER_GCMP)) ||
	     !(bss->wpa & 2)))
		goto no_wps; /* WPS2 does not allow WPA/TKIP-only
			      * configuration */

	if (ssid->wps_disabled
#if defined CONFIG_SAE
		|| ssid->key_mgmt & WPA_KEY_MGMT_SAE
#endif // CONFIG_SAE
#if defined CONFIG_OWE
		|| ssid->key_mgmt & WPA_KEY_MGMT_OWE
#endif // CONFIG_OWE 
		)
		goto no_wps;
	bss->eap_server = 1;

	if (!ssid->ignore_broadcast_ssid)
		bss->wps_state = 2;

	bss->ap_setup_locked = 2;
	if (wpa_s->conf->config_methods)
		bss->config_methods = os_strdup(wpa_s->conf->config_methods);
	os_memcpy(bss->device_type, wpa_s->conf->device_type,
		  WPS_DEV_TYPE_LEN);
	if (wpa_s->conf->device_name) {
		bss->device_name = os_strdup(wpa_s->conf->device_name);
		bss->friendly_name = os_strdup(wpa_s->conf->device_name);
	}
	if (wpa_s->conf->manufacturer)
		bss->manufacturer = os_strdup(wpa_s->conf->manufacturer);
	if (wpa_s->conf->model_name)
		bss->model_name = os_strdup(wpa_s->conf->model_name);
	if (wpa_s->conf->model_number)
		bss->model_number = os_strdup(wpa_s->conf->model_number);
	if (wpa_s->conf->serial_number)
		bss->serial_number = os_strdup(wpa_s->conf->serial_number);
	if (is_nil_uuid(wpa_s->conf->uuid))
		os_memcpy(bss->uuid, wpa_s->wps->uuid, WPS_UUID_LEN);
	else
		os_memcpy(bss->uuid, wpa_s->conf->uuid, WPS_UUID_LEN);
	os_memcpy(bss->os_version, wpa_s->conf->os_version, 4);
	bss->pbc_in_m1 = wpa_s->conf->pbc_in_m1;
	if (ssid->eap.fragment_size != DEFAULT_FRAGMENT_SIZE)
		bss->fragment_size = ssid->eap.fragment_size;
no_wps:
#endif /* CONFIG_WPS */

	if (wpa_s->max_stations &&
	    wpa_s->max_stations < wpa_s->conf->max_num_sta)
		bss->max_num_sta = wpa_s->max_stations;
	else
		bss->max_num_sta = wpa_s->conf->max_num_sta;

	if (!bss->isolate)
		bss->isolate = wpa_s->conf->ap_isolate;

	bss->disassoc_low_ack = wpa_s->conf->disassoc_low_ack;

#ifdef	CONFIG_AP_VENDOR_ELEM
	if (wpa_s->conf->ap_vendor_elements) {
		bss->vendor_elements =
			wpabuf_dup(wpa_s->conf->ap_vendor_elements);
	}
#endif	/* CONFIG_AP_VENDOR_ELEM */

	bss->ftm_responder = wpa_s->conf->ftm_responder;
	bss->ftm_initiator = wpa_s->conf->ftm_initiator;

#if 0
	hostapd_ctrl_iface_read_config(conf);
#endif /* 0 */
#ifdef CONFIG_AP_PARAMETERS /* munchang.jung_20160523 */
	conf->bss[0]->set_greenfield = wpa_s->conf->greenfield;
#endif /* CONFIG_AP_PARAMETERS */

	return 0;
}


static void ap_public_action_rx(void *ctx, const u8 *buf, size_t len, int freq)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	const struct _ieee80211_mgmt *mgmt;

	mgmt = (const struct _ieee80211_mgmt *) buf;
	if (len < IEEE80211_HDRLEN + 1)
		return;
	if (mgmt->u.action.category != WLAN_ACTION_PUBLIC)
		return;
	wpas_p2p_rx_action(wpa_s, mgmt->da, mgmt->sa, mgmt->bssid,
			   mgmt->u.action.category,
			   buf + IEEE80211_HDRLEN + 1,
			   len - IEEE80211_HDRLEN - 1, freq);
#endif /* CONFIG_P2P */
}


static void ap_wps_event_cb(void *ctx, enum wps_event event,
			    union wps_event_data *data)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;

	if (event == WPS_EV_FAIL) {
		struct wps_event_fail *fail = &data->fail;

		if (wpa_s->parent && wpa_s->parent != wpa_s &&
		    wpa_s == wpa_s->global->p2p_group_formation) {
			/*
			 * src/ap/wps_hostapd.c has already sent this on the
			 * main interface, so only send on the parent interface
			 * here if needed.
			 */
			wpa_msg(wpa_s->parent, MSG_INFO, WPS_EVENT_FAIL
				"msg=%d config_error=%d",
				fail->msg, fail->config_error);
		}
		wpas_p2p_wps_failed(wpa_s, fail);
	}
#endif /* CONFIG_P2P */
}


#ifdef CONFIG_P2P
static void ap_sta_authorized_cb(void *ctx, const u8 *mac_addr)
{
	struct wpa_supplicant *wpa_s = ctx;
	wpas_p2p_notify_ap_sta_authorized(wpa_s, mac_addr);
}


static void ap_new_psk_cb(void *ctx, const u8 *mac_addr, const u8 *p2p_dev_addr,
			  const u8 *psk, size_t psk_len)
{

	struct wpa_supplicant *wpa_s = ctx;
	if (wpa_s->ap_iface == NULL || wpa_s->current_ssid == NULL)
		return;
	wpas_p2p_new_psk_cb(wpa_s, mac_addr, p2p_dev_addr, psk, psk_len);
}
#endif /* CONFIG_P2P */


static int ap_vendor_action_rx(void *ctx, const u8 *buf, size_t len, int freq)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	const struct _ieee80211_mgmt *mgmt;

	mgmt = (const struct _ieee80211_mgmt *) buf;
	if (len < IEEE80211_HDRLEN + 1)
		return -1;
	wpas_p2p_rx_action(wpa_s, mgmt->da, mgmt->sa, mgmt->bssid,
			   mgmt->u.action.category,
			   buf + IEEE80211_HDRLEN + 1,
			   len - IEEE80211_HDRLEN - 1, freq);
#endif /* CONFIG_P2P */
	return 0;
}


static int ap_probe_req_rx(void *ctx, const u8 *sa, const u8 *da,
			   const u8 *bssid, const u8 *ie, size_t ie_len,
			   int ssi_signal)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
//	unsigned int freq = 0;

//	if (wpa_s->ap_iface)
//		freq = wpa_s->ap_iface->freq;

	return wpas_p2p_probe_req_rx(wpa_s, sa, da, bssid, ie, ie_len,
				     ssi_signal);
#else /* CONFIG_P2P */
	return 0;
#endif /* CONFIG_P2P */
}


static void ap_wps_reg_success_cb(void *ctx, const u8 *mac_addr,
				  const u8 *uuid_e)
{
#ifdef CONFIG_P2P
	struct wpa_supplicant *wpa_s = ctx;
	wpas_p2p_wps_success(wpa_s, mac_addr, 1);
#endif /* CONFIG_P2P */
}


static void wpas_ap_configured_cb(void *ctx)
{
	struct wpa_supplicant *wpa_s = ctx;

	wpa_printf_dbg(MSG_DEBUG, "AP interface setup completed - state %s",
		   hostapd_state_text(wpa_s->ap_iface->state));

	if (wpa_s->ap_iface->state == HAPD_IFACE_DISABLED) {
		wpa_supplicant_ap_deinit(wpa_s);
		return;
	}

#ifdef CONFIG_ACS
	if (wpa_s->current_ssid && wpa_s->current_ssid->acs) {
		wpa_s->assoc_freq = wpa_s->ap_iface->freq;
		wpa_s->current_ssid->frequency = wpa_s->ap_iface->freq;
	}
#endif /* CONFIG_ACS */

	wpa_supplicant_set_state(wpa_s, WPA_COMPLETED);

	if (wpa_s->ap_configured_cb)
		wpa_s->ap_configured_cb(wpa_s->ap_configured_cb_ctx,
					wpa_s->ap_configured_cb_data);

#if 1 /* SoftAP ACS */
	if (wpa_s->conf->ssid->mode == WPAS_MODE_AP && wpa_s->conf->ssid->frequency == 0) {
		//PRINTF("[%s] mode=%d\n", __func__, wpa_s->conf->ssid->mode);
		supplicant_done();
	}
#endif /* SoftAP ACS */

    if (is_in_softap_acs_mode()) {
        atcmd_asynchony_event(10, NULL); // send +INIT:DONE,1
    }
}


int wpa_supplicant_create_ap(struct wpa_supplicant *wpa_s,
			     struct wpa_ssid *ssid)
{
	struct wpa_driver_associate_params params;
	struct hostapd_iface *hapd_iface;
	struct hostapd_config *conf;
	size_t i;

	if (ssid->ssid == NULL || ssid->ssid_len == 0) {
		wpa_printf(MSG_ERROR, "No SSID configured for AP mode");
		return -1;
	}

	wpa_supplicant_ap_deinit(wpa_s);

	wpa_printf_dbg(MSG_DEBUG, "Setting up AP (SSID='%s')",
		   wpa_ssid_txt(ssid->ssid, ssid->ssid_len));

	os_memset(&params, 0, sizeof(params));
	params.ssid = ssid->ssid;
	params.ssid_len = ssid->ssid_len;

	switch (ssid->mode) {
		case WPAS_MODE_AP:
#ifdef	CONFIG_P2P
		case WPAS_MODE_P2P_GO:
		case WPAS_MODE_P2P_GROUP_FORMATION:
#endif	/* CONFIG_P2P */
			params.mode = IEEE80211_MODE_AP;
			break;

#ifdef CONFIG_MESH
		case WPAS_MODE_MESH:
			params.mode = IEEE80211_MODE_MESH;
			break;		
#endif /* CONFIG_MESH */

		default:
			return -1;
	}

#if 1 // FC9000 Only
#ifdef CONFIG_ACS
	if (ssid->frequency == 0) {
		da16x_notice_prt(">>> AP Operating Channel: AUTO\n");
	}
	else
#endif /* CONFIG_ACS */
	{
		int startCH, endCH;
		extern int country_channel_range (char* country, int *startCH, int *endCH);
		
		country_channel_range(wpa_s->conf->country, &startCH, &endCH);
		
		if (i3ed11_freq_to_ch(ssid->frequency) < startCH
			|| i3ed11_freq_to_ch(ssid->frequency) > endCH) {
#ifdef CONFIG_ACS
			da16x_notice_prt(">>> AP ch Err: %d(%d) CCODE: %s, Change to AUTO ch\n",
				i3ed11_freq_to_ch(ssid->frequency), ssid->frequency, wpa_s->conf->country);
#else
			da16x_notice_prt(">>> AP ch Err: %d(%d) CCODE: %s, Change to default ch\n",
				i3ed11_freq_to_ch(ssid->frequency), ssid->frequency, wpa_s->conf->country);
#endif /* CONFIG_ACS */
			ssid->frequency = 0;
		} else {	
			da16x_notice_prt(">>> AP Operating Channel: %d(%d)\n",
				i3ed11_freq_to_ch(ssid->frequency), ssid->frequency);
		}
	}
#endif // FC9000 Only

#ifndef CONFIG_ACS
	if (ssid->frequency == 0)
		ssid->frequency = 2462; /* default channel 11 */
#endif /* CONFIG_ACS */

	params.freq.freq = ssid->frequency;

	params.wpa_proto = ssid->proto;
	if (ssid->key_mgmt & WPA_KEY_MGMT_PSK)
		wpa_s->key_mgmt = WPA_KEY_MGMT_PSK;
	else
		wpa_s->key_mgmt = WPA_KEY_MGMT_NONE;
	params.key_mgmt_suite = wpa_s->key_mgmt;

	wpa_s->pairwise_cipher = wpa_pick_pairwise_cipher(ssid->pairwise_cipher,
							  1);
	if (wpa_s->pairwise_cipher < 0) {
		da16x_err_prt("[%s] WPA: Failed to select pairwise cipher.\n",
			     __func__);
		return -1;
	}
	params.pairwise_suite = wpa_s->pairwise_cipher;
	params.group_suite = params.pairwise_suite;

#ifdef CONFIG_P2P
	if (ssid->mode == WPAS_MODE_P2P_GO ||
	    ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)
		params.p2p = 1;
#endif /* CONFIG_P2P */

	if (wpa_s->parent->set_ap_uapsd)
		params.uapsd = wpa_s->parent->ap_uapsd;
	else if (params.p2p && (wpa_s->drv_flags & WPA_DRIVER_FLAGS_AP_UAPSD))
		params.uapsd = 1; /* mandatory for P2P GO */
	else
		params.uapsd = -1;

#ifdef	CONFIG_P2P_UNUSED_CMD
	if (ieee80211_is_dfs(params.freq.freq, wpa_s->hw.modes,
			     wpa_s->hw.num_modes))
		params.freq.freq = 0; /* set channel after CAC */

	if (params.p2p)
		wpa_drv_get_ext_capa(wpa_s, WPA_IF_P2P_GO);
	else
		wpa_drv_get_ext_capa(wpa_s, WPA_IF_AP_BSS);
#endif	/* CONFIG_P2P_UNUSED_CMD */

	if (wpa_drv_associate(wpa_s, &params) < 0) {
		da16x_fatal_prt("[%s] Failed to start wpa_drv_addociate\n",
					__func__);
		return -1;
	}

	wpa_s->ap_iface = hapd_iface = hostapd_alloc_iface();
	if (hapd_iface == NULL)
		return -1;
	hapd_iface->owner = wpa_s;
	hapd_iface->drv_flags = wpa_s->drv_flags;
#ifdef	CONFIG_SUPP27_WPA_DRV_SMPS_MODE
	hapd_iface->smps_modes = wpa_s->drv_smps_modes;
#endif	/* CONFIG_SUPP27_WPA_DRV_SMPS_MODE */
	hapd_iface->probe_resp_offloads = wpa_s->probe_resp_offloads;
	hapd_iface->extended_capa = wpa_s->extended_capa;
	hapd_iface->extended_capa_mask = wpa_s->extended_capa_mask;
	hapd_iface->extended_capa_len = wpa_s->extended_capa_len;

	wpa_s->ap_iface->conf = conf = hostapd_config_defaults();
	if (conf == NULL) {
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

#ifdef	CONFIG_AP_WMM
	wpa_s->ap_iface->conf->bss[0]->wmm_enabled =
		wpa_s->conf->hostapd_wmm_enabled;

	wpa_s->ap_iface->conf->bss[0]->wmm_uapsd =
		wpa_s->conf->hostapd_wmm_ps_enabled;

	os_memcpy(wpa_s->ap_iface->conf->wmm_ac_params,
		  wpa_s->conf->wmm_ac_params,
		  sizeof(wpa_s->conf->wmm_ac_params));

	if (params.uapsd > 0) {
		conf->bss[0]->wmm_enabled = 1;
		conf->bss[0]->wmm_uapsd = 1;
	}
#endif	/* CONFIG_AP_WMM */

	if (wpa_supplicant_conf_ap(wpa_s, ssid, conf)) {
		da16x_err_prt("[%s] Failed to create AP configuration\n",
					__func__);
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

#ifdef CONFIG_P2P
	if (ssid->mode == WPAS_MODE_P2P_GO)
		conf->bss[0]->p2p = P2P_ENABLED | P2P_GROUP_OWNER;
	else if (ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)
		conf->bss[0]->p2p = P2P_ENABLED | P2P_GROUP_OWNER |
			P2P_GROUP_FORMATION;
#endif /* CONFIG_P2P */

#ifdef CONFIG_OWE_TRANS
	if (wpa_s->conf->owe_transition_mode == 1) {
		conf->num_bss = 2;
	}	
#endif // CONFIG_OWE_TRANS

	hapd_iface->num_bss = conf->num_bss;
	hapd_iface->bss = os_calloc(conf->num_bss,
				    sizeof(struct hostapd_data *));
	if (hapd_iface->bss == NULL) {
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

	for (i = 0; i < conf->num_bss; i++) {
		hapd_iface->bss[i] = hostapd_alloc_bss_data(hapd_iface, conf, conf->bss[i]);

		if (hapd_iface->bss[i] == NULL) {
			wpa_supplicant_ap_deinit(wpa_s);
			return -1;
		}

		hapd_iface->bss[i]->msg_ctx = wpa_s;
		hapd_iface->bss[i]->msg_ctx_parent = wpa_s->parent;
		hapd_iface->bss[i]->public_action_cb = ap_public_action_rx;
		hapd_iface->bss[i]->public_action_cb_ctx = wpa_s;
		hapd_iface->bss[i]->vendor_action_cb = ap_vendor_action_rx;
		hapd_iface->bss[i]->vendor_action_cb_ctx = wpa_s;
		hostapd_register_probereq_cb(hapd_iface->bss[i],
					     ap_probe_req_rx, wpa_s);
		hapd_iface->bss[i]->wps_reg_success_cb = ap_wps_reg_success_cb;
		hapd_iface->bss[i]->wps_reg_success_cb_ctx = wpa_s;
		hapd_iface->bss[i]->wps_event_cb = ap_wps_event_cb;
		hapd_iface->bss[i]->wps_event_cb_ctx = wpa_s;
#ifdef CONFIG_P2P
		hapd_iface->bss[i]->sta_authorized_cb = ap_sta_authorized_cb;
		hapd_iface->bss[i]->sta_authorized_cb_ctx = wpa_s;
		hapd_iface->bss[i]->new_psk_cb = ap_new_psk_cb;
		hapd_iface->bss[i]->new_psk_cb_ctx = wpa_s;
		hapd_iface->bss[i]->p2p = wpa_s->global->p2p;
		hapd_iface->bss[i]->p2p_group = wpas_p2p_group_init(wpa_s,
								    ssid);
#endif /* CONFIG_P2P */
		hapd_iface->bss[i]->setup_complete_cb = wpas_ap_configured_cb;
		hapd_iface->bss[i]->setup_complete_cb_ctx = wpa_s;
#ifdef CONFIG_TESTING_OPTIONS
		hapd_iface->bss[i]->ext_eapol_frame_io =
			wpa_s->ext_eapol_frame_io;
#endif /* CONFIG_TESTING_OPTIONS */
	}

	os_memcpy(hapd_iface->bss[0]->own_addr, wpa_s->own_addr, ETH_ALEN);
	hapd_iface->bss[0]->driver = wpa_s->driver;
	hapd_iface->bss[0]->drv_priv = wpa_s->drv_priv;

#ifdef CONFIG_OWE_TRANS
	if (conf->num_bss == 2) {
		hapd_iface->bss[1]->driver = wpa_s->parent->driver;
		hapd_iface->bss[1]->drv_priv = wpa_s->parent->drv_priv;
	}
#endif // CONFIG_OWE_TRANS
	
	wpa_s->current_ssid = ssid;
	eapol_sm_notify_config(wpa_s->eapol, NULL, NULL);
	os_memcpy(wpa_s->bssid, wpa_s->own_addr, ETH_ALEN);
	wpa_s->assoc_freq = ssid->frequency;

#ifdef CONFIG_AP_PARAMETERS
	hapd_iface->ht_op_mode = wpa_s->conf->ht_protection;
	hapd_iface->bss[0]->iconf->rts_threshold = wpa_s->conf->rts_threshold;
#endif /* CONFIG_AP_PARAMETERS */

#if 0 // defined(CONFIG_P2P) && defined(CONFIG_ACS)
	if (wpa_s->p2p_go_do_acs) {
		wpa_s->ap_iface->conf->channel = 0;
#if 0	/* by Bright : Merge 2.7 */
		wpa_s->ap_iface->conf->hw_mode = wpa_s->p2p_go_acs_band;
#endif	/* 0 */
		ssid->acs = 1;
	}
#endif /* CONFIG_P2P && CONFIG_ACS */

	if (hostapd_setup_interface(wpa_s->ap_iface)) {
		da16x_fatal_prt("[%s] Failed to init AP interface\n",
					__func__);
		wpa_supplicant_ap_deinit(wpa_s);
		return -1;
	}

#ifdef	CONFIG_ACL
	if (hostapd_ap_acl_init(wpa_s) < 0) {
		return -1;
	}
#endif	/* CONFIG_ACL */

#ifdef CONFIG_AP_POWER
	if (ssid->ap_power != NULL) {
		ap_ctrl_iface_set_ap_power(wpa_s, ssid->ap_power);
	}
#endif /* CONFIG_AP_POWER */
	//[[ trinity_0170201_BEGIN -- p2p go mode crash bugfix
	wpa_s->ap_active = 1;
	//]] trinity_0170201_END

#ifdef	CONFIG_P2P_POWER_SAVE /* by Shingu 20180208 (P2P PS opt.) */
	if (get_run_mode() == P2P_GO_FIXED_MODE)
		hostapd_p2p_default_noa(hapd_iface->bss[0], 1);
	else if (get_run_mode() == P2P_ONLY_MODE)
		hostapd_p2p_default_noa(hapd_iface->bss[0], 0);
#endif	/* CONFIG_P2P_POWER_SAVE */

#ifdef CONFIG_OWE_TRANS
	if ((conf->num_bss == 2) && (wpa_s->parent->ap_iface == NULL)) {
		wpa_s->parent->ap_iface = wpa_s->ap_iface;
	}	
#endif // CONFIG_OWE_TRANS

	return 0;
}


void wpa_supplicant_ap_deinit(struct wpa_supplicant *wpa_s)
{
#ifdef CONFIG_WPS
#ifdef CONFIG_WPS_PIN
	da16x_eloop_cancel_timeout(wpas_wps_ap_pin_timeout, wpa_s, NULL);
#else
	da16x_ap_prt("[%s] Have to cancel timer wpas_wps_ap_pin_timeout()\n",
				__func__);
#endif	/* CONFIG_WPS_PIN */
#endif /* CONFIG_WPS */

	if (wpa_s->ap_iface == NULL)
		return;

	wpa_s->current_ssid = NULL;
	eapol_sm_notify_config(wpa_s->eapol, NULL, NULL);
	wpa_s->assoc_freq = 0;
#ifdef CONFIG_P2P
	if (wpa_s->global->p2p) {
		if (wpa_s->ap_iface->bss)
			wpa_s->ap_iface->bss[0]->p2p_group = NULL;
		wpas_p2p_group_deinit(wpa_s);
	}
#endif /* CONFIG_P2P */
	wpa_s->ap_iface->driver_ap_teardown =
		!!(wpa_s->drv_flags & WPA_DRIVER_FLAGS_AP_TEARDOWN_SUPPORT);

	hostapd_interface_deinit(wpa_s->ap_iface);
#if 0	/* by Shingu 20160622 (P2P GO Inactivity) */
	if (hostapd_drv_chk_deauth_send_done(wpa_s->ap_iface->bss[0]) < 0)
		da16x_wnm_prt("[%s] Deauthentication packet send fail\n",
			     __func__);
#endif	/* 0 */

	hostapd_interface_free(wpa_s->ap_iface);
	wpa_s->ap_iface = NULL;
	//[[ trinity_0170201_BEGIN -- p2p go mode crash bugfix
	wpa_s->ap_active = 0;
	//]] trinity_0170201_END
#if 1
	if (wpa_drv_deinit_ap(wpa_s) != 0)
	{
		da16x_fatal_prt("[%s] ap deinit falil\n", __func__);
	}
#else
	wpa_drv_deinit_ap(wpa_s);
#endif /* 1 */
}


void ap_tx_status(void *ctx, const u8 *addr,
		  const u8 *buf, size_t len, int ack)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	hostapd_tx_status(wpa_s->ap_iface->bss[0], addr, buf, len, ack);
#endif /* NEED_AP_MLME */
}


void ap_eapol_tx_status(void *ctx, const u8 *dst,
			const u8 *data, size_t len, int ack)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	if (!wpa_s->ap_iface)
		return;
	hostapd_tx_status(wpa_s->ap_iface->bss[0], dst, data, len, ack);
#endif /* NEED_AP_MLME */
}


void ap_client_poll_ok(void *ctx, const u8 *addr)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	if (wpa_s->ap_iface)
		hostapd_client_poll_ok(wpa_s->ap_iface->bss[0], addr);
#endif /* NEED_AP_MLME */
}


void ap_rx_from_unknown_sta(void *ctx, const u8 *addr, int wds)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	ieee802_11_rx_from_unknown(wpa_s->ap_iface->bss[0], addr, wds);
#endif /* NEED_AP_MLME */
}


#if 1 /* FC9000 Only */
int ap_mgmt_rx(void *ctx, struct rx_mgmt *rx_mgmt)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	struct hostapd_frame_info fi;
	os_memset(&fi, 0, sizeof(fi));
	fi.datarate = rx_mgmt->datarate;
	fi.ssi_signal = rx_mgmt->ssi_signal;
#ifdef CONFIG_OWE_TRANS
	struct hostapd_data *hapd;

	hapd = wpa_s->ap_iface->bss[0];

	if(os_memcmp(wpa_s->own_addr, hapd->own_addr, ETH_ALEN) !=0)
		hapd = hapd->iface->bss[1];
	else
		hapd =hapd->iface->bss[0];
	
	return ieee802_11_mgmt(hapd, rx_mgmt->frame,
		rx_mgmt->frame_len, &fi); 
#else
	return ieee802_11_mgmt(wpa_s->ap_iface->bss[0], rx_mgmt->frame,
			rx_mgmt->frame_len, &fi);
#endif // CONFIG_OWE_TRANS
#endif /* NEED_AP_MLME */
}

#else

void ap_mgmt_rx(void *ctx, struct rx_mgmt *rx_mgmt)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	struct hostapd_frame_info fi;
	os_memset(&fi, 0, sizeof(fi));
	fi.datarate = rx_mgmt->datarate;
	fi.ssi_signal = rx_mgmt->ssi_signal;
	ieee802_11_mgmt(wpa_s->ap_iface->bss[0], rx_mgmt->frame,
			rx_mgmt->frame_len, &fi);
#endif /* NEED_AP_MLME */
}
#endif /* FC9000 Only */


void ap_mgmt_tx_cb(void *ctx, const u8 *buf, size_t len, u16 stype, int ok)
{
#ifdef NEED_AP_MLME
	struct wpa_supplicant *wpa_s = ctx;
	ieee802_11_mgmt_cb(wpa_s->ap_iface->bss[0], buf, len, stype, ok);
#endif /* NEED_AP_MLME */
}


void wpa_supplicant_ap_rx_eapol(struct wpa_supplicant *wpa_s,
				const u8 *src_addr, const u8 *buf, size_t len)
{
	ieee802_1x_receive(wpa_s->ap_iface->bss[0], src_addr, buf, len);
}


#ifdef CONFIG_WPS

int wpa_supplicant_ap_wps_pbc(struct wpa_supplicant *wpa_s, const u8 *bssid,
			      const u8 *p2p_dev_addr)
{
	if (!wpa_s->ap_iface)
		return -1;
	return hostapd_wps_button_pushed(wpa_s->ap_iface->bss[0],
					 p2p_dev_addr);
}


int wpa_supplicant_ap_wps_cancel(struct wpa_supplicant *wpa_s)
{
	struct wps_registrar *reg;
	int reg_sel = 0, wps_sta = 0;

	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0]->wps)
		return -1;

	reg = wpa_s->ap_iface->bss[0]->wps->registrar;
	reg_sel = wps_registrar_wps_cancel(reg);
	wps_sta = ap_for_each_sta(wpa_s->ap_iface->bss[0],
				  ap_sta_wps_cancel, NULL);

	if (!reg_sel && !wps_sta) {
		da16x_ap_prt("[%s] No WPS operation in progress at this time\n",
				__func__);
		return -1;
	}

	/*
	 * There are 2 cases to return wps cancel as success:
	 * 1. When wps cancel was initiated but no connection has been
	 *    established with client yet.
	 * 2. Client is in the middle of exchanging WPS messages.
	 */

	return 0;
}


int wpa_supplicant_ap_wps_pin(struct wpa_supplicant *wpa_s, const u8 *bssid,
                              const char *pin, char *buf, size_t buflen,
                              int timeout)
{
        int ret, ret_len = 0;

        if (!wpa_s->ap_iface)
                return -1;

        if (pin == NULL) {
                unsigned int rpin = wps_generate_pin();
                ret_len = os_snprintf(buf, buflen, "%08d", rpin);
                pin = buf;
        } else {
                ret_len = os_snprintf(buf, buflen, "%s", pin);
        }

        ret = hostapd_wps_add_pin(wpa_s->ap_iface->bss[0], bssid, "any", pin,
                                  timeout);
        if (ret)
                return -1;

        return ret_len;
}

#ifdef CONFIG_WPS_PIN
static void wpas_wps_ap_pin_timeout(void *eloop_data, void *user_ctx)
{
        struct wpa_supplicant *wpa_s = eloop_data;
        da16x_ap_prt("[%s] WPS: AP PIN timed out\n", __func__);
        wpas_wps_ap_pin_disable(wpa_s);
}

static void wpas_wps_ap_pin_enable(struct wpa_supplicant *wpa_s, int timeout)
{
        struct hostapd_data *hapd;

        if (wpa_s->ap_iface == NULL)
                return;
        hapd = wpa_s->ap_iface->bss[0];

        da16x_ap_prt("[%s] WPS: Enabling AP PIN (timeout=%d)\n",
                        __func__, timeout);
        hapd->ap_pin_failures = 0;

        da16x_eloop_cancel_timeout(wpas_wps_ap_pin_timeout, wpa_s, NULL);

        if (timeout > 0) {
                da16x_eloop_register_timeout(timeout, 0,
                                       wpas_wps_ap_pin_timeout, wpa_s, NULL);
        }
}


void wpas_wps_ap_pin_disable(struct wpa_supplicant *wpa_s)
{
        struct hostapd_data *hapd;

        if (wpa_s->ap_iface == NULL)
                return;

        da16x_ap_prt("[%s] WPS: Disabling AP PIN\n", __func__);

        hapd = wpa_s->ap_iface->bss[0];
        os_free(hapd->conf->ap_pin);
        hapd->conf->ap_pin = NULL;

        da16x_eloop_cancel_timeout(wpas_wps_ap_pin_timeout, wpa_s, NULL);
}

const char * wpas_wps_ap_pin_random(struct wpa_supplicant *wpa_s, int timeout)
{
        struct hostapd_data *hapd;
        unsigned int pin;
        char pin_txt[9];

        if (wpa_s->ap_iface == NULL)
                return NULL;
        hapd = wpa_s->ap_iface->bss[0];
        pin = wps_generate_pin();
        os_snprintf(pin_txt, sizeof(pin_txt), "%08u", pin);
        os_free(hapd->conf->ap_pin);
        hapd->conf->ap_pin = os_strdup(pin_txt);
        if (hapd->conf->ap_pin == NULL)
                return NULL;
        wpas_wps_ap_pin_enable(wpa_s, timeout);

        return hapd->conf->ap_pin;
}


const char * wpas_wps_ap_pin_get(struct wpa_supplicant *wpa_s)
{
        struct hostapd_data *hapd;
        if (wpa_s->ap_iface == NULL)
                return NULL;
        hapd = wpa_s->ap_iface->bss[0];
        return hapd->conf->ap_pin;
}

int wpas_wps_ap_pin_set(struct wpa_supplicant *wpa_s, const char *pin,
                        int timeout)
{
        struct hostapd_data *hapd;
        char pin_txt[9];
        int ret;

        if (wpa_s->ap_iface == NULL)
                return -1;
        hapd = wpa_s->ap_iface->bss[0];
        ret = os_snprintf(pin_txt, sizeof(pin_txt), "%s", pin);
        if (ret < 0 || ret >= (int) sizeof(pin_txt))
                return -1;
        os_free(hapd->conf->ap_pin);
        hapd->conf->ap_pin = os_strdup(pin_txt);
        if (hapd->conf->ap_pin == NULL)
                return -1;
        wpas_wps_ap_pin_enable(wpa_s, timeout);

        return 0;
}
#endif /* CONFIG_WPS */


void wpa_supplicant_ap_pwd_auth_fail(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return;
	hapd = wpa_s->ap_iface->bss[0];

	/*
	 * Registrar failed to prove its knowledge of the AP PIN. Disable AP
	 * PIN if this happens multiple times to slow down brute force attacks.
	 */
	hapd->ap_pin_failures++;

	da16x_ap_prt("[%s] WPS: AP PIN authentication failure number %u\n",
			__func__, hapd->ap_pin_failures);

	if (hapd->ap_pin_failures < 3)
		return;

	da16x_ap_prt("[%s] WPS: Disable AP PIN\n", __func__);

	hapd->ap_pin_failures = 0;
	os_free(hapd->conf->ap_pin);
	hapd->conf->ap_pin = NULL;
}
#endif /* CONFIG_WPS_PIN */

int ap_ctrl_iface_set_country(struct wpa_supplicant *wpa_s,
		char *country)
{
	struct hostapd_data *hapd;
	struct hostapd_config *conf;

	if (*country == 0)
		return -1;

	if (wpa_s->ap_iface == NULL)
		return -1;

	hapd = wpa_s->ap_iface->bss[0];
	os_memset(hapd->iconf->country, NULL, sizeof(conf->country));
	os_memcpy(hapd->iconf->country, country, sizeof(conf->country));
	return 0;
}

char * ap_ctrl_iface_get_country(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;
	struct hostapd_config *conf;

	if (wpa_s->ap_iface == NULL)
		return NULL;
	hapd = wpa_s->ap_iface->bss[0];
	conf = hapd->iconf;

	return conf->country;
}

#if 1	/* by Shingu 20160901 (Concurrent) */
int ap_ctrl_iface_ap(struct wpa_supplicant *wpa_s, char *cmd, char *buf, size_t buflen)
{
	struct wpa_ssid *ssid_p = NULL;
	int set_freq = 0;
	
	if (!wpa_s->conf->ssid ||
	    (os_strcmp(wpa_s->ifname, SOFTAP_DEVICE_NAME) != 0))
		return -1;

	for (ssid_p = wpa_s->conf->ssid ; ssid_p ; ssid_p = ssid_p->next)
	{
		if (ssid_p->ssid == NULL || ssid_p->ssid_len == 0) {
			da16x_ap_prt("[%s] No SSID configured for AP mode\n", __func__);
			continue;
		} else if (ssid_p->mode != WPAS_MODE_AP) {
			da16x_ap_prt("[%s] Not AP mode\n", __func__);
			continue;
		}
		break;
	}

	if (ssid_p == NULL) {
		return -1;
	}

	if (os_strcasecmp(cmd, "START") == 0) {
		if (wpa_s->ap_iface) {
			da16x_err_prt("AP iface is already running.\n");
			return -1;
		}

		set_freq = ssid_p->frequency;
	} else if (os_strcasecmp(cmd, "STOP") == 0) {
		if (!wpa_s->ap_iface) {
			da16x_err_prt("NO AP iface.\n");
			return -1;
		}
	} else if (os_strcasecmp(cmd, "RESTART") == 0) {
#if 1
		set_freq = ssid_p->frequency;
#else
		/*  Keep current channel. To reduce LMAC's burden caused by ACS procedure. 2020/Oct/28th. */
		set_freq = i3ed11_ch_to_freq(wpa_s->ap_iface->conf->channel, I3ED11_BAND_2GHZ);
#endif
	} else {
		int value = (int)ctoi(cmd);
		
		if (value > 14) // frequency
		{
			value = i3ed11_freq_to_ch(value);
		}

		if (value >= 1 && value <= 14) { // channel
			if (chk_channel_by_country(wpa_s->conf->country, value, 0, NULL) < 0)
			{
				return -1;
			}

			set_freq = i3ed11_ch_to_freq(value, I3ED11_BAND_2GHZ);

			if (set_freq != 0 && set_freq == ssid_p->frequency) {
				da16x_err_prt("Same Frequency\n");
				return -1;
			}
		} else {
			da16x_err_prt("Wrong argument !!!\n");
			return -1;
		}
	}

	/* from wpa_supp_ctrl_iface_remove_network() */
	if (wpa_s->current_ssid->mode == WPAS_MODE_AP) {
		//ap_ctrl_iface_sta_disassociate(wpa_s, "FF:FF:FF:FF:FF:FF reason=3");
		ap_ctrl_iface_sta_deauthenticate(wpa_s, "FF:FF:FF:FF:FF:FF reason=3");
		vTaskDelay(20);
	}

	if (os_strcasecmp(cmd, "STOP") == 0) {
		wpa_supplicant_ap_deinit(wpa_s);
		goto end;
	}

#ifdef	CONFIG_CONCURRENT
	if (   get_run_mode() == STA_SOFT_AP_CONCURRENT_MODE
#if defined ( P2P_ONLY_MODE )
		|| get_run_mode() == STA_P2P_CONCURRENT_MODE
#endif	// P2P_ONLY_MODE
	    ) {
		struct wpa_supplicant *sta_wpa_s = select_sta0(wpa_s);

		if (sta_wpa_s->current_bss &&
		    set_freq != sta_wpa_s->current_bss->freq) {
			da16x_err_prt("Input ch is different from STA ch.\n");
			return -1;
		}

		if (set_freq == 0) {
			da16x_err_prt("Not support the ACS in concurrent mode.\n");
			return -1;
		}
	}
#endif	/* CONFIG_CONCURRENT */

	ssid_p->frequency = set_freq;

	if (ssid_p->acs == 1 && ssid_p->frequency > 0)
	{
		ssid_p->acs = 0;
	
		if (wpa_supplicant_create_ap(wpa_s, ssid_p) < 0) {
			da16x_fatal_prt("[%s] Failed to Create AP\n", __func__);
			return -1;
		}

		ssid_p->acs = 1;
	}
	else
	{
		if (wpa_supplicant_create_ap(wpa_s, ssid_p) < 0) {
			da16x_fatal_prt("[%s] Failed to Create AP\n", __func__);
			return -1;
		}
	}

end:

	if (buflen > 2 && buf != NULL)
	{
		strcpy(buf, "OK");
		return 2; // reply_len
	}
	else if (buflen != 0 && buf != NULL)
	{
		da16x_fatal_prt("[%s] Buffer size is small\n", __func__);
	}
	return 0; // reply_len
}

#if 1 /* munchang.jung_20160118 */
int ap_ctrl_iface_status(struct wpa_supplicant *wpa_s, char *buf, size_t buflen)
{
	if (wpa_s->ap_iface == NULL)
		return -1;
	return hostapd_ctrl_iface_status(wpa_s->ap_iface->bss[0],
					    buf, buflen);
}
#endif /* 1 */

#ifdef CONFIG_WPS_NFC

struct wpabuf * wpas_ap_wps_nfc_config_token(struct wpa_supplicant *wpa_s,
					     int ndef)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return NULL;
	hapd = wpa_s->ap_iface->bss[0];
	return hostapd_wps_nfc_config_token(hapd, ndef);
}

struct wpabuf * wpas_ap_wps_nfc_handover_sel(struct wpa_supplicant *wpa_s,
					     int ndef)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return NULL;
	hapd = wpa_s->ap_iface->bss[0];
	return hostapd_wps_nfc_hs_cr(hapd, ndef);
}

int wpas_ap_wps_nfc_report_handover(struct wpa_supplicant *wpa_s,
				    const struct wpabuf *req,
				    const struct wpabuf *sel)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL)
		return -1;
	hapd = wpa_s->ap_iface->bss[0];
	return hostapd_wps_nfc_report_handover(hapd, req, sel);
}

#endif /* CONFIG_WPS_NFC */

#endif /* CONFIG_WPS */

int ap_ctrl_iface_sta_first(struct wpa_supplicant *wpa_s,
			    char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface)
		hapd = wpa_s->ap_iface->bss[0];
#ifdef	CONFIG_MESH
	else if (wpa_s->ifmsh)
		hapd = wpa_s->ifmsh->bss[0];
#endif	/* CONFIG_MESH */
	else
		return -1;
	return hostapd_ctrl_iface_sta_first(hapd, buf, buflen);
}

int ap_ctrl_iface_sta(struct wpa_supplicant *wpa_s, const char *txtaddr,
		      char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface)
		hapd = wpa_s->ap_iface->bss[0];
#ifdef CONFIG_MESH
	else if (wpa_s->ifmsh)
		hapd = wpa_s->ifmsh->bss[0];
#endif /* CONFIG_MESH */
	else
		return -1;
	return hostapd_ctrl_iface_sta(hapd, txtaddr, buf, buflen);
}

int ap_ctrl_iface_sta_next(struct wpa_supplicant *wpa_s, const char *txtaddr,
			   char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface)
		hapd = wpa_s->ap_iface->bss[0];
#ifdef CONFIG_MESH
	else if (wpa_s->ifmsh)
		hapd = wpa_s->ifmsh->bss[0];
#endif /* CONFIG_MESH */
	else
		return -1;
	return hostapd_ctrl_iface_sta_next(hapd, txtaddr, buf, buflen);
}

#ifdef CONFIG_AP_MANAGE_CLIENT
int ap_ctrl_iface_sta_disassociate(struct wpa_supplicant *wpa_s,
				   const char *txtaddr)
{
	u8 mac_addr[ETH_ALEN];

	if (wpa_s->ap_iface == NULL)
		return -1;
	
	// Check MAC address validation,, aa:bb:cc:dd:ee:ff
	if (!((os_strcmp(txtaddr, "all") == 0)	/* all */
		|| (os_strlen(txtaddr) == 17 && hwaddr_aton(txtaddr, mac_addr) == 0)	/* ff:ff:ff:ff:ff:ff */
		|| (os_strstr(txtaddr, "all") != NULL && os_strlen(txtaddr) > 3 && txtaddr[3] == ' ' && os_strstr(txtaddr+3, "="))			/* all reason=3 */
		|| (hwaddr_aton(txtaddr, mac_addr) == 0 && os_strlen(txtaddr) > 17 && txtaddr[17] == ' ' && os_strstr(txtaddr+17, "="))))	/* ff:ff:ff:ff:ff:ff reason=3 */
	{
		da16x_err_prt("[%s] Invalid disassociate MAC Address '%s'\n",
			     __func__, txtaddr);
		return -1;
	}
	return hostapd_ctrl_iface_disassociate(wpa_s->ap_iface->bss[0],
					       txtaddr);
}

#ifdef CONFIG_AP_ISOLATION
int ap_ctrl_iface_isolate(struct wpa_supplicant *wpa_s,
					const int isolate)
{
	if (wpa_s->ap_iface == NULL)
		return -1;
	return hostapd_drv_set_ap_isolate(wpa_s->ap_iface->bss[0],
					       isolate);
}
#endif /* CONFIG_AP_ISOLATION */

int ap_ctrl_iface_sta_deauthenticate(struct wpa_supplicant *wpa_s,
				     const char *txtaddr)
{
	u8 mac_addr[ETH_ALEN];

	if (wpa_s->ap_iface == NULL)
		return -1;
	
	// Check MAC address validation,, aa:bb:cc:dd:ee:ff
	if (!((os_strcmp(txtaddr, "all") == 0)	/* all */
		|| (os_strlen(txtaddr) == 17 && hwaddr_aton(txtaddr, mac_addr) == 0)	/* ff:ff:ff:ff:ff:ff */
		|| (os_strstr(txtaddr, "all") != NULL && os_strlen(txtaddr) > 3 && txtaddr[3] == ' ' && os_strstr(txtaddr+3, "="))			/* all reason=3 */
		|| (hwaddr_aton(txtaddr, mac_addr) == 0 && os_strlen(txtaddr) > 17 && txtaddr[17] == ' ' && os_strstr(txtaddr+17, "="))))	/* ff:ff:ff:ff:ff:ff reason=3 */
	{
		da16x_err_prt("[%s] Invalid deauthenticate MAC Address '%s'\n",
			     __func__, txtaddr);
		return -1;
	}

	return hostapd_ctrl_iface_deauthenticate(wpa_s->ap_iface->bss[0],
						 txtaddr);
}
#endif /* CONFIG_AP_MANAGE_CLIENT */

int ap_ctrl_iface_wpa_get_status(struct wpa_supplicant *wpa_s, char *buf,
				 size_t buflen, int verbose)
{
	char *pos = buf, *end = buf + buflen;
	int ret;
	struct hostapd_bss_config *conf;
	char *start;

	if (wpa_s->ap_iface == NULL
#ifdef CONFIG_MESH
		&& wpa_s->ifmsh == NULL
#endif /* CONFIG_MESH */
	) {
		return -1;
	}

#ifdef CONFIG_MESH
	if (wpa_s->current_ssid->mode == WPAS_MODE_MESH) {
		conf = wpa_s->ifmsh->bss[0]->conf;
	} else
#endif /* CONFIG_MESH */
	{
		conf = wpa_s->ap_iface->bss[0]->conf;
	}

	if (wpa_s->current_ssid) {
		// After "cli save_config" without changing the current channel information,
		// wpa_s->current_ssid->frequency is cleared.
//		ret = os_snprintf(pos, end - pos, "channel=%d\n", i3ed11_freq_to_ch(wpa_s->current_ssid->frequency));
		ret = os_snprintf(pos, end - pos, "channel=%d\n", i3ed11_freq_to_ch(wpa_s->assoc_freq));
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}

#if 0 /* key_mgmt ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿?ap cli statusï¿½ï¿½ unkownï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ */
	ret = os_snprintf(pos, end - pos,
			  "pairwise_cipher=%s\n"
			  "group_cipher=%s\n"
			  "key_mgmt=%s\n",
			  wpa_cipher_txt(conf->rsn_pairwise),
			  wpa_cipher_txt(conf->wpa_group),
			  wpa_key_mgmt_txt(conf->wpa_key_mgmt,
					   conf->wpa));

#else

	if (conf->wpa != 0) {

#ifdef CONFIG_IEEE80211W
		ret = os_snprintf(pos, end - pos,
				"pmf=%s\n"
				"key_mgmt=",
				pmf_mode_txt(conf->ieee80211w));
#else
		ret = os_snprintf(pos, end - pos, "key_mgmt=");
#endif /* CONFIG_IEEE80211W */

		pos += ret;

		/* ï¿½ï¿½ï¿½ï¿½ ï¿½Úµï¿½ wpa_key_mgmt_txt() ï¿½ï¿½ wpa_supplicant_ie_txt() ï¿½ï¿½ï¿½ï¿½ ï¿½Úµï¿½ ï¿½ï¿½ï¿½ï¿½ */
		start = pos;

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_IEEE8021X) {
			if (conf->wpa == (WPA_PROTO_RSN | WPA_PROTO_WPA))
				ret = os_snprintf(pos, end - pos, "%sWPA2+WPA/IEEE 802.1X/EAP",
							pos == start ? "" : "+");
			else
				ret = os_snprintf(pos, end - pos, conf->wpa == WPA_PROTO_RSN ? "%sWPA2/IEEE 802.1X/EAP" : "%sWPA/IEEE 802.1X/EAP",
							pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_PSK) {
			if (conf->wpa == (WPA_PROTO_RSN | WPA_PROTO_WPA))
				ret = os_snprintf(pos, end - pos, "%sWPA2-PSK+WPA-PSK",
						pos == start ? "" : "+");
			else
				ret = os_snprintf(pos, end - pos, conf->wpa == WPA_PROTO_RSN ? "%sWPA2-PSK" : "%sWPA-PSK",
						pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_WPA_NONE) {
			ret = os_snprintf(pos, end - pos, "%sNone",
					  pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_SAE) {
			ret = os_snprintf(pos, end - pos, "%sSAE",
					  pos == start ? "" : "+");
			pos += ret;
		}

#ifdef CONFIG_IEEE80211R
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_IEEE8021X) {
			ret = os_snprintf(pos, end - pos, "%sFT/EAP",
					  pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_PSK) {
			ret = os_snprintf(pos, end - pos, "%sFT/PSK",
					  pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_SAE) {
			ret = os_snprintf(pos, end - pos, "%sFT/SAE",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_IEEE80211R */

#ifdef CONFIG_IEEE80211W
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_IEEE8021X_SHA256) {
			ret = os_snprintf(pos, end - pos, "%sEAP-SHA256",
					  pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_PSK_SHA256) {
			ret = os_snprintf(pos, end - pos, "%sPSK-SHA256",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_IEEE80211W */

#ifdef CONFIG_SUITEB
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_IEEE8021X_SUITE_B) {
			ret = os_snprintf(pos, end - pos, "%sEAP-SUITE-B",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_SUITEB */

#ifdef CONFIG_SUITEB192
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_IEEE8021X_SUITE_B_192) {
			ret = os_snprintf(pos, end - pos, "%sEAP-SUITE-B-192",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_SUITEB192 */

#ifdef CONFIG_FILS
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FILS_SHA256) {
			ret = os_snprintf(pos, end - pos, "%sFILS-SHA256",
					  pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FILS_SHA384) {
			ret = os_snprintf(pos, end - pos, "%sFILS-SHA384",
					  pos == start ? "" : "+");
			pos += ret;
		}

#ifdef CONFIG_IEEE80211R
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_FILS_SHA256) {
			ret = os_snprintf(pos, end - pos, "%sFT-FILS-SHA256",
					  pos == start ? "" : "+");
			pos += ret;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_FILS_SHA384) {
			ret = os_snprintf(pos, end - pos, "%sFT-FILS-SHA384",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_IEEE80211R */
#endif /* CONFIG_FILS */

#ifdef CONFIG_OWE_AP
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_OWE) {
			ret = os_snprintf(pos, end - pos, "%sOWE",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_OWE_AP */

#ifdef CONFIG_DPP
		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_DPP) {
			ret = os_snprintf(pos, end - pos, "%sDPP",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* CONFIG_DPP */

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_OSEN) {
			ret = os_snprintf(pos, end - pos, "%sOSEN",
					  pos == start ? "" : "+");
			pos += ret;
		}
#endif /* 1 */

		if (os_snprintf_error(end - pos, ret))
			return pos - buf;

		ret = os_snprintf(pos, end - pos, "\n");
		pos += ret;

		if (os_snprintf_error(end - pos, ret))
			return pos - buf;

#ifdef CONFIG_SAE
		if ((wpa_s->current_ssid->mode == WPAS_MODE_AP
			|| wpa_s->current_ssid->mode == WPAS_MODE_INFRA
#ifdef CONFIG_MESH
			||wpa_s->current_ssid->mode == WPAS_MODE_MESH
#endif /* CONFIG_MESH */
			)
			&& (wpa_key_mgmt_wpa_psk(wpa_s->current_ssid->key_mgmt) || wpa_key_mgmt_wpa_psk(wpa_s->ap_iface->bss[0]->conf->wpa_key_mgmt))
			&& wpa_s->current_ssid->sae_password) {
			ret = os_snprintf(pos, end - pos, "sae_password=%s\n", wpa_s->current_ssid->sae_password);

			pos += ret;

			if (os_snprintf_error(end - pos, ret))
				return pos - buf;
		}

		if (conf->wpa_key_mgmt & WPA_KEY_MGMT_SAE) {
			ret = os_snprintf(pos, end - pos, "sae_groups=");
			pos += ret;


#ifdef CONFIG_MESH
			if (wpa_s->current_ssid->mode == WPAS_MODE_MESH) {
				for (int i = 0; conf->sae_groups[i] > 0; i++) {
					ret = os_snprintf(pos, end - pos, "%s%d", i == 0 ? "":" ", conf->sae_groups[i]);
					pos += ret;
				}
			} else
#endif /* CONFIG_MESH */
			{
				for (int i = 0; wpa_s->conf->sae_groups[i] != 0; i++) {
					ret = os_snprintf(pos, end - pos, "%s%d", i == 0 ? "":" ", wpa_s->conf->sae_groups[i]);
					pos += ret;
				}
			}
			
			if (os_snprintf_error(end - pos, ret))
				return pos - buf;

			ret = os_snprintf(pos, end - pos, "\n");
			pos += ret;

			if (os_snprintf_error(end - pos, ret))
				return pos - buf;
		}
#endif /* CONFIG_SAE */

		if (!(conf->wpa_key_mgmt & WPA_KEY_MGMT_SAE)) {
			ret = os_snprintf(pos, end - pos,
					"pairwise_cipher=%s\n"
					"group_cipher=%s\n",
					wpa_cipher_txt(conf->wpa == WPA_PROTO_WPA ? conf->wpa_pairwise : conf->rsn_pairwise),
					wpa_cipher_txt(conf->wpa_group));
			pos += ret;
		}
	}

	if (os_snprintf_error(end - pos, ret))
		return pos - buf;

	return pos - buf;
}


int wpa_supplicant_ap_update_beacon(struct wpa_supplicant *wpa_s)
{
	struct hostapd_iface *iface = wpa_s->ap_iface;
	struct wpa_ssid *ssid = wpa_s->current_ssid;
	struct hostapd_data *hapd;

	if (ssid == NULL || ssid->mode == WPAS_MODE_INFRA || ssid->mode == WPAS_MODE_IBSS
		|| (wpa_s->ap_iface == NULL
#ifdef CONFIG_MESH
		    && wpa_s->ifmsh == NULL
#endif /* CONFIG_MESH */
		)) {
		return -1;
	}

#ifdef CONFIG_P2P
	if (ssid->mode == WPAS_MODE_P2P_GO)
		iface->conf->bss[0]->p2p = P2P_ENABLED | P2P_GROUP_OWNER;
	else if (ssid->mode == WPAS_MODE_P2P_GROUP_FORMATION)
		iface->conf->bss[0]->p2p = P2P_ENABLED | P2P_GROUP_OWNER |
			P2P_GROUP_FORMATION;
#endif /* CONFIG_P2P */

	hapd = iface->bss[0];
	if (hapd->drv_priv == NULL)
		return -1;
	ieee802_11_set_beacons(iface);
	hostapd_set_ap_wps_ie(hapd);

	return 0;
}

#ifdef CONFIG_AP_POWER
/**
 * nl80211.h
 * enum nld11_tx_power_setting - TX power adjustment
 * @NLD11_TX_PWR_AUTOMATIC: automatically determine transmit power
 * @NLD11_TX_PWR_LIMITED: limit TX power by the mBm parameter
 * @NLD11_TX_PWR_FIXED: fix TX power to the mBm parameter
 */
#define 	POWER_AUTOMATIC	0
#define		POWER_LIMITED	1
#define 	POWER_FIXED		2
#define 	POWER_GET		9

int ap_ctrl_iface_set_ap_power(struct wpa_supplicant *wpa_s, char *ap_pwr)
{
	struct hostapd_data *hapd = wpa_s->ap_iface->bss[0];
	int type, power, ret;
#if 0
	int i;
#endif	/* 0 */

	if (hapd->drv_priv == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	if (os_strncasecmp(ap_pwr, "auto", 4) == 0) {
		type = POWER_AUTOMATIC;
		power = 0;
	} else {
#if 0 // ???
		power = atoi(ap_pwr);
		if (power > 0) {
			for (i = 0; i < NUM_SUPPORTS_COUNTRY; i++) {
#ifdef __NEW_COUNTRYCODE__
				const struct country_ch_power_level *field = (void*)cc_power_table(i);
				int min_power, max_pwr;

				country_ch_pwr_MinMax(field->country, &max_pwr, &max_pwr);
#else
				const struct country_data *field = &country_fields[i];
#endif /* __NEW_COUNTRYCODE__ */
				if (os_strcasecmp(wpa_s->conf->country, field->country) != 0)
					continue;

#ifdef __NEW_COUNTRYCODE__
				if ((power < min_power) || (power > max_pwr)) {
					da16x_err_prt("Country Code [%s] ap_power [%d mW ~ %d mW]\n",
						     wpa_s->conf->country, min_power, max_pwr);
					goto ret_error;
				}
#else
				if ((power < field->min_power) || (power > field->max_pwr)) {
					da16x_err_prt("Country Code [%s] ap_power [%d mW ~ %d mW]\n",
						     wpa_s->conf->country, field->min_power, field->max_pwr);
					goto ret_error;
				}
#endif /* __NEW_COUNTRYCODE__ */
				type = POWER_FIXED;
				break;
			}
		} else {
			goto ret_error;
		}
#endif /* 0 */
	}

	ret = hostapd_drv_set_ap_power(hapd, type, power, NULL);
	return ret;

#if 0
ret_error:
	da16x_err_prt("ap_power [\"auto\" / power(mW)]\n");
	return -1;
#endif	/* 0 */
}

int ap_ctrl_iface_get_ap_power(struct wpa_supplicant *wpa_s, char *ap_pwr)
{
	struct hostapd_data *hapd = wpa_s->ap_iface->bss[0];
	int type, power, ret = 0;
	int *get_power;

	if (hapd->drv_priv == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	type = POWER_GET;
	power = 0;
	get_power = os_malloc(sizeof(int));
	if(get_power == NULL)
		return -1;

	if (hostapd_drv_set_ap_power(hapd, type, power, get_power) >= 0)
	{
		da16x_cli_prt("AP power is = %d dbm\n", *get_power);		
		ret = *get_power;
	}

	os_free(get_power);

	return ret;
}
#endif /* CONFIG_AP_POWER */

#ifdef CONFIG_AP_WMM
int ap_ctrl_iface_wmm_enabled(struct wpa_supplicant *wpa_s, char *pos)
{
	struct hostapd_data *hapd;
	int val = -1;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	// Check validation of input argument
	if ((os_strlen(pos) > 1) || (pos[0] != '0' && pos[0] != '1')) {
		da16x_cli_prt("Wrong input argument value\n");
		da16x_err_prt("Invalid Cmd\n"
			     "1: enable / 0: disable\n");

		return -1;
	}

	val = atoi(pos);
	wpa_config_write_nvram_int(ENV_WMM_ENABLED, val, 1);

	hapd = wpa_s->ap_iface->bss[0];
	hapd->conf->wmm_enabled = val;
	wpa_s->conf->hostapd_wmm_enabled = val;
	return wpa_supplicant_ap_update_beacon(wpa_s);
}

int ap_ctrl_iface_get_wmm_enabled(struct wpa_supplicant *wpa_s,
				  char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}
	
	hapd = wpa_s->ap_iface->bss[0];
	hapd->conf->wmm_enabled = wpa_s->conf->hostapd_wmm_enabled;
	sprintf(buf, "%d", hapd->conf->wmm_enabled == 1 ? 1 : 0);

	return os_strlen(buf);;
}

int ap_ctrl_iface_wmm_ps_enabled(struct wpa_supplicant *wpa_s, char *pos)
{
	struct hostapd_data *hapd;
	int val = -1;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	// Check validation of input argument
	if ((os_strlen(pos) > 1) || (pos[0] != '0' && pos[0] != '1')) {
		da16x_cli_prt("Wrong input argument value\n");
		da16x_err_prt("Invalid Cmd\n"
				     "1: enable / 0: disable\n");
		return -1;
	}

	val = atoi(pos);
	wpa_config_write_nvram_int(ENV_WMM_PS_ENABLED, val, 0);

	hapd = wpa_s->ap_iface->bss[0];
	hapd->conf->wmm_uapsd = val;
	wpa_s->conf->hostapd_wmm_ps_enabled = val;
	return wpa_supplicant_ap_update_beacon(wpa_s);
}

int ap_ctrl_iface_get_wmm_ps_enabled(struct wpa_supplicant *wpa_s,
				     char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	hapd = wpa_s->ap_iface->bss[0];
	sprintf(buf, "%d", hapd->conf->wmm_uapsd == 1 ? 1 : 0);

	return os_strlen(buf);
}

static int exp2_int_only(int input)
{
	if (input == 1)
		return 1;
	else if (input == 2)
		return 3;
	else if (input == 3)
		return 7;
	else if (input == 4)
		return 15;
	else if (input == 5)
		return 31;
	else if (input == 6)
		return 63;
	else if (input == 7)
		return 127;
	else if (input == 8)
		return 255;
	else if (input == 9)
		return 511;
	else if (input == 10)
		return 1023;
	else
		return -1;
}

static int log2_int_only(int input)
{
	if (input == 1)
		return 1;
	else if (input == 3)
		return 2;
	else if (input == 7)
		return 3;
	else if (input == 15)
		return 4;
	else if (input == 31)
		return 5;
	else if (input == 63)
		return 6;
	else if (input == 127)
		return 7;
	else if (input == 255)
		return 8;
	else if (input == 511)
		return 9;
	else if (input == 1023)
		return 10;
	else
		return -1;
}

int ap_ctrl_iface_wmm_params(struct wpa_supplicant *wpa_s, char *pos)
{
	struct hostapd_data *hapd;
	struct hostapd_config *iconf;
	char *str1;
	char str2[4][5] = {0};
	int index = 0;
	int str_num = 0;

	int wmm_params[4];
	int is_ap;
	int ac_idx;

	int ecwmin, ecwmax;
	int ret = 0;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	hapd = wpa_s->ap_iface->bss[0];
	iconf = hapd->iconf;

	str1 = pos;
	if (os_strncmp(str1, "ap ", 3) == 0) {
		is_ap = 1;
		str1 += 3;
	} else if (os_strncmp(str1, "sta ", 4) == 0) {
		is_ap = 0;
		str1 += 4;
	} else
		goto invalid_cmd;

	if (os_strncmp(str1, "be ", 3) == 0)
		ac_idx = 0;
	else if (os_strncmp(str1, "bk ", 3) == 0)
		ac_idx = 1;
	else if (os_strncmp(str1, "vi ", 3) == 0)
		ac_idx = 2;
	else if (os_strncmp(str1, "vo ", 3) == 0)
		ac_idx = 3;
	else
		goto invalid_cmd;

	str1 += 3;

	while (*str1) {
		if (*str1 != ' ') {
			str2[index][str_num] = *str1;
			str_num++;
		} else  {
			wmm_params[index] = atoi(str2[index]);
			str_num = 0;
			index++;
		}

		if (index > 3 || str_num > 4)
			goto invalid_cmd;

		str1++;
	}
	wmm_params[index] = atoi(str2[index]);

	ecwmin = log2_int_only(wmm_params[1]);
	ecwmax = log2_int_only(wmm_params[2]);
	if ((ecwmin == -1 || ecwmax == -1) || ecwmin >= ecwmax) {
		da16x_err_prt("Invalid CW Parameter! "
			     "(1, 3, 7, 15, 31, 63, "
			     "127, 255, 511, 1023)\n");
		goto invalid_params;
	}

	if (is_ap) {
		int tmp = ac_idx;

		if (tmp == 0)
			ac_idx = 2;
		else if (tmp == 1)
			ac_idx = 3;
		else if (tmp == 2)
			ac_idx = 1;
		else if (tmp == 3)
			ac_idx = 0;

		iconf->tx_queue[ac_idx].aifs = wmm_params[0];
		iconf->tx_queue[ac_idx].cwmin = wmm_params[1];
		iconf->tx_queue[ac_idx].cwmax = wmm_params[2];
		iconf->tx_queue[ac_idx].burst = wmm_params[3];

		hostapd_tx_queue_params(wpa_s->ap_iface);
	} else {
		iconf->wmm_ac_params[ac_idx].aifs = wmm_params[0];
		iconf->wmm_ac_params[ac_idx].cwmin = ecwmin;
		iconf->wmm_ac_params[ac_idx].cwmax = ecwmax;
		iconf->wmm_ac_params[ac_idx].txop_limit = wmm_params[3];
		ret = wpa_supplicant_ap_update_beacon(wpa_s);
	}

	return ret;

invalid_cmd:
	da16x_warn_prt("Invalid Cmd : cli wmm_params "
		      "[ap/sta] [be/bk/vi/vo] [AIFS] [CWmin] [CWmax] "
		      "[Burst(AP) or TXOP Limit(STA)]\n");

invalid_params:
	return -1;
}

int ap_ctrl_iface_get_wmm_params(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;
	struct hostapd_config *iconf;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	hapd = wpa_s->ap_iface->bss[0];
	iconf = hapd->iconf;

	da16x_notice_prt("\n[AP]\n"
		        "Category\tAIFS\tCWmin\tCWmax\tBurst\n"
		        "AC_BE:\t\t%d\t%d\t%d\t%d\n"
		        "AC_BK:\t\t%d\t%d\t%d\t%d\n"
		        "AC_VI:\t\t%d\t%d\t%d\t%d\n"
		        "AC_VO:\t\t%d\t%d\t%d\t%d\n\n",
		        iconf->tx_queue[2].aifs, iconf->tx_queue[2].cwmin,
		        iconf->tx_queue[2].cwmax, iconf->tx_queue[2].burst,
		        iconf->tx_queue[3].aifs, iconf->tx_queue[3].cwmin,
		        iconf->tx_queue[3].cwmax, iconf->tx_queue[3].burst,
		        iconf->tx_queue[1].aifs, iconf->tx_queue[1].cwmin,
		        iconf->tx_queue[1].cwmax, iconf->tx_queue[1].burst,
		        iconf->tx_queue[0].aifs, iconf->tx_queue[0].cwmin,
		        iconf->tx_queue[0].cwmax, iconf->tx_queue[0].burst);

	da16x_notice_prt("\n[STA]\n"
		        "Category\tAIFS\tCWmin\tCWmax\tTXOP Limit\n"
		        "AC_BE:\t\t%d\t%d\t%d\t%d\n"
		        "AC_BK:\t\t%d\t%d\t%d\t%d\n"
		        "AC_VI:\t\t%d\t%d\t%d\t%d\n"
		        "AC_VO:\t\t%d\t%d\t%d\t%d\n\n",
		        iconf->wmm_ac_params[0].aifs,
		        exp2_int_only(iconf->wmm_ac_params[0].cwmin),
		        exp2_int_only(iconf->wmm_ac_params[0].cwmax),
		        iconf->wmm_ac_params[0].txop_limit,
		        iconf->wmm_ac_params[1].aifs,
		        exp2_int_only(iconf->wmm_ac_params[1].cwmin),
		        exp2_int_only(iconf->wmm_ac_params[1].cwmax),
		        iconf->wmm_ac_params[1].txop_limit,
		        iconf->wmm_ac_params[2].aifs,
		        exp2_int_only(iconf->wmm_ac_params[2].cwmin),
		        exp2_int_only(iconf->wmm_ac_params[2].cwmax),
		        iconf->wmm_ac_params[2].txop_limit,
		        iconf->wmm_ac_params[3].aifs,
		        exp2_int_only(iconf->wmm_ac_params[3].cwmin),
		        exp2_int_only(iconf->wmm_ac_params[3].cwmax),
		        iconf->wmm_ac_params[3].txop_limit);

	return 0;
}
#endif /* CONFIG_AP_WMM */


#ifdef CONFIG_ACL
int hostapd_ap_acl_init(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;
	struct hostapd_config *conf;
	int i;

	hapd = wpa_s->ap_iface->bss[0];
	conf = hapd->iconf;

	hapd->iface->drv_max_acl_mac_addrs = NUM_MAX_ACL;
	conf->bss[0]->accept_mac = os_zalloc(NUM_MAX_ACL * sizeof(struct mac_acl_entry));
	if(conf->bss[0]->accept_mac == NULL) {
		return -1;
	}
	
	conf->bss[0]->deny_mac = os_zalloc(NUM_MAX_ACL * sizeof(struct mac_acl_entry));
	if(conf->bss[0]->deny_mac == NULL) {
		os_free(conf->bss[0]->accept_mac);
		return -1;
	}

	if (wpa_s->conf->macaddr_acl == DENY_UNLESS_ACCEPTED) {
		conf->bss[0]->macaddr_acl = DENY_UNLESS_ACCEPTED;
		os_memcpy (conf->bss[0]->accept_mac, wpa_s->conf->acl_mac,
						NUM_MAX_ACL*sizeof(struct mac_acl_entry));
		conf->bss[0]->num_accept_mac = wpa_s->conf->acl_num;
		conf->bss[0]->num_deny_mac = 0;
	} else if (wpa_s->conf->macaddr_acl == ACCEPT_UNLESS_DENIED ) {
		conf->bss[0]->macaddr_acl = ACCEPT_UNLESS_DENIED;
		os_memcpy (conf->bss[0]->deny_mac, wpa_s->conf->acl_mac,
						NUM_MAX_ACL*sizeof(struct mac_acl_entry));
		conf->bss[0]->num_deny_mac = wpa_s->conf->acl_num;
		conf->bss[0]->num_accept_mac = 0;
	} else {
		conf->bss[0]->num_accept_mac = 0;
		conf->bss[0]->num_deny_mac = 0;
		for (i=0; i < NUM_MAX_ACL; i++) {
			os_memset(conf->bss[0]->accept_mac[i].addr, 0, ETH_ALEN);
			os_memset(conf->bss[0]->deny_mac[i].addr, 0, ETH_ALEN);
		}

	}

	return 0;
}

int hostapd_ap_acl_set(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;
	struct hostapd_config *conf;
	int i;

	if (!wpa_s->ap_iface)
		return -1;

	hapd = wpa_s->ap_iface->bss[0];
	conf = hapd->iconf;

	if (wpa_s->conf->macaddr_acl == DENY_UNLESS_ACCEPTED) {
		conf->bss[0]->macaddr_acl = DENY_UNLESS_ACCEPTED;
		os_memcpy (conf->bss[0]->accept_mac, wpa_s->conf->acl_mac,
				NUM_MAX_ACL*sizeof(struct mac_acl_entry));
		conf->bss[0]->num_accept_mac = wpa_s->conf->acl_num;
		conf->bss[0]->num_deny_mac = 0;
	} else if (wpa_s->conf->macaddr_acl == ACCEPT_UNLESS_DENIED ) {
		conf->bss[0]->macaddr_acl = ACCEPT_UNLESS_DENIED;
		os_memcpy (conf->bss[0]->deny_mac, wpa_s->conf->acl_mac,
				NUM_MAX_ACL*sizeof(struct mac_acl_entry));
		conf->bss[0]->num_deny_mac = wpa_s->conf->acl_num;
		conf->bss[0]->num_accept_mac = 0;
	} else {
		conf->bss[0]->macaddr_acl = ACCEPT_UNLESS_DENIED;
		conf->bss[0]->num_accept_mac = 0;
		conf->bss[0]->num_deny_mac = 0;
		for (i=0; i < NUM_MAX_ACL; i++) {
			os_memset(conf->bss[0]->accept_mac[i].addr, 0, ETH_ALEN);
			os_memset(conf->bss[0]->deny_mac[i].addr, 0, ETH_ALEN);
		}
	}

	return 0;

}

#endif /* CONFIG_ACL */


extern int get_int_val_from_str(char* param, int* int_val, int policy);

int ap_ctrl_iface_ap_max_inactivity(struct wpa_supplicant *wpa_s, char *pos)
{
	struct hostapd_data *hapd;
	int val;
#define	POL_1	1

	// Check parameter validity
	if (get_int_val_from_str(pos, &val, POL_1) != 0) {
		goto err_rtn;
	}

	if ((val != 0 && (val < 30 || val > 86400)) || (val % 10 != 0)) {
err_rtn :
		da16x_err_prt("Invalid Cmd\n"
			     "Timeout: 30 ~ 86400 secs (step=10)\n"
			     "(If 0, use default - 300 secs)\n");
		return -1;
	}
	wpa_s->conf->ap_max_inactivity = val;

	if (wpa_s->ap_iface != NULL) {
		hapd = wpa_s->ap_iface->bss[0];
		hapd->conf->ap_max_inactivity = val;
	}

	return 0;
}

int ap_ctrl_iface_get_ap_max_inactivity(struct wpa_supplicant *wpa_s,
					char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface != NULL) {
		hapd = wpa_s->ap_iface->bss[0];
		sprintf(buf, "%d", (hapd->conf->ap_max_inactivity)? 
		                hapd->conf->ap_max_inactivity : DEFAULT_AP_MAX_INACTIVITY );
	} else {
		sprintf(buf, "%d", (wpa_s->conf->ap_max_inactivity)?
                        wpa_s->conf->ap_max_inactivity : DEFAULT_AP_MAX_INACTIVITY);
	}

	return os_strlen(buf);
}

#if 1	/* by Shingu 20161010 (Keep-alive) */
int ap_ctrl_iface_ap_send_ka(struct wpa_supplicant *wpa_s, char *pos)
{
	struct hostapd_data *hapd;
	int val = -1;

	// Check validation of input argument
	if ((os_strlen(pos) > 1) || (pos[0] != '0' && pos[0] != '1')) {
		da16x_cli_prt("Wrong input argument value\n");
		da16x_err_prt("Invalid Cmd\n"
			     "1: enable / 0: disable\n");
		return -1;
	}

	val = atoi(pos);
	wpa_s->conf->ap_send_ka = val;

	if (wpa_s->ap_iface == NULL) {
		da16x_cli_prt("No AP or P2P GO Interface\n");
		return -1;
	}

	hapd = wpa_s->ap_iface->bss[0];
	hapd->conf->ap_send_ka = val;

	return 0;
}

int ap_ctrl_iface_get_ap_send_ka(struct wpa_supplicant *wpa_s,
					char *buf, size_t buflen)
{
	struct hostapd_data *hapd;

	if (wpa_s->ap_iface != NULL) {
		hapd = wpa_s->ap_iface->bss[0];
		sprintf(buf, "%d", hapd->conf->ap_send_ka);
	} else {
		sprintf(buf, "%d", wpa_s->conf->ap_send_ka);
	}

	return os_strlen(buf);
}
#endif	/* 1 */

int wpas_ap_pwrsave(struct wpa_supplicant *wpa_s, int ps_state, int timeout)
{
	struct hostapd_data *hapd = wpa_s->ap_iface->bss[0];
	return hostapd_drv_ap_pwrsave(hapd, ps_state, timeout);
}
int wpas_ap_rts(struct wpa_supplicant *wpa_s, int rts_threshold)
{
	struct hostapd_data *hapd = wpa_s->ap_iface->bss[0];
	wpa_s->conf->rts_threshold = rts_threshold;
	hapd->iconf->rts_threshold = rts_threshold;
	return hostapd_set_rts(hapd, hapd->iconf->rts_threshold);
}


#ifdef __FRAG_ENABLE__
int wpas_ap_frag(struct wpa_supplicant *wpa_s, const int fragm_threshold)
{
	struct hostapd_data *hapd = wpa_s->ap_iface->bss[0];
	hapd->iconf->fragm_threshold = fragm_threshold;
	return hostapd_set_frag(hapd, hapd->iconf->fragm_threshold);
}
#endif /* __FRAG_ENABLE__ */


int wpas_addba_reject(struct wpa_supplicant *wpa_s, u8 addba_reject)
{
	struct hostapd_data *hapd = wpa_s->ap_iface->bss[0];
	hapd->iconf->addba_reject = addba_reject;
	return hostapd_addba_reject(hapd, hapd->iconf->addba_reject);
}

int wpas_ap_set_greenfield(struct wpa_supplicant *wpa_s, u16 set_greenfield)
{
	struct hostapd_data *hapd  = wpa_s->ap_iface->bss[0];
	hapd->conf->set_greenfield = set_greenfield;
#ifdef CONFIG_AP_PARAMETERS
	wpa_s->conf->greenfield = set_greenfield;
#endif /* CONFIG_AP_PARAMETERS */
	return hapd->conf->set_greenfield;
}

int wpas_ap_ht_protection(struct wpa_supplicant *wpa_s, int ht_protection)
{
	struct hostapd_iface *iface = wpa_s->ap_iface;

	iface->ht_op_mode = ht_protection;
#ifdef CONFIG_AP_PARAMETERS
	wpa_s->conf->ht_protection = ht_protection;
#endif /* CONFIG_AP_PARAMETERS */
	return wpa_supplicant_ap_update_beacon(wpa_s);
}

int ap_ctrl_iface_save_config(struct wpa_supplicant *wpa_s)
{
	if (wpa_s->ap_iface == NULL)
		return -1;
	return hostapd_ctrl_iface_save_config(wpa_s->ap_iface->bss[0]);
}


#ifdef	CONFIG_5G_SUPPORT
int ap_switch_channel(struct wpa_supplicant *wpa_s,
		      struct csa_settings *settings)
{
#ifdef NEED_AP_MLME
	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0])
		return -1;

	return hostapd_switch_channel(wpa_s->ap_iface->bss[0], settings);
#else /* NEED_AP_MLME */
	return -1;
#endif /* NEED_AP_MLME */
}
#endif	/* CONFIG_5G_SUPPORT */


#ifdef CONFIG_CTRL_IFACE
int ap_ctrl_iface_chanswitch(struct wpa_supplicant *wpa_s, const char *pos)
{
	struct csa_settings settings;
	int ret = hostapd_parse_csa_settings(pos, &settings);

	if (ret)
		return ret;

	return ap_switch_channel(wpa_s, &settings);
}
#endif /* CONFIG_CTRL_IFACE */


void wpas_ap_ch_switch(struct wpa_supplicant *wpa_s, int freq, int ht,
		       int offset, int width, int cf1, int cf2)
{
	if (!wpa_s->ap_iface)
		return;

	wpa_s->assoc_freq = freq;
	if (wpa_s->current_ssid)
		wpa_s->current_ssid->frequency = freq;
	hostapd_event_ch_switch(wpa_s->ap_iface->bss[0], freq, ht,
				offset, width, cf1, cf2);
}


int wpa_supplicant_ap_mac_addr_filter(struct wpa_supplicant *wpa_s,
				      const u8 *addr)
{
	struct hostapd_data *hapd;
	struct hostapd_bss_config *conf;

	if (!wpa_s->ap_iface)
		return -1;

	if (addr) {
		wpa_printf_dbg(MSG_DEBUG, "AP: Set MAC address filter: " MACSTR,
			   MAC2STR(addr));
	} else {
		wpa_printf_dbg(MSG_DEBUG, "AP: Clear MAC address filter");
	}

	hapd = wpa_s->ap_iface->bss[0];
	conf = hapd->conf;

	os_free(conf->accept_mac);
	conf->accept_mac = NULL;
	conf->num_accept_mac = 0;
	os_free(conf->deny_mac);
	conf->deny_mac = NULL;
	conf->num_deny_mac = 0;

	if (addr == NULL) {
		conf->macaddr_acl = ACCEPT_UNLESS_DENIED;
		return 0;
	}

	conf->macaddr_acl = DENY_UNLESS_ACCEPTED;
	conf->accept_mac = os_zalloc(sizeof(struct mac_acl_entry));
	if (conf->accept_mac == NULL)
		return -1;
	os_memcpy(conf->accept_mac[0].addr, addr, ETH_ALEN);
	conf->num_accept_mac = 1;

	return 0;
}


#ifdef CONFIG_WPS_NFC
int wpas_ap_wps_add_nfc_pw(struct wpa_supplicant *wpa_s, u16 pw_id,
			   const struct wpabuf *pw, const u8 *pubkey_hash)
{
	struct hostapd_data *hapd;
	struct wps_context *wps;

	if (!wpa_s->ap_iface)
		return -1;
	hapd = wpa_s->ap_iface->bss[0];
	wps = hapd->wps;

	if (wpa_s->p2pdev->conf->wps_nfc_dh_pubkey == NULL ||
	    wpa_s->p2pdev->conf->wps_nfc_dh_privkey == NULL) {
		wpa_printf_dbg(MSG_DEBUG, "P2P: No NFC DH key known");
		return -1;
	}

	dh5_free(wps->dh_ctx);
	wpabuf_free(wps->dh_pubkey);
	wpabuf_free(wps->dh_privkey);
	wps->dh_privkey = wpabuf_dup(
		wpa_s->p2pdev->conf->wps_nfc_dh_privkey);
	wps->dh_pubkey = wpabuf_dup(
		wpa_s->p2pdev->conf->wps_nfc_dh_pubkey);
	if (wps->dh_privkey == NULL || wps->dh_pubkey == NULL) {
		wps->dh_ctx = NULL;
		wpabuf_free(wps->dh_pubkey);
		wps->dh_pubkey = NULL;
		wpabuf_free(wps->dh_privkey);
		wps->dh_privkey = NULL;
		return -1;
	}
	wps->dh_ctx = dh5_init_fixed(wps->dh_privkey, wps->dh_pubkey);
	if (wps->dh_ctx == NULL)
		return -1;

	return wps_registrar_add_nfc_pw_token(hapd->wps->registrar, pubkey_hash,
					      pw_id,
					      pw ? wpabuf_head(pw) : NULL,
					      pw ? wpabuf_len(pw) : 0, 1);
}
#endif /* CONFIG_WPS_NFC */


#ifdef CONFIG_CTRL_IFACE
int wpas_ap_stop_ap(struct wpa_supplicant *wpa_s)
{
	struct hostapd_data *hapd;

	if (!wpa_s->ap_iface)
		return -1;
	hapd = wpa_s->ap_iface->bss[0];
	return hostapd_ctrl_iface_stop_ap(hapd);
}
#endif /* CONFIG_CTRL_IFACE */

int wpas_ap_pmksa_cache_list(struct wpa_supplicant *wpa_s, char *buf,
			     size_t len)
{
	size_t reply_len = 0, i;
	char ap_delimiter[] = "---- AP ----\n";
	char mesh_delimiter[] = "---- mesh ----\n";
	size_t dlen;

	if (wpa_s->ap_iface) {
		dlen = os_strlen(ap_delimiter);
		if (dlen > len - reply_len)
			return reply_len;
		os_memcpy(&buf[reply_len], ap_delimiter, dlen);
		reply_len += dlen;

		for (i = 0; i < wpa_s->ap_iface->num_bss; i++) {
			reply_len += hostapd_ctrl_iface_pmksa_list(
				wpa_s->ap_iface->bss[i],
				&buf[reply_len], len - reply_len);
		}
	}

	if (wpa_s->ifmsh) {
		dlen = os_strlen(mesh_delimiter);
		if (dlen > len - reply_len)
			return reply_len;
		os_memcpy(&buf[reply_len], mesh_delimiter, dlen);
		reply_len += dlen;

		reply_len += hostapd_ctrl_iface_pmksa_list(
			wpa_s->ifmsh->bss[0], &buf[reply_len],
			len - reply_len);
	}

	return reply_len;
}


#ifdef CONFIG_ENTERPRISE
void wpas_ap_pmksa_cache_flush(struct wpa_supplicant *wpa_s)
{
	size_t i;

	if (wpa_s->ap_iface) {
		for (i = 0; i < wpa_s->ap_iface->num_bss; i++)
			hostapd_ctrl_iface_pmksa_flush(wpa_s->ap_iface->bss[i]);
	}

	if (wpa_s->ifmsh)
		hostapd_ctrl_iface_pmksa_flush(wpa_s->ifmsh->bss[0]);
}
#endif	/* CONFIG_ENTERPRISE */

#ifdef CONFIG_CTRL_IFACE

#ifdef CONFIG_PMKSA_CACHE_EXTERNAL
#ifdef CONFIG_MESH

int wpas_ap_pmksa_cache_list_mesh(struct wpa_supplicant *wpa_s, const u8 *addr,
				  char *buf, size_t len)
{
	return hostapd_ctrl_iface_pmksa_list_mesh(wpa_s->ifmsh->bss[0], addr,
						  &buf[0], len);
}


int wpas_ap_pmksa_cache_add_external(struct wpa_supplicant *wpa_s, char *cmd)
{
	struct external_pmksa_cache *entry;
	void *pmksa_cache;

	pmksa_cache = hostapd_ctrl_iface_pmksa_create_entry(wpa_s->own_addr,
							    cmd);
	if (!pmksa_cache)
		return -1;

	entry = os_zalloc(sizeof(struct external_pmksa_cache));
	if (!entry)
		return -1;

	entry->pmksa_cache = pmksa_cache;

	dl_list_add(&wpa_s->mesh_external_pmksa_cache, &entry->list);

	return 0;
}

#endif /* CONFIG_MESH */
#endif /* CONFIG_PMKSA_CACHE_EXTERNAL */

#endif /* CONFIG_CTRL_IFACE */


#ifdef NEED_AP_MLME
#ifdef CONFIG_IEEE80211H
void wpas_ap_event_dfs_radar_detected(struct wpa_supplicant *wpa_s,
				      struct dfs_event *radar)
{
	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0])
		return;
	wpa_printf_dbg(MSG_DEBUG, "DFS radar detected on %d MHz", radar->freq);
	hostapd_dfs_radar_detected(wpa_s->ap_iface, radar->freq,
				   radar->ht_enabled, radar->chan_offset,
				   radar->chan_width,
				   radar->cf1, radar->cf2);
}


void wpas_ap_event_dfs_cac_started(struct wpa_supplicant *wpa_s,
				   struct dfs_event *radar)
{
	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0])
		return;
	wpa_printf_dbg(MSG_DEBUG, "DFS CAC started on %d MHz", radar->freq);
	hostapd_dfs_start_cac(wpa_s->ap_iface, radar->freq,
			      radar->ht_enabled, radar->chan_offset,
			      radar->chan_width, radar->cf1, radar->cf2);
}


void wpas_ap_event_dfs_cac_finished(struct wpa_supplicant *wpa_s,
				    struct dfs_event *radar)
{
	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0])
		return;
	wpa_printf_dbg(MSG_DEBUG, "DFS CAC finished on %d MHz", radar->freq);
	hostapd_dfs_complete_cac(wpa_s->ap_iface, 1, radar->freq,
				 radar->ht_enabled, radar->chan_offset,
				 radar->chan_width, radar->cf1, radar->cf2);
}


void wpas_ap_event_dfs_cac_aborted(struct wpa_supplicant *wpa_s,
				   struct dfs_event *radar)
{
	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0])
		return;
	wpa_printf_dbg(MSG_DEBUG, "DFS CAC aborted on %d MHz", radar->freq);
	hostapd_dfs_complete_cac(wpa_s->ap_iface, 0, radar->freq,
				 radar->ht_enabled, radar->chan_offset,
				 radar->chan_width, radar->cf1, radar->cf2);
}


void wpas_ap_event_dfs_cac_nop_finished(struct wpa_supplicant *wpa_s,
					struct dfs_event *radar)
{
	if (!wpa_s->ap_iface || !wpa_s->ap_iface->bss[0])
		return;
	wpa_printf_dbg(MSG_DEBUG, "DFS NOP finished on %d MHz", radar->freq);
	hostapd_dfs_nop_finished(wpa_s->ap_iface, radar->freq,
				 radar->ht_enabled, radar->chan_offset,
				 radar->chan_width, radar->cf1, radar->cf2);
}
#endif /* CONFIG_IEEE80211H */
#endif /* NEED_AP_MLME */


#ifdef	UNUSED_CODE
void ap_periodic(struct wpa_supplicant *wpa_s)
{
	if (wpa_s->ap_iface)
		hostapd_periodic_iface(wpa_s->ap_iface);
}
#endif	/* UNUSED_CODE */

#endif	/* CONFIG_AP */

/* EOF */
