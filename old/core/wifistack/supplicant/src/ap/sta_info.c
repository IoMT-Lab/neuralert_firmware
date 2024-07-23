/*
 * hostapd / Station table
 * Copyright (c) 2002-2017, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "utils/includes.h"

#ifdef	CONFIG_AP

#include "supp_common.h"
#include "supp_eloop.h"
#include "common/ieee802_11_defs.h"
#include "wpa_ctrl.h"
#include "common/sae.h"

#ifdef	CONFIG_RADIUS
#include "radius/radius.h"
#include "radius/radius_client.h"
#endif	/* CONFIG_RADIUS */

//#include "fst.h"
#include "wpa_supplicant_i.h"
#include "crypto/crypto.h"
#include "hostapd.h"
#include "accounting.h"
#include "ieee802_1x.h"
#include "ieee802_11.h"
#include "ieee802_11_auth.h"
#include "wpa_auth.h"
#include "preauth_auth.h"
#include "ap_config.h"
#include "beacon.h"
#include "ap_mlme.h"
#include "vlan_init.h"
#include "p2p_hostapd.h"
#include "ap_drv_ops.h"
#ifdef CONFIG_GAS
#include "gas_serv.h"
#endif /* CONFIG_GAS */
#include "wnm_ap.h"
#ifdef CONFIG_MBO
#include "mbo_ap.h"
#endif /* CONFIG_MBO */
#if defined(CONFIG_PROXYARP) && defined(CONFIG_IPV6)
#include "ndisc_snoop.h"
#endif /* defined(CONFIG_PROXYARP) && defined(CONFIG_IPV6) */
#include "sta_info.h"
#ifndef CONFIG_NO_VLAN
#include "vlan.h"
#endif /* CONFIG_NO_VLAN */
#include "wps_hostapd.h"

#ifdef CONFIG_P2P
#include "p2p_supplicant.h"
#include "supp_p2p.h"
#include "p2p_i.h"
#endif /* CONFIG_P2P */


extern int get_run_mode(void);

static void ap_sta_remove_in_other_bss(struct hostapd_data *hapd,
				       struct sta_info *sta);
static void ap_handle_session_timer(void *eloop_ctx, void *timeout_ctx);
static void ap_handle_session_warning_timer(void *eloop_ctx, void *timeout_ctx);
static void ap_sta_deauth_cb_timeout(void *eloop_ctx, void *timeout_ctx);
static void ap_sta_disassoc_cb_timeout(void *eloop_ctx, void *timeout_ctx);
#ifdef CONFIG_IEEE80211W
static void ap_sa_query_timer(void *eloop_ctx, void *timeout_ctx);
#endif /* CONFIG_IEEE80211W */
static int ap_sta_remove(struct hostapd_data *hapd, struct sta_info *sta);
static void ap_sta_delayed_1x_auth_fail_cb(void *eloop_ctx, void *timeout_ctx);

int ap_for_each_sta(struct hostapd_data *hapd,
		    int (*cb)(struct hostapd_data *hapd, struct sta_info *sta,
			      void *ctx),
		    void *ctx)
{
	struct sta_info *sta;

	for (sta = hapd->sta_list; sta; sta = sta->next) {
		if (cb(hapd, sta, ctx))
			return 1;
	}

	return 0;
}


struct sta_info * ap_get_sta(struct hostapd_data *hapd, const u8 *sta)
{
	struct sta_info *s;

	s = hapd->sta_hash[STA_HASH(sta)];
	while (s != NULL && os_memcmp(s->addr, sta, 6) != 0)
		s = s->hnext;
	return s;
}


#ifdef CONFIG_P2P
struct sta_info * ap_get_sta_p2p(struct hostapd_data *hapd, const u8 *addr)
{
	struct sta_info *sta;

	for (sta = hapd->sta_list; sta; sta = sta->next) {
		const u8 *p2p_dev_addr;

		if (sta->p2p_ie == NULL)
			continue;

		p2p_dev_addr = p2p_get_go_dev_addr(sta->p2p_ie);
		if (p2p_dev_addr == NULL)
			continue;

		if (os_memcmp(p2p_dev_addr, addr, ETH_ALEN) == 0)
			return sta;
	}

	return NULL;
}
#endif /* CONFIG_P2P */


static void ap_sta_list_del(struct hostapd_data *hapd, struct sta_info *sta)
{
	struct sta_info *tmp;

	if (hapd->sta_list == sta) {
		hapd->sta_list = sta->next;
		return;
	}

	tmp = hapd->sta_list;
	while (tmp != NULL && tmp->next != sta)
		tmp = tmp->next;
	if (tmp == NULL) {
		da16x_ap_prt("[%s] Could not remove STA " MACSTR " from "
			    "list.\n", __func__, MAC2STR(sta->addr));
	} else
		tmp->next = sta->next;
}


void ap_sta_hash_add(struct hostapd_data *hapd, struct sta_info *sta)
{
	sta->hnext = hapd->sta_hash[STA_HASH(sta->addr)];
	hapd->sta_hash[STA_HASH(sta->addr)] = sta;
}


static void ap_sta_hash_del(struct hostapd_data *hapd, struct sta_info *sta)
{
	struct sta_info *s;

	s = hapd->sta_hash[STA_HASH(sta->addr)];

	if (s == NULL)
		return;
	
	if (os_memcmp(s->addr, sta->addr, 6) == 0) {
		hapd->sta_hash[STA_HASH(sta->addr)] = s->hnext;
		return;
	}

	while (s->hnext != NULL &&
	       os_memcmp(s->hnext->addr, sta->addr, ETH_ALEN) != 0)
		s = s->hnext;
	if (s->hnext != NULL)
		s->hnext = s->hnext->hnext;
	else
		da16x_ap_prt("[%s] AP: could not remove STA " MACSTR
			   " from hash table\n",
				__func__, MAC2STR(sta->addr));
}


#if 0
void ap_sta_ip6addr_del(struct hostapd_data *hapd, struct sta_info *sta)
{
	sta_ip6addr_del(hapd, sta);
}
#endif /* 0 */


void ap_free_sta(struct hostapd_data *hapd, struct sta_info *sta)
{
	int set_beacon = 0;

#ifdef CONFIG_MESH
	if (hapd == NULL)
		return;	
#endif // CONFIG_MESH

#ifdef	CONFIG_RADIUS
	accounting_sta_stop(hapd, sta);
#endif	/* CONFIG_RADIUS */

	/* just in case */
	ap_sta_set_authorized(hapd, sta, 0);

#ifdef	CONFIG_AP_WDS
	if (sta->flags & WLAN_STA_WDS)
		hostapd_set_wds_sta(hapd, NULL, sta->addr, sta->aid, 0);
#endif	/* CONFIG_AP_WDS */

#if 0
	if (sta->ipaddr)
		hostapd_drv_br_delete_ip_neigh(hapd, 4, (u8 *) &sta->ipaddr);
	ap_sta_ip6addr_del(hapd, sta);
#endif /* 0 */

	if (!hapd->iface->driver_ap_teardown &&
	    !(sta->flags & WLAN_STA_PREAUTH)) {
		hostapd_drv_sta_remove(hapd, sta->addr);
		sta->added_unassoc = 0;
	}

	ap_sta_hash_del(hapd, sta);
	ap_sta_list_del(hapd, sta);

	if (sta->aid > 0)
		hapd->sta_aid[(sta->aid - 1) / 32] &=
			~BIT((sta->aid - 1) % 32);

	hapd->num_sta--;
	if (sta->nonerp_set) {
		sta->nonerp_set = 0;
		hapd->iface->num_sta_non_erp--;
		if (hapd->iface->num_sta_non_erp == 0)
			set_beacon++;
	}

	if (sta->no_short_slot_time_set) {
		sta->no_short_slot_time_set = 0;
		hapd->iface->num_sta_no_short_slot_time--;
		if (hapd->iface->current_mode &&
		    hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211G
		    && hapd->iface->num_sta_no_short_slot_time == 0)
			set_beacon++;
	}

	if (sta->no_short_preamble_set) {
		sta->no_short_preamble_set = 0;
		hapd->iface->num_sta_no_short_preamble--;
		if (hapd->iface->current_mode &&
		    hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211G
		    && hapd->iface->num_sta_no_short_preamble == 0)
			set_beacon++;
	}

	if (sta->no_ht_gf_set) {
		sta->no_ht_gf_set = 0;
		hapd->iface->num_sta_ht_no_gf--;
	}

	if (sta->no_ht_set) {
		sta->no_ht_set = 0;
		hapd->iface->num_sta_no_ht--;
	}

	if (sta->ht_20mhz_set) {
		sta->ht_20mhz_set = 0;
		hapd->iface->num_sta_ht_20mhz--;
	}

#ifdef CONFIG_TAXONOMY
	wpabuf_free(sta->probe_ie_taxonomy);
	sta->probe_ie_taxonomy = NULL;
	wpabuf_free(sta->assoc_ie_taxonomy);
	sta->assoc_ie_taxonomy = NULL;
#endif /* CONFIG_TAXONOMY */

#ifdef CONFIG_IEEE80211N
	ht40_intolerant_remove(hapd->iface, sta);
#endif /* CONFIG_IEEE80211N */

#ifdef CONFIG_P2P
#ifdef CONFIG_P2P_UNUSED_CMD
	if (sta->no_p2p_set) {
		sta->no_p2p_set = 0;
		hapd->num_sta_no_p2p--;
		if (hapd->num_sta_no_p2p == 0)
			hostapd_p2p_non_p2p_sta_disconnected(hapd);
	}
#endif /* CONFIG_P2P_UNUSED_CMD */
#endif /* CONFIG_P2P */

#if defined(NEED_AP_MLME) && defined(CONFIG_IEEE80211N)
	if (hostapd_ht_operation_update(hapd->iface) > 0)
		set_beacon++;
#endif /* NEED_AP_MLME && CONFIG_IEEE80211N */

#ifdef CONFIG_MESH
	if (hapd->mesh_sta_free_cb)
		hapd->mesh_sta_free_cb(hapd, sta);
#endif /* CONFIG_MESH */

	if (set_beacon)
		ieee802_11_set_beacons(hapd->iface);

	da16x_ap_prt("[%s] cancel ap_handle_timer for " MACSTR "\n",
            __func__, MAC2STR(sta->addr));

	da16x_eloop_cancel_timeout(ap_handle_timer, hapd, sta);
	da16x_eloop_cancel_timeout(ap_handle_session_timer, hapd, sta);
	da16x_eloop_cancel_timeout(ap_handle_session_warning_timer, hapd, sta);
	ap_sta_clear_disconnect_timeouts(hapd, sta);
	sae_clear_retransmit_timer(hapd, sta);

	ieee802_1x_free_station(hapd, sta);
	wpa_auth_sta_deinit(sta->wpa_sm);
	rsn_preauth_free_station(hapd, sta);

#ifndef CONFIG_NO_RADIUS
	if (hapd->radius)
		radius_client_flush_auth(hapd->radius, sta->addr);
#endif /* CONFIG_NO_RADIUS */

#ifndef CONFIG_NO_VLAN
	/*
	 * sta->wpa_sm->group needs to be released before so that
	 * vlan_remove_dynamic() can check that no stations are left on the
	 * AP_VLAN netdev.
	 */
	if (sta->vlan_id)
		vlan_remove_dynamic(hapd, sta->vlan_id);
	if (sta->vlan_id_bound) {
		/*
		 * Need to remove the STA entry before potentially removing the
		 * VLAN.
		 */
		if (hapd->iface->driver_ap_teardown &&
		    !(sta->flags & WLAN_STA_PREAUTH)) {
			hostapd_drv_sta_remove(hapd, sta->addr);
			sta->added_unassoc = 0;
		}
		vlan_remove_dynamic(hapd, sta->vlan_id_bound);
	}
#endif /* CONFIG_NO_VLAN */

	os_free(sta->challenge);

#ifdef CONFIG_IEEE80211W
	os_free(sta->sa_query_trans_id);
	da16x_eloop_cancel_timeout(ap_sa_query_timer, hapd, sta);
#endif /* CONFIG_IEEE80211W */

#ifdef CONFIG_P2P
	p2p_group_notif_disassoc(hapd->p2p_group, sta->addr);
#endif /* CONFIG_P2P */

#ifdef CONFIG_INTERWORKING
	if (sta->gas_dialog) {
		int i;
		for (i = 0; i < GAS_DIALOG_MAX; i++)
			gas_serv_dialog_clear(&sta->gas_dialog[i]);
		os_free(sta->gas_dialog);
	}
#endif /* CONFIG_INTERWORKING */

	wpabuf_free(sta->wps_ie);
	wpabuf_free(sta->p2p_ie);
	wpabuf_free(sta->hs20_ie);
#ifdef	CONFIG_SUPP27_ROAM_CONSORTIUM
	wpabuf_free(sta->roaming_consortium);
#endif	/* CONFIG_SUPP27_ROAM_CONSORTIUM */

#ifdef CONFIG_FST
	wpabuf_free(sta->mb_ies);
#endif /* CONFIG_FST */

	os_free(sta->ht_capabilities);
#ifdef CONFIG_IEEE80211AC
	os_free(sta->vht_capabilities);
#endif /* CONFIG_IEEE80211AC */
	hostapd_free_psk_list(sta->psk);
#ifdef CONFIG_RADIUS
	os_free(sta->identity);
	os_free(sta->radius_cui);
#endif /* CONFIG_RADIUS */
#ifdef CONFIG_HS20
	os_free(sta->remediation_url);
	os_free(sta->t_c_url);
	wpabuf_free(sta->hs20_deauth_req);
	os_free(sta->hs20_session_info_url);
#endif /* CONFIG_HS20 */

#ifdef CONFIG_SAE
	sae_clear_data(sta->sae);
	os_free(sta->sae);
#endif /* CONFIG_SAE */

#ifdef CONFIG_MBO
	mbo_ap_sta_free(sta);
#endif /* CONFIG_MBO */
#ifdef CONFIG_SUPP27_STA_INFO
	os_free(sta->supp_op_classes);
#endif /* CONFIG_SUPP27_STA_INFO */

#ifdef CONFIG_FILS
	os_free(sta->fils_pending_assoc_req);
	wpabuf_free(sta->fils_hlp_resp);
	wpabuf_free(sta->hlp_dhcp_discover);
	da16x_eloop_cancel_timeout(fils_hlp_timeout, hapd, sta);
#ifdef CONFIG_FILS_SK_PFS
	crypto_ecdh_deinit(sta->fils_ecdh);
	wpabuf_clear_free(sta->fils_dh_ss);
	wpabuf_free(sta->fils_g_sta);
#endif /* CONFIG_FILS_SK_PFS */
#endif /* CONFIG_FILS */

#ifdef CONFIG_OWE
#ifdef	CONFIG_SUPP27_BIN_CLR_FREE
	bin_clear_free(sta->owe_pmk, sta->owe_pmk_len);
#else
	os_free(sta->owe_pmk);
#endif	/* CONFIG_SUPP27_BIN_CLR_FREE */
	crypto_ecdh_deinit(sta->owe_ecdh);
#endif /* CONFIG_OWE */

#ifdef CONFIG_STA_EXT_CAPAB
	os_free(sta->ext_capability);
#endif /* CONFIG_STA_EXT_CAPAB */

#ifdef CONFIG_WNM_AP
	da16x_eloop_cancel_timeout(ap_sta_reset_steer_flag_timer, hapd, sta);
#endif /* CONFIG_WNM_AP */

#ifdef CONFIG_AP_WDS
	os_free(sta->ifname_wds);
#endif /* CONFIG_AP_WDS */

	os_free(sta);

#ifdef	CONFIG_P2P
	if (hapd->p2p && hapd->num_sta == 0 && hapd->p2p_go_has_peers) {
		if (get_run_mode() != P2P_GO_FIXED_MODE) {            
			wpas_p2p_disconnect(hapd->iface->owner);
		}
	}
#endif	/* CONFIG_P2P */
}


void hostapd_free_stas(struct hostapd_data *hapd)
{
	struct sta_info *sta, *prev;

	sta = hapd->sta_list;

	while (sta) {
		prev = sta;
		if (sta->flags & WLAN_STA_AUTH) {
			mlme_deauthenticate_indication(
				hapd, sta, WLAN_REASON_UNSPECIFIED);
		}
		sta = sta->next;
		da16x_notice_prt("[%s] Removing STA " MACSTR "\n",
			   __func__, MAC2STR(prev->addr));
		ap_free_sta(hapd, prev);
	}
}

/**
 * ap_handle_timer - Per STA timer handler
 * @eloop_ctx: struct hostapd_data *
 * @timeout_ctx: struct sta_info *
 *
 * This function is called to check station activity and to remove inactive
 * stations.
 */
#if 1  /* by Shingu 20190620 */

#undef MESH_KEEP_ALIVE /* MESH ��忡��?Null packet �� ���� ���� ó�� üũ ������ ��. */

void ap_handle_timer(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;
	struct hostap_sta_driver_data data;
	unsigned long next_time = INACTIVE_CHECK_INTERVAL;
	int reason;
	int ret;

	da16x_wnm_prt("[%s] " MACSTR " flags=0x%x timeout_next=%s(%d)\n",
		    __func__, MAC2STR(sta->addr), sta->flags,
		    timeout_next_str(sta->timeout_next), sta->timeout_next);

	if (sta->timeout_next == STA_REMOVE) {
		da16x_wnm_prt("[%s] " MACSTR " deauthenticated due to local deauth request\n",
				__func__, MAC2STR(sta->addr));

		ap_free_sta(hapd, sta);
		return;
	}

	if (sta->timeout_next == STA_DEAUTH)
		next_time = 0;

	ret = hostapd_drv_read_sta_data(hapd, &data, sta->addr);

	if (ret < 0) {
		da16x_wnm_prt("[%s] Check inactivity: Could not "
				"get station info from kernel driver for " MACSTR "\n",
				__func__, MAC2STR(sta->addr));
	}

	if (   (sta->flags & WLAN_STA_ASSOC)
	    && (sta->timeout_next == STA_NULLFUNC ||
	        sta->timeout_next == STA_DISASSOC)
	    && hapd->conf->ap_max_inactivity) {
		if (ret < 0) {
			sta->inact_cnt++;

#if 1   /* by Shingu 20161010 (Keep-alive) */
		} else if (
#if defined CONFIG_MESH && defined MESH_KEEP_ALIVE
			(hapd->conf->ap_send_ka || hapd->conf->mesh & MESH_ENABLED) && data.rx_packets == sta->last_rx_packets
#else
			hapd->conf->ap_send_ka
#endif /* CONFIG_MESH && MESH_KEEP_ALIVE */
		) {
		
#if defined CONFIG_MESH && defined MESH_KEEP_ALIVE
			if (hapd->conf->mesh & MESH_ENABLED) {
				hostapd_drv_chk_keep_alive(hapd, sta->addr); /* data ������ ���� ���?Action msg �켱 ������ ���� */
			}
#endif /* CONFIG_MESH && MESH_KEEP_ALIVE */

			if (hostapd_drv_chk_keep_alive(hapd, sta->addr)) {
				sta->inact_cnt++;
				da16x_wnm_prt("[%s] " MACSTR " Not Alive (inactive >= %d/%d sec)\n",
					__func__, MAC2STR(sta->addr),
					sta->inact_cnt*INACTIVE_CHECK_INTERVAL, hapd->conf->ap_max_inactivity);
			} else {
				sta->inact_cnt = 0;
			}
#endif  /* 1 */
		} else {
#ifdef CONFIG_WNM_BSS_MAX_IDLE_PERIOD_TEST
			sta->inact_cnt++;
#else /* CONFIG_WNM_BSS_MAX_IDLE_PERIOD_TEST */
			da16x_wnm_prt("[%s] " MACSTR " rx_packets=%d last_rx_packets=%d / rx_data_packets=%d last_rx_data_packets=%d\n",
					__func__, MAC2STR(sta->addr),
					data.rx_packets, sta->last_rx_packets,
					data.rx_data_packets, sta->last_rx_data_packets);
#if 0
			da16x_wnm_prt("[%s] tx_packets = %d, last_tx_packets = %d \n",
					__func__, data.tx_packets, sta->last_tx_packets);
#endif /* 0 */

			/* Check inactivity */
			if (
#ifdef CONFIG_MESH
				((hapd->conf->mesh & MESH_ENABLED) && (data.rx_packets == sta->last_rx_packets)) /* MESH */
					|| (!(hapd->conf->mesh & MESH_ENABLED) && (data.rx_data_packets == sta->last_rx_data_packets)) /* SoftAP */
#else
				data.rx_data_packets == sta->last_rx_data_packets
#endif /* CONFIG_MESH */
			) { /* SoftAP : rx_data_packets: X, MESH: rx_packets: X*/
#ifdef CONFIG_MESH				
				if (hapd->conf->mesh & MESH_ENABLED) { /* MESH */
					da16x_notice_prt("Inactive too long: Remove connected Mesh Station(" MACSTR ")\n", MAC2STR(sta->addr));
					next_time = 0;
					sta->timeout_next = STA_REMOVE;	
					//retrun to original channel
					if (get_run_mode() != STA_MESH_PORTAL_MODE) {
						extern int fc80211_mesh_switch_channel(int , int);
						fc80211_mesh_switch_channel(1, 0);
					}
				} else 
#endif /* CONFIG_MESH */
				{	/* SoftAP */
					sta->inact_cnt++;
				}
			} else {
				sta->inact_cnt = 0;
			}
#endif /* CONFIG_WNM_BSS_MAX_IDLE_PERIOD_TEST */
		}

		if (
#ifdef CONFIG_WNM_BSS_MAX_IDLE_PERIOD_TEST
			sta->inact_cnt * INACTIVE_CHECK_INTERVAL >= 60)
#else
			sta->inact_cnt * INACTIVE_CHECK_INTERVAL >= hapd->conf->ap_max_inactivity
#endif /* CONFIG_WNM_BSS_MAX_IDLE_PERIOD_TEST */
		) {
			da16x_notice_prt("STA " MACSTR " has been "
					"inactive too long: max allowed: %d\n",
					MAC2STR(sta->addr),
					hapd->conf->ap_max_inactivity);

			sta->timeout_next = STA_DISASSOC;
			next_time = 0;
#if 1	/* by Shingu 20161017 (Keep-alive) */
		} else if (hapd->conf->ap_max_inactivity - (sta->inact_cnt * INACTIVE_CHECK_INTERVAL) < INACTIVE_CHECK_INTERVAL) {
			next_time = hapd->conf->ap_max_inactivity - (sta->inact_cnt * INACTIVE_CHECK_INTERVAL);
#endif	/* 1 */
		}
	}
#ifdef CONFIG_MESH
	else if (hapd->conf->mesh & MESH_ENABLED && !(sta->flags & WLAN_STA_ASSOC)) { /* �� ���� �ܸ� */
		if (sta->plink_state == PLINK_BLOCKED || (sta->plink_state == PLINK_HOLDING && data.rx_packets == sta->last_rx_packets))
		{	/* Peer Unassoc. ���� �̸�, PLINK_BLOCKED �Ǵ� PLINK_HOLDING �̰� Rx_packet(mgmt ����) ��ȭ ���� ���? ���?Remove */
			da16x_notice_prt("Remove unconnected Mesh Station(" MACSTR ") plink_state=%d\n", MAC2STR(sta->addr), sta->plink_state);
			next_time = 0;
			sta->timeout_next = STA_REMOVE;
		}
	}
#endif /* CONFIG_MESH */

	/* Last packets update */
	sta->last_rx_packets = data.rx_packets; 		// All Rx
	sta->last_rx_data_packets = data.rx_data_packets;	// Data only
	sta->last_tx_packets = data.tx_packets;

	if (next_time) {
		da16x_wnm_prt("[%s] " MACSTR " Register <ap_handle_timer> timeout (%lu seconds)\n",
			    __func__, MAC2STR(sta->addr), next_time);

		da16x_eloop_register_timeout(next_time, 0, ap_handle_timer,
					    hapd, sta);

		return;
	}

	if (sta->timeout_next != STA_REMOVE) {
		int deauth = sta->timeout_next == STA_DEAUTH;

		da16x_wnm_prt("[%s] " MACSTR "Timeout, sending %s info to STA\n",
				__func__, MAC2STR(sta->addr),
				deauth ? "deauthentication" : "disassociation");

		if (deauth) {
			hostapd_drv_sta_deauth(hapd, sta->addr,
					WLAN_REASON_PREV_AUTH_NOT_VALID);
		} else {
			reason = (sta->timeout_next == STA_DISASSOC) ?
				WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY :
				WLAN_REASON_PREV_AUTH_NOT_VALID;

			hostapd_drv_sta_disassoc(hapd, sta->addr, reason);
		}
	}

	switch (sta->timeout_next) {
		case STA_NULLFUNC:
			sta->timeout_next = STA_DISASSOC;
			da16x_wnm_prt("[%s] " MACSTR " Register <ap_handle_timer> timeout (%d seconds - AP_DISASSOC_DELAY)\n",
				    __func__, MAC2STR(sta->addr), AP_DISASSOC_DELAY);

			da16x_eloop_register_timeout(AP_DISASSOC_DELAY, 0,
						    ap_handle_timer, hapd, sta);
			break;

		case STA_DISASSOC:
		case STA_DISASSOC_FROM_CLI:
			ap_sta_set_authorized(hapd, sta, 0);
			sta->flags &= ~WLAN_STA_ASSOC;
#ifdef	CONFIG_RADIUS
			ieee802_1x_notify_port_enabled(sta->eapol_sm, 0);
			if (!sta->acct_terminate_cause)
				sta->acct_terminate_cause =
					RADIUS_ACCT_TERMINATE_CAUSE_IDLE_TIMEOUT;

			accounting_sta_stop(hapd, sta);
#endif	/* CONFIG_RADIUS */
			ieee802_1x_free_station(hapd, sta);

			da16x_wnm_prt("[%s] " MACSTR " disassociated due to inactivity\n", __func__, MAC2STR(sta->addr));

			reason = (sta->timeout_next == STA_DISASSOC) ?
				WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY :
				WLAN_REASON_PREV_AUTH_NOT_VALID;
			sta->timeout_next = STA_DEAUTH;

			da16x_ap_prt("[%s] " MACSTR " register <ap_handle_timer> timeout (%d seconds - AP_DEAUTH_DELAY)\n",
				    __func__, MAC2STR(sta->addr), AP_DEAUTH_DELAY);

			da16x_eloop_register_timeout(AP_DEAUTH_DELAY, 0, ap_handle_timer,
						    hapd, sta);

			mlme_disassociate_indication(hapd, sta, reason);
			ap_free_sta(hapd, sta);
			break;

		case STA_DEAUTH:
		case STA_REMOVE:
			da16x_ap_prt("[%s] " MACSTR " deauthenticated due to "
				    "inactivity (timer DEAUTH/REMOVE)\n", __func__, MAC2STR(sta->addr));
#ifdef	CONFIG_RADIUS
			if (!sta->acct_terminate_cause)
				sta->acct_terminate_cause =
					RADIUS_ACCT_TERMINATE_CAUSE_IDLE_TIMEOUT;
#endif	/* CONFIG_RADIUS */

			mlme_deauthenticate_indication(hapd, sta,
						       WLAN_REASON_PREV_AUTH_NOT_VALID);
			ap_free_sta(hapd, sta);
			break;
	}
}
#else
void ap_handle_timer(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;
	unsigned long next_time = INACTIVE_CHECK_INTERVAL;
	int reason;

	wpa_printf_dbg(MSG_DEBUG, "%s: %s: " MACSTR " flags=0x%x timeout_next=%d",
		   hapd->conf->iface, __func__, MAC2STR(sta->addr), sta->flags,
		   sta->timeout_next);
	if (sta->timeout_next == STA_REMOVE) {
		hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_INFO, "deauthenticated due to "
			       "local deauth request");
		ap_free_sta(hapd, sta);
		return;
	}

	if ((sta->flags & WLAN_STA_ASSOC) &&
	    (sta->timeout_next == STA_NULLFUNC ||
	     sta->timeout_next == STA_DISASSOC) && hapd->conf->ap_max_inactivity) {
		int inactive_sec;
		/*
		 * Add random value to timeout so that we don't end up bouncing
		 * all stations at the same time if we have lots of associated
		 * stations that are idle (but keep re-associating).
		 */
		int fuzz = os_random() % 20;
		inactive_sec = hostapd_drv_get_inact_sec(hapd, sta->addr);
		if (inactive_sec == -1) {
			wpa_msg(hapd->msg_ctx, MSG_DEBUG,
				"Check inactivity: Could not "
				"get station info from kernel driver for "
				MACSTR, MAC2STR(sta->addr));
			/*
			 * The driver may not support this functionality.
			 * Anyway, try again after the next inactivity timeout,
			 * but do not disconnect the station now.
			 */
			next_time = hapd->conf->ap_max_inactivity + fuzz;
		} else if (inactive_sec == -ENOENT) {
			wpa_msg(hapd->msg_ctx, MSG_DEBUG,
				"Station " MACSTR " has lost its driver entry",
				MAC2STR(sta->addr));

			/* Avoid sending client probe on removed client */
			sta->timeout_next = STA_DISASSOC;
			goto skip_poll;
		} else if (inactive_sec < hapd->conf->ap_max_inactivity) {
			/* station activity detected; reset timeout state */
			wpa_msg(hapd->msg_ctx, MSG_DEBUG,
				"Station " MACSTR " has been active %is ago",
				MAC2STR(sta->addr), inactive_sec);
			sta->timeout_next = STA_NULLFUNC;
			next_time = hapd->conf->ap_max_inactivity + fuzz -
				inactive_sec;
		} else {
			wpa_msg(hapd->msg_ctx, MSG_DEBUG,
				"Station " MACSTR " has been "
				"inactive too long: %d sec, max allowed: %d",
				MAC2STR(sta->addr), inactive_sec,
				hapd->conf->ap_max_inactivity);

			if (hapd->conf->skip_inactivity_poll)
				sta->timeout_next = STA_DISASSOC;
		}
	}

	if ((sta->flags & WLAN_STA_ASSOC) &&
	    sta->timeout_next == STA_DISASSOC &&
	    !(sta->flags & WLAN_STA_PENDING_POLL) &&
	    !hapd->conf->skip_inactivity_poll) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG, "Station " MACSTR
			" has ACKed data poll", MAC2STR(sta->addr));
		/* data nullfunc frame poll did not produce TX errors; assume
		 * station ACKed it */
		sta->timeout_next = STA_NULLFUNC;
		next_time = hapd->conf->ap_max_inactivity;
	}

skip_poll:
	if (next_time) {
		wpa_printf_dbg(MSG_DEBUG, "%s: register ap_handle_timer timeout "
			   "for " MACSTR " (%lu seconds)",
			   __func__, MAC2STR(sta->addr), next_time);
		da16x_eloop_register_timeout(next_time, 0, ap_handle_timer, hapd,
				       sta);
		return;
	}

	if (sta->timeout_next == STA_NULLFUNC &&
	    (sta->flags & WLAN_STA_ASSOC)) {
		wpa_printf_dbg(MSG_DEBUG, "  Polling STA");
		sta->flags |= WLAN_STA_PENDING_POLL;
		hostapd_drv_poll_client(hapd, hapd->own_addr, sta->addr,
					sta->flags & WLAN_STA_WMM);
	} else if (sta->timeout_next != STA_REMOVE) {
		int deauth = sta->timeout_next == STA_DEAUTH;

		wpa_dbg(hapd->msg_ctx, MSG_DEBUG,
			"Timeout, sending %s info to STA " MACSTR,
			deauth ? "deauthentication" : "disassociation",
			MAC2STR(sta->addr));

		if (deauth) {
			hostapd_drv_sta_deauth(
				hapd, sta->addr,
				WLAN_REASON_PREV_AUTH_NOT_VALID);
		} else {
			reason = (sta->timeout_next == STA_DISASSOC) ?
				WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY :
				WLAN_REASON_PREV_AUTH_NOT_VALID;

			hostapd_drv_sta_disassoc(hapd, sta->addr, reason);
		}
	}

	switch (sta->timeout_next) {
	case STA_NULLFUNC:
		sta->timeout_next = STA_DISASSOC;
		wpa_printf_dbg(MSG_DEBUG, "%s: register ap_handle_timer timeout "
			   "for " MACSTR " (%d seconds - AP_DISASSOC_DELAY)",
			   __func__, MAC2STR(sta->addr), AP_DISASSOC_DELAY);
		da16x_eloop_register_timeout(AP_DISASSOC_DELAY, 0, ap_handle_timer,
				       hapd, sta);
		break;
	case STA_DISASSOC:
	case STA_DISASSOC_FROM_CLI:
		ap_sta_set_authorized(hapd, sta, 0);
		sta->flags &= ~WLAN_STA_ASSOC;
#ifdef	CONFIG_RADIUS
		ieee802_1x_notify_port_enabled(sta->eapol_sm, 0);
		if (!sta->acct_terminate_cause)
			sta->acct_terminate_cause =
				RADIUS_ACCT_TERMINATE_CAUSE_IDLE_TIMEOUT;

		accounting_sta_stop(hapd, sta);
#endif	/* CONFIG_RADIUS */
		ieee802_1x_free_station(hapd, sta);
		hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_INFO, "disassociated due to "
			       "inactivity");
		reason = (sta->timeout_next == STA_DISASSOC) ?
			WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY :
			WLAN_REASON_PREV_AUTH_NOT_VALID;
		sta->timeout_next = STA_DEAUTH;
		wpa_printf_dbg(MSG_DEBUG, "%s: register ap_handle_timer timeout "
			   "for " MACSTR " (%d seconds - AP_DEAUTH_DELAY)",
			   __func__, MAC2STR(sta->addr), AP_DEAUTH_DELAY);
		da16x_eloop_register_timeout(AP_DEAUTH_DELAY, 0, ap_handle_timer,
				       hapd, sta);
		mlme_disassociate_indication(hapd, sta, reason);
		ap_free_sta(hapd, sta);
		break;

	case STA_DEAUTH:
	case STA_REMOVE:
		hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_INFO, "deauthenticated due to "
			       "inactivity (timer DEAUTH/REMOVE)");

#ifdef	CONFIG_RADIUS
		if (!sta->acct_terminate_cause)
			sta->acct_terminate_cause =
				RADIUS_ACCT_TERMINATE_CAUSE_IDLE_TIMEOUT;
#endif	/* CONFIG_RADIUS */

		mlme_deauthenticate_indication(
			hapd, sta,
			WLAN_REASON_PREV_AUTH_NOT_VALID);
		ap_free_sta(hapd, sta);
		break;
	}
}
#endif

static void ap_handle_session_timer(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;

	if (!(sta->flags & WLAN_STA_AUTH)) {
		if (sta->flags & WLAN_STA_GAS) {
			da16x_ap_prt("[%s] GAS: Remove temporary STA "
				    "entry " MACSTR "\n",
				    __func__, MAC2STR(sta->addr));

			ap_free_sta(hapd, sta);
		}
		return;
	}

	hostapd_drv_sta_deauth(hapd, sta->addr,
			       WLAN_REASON_PREV_AUTH_NOT_VALID);
	mlme_deauthenticate_indication(hapd, sta,
				       WLAN_REASON_PREV_AUTH_NOT_VALID);
	da16x_ap_prt("[%s] deauthenticated due to session timeout\n", __func__);

#ifdef	CONFIG_RADIUS
	sta->acct_terminate_cause =
		RADIUS_ACCT_TERMINATE_CAUSE_SESSION_TIMEOUT;
#endif	/* CONFIG_RADIUS */
	ap_free_sta(hapd, sta);
}

#if 0 //[[ trinity_0150416_BEGIN -- not used
void ap_sta_replenish_timeout(struct hostapd_data *hapd, struct sta_info *sta,
			      u32 session_timeout)
{
#if 0	/* by Bright */
	if (eloop_replenish_timeout(session_timeout, 0,
				    ap_handle_session_timer, hapd, sta) == 1) {
		da16x_ap_prt("[%s] setting session timeout to %d seconds\n",
			    __func__, session_timeout);
	}
#else
	da16x_ap_prt("[%s] What do I do for this ???\n", __func__);
#endif	/* 0 */
}
#endif //]] trinity_0150416_END -- not used


void ap_sta_session_timeout(struct hostapd_data *hapd, struct sta_info *sta,
			    u32 session_timeout)
{
	da16x_ap_prt("[%s] setting session timeout to %d seconds\n",
		    __func__, session_timeout);

	da16x_eloop_cancel_timeout(ap_handle_session_timer, hapd, sta);
	da16x_eloop_register_timeout(session_timeout, 0, ap_handle_session_timer,
			       hapd, sta);

}


void ap_sta_no_session_timeout(struct hostapd_data *hapd, struct sta_info *sta)
{
#if 0	/* by Bright */
	eloop_cancel_timeout(ap_handle_session_timer, hapd, sta);
#else
	da16x_ap_prt("[%s] What do I do for this ???\n", __func__);
#endif	/* 0 */
}

static void ap_handle_session_warning_timer(void *eloop_ctx, void *timeout_ctx)
{
#ifdef CONFIG_WNM_BSS_TRANS_MGMT
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;

	da16x_ap_prt("[%s] WNM: Session warning time reached for " MACSTR "\n",
		    __func__, MAC2STR(sta->addr));

	if (sta->hs20_session_info_url == NULL)
		return;

	wnm_send_ess_disassoc_imminent(hapd, sta, sta->hs20_session_info_url,
				       sta->hs20_disassoc_timer);
#endif /* CONFIG_WNM_BSS_TRANS_MGMT */
}

void ap_sta_session_warning_timeout(struct hostapd_data *hapd,
				    struct sta_info *sta, int warning_time)
{
	da16x_eloop_cancel_timeout(ap_handle_session_warning_timer, hapd, sta);
	da16x_eloop_register_timeout(warning_time, 0, ap_handle_session_warning_timer,
			       hapd, sta);
}


struct sta_info * ap_sta_add(struct hostapd_data *hapd, const u8 *addr)
{
	struct sta_info *sta;

	sta = ap_get_sta(hapd, addr);
	if (sta)
		return sta;

	da16x_ap_prt("[%s] New STA - " MACSTR "\n",
		__func__, MAC2STR(addr));

	if (hapd->num_sta >= hapd->conf->max_num_sta) {
		/* FIX: might try to remove some old STAs first? */
		da16x_debug_prt("[%s] No more free space to connect new STAs (%d/%d) - " MACSTR "\n",
			    __func__, hapd->num_sta, hapd->conf->max_num_sta,
			    MAC2STR(addr));
		return NULL;
	}

	sta = os_zalloc(sizeof(struct sta_info));
	if (sta == NULL) {
		da16x_err_prt("[%s] malloc failed\n", __func__);
		return NULL;
	}
#ifdef	CONFIG_P2P_UNUSED_CMD
	sta->acct_interim_interval = hapd->conf->acct_interim_interval;
#endif	/* CONFIG_P2P_UNUSED_CMD */

#ifdef	CONFIG_RADIUS
	if (accounting_sta_get_id(hapd, sta) < 0) {
		os_free(sta);
		return NULL;
	}
#endif	/* CONFIG_RADIUS */

	if (!(hapd->iface->drv_flags & WPA_DRIVER_FLAGS_INACTIVITY_TIMER)) {
		da16x_ap_prt("[%s] register ap_handle_timer timeout "
			    "for " MACSTR " (%d seconds)\n",
			    __func__, MAC2STR(addr), INACTIVE_CHECK_INTERVAL);

		da16x_eloop_register_timeout(INACTIVE_CHECK_INTERVAL, 0,
				   ap_handle_timer, hapd, sta);
	}

	/* initialize STA info data */
	sta->inact_cnt = 0;
	sta->last_rx_packets = sta->last_tx_packets = 0;
	os_memcpy(sta->addr, addr, ETH_ALEN);
	sta->next = hapd->sta_list;
	hapd->sta_list = sta;
	hapd->num_sta++;
	ap_sta_hash_add(hapd, sta);
//	sta->ssid = &hapd->conf->ssid;
	ap_sta_remove_in_other_bss(hapd, sta);

	return sta;
}


static int ap_sta_remove(struct hostapd_data *hapd, struct sta_info *sta)
{
#ifdef	CONFIG_RADIUS
	ieee802_1x_notify_port_enabled(sta->eapol_sm, 0);
#endif	/* CONFIG_RADIUS */

	da16x_notice_prt("[%s] Removing STA " MACSTR " from kernel driver\n",
		    __func__, MAC2STR(sta->addr));

	if (hostapd_drv_sta_remove(hapd, sta->addr) &&
	    sta->flags & WLAN_STA_ASSOC) {
		da16x_err_prt("[%s] Could not remove STA " MACSTR
			    " from kernel driver.\n",
			    __func__, MAC2STR(sta->addr));

		return -1;
	}
	sta->added_unassoc = 0;
	return 0;
}


static void ap_sta_remove_in_other_bss(struct hostapd_data *hapd,
				       struct sta_info *sta)
{
	struct hostapd_iface *iface = hapd->iface;
	size_t i;

	for (i = 0; i < iface->num_bss; i++) {
		struct hostapd_data *bss = iface->bss[i];
		struct sta_info *sta2;
		/* bss should always be set during operation, but it may be
		 * NULL during reconfiguration. Assume the STA is not
		 * associated to another BSS in that case to avoid NULL pointer
		 * dereferences. */
		if (bss == hapd || bss == NULL)
			continue;
		sta2 = ap_get_sta(bss, sta->addr);
		if (!sta2)
			continue;

		ap_sta_disconnect(bss, sta2, sta2->addr,
				  WLAN_REASON_PREV_AUTH_NOT_VALID);
	}
}


static void ap_sta_disassoc_cb_timeout(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;

	ap_sta_remove(hapd, sta);
	mlme_disassociate_indication(hapd, sta, sta->disassoc_reason);
}


void ap_sta_disassociate(struct hostapd_data *hapd, struct sta_info *sta,
			 u16 reason)
{
	da16x_notice_prt("[%s] disassoc STA " MACSTR "\n",
			__func__, MAC2STR(sta->addr));

	sta->flags &= ~(WLAN_STA_ASSOC | WLAN_STA_ASSOC_REQ_OK);
	ap_sta_set_authorized(hapd, sta, 0);
	sta->timeout_next = STA_DEAUTH;

	da16x_ap_prt("[%s] reschedule ap_handle_timer timeout "
		    "for " MACSTR " (%d seconds - "
			    "AP_DISASSOC_DELAY)\n",
			    __func__, MAC2STR(sta->addr),
			    AP_DISASSOC_DELAY);

	da16x_eloop_cancel_timeout(ap_handle_timer, hapd, sta);
	da16x_eloop_register_timeout(AP_DISASSOC_DELAY, 0,
				    ap_handle_timer, hapd, sta);

#ifdef	CONFIG_RADIUS
	accounting_sta_stop(hapd, sta);
#endif	/* CONFIG_RADIUS */
	ieee802_1x_free_station(hapd, sta);

	sta->disassoc_reason = reason;
	sta->flags |= WLAN_STA_PENDING_DISASSOC_CB;

	da16x_eloop_cancel_timeout(ap_sta_disassoc_cb_timeout, hapd, sta);
	da16x_eloop_register_timeout(hapd->iface->drv_flags &
			       WPA_DRIVER_FLAGS_DEAUTH_TX_STATUS ? 2 : 0, 0,
			       ap_sta_disassoc_cb_timeout, hapd, sta);
}


static void ap_sta_deauth_cb_timeout(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;

	ap_sta_remove(hapd, sta);
	mlme_deauthenticate_indication(hapd, sta, sta->deauth_reason);
}


void ap_sta_deauthenticate(struct hostapd_data *hapd, struct sta_info *sta,
			   u16 reason)
{
	da16x_notice_prt("[%s] deauth STA " MACSTR "\n",
			__func__, MAC2STR(sta->addr));

	wpa_printf_dbg(MSG_DEBUG, "%s: deauthenticate STA " MACSTR,
		   hapd->conf->iface, MAC2STR(sta->addr));
#ifdef CONFIG_SUPP27_STA_INFO
	sta->last_seq_ctrl = WLAN_INVALID_MGMT_SEQ;
#endif /* CONFIG_SUPP27_STA_INFO */
	sta->flags &= ~(WLAN_STA_AUTH | WLAN_STA_ASSOC | WLAN_STA_ASSOC_REQ_OK);
	ap_sta_set_authorized(hapd, sta, 0);
	sta->timeout_next = STA_REMOVE;
	wpa_printf_dbg(MSG_DEBUG, "%s: reschedule ap_handle_timer timeout "
		   "for " MACSTR " (%d seconds - "
		   "AP_MAX_INACTIVITY_AFTER_DEAUTH)",
		   __func__, MAC2STR(sta->addr),
		   AP_MAX_INACTIVITY_AFTER_DEAUTH);
	da16x_eloop_cancel_timeout(ap_handle_timer, hapd, sta);
	da16x_eloop_register_timeout(AP_MAX_INACTIVITY_AFTER_DEAUTH, 0,
			       ap_handle_timer, hapd, sta);
#ifdef	CONFIG_RADIUS
	accounting_sta_stop(hapd, sta);
#endif	/* CONFIG_RADIUS */
	ieee802_1x_free_station(hapd, sta);

	sta->deauth_reason = reason;
	sta->flags |= WLAN_STA_PENDING_DEAUTH_CB;
	da16x_eloop_cancel_timeout(ap_sta_deauth_cb_timeout, hapd, sta);
	da16x_eloop_register_timeout(hapd->iface->drv_flags &
			       WPA_DRIVER_FLAGS_DEAUTH_TX_STATUS ? 2 : 0, 0,
			       ap_sta_deauth_cb_timeout, hapd, sta);
}


#ifdef CONFIG_WPS
int ap_sta_wps_cancel(struct hostapd_data *hapd,
		      struct sta_info *sta, void *ctx)
{
	if (sta && (sta->flags & WLAN_STA_WPS)) {
		ap_sta_deauthenticate(hapd, sta,
				      WLAN_REASON_PREV_AUTH_NOT_VALID);
		wpa_printf_dbg(MSG_DEBUG, "WPS: %s: Deauth sta=" MACSTR,
			   __func__, MAC2STR(sta->addr));
		return 1;
	}

	return 0;
}
#endif /* CONFIG_WPS */


#ifndef CONFIG_NO_VLAN
static int ap_sta_get_free_vlan_id(struct hostapd_data *hapd)
{
	struct hostapd_vlan *vlan;
	int vlan_id = MAX_VLAN_ID + 2;

retry:
	for (vlan = hapd->conf->vlan; vlan; vlan = vlan->next) {
		if (vlan->vlan_id == vlan_id) {
			vlan_id++;
			goto retry;
		}
	}
	return vlan_id;
}

int ap_sta_set_vlan(struct hostapd_data *hapd, struct sta_info *sta,
		    struct vlan_description *vlan_desc)
{

	struct hostapd_vlan *vlan = NULL, *wildcard_vlan = NULL;
	int old_vlan_id, vlan_id = 0, ret = 0;

	if (hapd->conf->ssid.dynamic_vlan == DYNAMIC_VLAN_DISABLED)
		vlan_desc = NULL;

	/* Check if there is something to do */
	if (hapd->conf->ssid.per_sta_vif && !sta->vlan_id) {
		/* This sta is lacking its own vif */
	} else if (hapd->conf->ssid.dynamic_vlan == DYNAMIC_VLAN_DISABLED &&
		   !hapd->conf->ssid.per_sta_vif && sta->vlan_id) {
		/* sta->vlan_id needs to be reset */
	} else if (!vlan_compare(vlan_desc, sta->vlan_desc)) {
		return 0; /* nothing to change */
	}

	/* Now the real VLAN changed or the STA just needs its own vif */
	if (hapd->conf->ssid.per_sta_vif) {
		/* Assign a new vif, always */
		/* find a free vlan_id sufficiently big */
		vlan_id = ap_sta_get_free_vlan_id(hapd);
		/* Get wildcard VLAN */
		for (vlan = hapd->conf->vlan; vlan; vlan = vlan->next) {
			if (vlan->vlan_id == VLAN_ID_WILDCARD)
				break;
		}
		if (!vlan) {
			hostapd_logger(hapd, sta->addr,
				       HOSTAPD_MODULE_IEEE80211,
				       HOSTAPD_LEVEL_DEBUG,
				       "per_sta_vif missing wildcard");
			vlan_id = 0;
			ret = -1;
			goto done;
		}
	} else if (vlan_desc && vlan_desc->notempty) {
		for (vlan = hapd->conf->vlan; vlan; vlan = vlan->next) {
			if (!vlan_compare(&vlan->vlan_desc, vlan_desc))
				break;
			if (vlan->vlan_id == VLAN_ID_WILDCARD)
				wildcard_vlan = vlan;
		}
		if (vlan) {
			vlan_id = vlan->vlan_id;
		} else if (wildcard_vlan) {
			vlan = wildcard_vlan;
			vlan_id = vlan_desc->untagged;
			if (vlan_desc->tagged[0]) {
				/* Tagged VLAN configuration */
				vlan_id = ap_sta_get_free_vlan_id(hapd);
			}
		} else {
			hostapd_logger(hapd, sta->addr,
				       HOSTAPD_MODULE_IEEE80211,
				       HOSTAPD_LEVEL_DEBUG,
				       "missing vlan and wildcard for vlan=%d%s",
				       vlan_desc->untagged,
				       vlan_desc->tagged[0] ? "+" : "");
			vlan_id = 0;
			ret = -1;
			goto done;
		}
	}

	if (vlan && vlan->vlan_id == VLAN_ID_WILDCARD) {
		vlan = vlan_add_dynamic(hapd, vlan, vlan_id, vlan_desc);
		if (vlan == NULL) {
			hostapd_logger(hapd, sta->addr,
				       HOSTAPD_MODULE_IEEE80211,
				       HOSTAPD_LEVEL_DEBUG,
				       "could not add dynamic VLAN interface for vlan=%d%s",
				       vlan_desc ? vlan_desc->untagged : -1,
				       (vlan_desc && vlan_desc->tagged[0]) ?
				       "+" : "");
			vlan_id = 0;
			ret = -1;
			goto done;
		}

		hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_DEBUG,
			       "added new dynamic VLAN interface '%s'",
			       vlan->ifname);
	} else if (vlan && vlan->dynamic_vlan > 0) {
		vlan->dynamic_vlan++;
		hostapd_logger(hapd, sta->addr,
			       HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_DEBUG,
			       "updated existing dynamic VLAN interface '%s'",
			       vlan->ifname);
	}
done:
	old_vlan_id = sta->vlan_id;
	sta->vlan_id = vlan_id;
	sta->vlan_desc = vlan ? &vlan->vlan_desc : NULL;

	if (vlan_id != old_vlan_id && old_vlan_id)
		vlan_remove_dynamic(hapd, old_vlan_id);

	return ret;
}

int ap_sta_bind_vlan(struct hostapd_data *hapd, struct sta_info *sta)
{
	const char *iface;
	struct hostapd_vlan *vlan = NULL;
	int ret;
	int old_vlanid = sta->vlan_id_bound;

	iface = hapd->conf->iface;
	if (hapd->conf->ssid.vlan[0])
		iface = hapd->conf->ssid.vlan;

	if (sta->vlan_id > 0) {
		for (vlan = hapd->conf->vlan; vlan; vlan = vlan->next) {
			if (vlan->vlan_id == sta->vlan_id)
				break;
		}
		if (vlan)
			iface = vlan->ifname;
	}

	/*
	 * Do not increment ref counters if the VLAN ID remains same, but do
	 * not skip hostapd_drv_set_sta_vlan() as hostapd_drv_sta_remove() might
	 * have been called before.
	 */
	if (sta->vlan_id == old_vlanid)
		goto skip_counting;

	if (sta->vlan_id > 0 && vlan == NULL) {
		hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_DEBUG, "could not find VLAN for "
			       "binding station to (vlan_id=%d)",
			       sta->vlan_id);
		ret = -1;
		goto done;
	} else if (vlan && vlan->dynamic_vlan > 0) {
		vlan->dynamic_vlan++;
		hostapd_logger(hapd, sta->addr,
			       HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_DEBUG,
			       "updated existing dynamic VLAN interface '%s'",
			       iface);
	}

	/* ref counters have been increased, so mark the station */
	sta->vlan_id_bound = sta->vlan_id;

skip_counting:
	hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
		       HOSTAPD_LEVEL_DEBUG, "binding station to interface "
		       "'%s'", iface);

	if (wpa_auth_sta_set_vlan(sta->wpa_sm, sta->vlan_id) < 0)
		wpa_printf(MSG_INFO, "Failed to update VLAN-ID for WPA");

	ret = hostapd_drv_set_sta_vlan(iface, hapd, sta->addr, sta->vlan_id);
	if (ret < 0) {
		hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_DEBUG, "could not bind the STA "
			       "entry to vlan_id=%d", sta->vlan_id);
	}

	/* During 1x reauth, if the vlan id changes, then remove the old id. */
	if (old_vlanid > 0 && old_vlanid != sta->vlan_id)
		vlan_remove_dynamic(hapd, old_vlanid);
done:

	return ret;
}
#endif /* CONFIG_NO_VLAN */


#ifdef CONFIG_IEEE80211W

int ap_check_sa_query_timeout(struct hostapd_data *hapd, struct sta_info *sta)
{
	u32 tu;
	struct os_reltime now, passed;
	os_get_reltime(&now);
	os_reltime_sub(&now, &sta->sa_query_start, &passed);
	tu = (passed.sec * 1000000 + passed.usec) / 1024;
	if (hapd->conf->assoc_sa_query_max_timeout < tu) {
		hostapd_logger(hapd, sta->addr,
			       HOSTAPD_MODULE_IEEE80211,
			       HOSTAPD_LEVEL_DEBUG,
			       "association SA Query timed out");
		sta->sa_query_timed_out = 1;
		os_free(sta->sa_query_trans_id);
		sta->sa_query_trans_id = NULL;
		sta->sa_query_count = 0;
		da16x_eloop_cancel_timeout(ap_sa_query_timer, hapd, sta);
		return 1;
	}

	return 0;
}


static void ap_sa_query_timer(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;
	unsigned int timeout, sec, usec;
	u8 *trans_id, *nbuf;

	wpa_printf_dbg(MSG_DEBUG, "%s: SA Query timer for STA " MACSTR
		   " (count=%d)",
		   hapd->conf->iface, MAC2STR(sta->addr), sta->sa_query_count);

	if (sta->sa_query_count > 0 &&
	    ap_check_sa_query_timeout(hapd, sta))
		return;

	nbuf = os_realloc_array(sta->sa_query_trans_id,
				sta->sa_query_count + 1,
				WLAN_SA_QUERY_TR_ID_LEN);
	if (nbuf == NULL)
		return;
	if (sta->sa_query_count == 0) {
		/* Starting a new SA Query procedure */
		os_get_reltime(&sta->sa_query_start);
	}
	trans_id = nbuf + sta->sa_query_count * WLAN_SA_QUERY_TR_ID_LEN;
	sta->sa_query_trans_id = nbuf;
	sta->sa_query_count++;

	if (os_get_random(trans_id, WLAN_SA_QUERY_TR_ID_LEN) < 0) {
		/*
		 * We don't really care which ID is used here, so simply
		 * hardcode this if the mostly theoretical os_get_random()
		 * failure happens.
		 */
		trans_id[0] = 0x12;
		trans_id[1] = 0x34;
	}

	timeout = hapd->conf->assoc_sa_query_retry_timeout;
	sec = ((timeout / 1000) * 1024) / 1000;
	usec = (timeout % 1000) * 1024;
	da16x_eloop_register_timeout(sec, usec, ap_sa_query_timer, hapd, sta);

	hostapd_logger(hapd, sta->addr, HOSTAPD_MODULE_IEEE80211,
		       HOSTAPD_LEVEL_DEBUG,
		       "association SA Query attempt %d", sta->sa_query_count);

	ieee802_11_send_sa_query_req(hapd, sta->addr, trans_id);
}


void ap_sta_start_sa_query(struct hostapd_data *hapd, struct sta_info *sta)
{
	ap_sa_query_timer(hapd, sta);
}


void ap_sta_stop_sa_query(struct hostapd_data *hapd, struct sta_info *sta)
{
	da16x_eloop_cancel_timeout(ap_sa_query_timer, hapd, sta);
	os_free(sta->sa_query_trans_id);
	sta->sa_query_trans_id = NULL;
	sta->sa_query_count = 0;
}

#endif /* CONFIG_IEEE80211W */


void ap_sta_set_authorized(struct hostapd_data *hapd, struct sta_info *sta,
			   int authorized)
{
	char buf[100];
#ifdef CONFIG_P2P
	const u8 *dev_addr = NULL;
	u8 addr[ETH_ALEN];
	u8 ip_addr_buf[4];
#endif /* CONFIG_P2P */

#ifdef CONFIG_MESH
	if(hapd == NULL)
		return; 
#endif

	if (!!authorized == !!(sta->flags & WLAN_STA_AUTHORIZED))
		return;

	if (authorized)
		sta->flags |= WLAN_STA_AUTHORIZED;
	else
		sta->flags &= ~WLAN_STA_AUTHORIZED;

#ifdef CONFIG_P2P
	if (hapd->p2p_group == NULL) {
		if (sta->p2p_ie != NULL &&
		    p2p_parse_dev_addr_in_p2p_ie(sta->p2p_ie, addr) == 0)
			dev_addr = addr;
	} else
		dev_addr = p2p_group_get_dev_addr(hapd->p2p_group, sta->addr);

	if (dev_addr)
		os_snprintf(buf, sizeof(buf), MACSTR " p2p_dev_addr=" MACSTR,
			    MAC2STR(sta->addr), MAC2STR(dev_addr));
	else
#endif /* CONFIG_P2P */
		os_snprintf(buf, sizeof(buf), MACSTR, MAC2STR(sta->addr));

	if (hapd->sta_authorized_cb)
		hapd->sta_authorized_cb(hapd->sta_authorized_cb_ctx, sta->addr);

	if (authorized) {
#ifdef CONFIG_P2P
		char ip_addr[100];
		ip_addr[0] = '\0';

		if (wpa_auth_get_ip_addr(sta->wpa_sm, ip_addr_buf) == 0) {
			os_snprintf(ip_addr, sizeof(ip_addr),
				    " ip_addr=%u.%u.%u.%u",
				    ip_addr_buf[0], ip_addr_buf[1],
				    ip_addr_buf[2], ip_addr_buf[3]);
		}
#endif /* CONFIG_P2P */

		da16x_notice_prt(AP_STA_CONNECTED "%s\n", buf);

		/* AP MODE */
		/* Notify Wi-Fi connection status on user callback function */
		if (wifi_conn_notify_cb != NULL) {
			wifi_conn_notify_cb();
		}

		atcmd_asynchony_event(2, buf); // ATC_EV_APCONN

#ifdef	CONFIG_P2P
		if (hapd->p2p) {
#ifdef	CONFIG_P2P_POWER_SAVE /* by Shingu 20180208 (P2P PS opt.) */
			if (!dev_addr) 
				hapd->num_sta_no_p2p++;
			if (hapd->num_sta_no_p2p == 0)
				hostapd_p2p_default_noa(hapd, 1);
#endif	/* CONFIG_P2P_POWER_SAVE */
			p2p_remove_device(hapd->p2p, dev_addr, 0);
			hapd->p2p_go_has_peers = 1;
		}
#endif  /* CONFIG_P2P */
	} else {
		atcmd_asynchony_event(3, buf); // ATC_EV_APDISCONN

		if (ap_sta_disconnected_notify_cb != NULL) {
			ap_sta_disconnected_notify_cb(sta->addr);
		}

		da16x_notice_prt(AP_STA_DISCONNECTED "%s\n", buf);

#ifdef	CONFIG_P2P_POWER_SAVE /* by Shingu 20180208 (P2P PS opt.) */
		if (hapd->p2p) {
			if (!dev_addr) {
				hapd->num_sta_no_p2p--;
				if ((get_run_mode() == P2P_ONLY_MODE &&
				     hapd->num_sta_no_p2p == 0 && hapd->num_sta > 1) ||
				    (get_run_mode() == P2P_GO_FIXED_MODE &&
				     hapd->num_sta_no_p2p == 0))
					hostapd_p2p_default_noa(hapd, 1);
			}
		}
#endif	/* CONFIG_P2P_POWER_SAVE */
	}

#ifdef  CONFIG_P2P
	if (hapd->sta_authorized_cb)
		hapd->sta_authorized_cb(hapd->sta_authorized_cb_ctx, sta->addr);
#endif  /* CONFIG_P2P */
#ifdef CONFIG_FST
	if (hapd->iface->fst) {
		if (authorized)
			fst_notify_peer_connected(hapd->iface->fst, sta->addr);
		else
			fst_notify_peer_disconnected(hapd->iface->fst,
						     sta->addr);
	}
#endif /* CONFIG_FST */
}


void ap_sta_disconnect(struct hostapd_data *hapd, struct sta_info *sta,
		       const u8 *addr, u16 reason)
{

	if (sta == NULL && addr)
		sta = ap_get_sta(hapd, addr);

	if (addr)
		hostapd_drv_sta_deauth(hapd, addr, reason);

	if (sta == NULL)
		return;
	ap_sta_set_authorized(hapd, sta, 0);
	wpa_auth_sm_event(sta->wpa_sm, WPA_DEAUTH);
#ifdef	CONFIG_RADIUS
	ieee802_1x_notify_port_enabled(sta->eapol_sm, 0);
#endif	/* CONFIG_RADIUS */
	sta->flags &= ~(WLAN_STA_AUTH | WLAN_STA_ASSOC);

	da16x_ap_prt("[%s] reschedule ap_handle_timer timeout "
		   "for " MACSTR " (%d seconds - "
		    "AP_MAX_INACTIVITY_AFTER_DEAUTH)\n",
		    __func__, MAC2STR(sta->addr),
		   AP_MAX_INACTIVITY_AFTER_DEAUTH);

	da16x_eloop_cancel_timeout(ap_handle_timer, hapd, sta);
	da16x_eloop_register_timeout(AP_MAX_INACTIVITY_AFTER_DEAUTH, 0,
			       ap_handle_timer, hapd, sta);

	sta->timeout_next = STA_REMOVE;

	if (hapd->iface->current_mode &&
	    hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211AD) {
		/* Deauthentication is not used in DMG/IEEE 802.11ad;
		 * disassociate the STA instead. */
		sta->disassoc_reason = reason;
		sta->flags |= WLAN_STA_PENDING_DISASSOC_CB;
		da16x_eloop_cancel_timeout(ap_sta_disassoc_cb_timeout, hapd, sta);
		da16x_eloop_register_timeout(hapd->iface->drv_flags &
				       WPA_DRIVER_FLAGS_DEAUTH_TX_STATUS ?
				       2 : 0, 0, ap_sta_disassoc_cb_timeout,
				       hapd, sta);
		return;
	}

	sta->deauth_reason = reason;
	sta->flags |= WLAN_STA_PENDING_DEAUTH_CB;

	da16x_eloop_cancel_timeout(ap_sta_deauth_cb_timeout, hapd, sta);
	da16x_eloop_register_timeout(hapd->iface->drv_flags &
			       WPA_DRIVER_FLAGS_DEAUTH_TX_STATUS ? 2 : 0, 0,
			       ap_sta_deauth_cb_timeout, hapd, sta);
}


void ap_sta_deauth_cb(struct hostapd_data *hapd, struct sta_info *sta)
{
	if (!(sta->flags & WLAN_STA_PENDING_DEAUTH_CB)) {
		da16x_ap_prt("[%s] Ignore deauth cb for test frame\n", __func__);
		return;
	}
	sta->flags &= ~WLAN_STA_PENDING_DEAUTH_CB;

	da16x_eloop_cancel_timeout(ap_sta_deauth_cb_timeout, hapd, sta);

	ap_sta_deauth_cb_timeout(hapd, sta);

	/* AP MODE */
	/* Notify Wi-Fi connection status on user callback function  */
	if (wifi_conn_fail_notify_cb != NULL) {
		wifi_conn_fail_notify_cb(sta->deauth_reason);
	}
}


void ap_sta_disassoc_cb(struct hostapd_data *hapd, struct sta_info *sta)
{
	if (!(sta->flags & WLAN_STA_PENDING_DISASSOC_CB)) {
		da16x_ap_prt("[%s] Ignore disassoc cb for test frame\n",
			    __func__);
		return;
	}
	sta->flags &= ~WLAN_STA_PENDING_DISASSOC_CB;

	da16x_eloop_cancel_timeout(ap_sta_disassoc_cb_timeout, hapd, sta);

	ap_sta_disassoc_cb_timeout(hapd, sta);
}


void ap_sta_clear_disconnect_timeouts(struct hostapd_data *hapd,
				      struct sta_info *sta)
{
	if (da16x_eloop_cancel_timeout(ap_sta_deauth_cb_timeout, hapd, sta) > 0)
		wpa_printf_dbg(MSG_DEBUG,
			   "%s: Removed ap_sta_deauth_cb_timeout timeout for "
			   MACSTR,
			   hapd->conf->iface, MAC2STR(sta->addr));
	if (da16x_eloop_cancel_timeout(ap_sta_disassoc_cb_timeout, hapd, sta) > 0)
		wpa_printf_dbg(MSG_DEBUG,
			   "%s: Removed ap_sta_disassoc_cb_timeout timeout for "
			   MACSTR,
			   hapd->conf->iface, MAC2STR(sta->addr));
	if (da16x_eloop_cancel_timeout(ap_sta_delayed_1x_auth_fail_cb, hapd, sta) > 0)
	{
		wpa_printf_dbg(MSG_DEBUG,
			   "%s: Removed ap_sta_delayed_1x_auth_fail_cb timeout for "
			   MACSTR,
			   hapd->conf->iface, MAC2STR(sta->addr));
		if (sta->flags & WLAN_STA_WPS)
			hostapd_wps_eap_completed(hapd);
	}
}


int ap_sta_flags_txt(u32 flags, char *buf, size_t buflen)
{
	int res;

	buf[0] = '\0';
	res = os_snprintf(buf, buflen, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
			  (flags & WLAN_STA_AUTH ? "[AUTH]" : ""),
			  (flags & WLAN_STA_ASSOC ? "[ASSOC]" : ""),
			  (flags & WLAN_STA_AUTHORIZED ? "[AUTHORIZED]" : ""),
			  (flags & WLAN_STA_PENDING_POLL ? "[PENDING_POLL" :
			   ""),
			  (flags & WLAN_STA_SHORT_PREAMBLE ?
			   "[SHORT_PREAMBLE]" : ""),
			  (flags & WLAN_STA_PREAUTH ? "[PREAUTH]" : ""),
			  (flags & WLAN_STA_WMM ? "[WMM]" : ""),
			  (flags & WLAN_STA_MFP ? "[MFP]" : ""),
			  (flags & WLAN_STA_WPS ? "[WPS]" : ""),
			  (flags & WLAN_STA_MAYBE_WPS ? "[MAYBE_WPS]" : ""),
			  (flags & WLAN_STA_WDS ? "[WDS]" : ""),
			  (flags & WLAN_STA_NONERP ? "[NonERP]" : ""),
			  (flags & WLAN_STA_WPS2 ? "[WPS2]" : ""),
			  (flags & WLAN_STA_GAS ? "[GAS]" : ""),
			  (flags & WLAN_STA_HT ? "[HT]" : ""),
			  (flags & WLAN_STA_VHT ? "[VHT]" : ""),
			  (flags & WLAN_STA_VENDOR_VHT ? "[VENDOR_VHT]" : ""),
			  (flags & WLAN_STA_WNM_SLEEP_MODE ?
			   "[WNM_SLEEP_MODE]" : ""));
	if (os_snprintf_error(buflen, res))
		res = -1;

	return res;
}


static void ap_sta_delayed_1x_auth_fail_cb(void *eloop_ctx, void *timeout_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	struct sta_info *sta = timeout_ctx;
	u16 reason;

	wpa_dbg(hapd->msg_ctx, MSG_DEBUG,
		"IEEE 802.1X: Scheduled disconnection of " MACSTR
		" after EAP-Failure", MAC2STR(sta->addr));

	reason = sta->disconnect_reason_code;
	if (!reason)
		reason = WLAN_REASON_IEEE_802_1X_AUTH_FAILED;
	ap_sta_disconnect(hapd, sta, sta->addr, reason);
	if (sta->flags & WLAN_STA_WPS)
		hostapd_wps_eap_completed(hapd);
}


void ap_sta_delayed_1x_auth_fail_disconnect(struct hostapd_data *hapd,
					    struct sta_info *sta)
{
	wpa_dbg(hapd->msg_ctx, MSG_DEBUG,
		"IEEE 802.1X: Force disconnection of " MACSTR
		" after EAP-Failure in 10 ms", MAC2STR(sta->addr));

	/*
	 * Add a small sleep to increase likelihood of previously requested
	 * EAP-Failure TX getting out before this should the driver reorder
	 * operations.
	 */
	da16x_eloop_cancel_timeout(ap_sta_delayed_1x_auth_fail_cb, hapd, sta);
	da16x_eloop_register_timeout(0, 10000, ap_sta_delayed_1x_auth_fail_cb,
			       hapd, sta);
}


int ap_sta_pending_delayed_1x_auth_fail_disconnect(struct hostapd_data *hapd,
						   struct sta_info *sta)
{
	return da16x_eloop_is_timeout_registered(ap_sta_delayed_1x_auth_fail_cb,
					   hapd, sta);
}
#endif /* CONFIG_AP */

/* EOF */
