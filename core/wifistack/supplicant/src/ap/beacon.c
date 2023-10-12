/*
 * hostapd / IEEE 802.11 Management: Beacon and Probe Request/Response
 * Copyright (c) 2002-2004, Instant802 Networks, Inc.
 * Copyright (c) 2005-2006, Devicescape Software, Inc.
 * Copyright (c) 2008-2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"

#ifdef	CONFIG_AP
#include "utils/supp_common.h"
#include "common/ieee802_11_defs.h"
#include "common/ieee802_11_common.h"

#if 0
#include "common/hw_features_common.h"
#include "common/wpa_ctrl.h"
#endif /* 0 */

#include "wps/wps_defs.h"
#include "p2p/supp_p2p.h"
#include "hostapd.h"
#include "ieee802_11.h"
#include "wpa_auth.h"
#include "wmm.h"
#include "ap_config.h"
#include "sta_info.h"
#include "p2p_hostapd.h"
#include "ap_drv_ops.h"
#include "beacon.h"
#ifdef CONFIG_HS20
#include "hs20.h"
#endif	/* CONFIG_HS20 */
#include "dfs.h"
#ifdef CONFIG_TAXONOMY
#include "taxonomy.h"
#endif /* CONFIG_TAXONOMY */
#include "ieee802_11_auth.h"

#ifdef	CONFIG_P2P
#include "p2p_hostapd.h"
#include "p2p_i.h"
#endif	/* CONFIG_P2P */

#pragma GCC diagnostic ignored "-Wimplicit-function-declaration"

extern int hostapd_set_freq_params(struct hostapd_freq_params *data,
			    enum hostapd_hw_mode mode,
			    int freq, int channel, int ht_enabled,
			    int vht_enabled, int sec_channel_offset,
			    int vht_oper_chwidth, int center_segment0,
			    int center_segment1, u32 vht_caps);

#ifdef NEED_AP_MLME

#ifdef CONFIG_RADIO_MEASUREMENTS
static u8 * hostapd_eid_rm_enabled_capab(struct hostapd_data *hapd, u8 *eid,
					 size_t len)
{
	size_t i;

	for (i = 0; i < RRM_CAPABILITIES_IE_LEN; i++) {
		if (hapd->conf->radio_measurements[i])
			break;
	}

	if (i == RRM_CAPABILITIES_IE_LEN || len < 2 + RRM_CAPABILITIES_IE_LEN)
		return eid;

	*eid++ = WLAN_EID_RRM_ENABLED_CAPABILITIES;
	*eid++ = RRM_CAPABILITIES_IE_LEN;
	os_memcpy(eid, hapd->conf->radio_measurements, RRM_CAPABILITIES_IE_LEN);

	return eid + RRM_CAPABILITIES_IE_LEN;
}
#endif /* CONFIG_RADIO_MEASUREMENTS */


static u8 * hostapd_eid_bss_load(struct hostapd_data *hapd, u8 *eid, size_t len)
{
	if (len < 2 + 5)
		return eid;

#ifdef CONFIG_TESTING_OPTIONS
	if (hapd->conf->bss_load_test_set) {
		*eid++ = WLAN_EID_BSS_LOAD;
		*eid++ = 5;
		os_memcpy(eid, hapd->conf->bss_load_test, 5);
		eid += 5;
		return eid;
	}
#endif /* CONFIG_TESTING_OPTIONS */
#ifdef	CONFIG_AP_BSS_LOAD_UPDATE
	if (hapd->conf->bss_load_update_period) {
		*eid++ = WLAN_EID_BSS_LOAD;
		*eid++ = 5;
		WPA_PUT_LE16(eid, hapd->num_sta);
		eid += 2;
#ifdef CONFIG_CHANNEL_UTILIZATION
		*eid++ = hapd->iface->channel_utilization;
#else
               	*eid++ = 0;
#endif /* CONFIG_CHANNEL_UTILIZATION */
		WPA_PUT_LE16(eid, 0); /* no available admission capabity */
		eid += 2;
	}
#endif	/* CONFIG_AP_BSS_LOAD_UPDATE */
	return eid;
}


static u8 ieee802_11_erp_info(struct hostapd_data *hapd)
{
	u8 erp = 0;

	if (hapd->iface->current_mode == NULL ||
	    hapd->iface->current_mode->mode != HOSTAPD_MODE_IEEE80211G)
		return 0;

	if (hapd->iface->olbc)
		erp |= ERP_INFO_USE_PROTECTION;
	if (hapd->iface->num_sta_non_erp > 0) {
		erp |= ERP_INFO_NON_ERP_PRESENT |
			ERP_INFO_USE_PROTECTION;
	}
	if (hapd->iface->num_sta_no_short_preamble > 0 ||
	    hapd->iconf->preamble == LONG_PREAMBLE)
		erp |= ERP_INFO_BARKER_PREAMBLE_MODE;

	return erp;
}


static u8 * hostapd_eid_ds_params(struct hostapd_data *hapd, u8 *eid)
{
	*eid++ = WLAN_EID_DS_PARAMS;
	*eid++ = 1;
	*eid++ = hapd->iconf->channel;
	return eid;
}


static u8 * hostapd_eid_erp_info(struct hostapd_data *hapd, u8 *eid)
{
	if (hapd->iface->current_mode == NULL ||
	    hapd->iface->current_mode->mode != HOSTAPD_MODE_IEEE80211G)
		return eid;

	/* Set NonERP_present and use_protection bits if there
	 * are any associated NonERP stations. */
	/* TODO: use_protection bit can be set to zero even if
	 * there are NonERP stations present. This optimization
	 * might be useful if NonERP stations are "quiet".
	 * See 802.11g/D6 E-1 for recommended practice.
	 * In addition, Non ERP present might be set, if AP detects Non ERP
	 * operation on other APs. */

	/* Add ERP Information element */
	*eid++ = WLAN_EID_ERP_INFO;
	*eid++ = 1;
	*eid++ = ieee802_11_erp_info(hapd);

	return eid;
}


static u8 * hostapd_eid_pwr_constraint(struct hostapd_data *hapd, u8 *eid)
{
	u8 *pos = eid;
	u8 local_pwr_constraint = 0;
#ifdef CONFIG_IEEE80211H
	int dfs;
#endif	/* CONFIG_IEEE80211H */

	if (hapd->iface->current_mode == NULL ||
	    hapd->iface->current_mode->mode != HOSTAPD_MODE_IEEE80211A)
		return eid;

#ifdef	CONFIG_AP_DFS
	/* Let host drivers add this IE if DFS support is offloaded */
	if (hapd->iface->drv_flags & WPA_DRIVER_FLAGS_DFS_OFFLOAD)
		return eid;
#endif	/* CONFIG_AP_DFS */

	/*
	 * There is no DFS support and power constraint was not directly
	 * requested by config option.
	 */
	if (
#ifdef	CONFIG_IEEE80211H
	    !hapd->iconf->ieee80211h &&
#endif	/* CONFIG_IEEE80211H */
	    hapd->iconf->local_pwr_constraint == -1)
		return eid;

#ifdef CONFIG_IEEE80211H
	/* Check if DFS is required by regulatory. */
	dfs = hostapd_is_dfs_required(hapd->iface);
	if (dfs < 0) {
		wpa_printf(MSG_WARNING, "Failed to check if DFS is required; ret=%d",
			   dfs);
		dfs = 0;
	}

	if (dfs == 0 && hapd->iconf->local_pwr_constraint == -1)
		return eid;
#endif	/* CONFIG_IEEE80211H */

	/*
	 * ieee80211h (DFS) is enabled so Power Constraint element shall
	 * be added when running on DFS channel whenever local_pwr_constraint
	 * is configured or not. In order to meet regulations when TPC is not
	 * implemented using a transmit power that is below the legal maximum
	 * (including any mitigation factor) should help. In this case,
	 * indicate 3 dB below maximum allowed transmit power.
	 */
	if (hapd->iconf->local_pwr_constraint == -1)
		local_pwr_constraint = 3;

	/*
	 * A STA that is not an AP shall use a transmit power less than or
	 * equal to the local maximum transmit power level for the channel.
	 * The local maximum transmit power can be calculated from the formula:
	 * local max TX pwr = max TX pwr - local pwr constraint
	 * Where max TX pwr is maximum transmit power level specified for
	 * channel in Country element and local pwr constraint is specified
	 * for channel in this Power Constraint element.
	 */

	/* Element ID */
	*pos++ = WLAN_EID_PWR_CONSTRAINT;
	/* Length */
	*pos++ = 1;
	/* Local Power Constraint */
	if (local_pwr_constraint)
		*pos++ = local_pwr_constraint;
	else
		*pos++ = hapd->iconf->local_pwr_constraint;

	return pos;
}


static u8 * hostapd_eid_country_add(u8 *pos, u8 *end, int chan_spacing,
				    struct hostapd_channel_data *start,
				    struct hostapd_channel_data *prev)
{
	if (end - pos < 3)
		return pos;

	/* first channel number */
	*pos++ = start->chan;
	/* number of channels */
	*pos++ = (prev->chan - start->chan) / chan_spacing + 1;
	/* maximum transmit power level */
	*pos++ = start->max_tx_power;

	return pos;
}


static u8 * hostapd_eid_country(struct hostapd_data *hapd, u8 *eid,
				int max_len)
{
	u8 *pos = eid;
	u8 *end = eid + max_len;
	int i;
	struct hostapd_hw_modes *mode;
	struct hostapd_channel_data *start, *prev;
	int chan_spacing = 1;

#ifdef	CONFIG_IEEE80211D
	if (!hapd->iconf->ieee80211d || max_len < 6 ||
	    hapd->iface->current_mode == NULL)
#else
	if (max_len < 6 || hapd->iface->current_mode == NULL)
#endif	/* CONFIG_IEEE80211D */
		return eid;

	*pos++ = WLAN_EID_COUNTRY;
	pos++; /* length will be set later */
	os_memcpy(pos, hapd->iconf->country, 3); /* e.g., 'US ' */
	pos += 3;

	mode = hapd->iface->current_mode;
	if (mode->mode == HOSTAPD_MODE_IEEE80211A)
		chan_spacing = 4;

	start = prev = NULL;
	for (i = 0; i < mode->num_channels; i++) {
		struct hostapd_channel_data *chan = &mode->channels[i];
		if (chan->flag & HOSTAPD_CHAN_DISABLED)
			continue;
		if (start && prev &&
		    prev->chan + chan_spacing == chan->chan &&
		    start->max_tx_power == chan->max_tx_power) {
			prev = chan;
			continue; /* can use same entry */
		}

		if (start && prev) {
			pos = hostapd_eid_country_add(pos, end, chan_spacing,
						      start, prev);
			start = NULL;
		}

		/* Start new group */
		start = prev = chan;
	}

	if (start) {
		pos = hostapd_eid_country_add(pos, end, chan_spacing,
					      start, prev);
	}

	if ((pos - eid) & 1) {
		if (end - pos < 1)
			return eid;
		*pos++ = 0; /* pad for 16-bit alignment */
	}

	eid[1] = (pos - eid) - 2;

	return pos;
}


static u8 * hostapd_eid_wpa(struct hostapd_data *hapd, u8 *eid, size_t len)
{
	const u8 *ie;
	size_t ielen;

	ie = wpa_auth_get_wpa_ie(hapd->wpa_auth, &ielen);
	if (ie == NULL || ielen > len)
		return eid;

	os_memcpy(eid, ie, ielen);
	return eid + ielen;
}


static u8 * hostapd_eid_csa(struct hostapd_data *hapd, u8 *eid)
{
#ifdef CONFIG_TESTING_OPTIONS
	if (hapd->iface->cs_oper_class && hapd->iconf->ecsa_ie_only)
		return eid;
#endif /* CONFIG_TESTING_OPTIONS */

	if (!hapd->cs_freq_params.channel)
		return eid;

	*eid++ = WLAN_EID_CHANNEL_SWITCH;
	*eid++ = 3;
	*eid++ = hapd->cs_block_tx;
	*eid++ = hapd->cs_freq_params.channel;
	*eid++ = hapd->cs_count;

	return eid;
}


#ifdef	CONFIG_SUPP27_BEACON
static u8 * hostapd_eid_ecsa(struct hostapd_data *hapd, u8 *eid)
{
	if (!hapd->cs_freq_params.channel || !hapd->iface->cs_oper_class)
		return eid;

	*eid++ = WLAN_EID_EXT_CHANSWITCH_ANN;
	*eid++ = 4;
	*eid++ = hapd->cs_block_tx;
	*eid++ = hapd->iface->cs_oper_class;
	*eid++ = hapd->cs_freq_params.channel;
	*eid++ = hapd->cs_count;

	return eid;
}


static u8 * hostapd_eid_supported_op_classes(struct hostapd_data *hapd, u8 *eid)
{
	u8 op_class, channel;

	if (!(hapd->iface->drv_flags & WPA_DRIVER_FLAGS_AP_CSA) ||
	    !hapd->iface->freq)
		return eid;

	if (ieee80211_freq_to_channel_ext(hapd->iface->freq,
					  hapd->iconf->secondary_channel,
#ifdef CONFIG_IEEE80211AC
					  hapd->iconf->vht_oper_chwidth,
#else
					  0,
#endif /* CONFIG_IEEE80211AC */
					  &op_class, &channel) ==
	    NUM_HOSTAPD_MODES)
		return eid;

	*eid++ = WLAN_EID_SUPPORTED_OPERATING_CLASSES;
	*eid++ = 2;

	/* Current Operating Class */
	*eid++ = op_class;

	/* TODO: Advertise all the supported operating classes */
	*eid++ = 0;

	return eid;
}
#endif	/* CONFIG_SUPP27_BEACON */


static u8 * hostapd_gen_probe_resp(struct hostapd_data *hapd,
				   const struct _ieee80211_mgmt *req,
				   int is_p2p, size_t *resp_len)
{
	struct _ieee80211_mgmt *resp;
	u8 *pos, *epos, *csa_pos;
	size_t buflen;

#define MAX_PROBERESP_LEN 768
	buflen = MAX_PROBERESP_LEN;
#ifdef CONFIG_WPS
	if (hapd->wps_probe_resp_ie)
		buflen += wpabuf_len(hapd->wps_probe_resp_ie);
#endif /* CONFIG_WPS */
#ifdef CONFIG_P2P
	if (hapd->p2p_probe_resp_ie)
		buflen += wpabuf_len(hapd->p2p_probe_resp_ie);
#endif /* CONFIG_P2P */
#ifdef CONFIG_FST
	if (hapd->iface->fst_ies)
		buflen += wpabuf_len(hapd->iface->fst_ies);
#endif /* CONFIG_FST */
#ifdef	CONFIG_AP_VENDOR_ELEM
	if (hapd->conf->vendor_elements)
		buflen += wpabuf_len(hapd->conf->vendor_elements);
#endif	/* CONFIG_AP_VENDOR_ELEM */
#ifdef	CONFIG_IEEE80211AC
	if (hapd->conf->vendor_vht) {
		buflen += 5 + 2 + sizeof(struct ieee80211_vht_capabilities) +
			2 + sizeof(struct ieee80211_vht_operation);
	}
#endif	/* CONFIG_IEEE80211AC */

#ifdef CONFIG_IEEE80211AX
	if (hapd->iconf->ieee80211ax) {
		buflen += 3 + sizeof(struct ieee80211_he_capabilities) +
			3 + sizeof(struct ieee80211_he_operation);
	}
#endif /* CONFIG_IEEE80211AX */

#ifdef CONFIG_MBO
	buflen += hostapd_mbo_ie_len(hapd);
#endif /* CONFIG_MBO */
#ifdef CONFIG_OWE_BEACON
	buflen += hostapd_eid_owe_trans_len(hapd);
#endif /* CONFIG_OWE_BEACON */

	resp = os_zalloc(buflen);
	if (resp == NULL)
		return NULL;

	epos = ((u8 *) resp) + MAX_PROBERESP_LEN;

	resp->frame_control = IEEE80211_FC(WLAN_FC_TYPE_MGMT,
					   WLAN_FC_STYPE_PROBE_RESP);
	if (req)
		os_memcpy(resp->da, req->sa, ETH_ALEN);
	os_memcpy(resp->sa, hapd->own_addr, ETH_ALEN);

	os_memcpy(resp->bssid, hapd->own_addr, ETH_ALEN);
	resp->u.probe_resp.beacon_int =
		host_to_le16(hapd->iconf->beacon_int);

	/* hardware or low-level driver will setup seq_ctrl and timestamp */
	resp->u.probe_resp.capab_info = host_to_le16(hostapd_own_capab_info(hapd));

	pos = resp->u.probe_resp.variable;
	*pos++ = WLAN_EID_SSID;
	*pos++ = hapd->conf->ssid.ssid_len;
	os_memcpy(pos, hapd->conf->ssid.ssid, hapd->conf->ssid.ssid_len);
	pos += hapd->conf->ssid.ssid_len;

	/* Supported rates */
	pos = hostapd_eid_supp_rates(hapd, pos);

	/* DS Params */
	pos = hostapd_eid_ds_params(hapd, pos);

	pos = hostapd_eid_country(hapd, pos, epos - pos);

	/* Power Constraint element */
	pos = hostapd_eid_pwr_constraint(hapd, pos);

	/* CSA IE */
	csa_pos = hostapd_eid_csa(hapd, pos);
	if (csa_pos != pos)
		hapd->cs_c_off_proberesp = csa_pos - (u8 *) resp - 1;
	pos = csa_pos;

	/* ERP Information element */
	pos = hostapd_eid_erp_info(hapd, pos);

	/* Extended supported rates */
	pos = hostapd_eid_ext_supp_rates(hapd, pos);

	/* RSN, MDIE, WPA */
	pos = hostapd_eid_wpa(hapd, pos, epos - pos);

	pos = hostapd_eid_bss_load(hapd, pos, epos - pos);

#ifdef CONFIG_RADIO_MEASUREMENTS
	pos = hostapd_eid_rm_enabled_capab(hapd, pos, epos - pos);
#endif /* CONFIG_RADIO_MEASUREMENTS */

#ifdef	CONFIG_SUPP27_BEACON
	/* eCSA IE */
	csa_pos = hostapd_eid_ecsa(hapd, pos);
	if (csa_pos != pos)
		hapd->cs_c_off_ecsa_proberesp = csa_pos - (u8 *) resp - 1;
	pos = csa_pos;

	pos = hostapd_eid_supported_op_classes(hapd, pos);
#endif	/* CONFIG_SUPP27_BEACON */

#ifdef CONFIG_IEEE80211N
#ifdef	CONFIG_AP_HT
	/* Secondary Channel Offset element */
	/* TODO: The standard doesn't specify a position for this element. */
	pos = hostapd_eid_secondary_channel(hapd, pos);

	pos = hostapd_eid_ht_capabilities(hapd, pos);
	pos = hostapd_eid_ht_operation(hapd, pos);
#endif	/* CONFIG_AP_HT */
#endif /* CONFIG_IEEE80211N */

	pos = hostapd_eid_ext_capab(hapd, pos);

#ifdef	CONFIG_TIME_ADV
	pos = hostapd_eid_time_adv(hapd, pos);
	pos = hostapd_eid_time_zone(hapd, pos);
#endif	/* CONFIG_TIME_ADV */

#ifdef	CONFIG_INTERWORKING
	pos = hostapd_eid_interworking(hapd, pos);
#endif	/* CONFIG_INTERWORKING */
#ifdef	CONFIG_ADV_PROTO
	pos = hostapd_eid_adv_proto(hapd, pos);
#endif	/* CONFIG_ADV_PROTO */
#ifdef	CONFIG_ROAM_CONSORTIUM
	pos = hostapd_eid_roaming_consortium(hapd, pos);
#endif	/* CONFIG_ROAM_CONSORTIUM */

#ifdef CONFIG_FST
	if (hapd->iface->fst_ies) {
		os_memcpy(pos, wpabuf_head(hapd->iface->fst_ies),
			  wpabuf_len(hapd->iface->fst_ies));
		pos += wpabuf_len(hapd->iface->fst_ies);
	}
#endif /* CONFIG_FST */

#ifdef CONFIG_IEEE80211AC
	if (hapd->iconf->ieee80211ac && !hapd->conf->disable_11ac) {
		pos = hostapd_eid_vht_capabilities(hapd, pos, 0);
		pos = hostapd_eid_vht_operation(hapd, pos);
		pos = hostapd_eid_txpower_envelope(hapd, pos);
		pos = hostapd_eid_wb_chsw_wrapper(hapd, pos);
	}
#endif /* CONFIG_IEEE80211AC */

#ifdef CONFIG_FILS
	pos = hostapd_eid_fils_indic(hapd, pos, 0);
#endif /* CONFIG_FILS */

#ifdef CONFIG_IEEE80211AX
	if (hapd->iconf->ieee80211ax) {
		pos = hostapd_eid_he_capab(hapd, pos);
		pos = hostapd_eid_he_operation(hapd, pos);
	}
#endif /* CONFIG_IEEE80211AX */

#ifdef CONFIG_IEEE80211AC
	if (hapd->conf->vendor_vht)
		pos = hostapd_eid_vendor_vht(hapd, pos);
#endif /* CONFIG_IEEE80211AC */

#ifdef CONFIG_AP_WMM
	/* Wi-Fi Alliance WMM */
	pos = hostapd_eid_wmm(hapd, pos);
#endif /* CONFIG_AP_WMM */

#ifdef CONFIG_WPS
	if (hapd->conf->wps_state && hapd->wps_probe_resp_ie) {
		os_memcpy(pos, wpabuf_head(hapd->wps_probe_resp_ie),
			  wpabuf_len(hapd->wps_probe_resp_ie));
		pos += wpabuf_len(hapd->wps_probe_resp_ie);
	}
#endif /* CONFIG_WPS */

#ifdef CONFIG_P2P
	if ((hapd->conf->p2p & P2P_ENABLED) && is_p2p &&
	    hapd->p2p_probe_resp_ie) {
		os_memcpy(pos, wpabuf_head(hapd->p2p_probe_resp_ie),
			  wpabuf_len(hapd->p2p_probe_resp_ie));
		pos += wpabuf_len(hapd->p2p_probe_resp_ie);
	}
#endif /* CONFIG_P2P */
#ifdef CONFIG_P2P_MANAGER
	if ((hapd->conf->p2p & (P2P_MANAGE | P2P_ENABLED | P2P_GROUP_OWNER)) ==
	    P2P_MANAGE)
		pos = hostapd_eid_p2p_manage(hapd, pos);
#endif /* CONFIG_P2P_MANAGER */

#ifdef CONFIG_HS20
	pos = hostapd_eid_hs20_indication(hapd, pos);
	pos = hostapd_eid_osen(hapd, pos);
#endif /* CONFIG_HS20 */

#ifdef CONFIG_MBO
	pos = hostapd_eid_mbo(hapd, pos, (u8 *) resp + buflen - pos);
#endif /* CONFIG_MBO */

#ifdef CONFIG_OWE_BEACON
	pos = hostapd_eid_owe_trans(hapd, pos, (u8 *) resp + buflen - pos);
#endif /* CONFIG_OWE_BEACON */

#ifdef	CONFIG_AP_VENDOR_ELEM
	if (hapd->conf->vendor_elements) {
		os_memcpy(pos, wpabuf_head(hapd->conf->vendor_elements),
			  wpabuf_len(hapd->conf->vendor_elements));
		pos += wpabuf_len(hapd->conf->vendor_elements);
	}
#endif	/* CONFIG_AP_VENDOR_ELEM */

	*resp_len = pos - (u8 *) resp;
	return (u8 *) resp;
}


enum ssid_match_result {
	NO_SSID_MATCH,
	EXACT_SSID_MATCH,
	WILDCARD_SSID_MATCH
};

static enum ssid_match_result ssid_match(struct hostapd_data *hapd,
					 const u8 *ssid, size_t ssid_len,
					 const u8 *ssid_list,
					 size_t ssid_list_len)
{
	const u8 *pos, *end;
	int wildcard = 0;

	if (ssid_len == 0)
		wildcard = 1;
	if (ssid_len == hapd->conf->ssid.ssid_len &&
	    os_memcmp(ssid, hapd->conf->ssid.ssid, ssid_len) == 0)
		return EXACT_SSID_MATCH;

	if (ssid_list == NULL)
		return wildcard ? WILDCARD_SSID_MATCH : NO_SSID_MATCH;

	pos = ssid_list;
	end = ssid_list + ssid_list_len;
	while (end - pos >= 1) {
		if (2 + pos[1] > end - pos)
			break;
		if (pos[1] == 0)
			wildcard = 1;
		if (pos[1] == hapd->conf->ssid.ssid_len &&
		    os_memcmp(pos + 2, hapd->conf->ssid.ssid, pos[1]) == 0)
			return EXACT_SSID_MATCH;
		pos += 2 + pos[1];
	}

	return wildcard ? WILDCARD_SSID_MATCH : NO_SSID_MATCH;
}


#ifdef	CONFIG_SUPP27_PROBE_REQ
void sta_track_expire(struct hostapd_iface *iface, int force)
{
	struct os_reltime now;
	struct hostapd_sta_info *info;

	if (!iface->num_sta_seen)
		return;

	os_get_reltime(&now);
	while ((info = dl_list_first(&iface->sta_seen, struct hostapd_sta_info,
				     list))) {
		if (!force &&
		    !os_reltime_expired(&now, &info->last_seen,
					iface->conf->track_sta_max_age))
			break;
		force = 0;

		wpa_printf(MSG_MSGDUMP, "%s: Expire STA tracking entry for "
			   MACSTR, iface->bss[0]->conf->iface,
			   MAC2STR(info->addr));
		dl_list_del(&info->list);
		iface->num_sta_seen--;
		sta_track_del(info);
	}
}


static struct hostapd_sta_info * sta_track_get(struct hostapd_iface *iface,
					       const u8 *addr)
{
	struct hostapd_sta_info *info;

	dl_list_for_each(info, &iface->sta_seen, struct hostapd_sta_info, list)
		if (os_memcmp(addr, info->addr, ETH_ALEN) == 0)
			return info;

	return NULL;
}


void sta_track_add(struct hostapd_iface *iface, const u8 *addr, int ssi_signal)
{
	struct hostapd_sta_info *info;

	info = sta_track_get(iface, addr);
	if (info) {
		/* Move the most recent entry to the end of the list */
		dl_list_del(&info->list);
		dl_list_add_tail(&iface->sta_seen, &info->list);
		os_get_reltime(&info->last_seen);
		info->ssi_signal = ssi_signal;
		return;
	}

	/* Add a new entry */
	info = os_zalloc(sizeof(*info));
	if (info == NULL)
		return;
	os_memcpy(info->addr, addr, ETH_ALEN);
	os_get_reltime(&info->last_seen);
	info->ssi_signal = ssi_signal;

	if (iface->num_sta_seen >= iface->conf->track_sta_max_num) {
		/* Expire oldest entry to make room for a new one */
		sta_track_expire(iface, 1);
	}

	wpa_printf(MSG_MSGDUMP, "%s: Add STA tracking entry for "
		   MACSTR, iface->bss[0]->conf->iface, MAC2STR(addr));
	dl_list_add_tail(&iface->sta_seen, &info->list);
	iface->num_sta_seen++;
}


struct hostapd_data *
sta_track_seen_on(struct hostapd_iface *iface, const u8 *addr,
		  const char *ifname)
{
	struct hapd_interfaces *interfaces = iface->interfaces;
	size_t i, j;

	for (i = 0; i < interfaces->count; i++) {
		struct hostapd_data *hapd = NULL;

		iface = interfaces->iface[i];
		for (j = 0; j < iface->num_bss; j++) {
			hapd = iface->bss[j];
			if (os_strcmp(ifname, hapd->conf->iface) == 0)
				break;
			hapd = NULL;
		}

		if (hapd && sta_track_get(iface, addr))
			return hapd;
	}

	return NULL;
}
#endif	/* CONFIG_SUPP27_PROBE_REQ */

#ifdef CONFIG_TAXONOMY
void sta_track_claim_taxonomy_info(struct hostapd_iface *iface, const u8 *addr,
				   struct wpabuf **probe_ie_taxonomy)
{
	struct hostapd_sta_info *info;

	info = sta_track_get(iface, addr);
	if (!info)
		return;

	wpabuf_free(*probe_ie_taxonomy);
	*probe_ie_taxonomy = info->probe_ie_taxonomy;
	info->probe_ie_taxonomy = NULL;
}
#endif /* CONFIG_TAXONOMY */


void handle_probe_req(struct hostapd_data *hapd,
		      const struct _ieee80211_mgmt *mgmt, size_t len,
		      int ssi_signal)
{
	u8 *resp;
	struct ieee802_11_elems elems;
	const u8 *ie;
	size_t ie_len;
	size_t i, resp_len;
	int noack;
	enum ssid_match_result res;

#ifdef CONFIG_SUPP27_PROBE_REQ
	int ret;
	u16 csa_offs[2];
	size_t csa_offs_len;
	u32 session_timeout, acct_interim_interval;
#endif /* CONFIG_SUPP27_PROBE_REQ */

#ifdef CONFIG_AP_VLAN
	struct vlan_description vlan_id;
#endif /* CONFIG_AP_VLAN */

#ifdef CONFIG_SUPP27_PROBE_REQ
	struct hostapd_sta_wpa_psk_short *psk = NULL;
	char *identity = NULL;
	char *radius_cui = NULL;
#endif /* CONFIG_SUPP27_PROBE_REQ */

#ifdef CONFIG_ACS
	if (hapd->iconf->channel == 0)
	{
		return;
	}
#endif // CONFIG_ACS

	if (len < IEEE80211_HDRLEN)
		return;
	ie = ((const u8 *) mgmt) + IEEE80211_HDRLEN;
#ifdef	CONFIG_SUPP27_PROBE_REQ
	if (hapd->iconf->track_sta_max_num)
		sta_track_add(hapd->iface, mgmt->sa, ssi_signal);
#endif	/* CONFIG_SUPP27_PROBE_REQ */
	ie_len = len - IEEE80211_HDRLEN;

#ifdef	CONFIG_SUPP27_PROBE_REQ
	ret = ieee802_11_allowed_address(hapd, mgmt->sa, (const u8 *) mgmt, len,
					 &session_timeout,
					 &acct_interim_interval,
#ifdef CONFIG_AP_VLAN
					 &vlan_id,
#endif /* CONFIG_AP_VLAN */
					 &psk, &identity, &radius_cui, 1);
	if (ret == HOSTAPD_ACL_REJECT) {
		wpa_dbg(hapd->msg_ctx, MSG_DEBUG,
			"Ignore Probe Request frame from " MACSTR
			" due to ACL reject ", MAC2STR(mgmt->sa));
		return;
	}
#endif	/* CONFIG_SUPP27_PROBE_REQ */

	for (i = 0; hapd->probereq_cb && i < hapd->num_probereq_cb; i++)
		if (hapd->probereq_cb[i].cb(hapd->probereq_cb[i].ctx,
					    mgmt->sa, mgmt->da, mgmt->bssid,
					    ie, ie_len, ssi_signal) > 0)
			return;

	if (!hapd->iconf->send_probe_response)
		return;

	if (ieee802_11_parse_elements(ie, ie_len, &elems, 0) == ParseFailed) {
		wpa_printf_dbg(MSG_DEBUG, "Could not parse ProbeReq from " MACSTR,
			   MAC2STR(mgmt->sa));
		return;
	}

#if defined CONFIG_OWE_BEACON || defined CONFIG_SAE
#ifdef __IGNORE_ALL_PROBE_REQUEST__
	if ((hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_OWE
			|| hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_SAE)
		&& hapd->conf->ignore_broadcast_ssid == 0) { /* WPA3_SAE ó�� �ӵ� ������... */
		//wpa_printf(MSG_ERROR, "Probe Request From " MACSTR, MAC2STR(mgmt->sa));
		return;
	}
#endif /* __IGNORE_ALL_PROBE_REQUEST__ */
#endif /* CONFIG_OWE_BEACON || CONFIG_SAE */

	if ((!elems.ssid || !elems.supp_rates)) {
		wpa_printf_dbg(MSG_DEBUG, "STA " MACSTR " sent probe request "
			   "without SSID or supported rates element",
			   MAC2STR(mgmt->sa));
		return;
	}

	/*
	 * No need to reply if the Probe Request frame was sent on an adjacent
	 * channel. IEEE Std 802.11-2012 describes this as a requirement for an
	 * AP with dot11RadioMeasurementActivated set to true, but strictly
	 * speaking does not allow such ignoring of Probe Request frames if
	 * dot11RadioMeasurementActivated is false. Anyway, this can help reduce
	 * number of unnecessary Probe Response frames for cases where the STA
	 * is less likely to see them (Probe Request frame sent on a
	 * neighboring, but partially overlapping, channel).
	 */
	if (elems.ds_params &&
	    hapd->iface->current_mode &&
	    (hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211G ||
	     hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211B) &&
	    hapd->iconf->channel != elems.ds_params[0]) {
		wpa_printf(MSG_MSGDUMP,
			   "Ignore Probe Request due to DS Params mismatch: chan=%u != ds.chan=%u",
			   hapd->iconf->channel, elems.ds_params[0]);
		return;
	}

#ifdef CONFIG_P2P
	if (hapd->p2p && hapd->p2p_group && elems.wps_ie) {
		struct wpabuf *wps;
		wps = ieee802_11_vendor_ie_concat(ie, ie_len, WPS_DEV_OUI_WFA);
		if (wps && !p2p_group_match_dev_type(hapd->p2p_group, wps)) {
			wpa_printf(MSG_MSGDUMP, "P2P: Ignore Probe Request "
				   "due to mismatch with Requested Device "
				   "Type");
			wpabuf_free(wps);
			return;
		}
		wpabuf_free(wps);
	}

	if (hapd->p2p && hapd->p2p_group && elems.p2p) {
		struct wpabuf *p2p;
		p2p = ieee802_11_vendor_ie_concat(ie, ie_len, P2P_IE_VENDOR_TYPE);
		if (p2p && !p2p_group_match_dev_id(hapd->p2p_group, p2p)) {
#ifdef	CONFIG_DBG_LOG_MSG
			wpa_printf(MSG_MSGDUMP, "P2P: Ignore Probe Request "
				   "due to mismatch with Device ID");
#else
			da16x_ap_prt("[%s] P2P: Ignore Probe Request "
                   "due to mismatch with Device ID\n",
                    __func__);
#endif	/* CONFIG_DBG_LOG_MSG */
			wpabuf_free(p2p);
			return;
		}
		wpabuf_free(p2p);
	}
#endif /* CONFIG_P2P */

	if (hapd->conf->ignore_broadcast_ssid && elems.ssid_len == 0 &&
	    elems.ssid_list_len == 0) {
#ifdef	CONFIG_DBG_LOG_MSG
		wpa_printf(MSG_MSGDUMP, "Probe Request from " MACSTR " for "
			   "broadcast SSID ignored", MAC2STR(mgmt->sa));
#else
		da16x_ap_prt("[%s] Probe Request from " MACSTR " for "
               "broadcast SSID ignored\n",
                __func__, MAC2STR(mgmt->sa));
#endif	/* CONFIG_DBG_LOG_MSG */
		return;
	}

#ifdef CONFIG_P2P
	if ((hapd->conf->p2p & P2P_GROUP_OWNER) &&
	    elems.ssid_len == P2P_WILDCARD_SSID_LEN &&
	    os_memcmp(elems.ssid, P2P_WILDCARD_SSID,
		      P2P_WILDCARD_SSID_LEN) == 0) {
		/* Process P2P Wildcard SSID like Wildcard SSID */
		elems.ssid_len = 0;
	}
#endif /* CONFIG_P2P */

#ifdef CONFIG_TAXONOMY
	{
		struct sta_info *sta;
		struct hostapd_sta_info *info;

		if ((sta = ap_get_sta(hapd, mgmt->sa)) != NULL) {
			taxonomy_sta_info_probe_req(hapd, sta, ie, ie_len);
		} else if ((info = sta_track_get(hapd->iface,
						 mgmt->sa)) != NULL) {
			taxonomy_hostapd_sta_info_probe_req(hapd, info,
							    ie, ie_len);
		}
	}
#endif /* CONFIG_TAXONOMY */

	res = ssid_match(hapd, elems.ssid, elems.ssid_len,
			 elems.ssid_list, elems.ssid_list_len);
	if (res == NO_SSID_MATCH) {
		if (!(mgmt->da[0] & 0x01)) {
			da16x_ap_prt("[%s] Probe Request from " MACSTR
				   " for foreign SSID '%s' (DA " MACSTR ")%s",
				   __func__, MAC2STR(mgmt->sa),
				   wpa_ssid_txt(elems.ssid, elems.ssid_len),
				   MAC2STR(mgmt->da),
				   elems.ssid_list ? " (SSID list)" : "");
		}

		return;
	}

#ifdef CONFIG_INTERWORKING
	if (hapd->conf->interworking &&
	    elems.interworking && elems.interworking_len >= 1) {
		u8 ant = elems.interworking[0] & 0x0f;
		if (ant != INTERWORKING_ANT_WILDCARD &&
		    ant != hapd->conf->access_network_type) {
			wpa_printf(MSG_MSGDUMP, "Probe Request from " MACSTR
				   " for mismatching ANT %u ignored",
				   MAC2STR(mgmt->sa), ant);
			return;
		}
	}

	if (hapd->conf->interworking && elems.interworking &&
	    (elems.interworking_len == 7 || elems.interworking_len == 9)) {
		const u8 *hessid;
		if (elems.interworking_len == 7)
			hessid = elems.interworking + 1;
		else
			hessid = elems.interworking + 1 + 2;
		if (!is_broadcast_ether_addr(hessid) &&
		    os_memcmp(hessid, hapd->conf->hessid, ETH_ALEN) != 0) {
			wpa_printf(MSG_MSGDUMP, "Probe Request from " MACSTR
				   " for mismatching HESSID " MACSTR
				   " ignored",
				   MAC2STR(mgmt->sa), MAC2STR(hessid));
			return;
		}
	}
#endif /* CONFIG_INTERWORKING */

#ifdef CONFIG_P2P
	if ((hapd->conf->p2p & P2P_GROUP_OWNER) &&
	    supp_rates_11b_only(&elems)) {
		/* Indicates support for 11b rates only */
		wpa_printf(MSG_EXCESSIVE, "P2P: Ignore Probe Request from "
			   MACSTR " with only 802.11b rates",
			   MAC2STR(mgmt->sa));
		return;
	}
#endif /* CONFIG_P2P */

	/* TODO: verify that supp_rates contains at least one matching rate
	 * with AP configuration */

#ifdef CONFIG_NO_PROBE_RESP_IF_SEEN_ON
	if (hapd->conf->no_probe_resp_if_seen_on &&
	    is_multicast_ether_addr(mgmt->da) &&
	    is_multicast_ether_addr(mgmt->bssid) &&
	    sta_track_seen_on(hapd->iface, mgmt->sa,
			      hapd->conf->no_probe_resp_if_seen_on)) {
		wpa_printf(MSG_MSGDUMP, "%s: Ignore Probe Request from " MACSTR
			   " since STA has been seen on %s",
			   hapd->conf->iface, MAC2STR(mgmt->sa),
			   hapd->conf->no_probe_resp_if_seen_on);
		return;
	}
#endif /* CONFIG_NO_PROBE_RESP_IF_SEEN_ON */

#ifdef CONFIG_NO_PROBE_RESP_IF_MAX_STA
	if (hapd->conf->no_probe_resp_if_max_sta &&
	    is_multicast_ether_addr(mgmt->da) &&
	    is_multicast_ether_addr(mgmt->bssid) &&
	    hapd->num_sta >= hapd->conf->max_num_sta &&
	    !ap_get_sta(hapd, mgmt->sa)) {
		wpa_printf(MSG_MSGDUMP, "%s: Ignore Probe Request from " MACSTR
			   " since no room for additional STA",
			   hapd->conf->iface, MAC2STR(mgmt->sa));
		return;
	}
#endif /* CONFIG_NO_PROBE_RESP_IF_MAX_STA */

#ifdef CONFIG_TESTING_OPTIONS
	if (hapd->iconf->ignore_probe_probability > 0.0 &&
	    drand48() < hapd->iconf->ignore_probe_probability) {
		wpa_printf(MSG_INFO,
			   "TESTING: ignoring probe request from " MACSTR,
			   MAC2STR(mgmt->sa));
		return;
	}
#endif /* CONFIG_TESTING_OPTIONS */


#if 1 // �ӽ�
#define RX_PROBE_REQUEST "RX-PROBE-REQUEST "
#endif /* 1 */

	wpa_msg_ctrl(hapd->msg_ctx, MSG_INFO, RX_PROBE_REQUEST "sa=" MACSTR
		     " signal=%d", MAC2STR(mgmt->sa), ssi_signal);

	resp = hostapd_gen_probe_resp(hapd, mgmt, elems.p2p != NULL,
				      &resp_len);
	if (resp == NULL)
		return;

#ifdef	CONFIG_SUPP27_PROBE_REQ
	/*
	 * If this is a broadcast probe request, apply no ack policy to avoid
	 * excessive retries.
	 */
	noack = !!(res == WILDCARD_SSID_MATCH &&
		   is_broadcast_ether_addr(mgmt->da));

	csa_offs_len = 0;
	if (hapd->csa_in_progress) {
		if (hapd->cs_c_off_proberesp)
			csa_offs[csa_offs_len++] =
				hapd->cs_c_off_proberesp;

		if (hapd->cs_c_off_ecsa_proberesp)
			csa_offs[csa_offs_len++] =
				hapd->cs_c_off_ecsa_proberesp;
	}

	hostapd_drv_send_mlme_csa(hapd, resp, resp_len, noack,
				  csa_offs_len ? csa_offs : NULL,
				  csa_offs_len);
#else
	/*
	 * If this is a broadcast probe request, apply no ack policy to avoid
	 * excessive retries.
	 */
	noack = !!(res == WILDCARD_SSID_MATCH &&
	is_broadcast_ether_addr(mgmt->da));

	if (hostapd_drv_send_mlme(hapd, resp, resp_len, noack) < 0)
		da16x_ap_prt("[%s] handle_probe_req: send failed\n", __func__);
#endif	/* CONFIG_SUPP27_PROBE_REQ */

	os_free(resp);

#ifdef	CONFIG_DBG_LOG_MSG
	wpa_printf(MSG_EXCESSIVE, "STA " MACSTR " sent probe request for %s "
		   "SSID", MAC2STR(mgmt->sa),
		   elems.ssid_len == 0 ? "broadcast" : "our");
#else
	da16x_ap_prt("[%s] STA " MACSTR " sent probe request for %s "
           "SSID\n", __func__, MAC2STR(mgmt->sa),
           elems.ssid_len == 0 ? "broadcast" : "our");
#endif	/* CONFIG_DBG_LOG_MSG */
}


static u8 * hostapd_probe_resp_offloads(struct hostapd_data *hapd,
					size_t *resp_len)
{
	/* check probe response offloading caps and print warnings */
	if (!(hapd->iface->drv_flags & WPA_DRIVER_FLAGS_PROBE_RESP_OFFLOAD))
		return NULL;

#ifdef CONFIG_WPS
	if (hapd->conf->wps_state && hapd->wps_probe_resp_ie &&
	    (!(hapd->iface->probe_resp_offloads &
	       (WPA_DRIVER_PROBE_RESP_OFFLOAD_WPS |
		WPA_DRIVER_PROBE_RESP_OFFLOAD_WPS2))))
		wpa_printf(MSG_WARNING, "Device is trying to offload WPS "
			   "Probe Response while not supporting this");
#endif /* CONFIG_WPS */

#ifdef CONFIG_P2P
	if ((hapd->conf->p2p & P2P_ENABLED) && hapd->p2p_probe_resp_ie &&
	    !(hapd->iface->probe_resp_offloads &
	      WPA_DRIVER_PROBE_RESP_OFFLOAD_P2P))
		wpa_printf(MSG_WARNING, "Device is trying to offload P2P "
			   "Probe Response while not supporting this");
#endif  /* CONFIG_P2P */

#ifdef	CONFIG_INTERWORKING
	if (hapd->conf->interworking &&
	    !(hapd->iface->probe_resp_offloads &
	      WPA_DRIVER_PROBE_RESP_OFFLOAD_INTERWORKING))
		wpa_printf(MSG_WARNING, "Device is trying to offload "
			   "Interworking Probe Response while not supporting "
			   "this");
#endif	/* CONFIG_INTERWORKING */

	/* Generate a Probe Response template for the non-P2P case */
	return hostapd_gen_probe_resp(hapd, NULL, 0, resp_len);
}

#endif /* NEED_AP_MLME */


void sta_track_del(struct hostapd_sta_info *info)
{
#ifdef CONFIG_TAXONOMY
	wpabuf_free(info->probe_ie_taxonomy);
	info->probe_ie_taxonomy = NULL;
#endif /* CONFIG_TAXONOMY */
	os_free(info);
}


int ieee802_11_build_ap_params(struct hostapd_data *hapd,
			       struct wpa_driver_ap_params *params)
{
	struct _ieee80211_mgmt *head = NULL;
	u8 *tail = NULL;
	size_t head_len = 0, tail_len = 0;
	u8 *resp = NULL;
	size_t resp_len = 0;
#ifdef NEED_AP_MLME
	u16 capab_info;
	u8 *pos, *tailpos, *csa_pos;

#define BEACON_HEAD_BUF_SIZE 256
#define BEACON_TAIL_BUF_SIZE 512
	head = os_zalloc(BEACON_HEAD_BUF_SIZE);
	tail_len = BEACON_TAIL_BUF_SIZE;
#ifdef CONFIG_WPS
	if (hapd->conf->wps_state && hapd->wps_beacon_ie)
		tail_len += wpabuf_len(hapd->wps_beacon_ie);
#endif /* CONFIG_WPS */
#ifdef CONFIG_P2P
	if (hapd->p2p_beacon_ie)
		tail_len += wpabuf_len(hapd->p2p_beacon_ie);
#endif /* CONFIG_P2P */
#ifdef CONFIG_FST
	if (hapd->iface->fst_ies)
		tail_len += wpabuf_len(hapd->iface->fst_ies);
#endif /* CONFIG_FST */
#ifdef	CONFIG_AP_VENDOR_ELEM
	if (hapd->conf->vendor_elements)
		tail_len += wpabuf_len(hapd->conf->vendor_elements);
#endif	/* CONFIG_AP_VENDOR_ELEM */

#ifdef CONFIG_IEEE80211AC
	if (hapd->conf->vendor_vht) {
		tail_len += 5 + 2 + sizeof(struct ieee80211_vht_capabilities) +
			2 + sizeof(struct ieee80211_vht_operation);
	}
#endif /* CONFIG_IEEE80211AC */

#ifdef CONFIG_IEEE80211AX
	if (hapd->iconf->ieee80211ax) {
		tail_len += 3 + sizeof(struct ieee80211_he_capabilities) +
			3 + sizeof(struct ieee80211_he_operation);
	}
#endif /* CONFIG_IEEE80211AX */

	tail_len += hostapd_mbo_ie_len(hapd);
#ifdef CONFIG_OWE_BEACON
	tail_len += hostapd_eid_owe_trans_len(hapd);
#endif /* CONFIG_OWE_BEACON */

	tailpos = tail = os_malloc(tail_len);
	if (head == NULL || tail == NULL) {
		wpa_printf(MSG_ERROR, "Failed to set beacon data");
		os_free(head);
		os_free(tail);
		return -1;
	}

	head->frame_control = IEEE80211_FC(WLAN_FC_TYPE_MGMT,
					   WLAN_FC_STYPE_BEACON);
	head->duration = host_to_le16(0);
	os_memset(head->da, 0xff, ETH_ALEN);

	os_memcpy(head->sa, hapd->own_addr, ETH_ALEN);
	os_memcpy(head->bssid, hapd->own_addr, ETH_ALEN);
	head->u.beacon.beacon_int =
		host_to_le16(hapd->iconf->beacon_int);

	/* hardware or low-level driver will setup seq_ctrl and timestamp */
	capab_info = hostapd_own_capab_info(hapd);
	head->u.beacon.capab_info = host_to_le16(capab_info);
	pos = &head->u.beacon.variable[0];

	/* SSID */
	*pos++ = WLAN_EID_SSID;
	if (hapd->conf->ignore_broadcast_ssid == 2) {
		/* clear the data, but keep the correct length of the SSID */
		*pos++ = hapd->conf->ssid.ssid_len;
		os_memset(pos, 0, hapd->conf->ssid.ssid_len);
		pos += hapd->conf->ssid.ssid_len;
	} else if (hapd->conf->ignore_broadcast_ssid) {
		*pos++ = 0; /* empty SSID */
	} else {
		*pos++ = hapd->conf->ssid.ssid_len;
		os_memcpy(pos, hapd->conf->ssid.ssid,
			  hapd->conf->ssid.ssid_len);
		pos += hapd->conf->ssid.ssid_len;
	}

	/* Supported rates */
	pos = hostapd_eid_supp_rates(hapd, pos);

	/* DS Params */
	pos = hostapd_eid_ds_params(hapd, pos);

	head_len = pos - (u8 *) head;

	tailpos = hostapd_eid_country(hapd, tailpos,
				      tail + BEACON_TAIL_BUF_SIZE - tailpos);

	/* Power Constraint element */
	tailpos = hostapd_eid_pwr_constraint(hapd, tailpos);

	/* CSA IE */
	csa_pos = hostapd_eid_csa(hapd, tailpos);
	if (csa_pos != tailpos)
		hapd->cs_c_off_beacon = csa_pos - tail - 1;
	tailpos = csa_pos;

	/* ERP Information element */
	tailpos = hostapd_eid_erp_info(hapd, tailpos);

	/* Extended supported rates */
	tailpos = hostapd_eid_ext_supp_rates(hapd, tailpos);

	/* RSN, MDIE, WPA */
	tailpos = hostapd_eid_wpa(hapd, tailpos, tail + BEACON_TAIL_BUF_SIZE -
				  tailpos);

#ifdef CONFIG_RADIO_MEASUREMENTS
	tailpos = hostapd_eid_rm_enabled_capab(hapd, tailpos,
					       tail + BEACON_TAIL_BUF_SIZE -
					       tailpos);
#endif /* CONFIG_RADIO_MEASUREMENTS */

	tailpos = hostapd_eid_bss_load(hapd, tailpos,
				       tail + BEACON_TAIL_BUF_SIZE - tailpos);

#ifdef	CONFIG_SUPP27_BEACON
	/* eCSA IE */
	csa_pos = hostapd_eid_ecsa(hapd, tailpos);
	if (csa_pos != tailpos)
		hapd->cs_c_off_ecsa_beacon = csa_pos - tail - 1;
	tailpos = csa_pos;

	tailpos = hostapd_eid_supported_op_classes(hapd, tailpos);
#endif	/* CONFIG_SUPP27_BEACON */

#ifdef CONFIG_IEEE80211N
#ifdef	CONFIG_AP_HT
#ifdef	CONFIG_SUPP27_BEACON
	/* Secondary Channel Offset element */
	/* TODO: The standard doesn't specify a position for this element. */
	tailpos = hostapd_eid_secondary_channel(hapd, tailpos);
#endif	/* CONFIG_SUPP27_BEACON */

	tailpos = hostapd_eid_ht_capabilities(hapd, tailpos);
	tailpos = hostapd_eid_ht_operation(hapd, tailpos);
#endif	/* CONFIG_AP_HT */
#endif /* CONFIG_IEEE80211N */

	tailpos = hostapd_eid_ext_capab(hapd, tailpos);

#ifdef	CONFIG_TIME_ADV
	/*
	 * TODO: Time Advertisement element should only be included in some
	 * DTIM Beacon frames.
	 */
	tailpos = hostapd_eid_time_adv(hapd, tailpos);
#endif	/* CONFIG_TIME_ADV */
#ifdef	CONFIG_INTERWORKING
	tailpos = hostapd_eid_interworking(hapd, tailpos);
#endif	/* CONFIG_INTERWORKING */
#ifdef	CONFIG_ADV_PROTO
	tailpos = hostapd_eid_adv_proto(hapd, tailpos);
#endif	/* CONFIG_ADV_PROTO */
#ifdef	CONFIG_ROAM_CONSORTIUM
	tailpos = hostapd_eid_roaming_consortium(hapd, tailpos);
#endif	/* CONFIG_ROAM_CONSORTIUM */

#ifdef CONFIG_FST
	if (hapd->iface->fst_ies) {
		os_memcpy(tailpos, wpabuf_head(hapd->iface->fst_ies),
			  wpabuf_len(hapd->iface->fst_ies));
		tailpos += wpabuf_len(hapd->iface->fst_ies);
	}
#endif /* CONFIG_FST */

#ifdef CONFIG_IEEE80211AC
	if (hapd->iconf->ieee80211ac && !hapd->conf->disable_11ac) {
		tailpos = hostapd_eid_vht_capabilities(hapd, tailpos, 0);
		tailpos = hostapd_eid_vht_operation(hapd, tailpos);
		tailpos = hostapd_eid_txpower_envelope(hapd, tailpos);
		tailpos = hostapd_eid_wb_chsw_wrapper(hapd, tailpos);
	}
#endif /* CONFIG_IEEE80211AC */

#ifdef CONFIG_FILS
	tailpos = hostapd_eid_fils_indic(hapd, tailpos, 0);
#endif /* CONFIG_FILS */

#ifdef CONFIG_IEEE80211AX
	if (hapd->iconf->ieee80211ax) {
		tailpos = hostapd_eid_he_capab(hapd, tailpos);
		tailpos = hostapd_eid_he_operation(hapd, tailpos);
	}
#endif /* CONFIG_IEEE80211AX */

#ifdef CONFIG_IEEE80211AC
	if (hapd->conf->vendor_vht)
		tailpos = hostapd_eid_vendor_vht(hapd, tailpos);
#endif /* CONFIG_IEEE80211AC */

#ifdef CONFIG_AP_WMM
	/* Wi-Fi Alliance WMM */
	tailpos = hostapd_eid_wmm(hapd, tailpos);
#endif /* CONFIG_AP_WMM */
#ifdef CONFIG_WPS
	if (hapd->conf->wps_state && hapd->wps_beacon_ie) {
		os_memcpy(tailpos, wpabuf_head(hapd->wps_beacon_ie),
			  wpabuf_len(hapd->wps_beacon_ie));
		tailpos += wpabuf_len(hapd->wps_beacon_ie);
	}
#endif /* CONFIG_WPS */

#ifdef CONFIG_P2P
	if ((hapd->conf->p2p & P2P_ENABLED) && hapd->p2p_beacon_ie) {
		os_memcpy(tailpos, wpabuf_head(hapd->p2p_beacon_ie),
			  wpabuf_len(hapd->p2p_beacon_ie));
		tailpos += wpabuf_len(hapd->p2p_beacon_ie);
	}
#endif /* CONFIG_P2P */
#ifdef CONFIG_P2P_MANAGER
	if ((hapd->conf->p2p & (P2P_MANAGE | P2P_ENABLED | P2P_GROUP_OWNER)) ==
	    P2P_MANAGE)
		tailpos = hostapd_eid_p2p_manage(hapd, tailpos);
#endif /* CONFIG_P2P_MANAGER */

#ifdef CONFIG_HS20
	tailpos = hostapd_eid_hs20_indication(hapd, tailpos);
	tailpos = hostapd_eid_osen(hapd, tailpos);
#endif /* CONFIG_HS20 */

	tailpos = hostapd_eid_mbo(hapd, tailpos, tail + tail_len - tailpos);
#ifdef CONFIG_OWE_BEACON
	tailpos = hostapd_eid_owe_trans(hapd, tailpos,
					tail + tail_len - tailpos);
#endif /* CONFIG_OWE_BEACON */

#ifdef	CONFIG_AP_VENDOR_ELEM
	if (hapd->conf->vendor_elements) {
		os_memcpy(tailpos, wpabuf_head(hapd->conf->vendor_elements),
			  wpabuf_len(hapd->conf->vendor_elements));
		tailpos += wpabuf_len(hapd->conf->vendor_elements);
	}
#endif	/* CONFIG_AP_VENDOR_ELEM */

	tail_len = tailpos > tail ? tailpos - tail : 0;

	resp = hostapd_probe_resp_offloads(hapd, &resp_len);
#endif /* NEED_AP_MLME */

	os_memset(params, 0, sizeof(*params));
	params->head = (u8 *) head;
	params->head_len = head_len;
	params->tail = tail;
	params->tail_len = tail_len;
	params->proberesp = resp;
	params->proberesp_len = resp_len;
	params->dtim_period = hapd->conf->dtim_period;
	params->beacon_int = hapd->iconf->beacon_int;
	params->basic_rates = hapd->iface->basic_rates;
	params->beacon_rate = hapd->iconf->beacon_rate;
	params->rate_type = hapd->iconf->rate_type;

#ifdef CONFIG_OWE_TRANS
	if (hapd->conf->ssid.ssid_len == 0)
		params->ssid = NULL;
	else
#endif // CONFIG_OWE_TRANS
		params->ssid = hapd->conf->ssid.ssid;

	params->ssid_len = hapd->conf->ssid.ssid_len;
	if ((hapd->conf->wpa & (WPA_PROTO_WPA | WPA_PROTO_RSN)) ==
	    (WPA_PROTO_WPA | WPA_PROTO_RSN))
		params->pairwise_ciphers = hapd->conf->wpa_pairwise |
			hapd->conf->rsn_pairwise;
	else if (hapd->conf->wpa & WPA_PROTO_RSN)
		params->pairwise_ciphers = hapd->conf->rsn_pairwise;
	else if (hapd->conf->wpa & WPA_PROTO_WPA)
		params->pairwise_ciphers = hapd->conf->wpa_pairwise;
	params->group_cipher = hapd->conf->wpa_group;
	params->key_mgmt_suites = hapd->conf->wpa_key_mgmt;
	params->auth_algs = hapd->conf->auth_algs;
	params->wpa_version = hapd->conf->wpa;
#ifdef	CONFIG_AP_PRE_AUTH
	params->privacy = hapd->conf->ssid.wep.keys_set || hapd->conf->wpa ||
		(hapd->conf->ieee802_1x &&
		 (hapd->conf->default_wep_key_len ||
		  hapd->conf->individual_wep_key_len));
#else
	params->privacy = hapd->conf->wpa;
#endif	/* CONFIG_AP_PRE_AUTH */

	switch (hapd->conf->ignore_broadcast_ssid) {
	case 0:
		params->hide_ssid = NO_SSID_HIDING;
		break;
	case 1:
		params->hide_ssid = HIDDEN_SSID_ZERO_LEN;
		break;
	case 2:
		params->hide_ssid = HIDDEN_SSID_ZERO_CONTENTS;
		break;
	}

#ifdef CONFIG_OWE_TRANS
	params->beacon_set = 1;
#endif // CONFIG_OWE_TRANS

#ifdef CONFIG_AP_ISOLATION
	params->isolate = hapd->conf->isolate;
#endif /* CONFIG_AP_ISOLATION */

	params->smps_mode = hapd->iconf->ht_capab & HT_CAP_INFO_SMPS_MASK;
#ifdef NEED_AP_MLME
	params->cts_protect = !!(ieee802_11_erp_info(hapd) &
				ERP_INFO_USE_PROTECTION);
	params->preamble = hapd->iface->num_sta_no_short_preamble == 0 &&
		hapd->iconf->preamble == SHORT_PREAMBLE;
	if (hapd->iface->current_mode &&
	    hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211G)
		params->short_slot_time =
			hapd->iface->num_sta_no_short_slot_time > 0 ? 0 : 1;
	else
		params->short_slot_time = -1;
	if (!hapd->iconf->ieee80211n || hapd->conf->disable_11n)
		params->ht_opmode = -1;
	else
		params->ht_opmode = hapd->iface->ht_op_mode;
#endif /* NEED_AP_MLME */

#ifdef	CONFIG_INTERWORKING
	params->interworking = hapd->conf->interworking;
	if (hapd->conf->interworking &&
	    !is_zero_ether_addr(hapd->conf->hessid))
		params->hessid = hapd->conf->hessid;
	params->access_network_type = hapd->conf->access_network_type;
#endif	/* CONFIG_INTERWORKING */

	params->ap_max_inactivity = hapd->conf->ap_max_inactivity;
#ifdef CONFIG_P2P
#ifdef	CONFIG_P2P_UNUSED_CMD
	params->p2p_go_ctwindow = hapd->iconf->p2p_go_ctwindow;
#endif	/* CONFIG_P2P_UNUSED_CMD */
#endif /* CONFIG_P2P */

#ifdef CONFIG_HS20
	params->disable_dgaf = hapd->conf->disable_dgaf;
	if (hapd->conf->osen) {
		params->privacy = 1;
		params->osen = 1;
	}
#endif /* CONFIG_HS20 */
#ifdef CONFIG_M2U_BC_DEAUTH
	params->multicast_to_unicast = hapd->conf->multicast_to_unicast;
#endif /* CONFIG_M2U_BC_DEAUTH */
	params->pbss = hapd->conf->pbss;
	return 0;
}


void ieee802_11_free_ap_params(struct wpa_driver_ap_params *params)
{
	os_free(params->tail);
	params->tail = NULL;
	os_free(params->head);
	params->head = NULL;
	os_free(params->proberesp);
	params->proberesp = NULL;
}


int ieee802_11_set_beacon(struct hostapd_data *hapd)
{
	struct wpa_driver_ap_params params;
	struct hostapd_freq_params freq;
	struct hostapd_iface *iface = hapd->iface;
	struct hostapd_config *iconf = iface->conf;
	struct wpabuf *beacon, *proberesp, *assocresp;
	int res, ret = -1;

	if (hapd->csa_in_progress) {
		wpa_printf(MSG_ERROR, "Cannot set beacons during CSA period");
		return -1;
	}

	hapd->beacon_set_done = 1;

	if (ieee802_11_build_ap_params(hapd, &params) < 0)
		return -1;

	if (hostapd_build_ap_extra_ies(hapd, &beacon, &proberesp, &assocresp) < 0)
		goto fail;

	params.beacon_ies = beacon;
	params.proberesp_ies = proberesp;
	params.assocresp_ies = assocresp;
	params.reenable = hapd->reenable_beacon;
	hapd->reenable_beacon = 0;

	if (iface->current_mode &&
#ifdef	CONFIG_IEEE80211AC
	    hostapd_set_freq_params(&freq, iconf->hw_mode, iface->freq,
				    iconf->channel, iconf->ieee80211n,
				    iconf->ieee80211ac,
				    iconf->secondary_channel,
				    iconf->vht_oper_chwidth,
				    iconf->vht_oper_centr_freq_seg0_idx,
				    iconf->vht_oper_centr_freq_seg1_idx,
				    iface->current_mode->vht_capab) == 0
#else
	    hostapd_set_freq_params(&freq, iconf->hw_mode, iface->freq,
				    iconf->channel, iconf->ieee80211n,
				    0, iconf->secondary_channel,
				    0, 0, 0, 0) == 0
#endif	/* CONFIG_IEEE80211AC */
	) {
		params.freq = &freq;
	}

	res = hostapd_drv_set_ap(hapd, &params);
	hostapd_free_ap_extra_ies(hapd, beacon, proberesp, assocresp);
	if (res) {
		wpa_printf(MSG_ERROR, "Failed to set beacon parameters");
	} else
		ret = 0;
fail:
	ieee802_11_free_ap_params(&params);
	return ret;
}


int ieee802_11_set_beacons(struct hostapd_iface *iface)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < iface->num_bss; i++) {
		if (iface->bss[i]->started &&
		    ieee802_11_set_beacon(iface->bss[i]) < 0)
			ret = -1;
	}

	return ret;
}


/* only update beacons if started */
int ieee802_11_update_beacons(struct hostapd_iface *iface)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < iface->num_bss; i++) {
		if (iface->bss[i]->beacon_set_done && iface->bss[i]->started &&
		    ieee802_11_set_beacon(iface->bss[i]) < 0)
			ret = -1;
	}

	return ret;
}

#endif	/* CONFIG_AP */

/* EOF */
