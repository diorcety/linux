/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*
*   \file 
*   \brief
*   \author Montage
*/
#include <os_compat.h>
#include <linux_wla.h>
#include <mac_common.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <asm/mach-cheetah/common.h>
#include <wla_debug.h>
#include <rf.h>
#define cyg_drv_interrupt_unmask(n) enable_irq(n)
#define cyg_drv_interrupt_mask(n) disable_irq(n)

char *curfunc;

u32 __hw_mode = 0x4;
u32 __tx_power = 0x0;
u32 __wl_power_backoff = 0x0;
u32 __rfchip = 0;			/* 0 for auto-detect, works fine for RFCHIP_IP301_G */
u32 __ex_ldo = 0;
u32 __sw_forward = 0;		/* 0:hw path ; 1:sw path */
u32 __ap_isolated = 0;		/* 0:disable ; 1:enable */
u32 __bss_isolated = 0;     /* 0:disable ; 1:enable */
u32 __wds_mode = 0;			/* 0:disable ; 1:enable */
u32 __sta_mat = 0;			/* 0:disable ; 1:enable */
u32	__pw_save = 0;
u32	__no_ampdu = 0;			/* 0: enable RX ampdu; 1: disable ampdu RX */
u32	__ampdu_tx_mask = 0;	/* mask bit map */
u32	__apid = 0;				/* AP unit id */
u32	__staid = 1;			/* STA unit id */
u32	__fix_txrate = 0;		/* Fix tx rate */
u32	__rate_flags = 0;		/* rate flags */
u32 __recovery = 0;			/* MAC recovery */
u64 __wds_peer_addr[WDS_TABLE_ENTRIES];
u32 __cipher_type; /*0:None,1:WEP40,2:WEP104,3:TKIP,4:CCMP,5:WAPI*/
u32 __wds_keyid = 0;
u32 __uapsd_enable = 0;
struct wds_key __wds_key[WDS_TABLE_ENTRIES];

extern MAC_INFO *info;
extern struct macaddr_pool maddr_tables;
extern struct timer_list timer_g;
static struct cfg80211_ops wla_cfg_ops;

/* For ACS reset channel to 2412 channel 1 */
void wla_acs_reset(struct ieee80211_hw *hw)
{
	struct ieee80211_local *local;
	struct cfg80211_chan_def dfchan = {};
	struct ieee80211_supported_band *sband;
			
	local = container_of(hw, struct ieee80211_local, hw);

	sband = local->hw.wiphy->bands[IEEE80211_BAND_2GHZ];
		
	cfg80211_chandef_create(&dfchan,
							&sband->channels[0],
							NL80211_CHAN_NO_HT);
	local->hw.conf.chandef = dfchan;
	local->_oper_chandef = dfchan;

}

void wla_update_survey_stats(struct wla_softc *sc, char used)
{
	struct survey_info *survey = NULL;
	int chan;

	chan = wla_transfer_channel_num(sc->curr_center_freq);

	survey = (struct survey_info *)&sc->survey[chan-1];

	bb_rx_counter_ctrl(BB_RX_CNT_DISABLE);

	/*Record the stats*/
	survey->channel_time += bb_rx_counter_read(BB_RX_CHANNEL_CNT_ADDR);
	survey->channel_time_busy += bb_rx_counter_read(BB_RX_BUSY_CNT_ADDR);
	survey->noise = bb_read_noise_floor();
	
	survey->filled = SURVEY_INFO_CHANNEL_TIME | 
					 SURVEY_INFO_CHANNEL_TIME_BUSY |
					 SURVEY_INFO_NOISE_DBM;

	if (used)
		survey->filled |= SURVEY_INFO_IN_USE;

}

static void set_sta_info(struct wsta_cb *wcb, struct station_info *sinfo)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;

	captbl = &info->sta_cap_tbls[wcb->captbl_idx];

	sinfo->filled = STATION_INFO_RX_BYTES |
	        STATION_INFO_RX_PACKETS |
	        STATION_INFO_TX_BITRATE;

	sinfo->filled |= STATION_INFO_SIGNAL;
	sinfo->signal = -bb_rssi_decode1(wcb->rssi);

	if(wcb->rate_flags & (RATE_FLAGS_HT_SGI20|RATE_FLAGS_HT_SGI40))
		sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;

	if(sinfo->rx_packets != captbl->rx_pkt_cnt)
	{
		sinfo->filled |= STATION_INFO_INACTIVE_TIME ;
		sinfo->inactive_time = jiffies_to_msecs(jiffies - wcb->last_rx);
		sinfo->rx_packets = captbl->rx_pkt_cnt;
		sinfo->rx_bytes = captbl->rx_byte_cnt;
	}
	
	if(captbl->rate[0].bit.ht == 1) {
		sinfo->txrate.flags |= RATE_INFO_FLAGS_MCS;
		sinfo->txrate.mcs = (captbl->rate[0].val & 7);
		if(captbl->rate[0].bit.ch_offset > 0)
			sinfo->txrate.flags |= RATE_INFO_FLAGS_40_MHZ_WIDTH;
	}
	else
		sinfo->txrate.legacy = get_hw_bitrate(wcb->cur_tx_rate[0].rate_idx);
	
	return;
}


static int wla_cfg_get_station(struct wiphy *wiphy, struct net_device *dev,
								u8 *mac, struct station_info *sinfo)
{
	int ret = -ENOENT;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct sta_info *stainfo;
	struct ieee80211_sta *sta;
	struct wla_sta_priv *sp;
	struct wsta_cb *wcb;
	
	rcu_read_lock();
	
	stainfo = sta_info_get(sdata, mac);
	if (stainfo) {
	    ret = 0;
		sta=&stainfo->sta;
		sp = (struct wla_sta_priv *)sta->drv_priv;
		wcb = sp->wcb;
	    memcpy(mac, stainfo->sta.addr, ETH_ALEN);
	    set_sta_info(wcb, sinfo);
	}
	
	rcu_read_unlock();
	
	return ret;
}

static int wla_cfg_dump_station(struct wiphy *wiphy, struct net_device *dev,
								int idx, u8 *mac, struct station_info *sinfo)
{
	int ret = -ENOENT;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct sta_info *stainfo;
	struct ieee80211_sta *sta;
	struct wla_sta_priv *sp;

	struct wsta_cb *wcb;

	struct ieee80211_local *local = sdata->local;

	mutex_lock(&local->sta_mtx);
	
	stainfo = sta_info_get_by_idx(sdata, idx);
	if (stainfo) {
	    ret = 0;
		sta=&stainfo->sta;
		sp = (struct wla_sta_priv *)sta->drv_priv;
		wcb = sp->wcb;
	    memcpy(mac, stainfo->sta.addr, ETH_ALEN);
	    set_sta_info(wcb, sinfo);
	}
	
	mutex_unlock(&local->sta_mtx);

	return ret;
}

static int wla_cfg_set_wiphy(struct wiphy *wiphy, u32 changed)
{
	struct ieee80211_local *local = wiphy_priv(wiphy);
	MAC_INFO *info = WLA_MAC_INFO;

	if (changed & WIPHY_PARAM_RTS_THRESHOLD)
	{
		if(wiphy->rts_threshold)
			wla_cfg(SET_RTS_THRESHOLD,wiphy->rts_threshold - 24);
	}

	if (changed & WIPHY_PARAM_RETRY_SHORT)
	{		
		local->hw.conf.short_frame_max_tx_count = wiphy->retry_short;
		info->short_retry = wiphy->retry_short;
	}
	if (changed & WIPHY_PARAM_RETRY_LONG)
	{		
		local->hw.conf.long_frame_max_tx_count = wiphy->retry_long;
		info->long_retry = wiphy->retry_long;
	}
	
	/* Need to modify */
/*
	if (changed &
	    (WIPHY_PARAM_RETRY_SHORT | WIPHY_PARAM_RETRY_LONG))
		ieee80211_hw_config(local, IEEE80211_CONF_CHANGE_RETRY_LIMITS);
*/
	if((changed & WIPHY_PARAM_FRAG_THRESHOLD)) 
	{
		if(wiphy->frag_threshold)
			wla_cfg(SET_FRAG_THRESHOLD,wiphy->frag_threshold - 24);
	}

	return 0;

}

static int wla_cfg_set_tx_power(struct wiphy *wiphy, struct wireless_dev *wdev,
				enum nl80211_tx_power_setting type, int mbm)
{
	MAC_INFO *info = WLA_MAC_INFO;
	unsigned int power=0;
	
	power = MBM_TO_DBM(mbm);
	if(power > MAX_TX_POWER)
		power = MAX_TX_POWER;
	wla_tx_power(power);
	wla_set_channel(info);

	return 0;

}

void wla_cfg80211_ops(struct ieee80211_hw *hw)
{
	struct cfg80211_registered_device *rdev;

	rdev = container_of(hw->wiphy ,struct cfg80211_registered_device,wiphy);
	memcpy(&wla_cfg_ops,rdev->ops,sizeof(struct cfg80211_ops));
	if(rdev)
	{
		wla_cfg_ops.get_station = &wla_cfg_get_station;
		wla_cfg_ops.dump_station = &wla_cfg_dump_station;
		wla_cfg_ops.set_wiphy_params = &wla_cfg_set_wiphy;
		wla_cfg_ops.set_tx_power = &wla_cfg_set_tx_power;
		rdev->ops = &wla_cfg_ops;
	}
}

void wla_wds_setup(struct wla_softc *sc)
{
	int idx;
	char *mac;
	struct wds_peer *wds_peer;
	char psk[128];

	(void)psk;
	if((sc->wds_mode = __wds_mode) != AP_WDS_NONE)
	{		
		for(idx=0; idx<WDS_TABLE_ENTRIES; idx++)
		{	
			wds_peer = WLA_MALLOC(sizeof(struct wds_peer), DMA_MALLOC_BZERO);
			wds_peer->wcb = wla_get_wcb();
	
			mac = (char *)&__wds_peer_addr[idx];

			if(is_zero_ether_addr(&mac[2]))
				continue;
			
			memcpy(wds_peer->addr, &mac[2], WLAN_ADDR_LEN);
			
			wds_peer->next = sc->all_wds_peer;
			sc->all_wds_peer = wds_peer;

			wds_peer->wcb->supported_rates = 0xfff;
			wds_peer->wcb->qos = 1;
			wds_peer->wcb->controlled_port = 1;
			wla_get_sta((char *)wds_peer->addr, 0, wds_peer->wcb, WDS_LINKED_AP);

			/*Need key setup*/
			if((wds_peer->cipher = __wds_key[idx].cipher_type) != 0)
			{
				wds_peer->key_idx = __wds_key[idx].txkeyid;

				if((wds_peer->cipher == AUTH_CIPHER_WEP40) || (wds_peer->cipher == AUTH_CIPHER_WEP104))
				{
					memcpy(wds_peer->key, (char *)(__wds_key[idx].key[wds_peer->key_idx]) , WLAN_WEP_104BIT_KEYLEN);
				}
				else if((wds_peer->cipher == AUTH_CIPHER_TKIP) || (wds_peer->cipher == AUTH_CIPHER_CCMP))
				{
					memcpy(&wds_peer->key[0], (char *)(__wds_key[0].key[0]), __wds_key[0].keylen[0]);
					memcpy(&wds_peer->key[16+8], &wds_peer->key[16], 8);
					wds_peer->key_idx = 0;
				}

				wla_cipher_key(wds_peer->wcb, wds_peer->cipher, KEY_TYPE_PAIRWISE_KEY,
							(char *)wds_peer->key, wds_peer->key_idx, 1);
			}
		}

	}
}

void wla_chg_tx_rate(short bss, int rate_index, char type)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct tx_rate_t *trate;
	sta_cap_tbl *captbl;
	u8 txrate_idx[4];
	u8 tx_retry[4] = {4,2,1,1};
	int i;

	trate = &(info->mbss[bss].tx_rate[(short)type]);
	captbl = &info->rate_tbls[trate->rate_captbl];
	
	for(i=0; i<4; i++)
		txrate_idx[i] = rate_index;

	for(i = 0; i < 4; i++)
	{
		captbl->rate[i].val = wla_rate_encode(info, txrate_idx[i], tx_retry[i], 
					RATE_FLAGS_SHORT_PREAMBLE);
	}	

	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));

}


void wla_add_timer(unsigned int interval)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;

	cancel_delayed_work(&sc->hw_sync_work);
	ieee80211_queue_delayed_work(sc->hw, &sc->hw_sync_work, interval);
}

void wla_del_timer(void)
{
	struct ieee80211_local *local;
	struct ieee80211_hw *hw;
	struct wla_softc *sc;
	MAC_INFO *info = WLA_MAC_INFO;
	
	sc = info->sc;
	hw = sc->hw;
	local = container_of(hw, struct ieee80211_local, hw);
	//flush_workqueue(sc->irq_wq_rc);
	flush_workqueue(local->workqueue);
}

void wla_hal_cipher_key(struct ieee80211_sta *sta, struct ieee80211_vif *vif, bool add)
{
	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	struct wla_sta_priv *sp = (struct wla_sta_priv *)sta->drv_priv;
	struct wsta_cb *wcb = sp->wcb; 
	cipher_key *ghwkey;
	int gidx, i;
	MAC_INFO *info = WLA_MAC_INFO;

	if((vp->cipher_type == AUTH_CIPHER_WEP40) || (vp->cipher_type == AUTH_CIPHER_WEP104))
	{
		if(add)
		{		
			if(vif->type == NL80211_IFTYPE_AP)
			{	
				gidx = vp->bss_desc;
				ghwkey = &info->group_keys[gidx].group_key;
				for(i=0; i<WLAN_WEP_NKID; i++)
				{
					wla_cipher_key(wcb, vp->cipher_type, KEY_TYPE_PAIRWISE_KEY,
						(char *)&(ghwkey->wep_key.key[i]),i,(ghwkey->wep_key.txkeyid == i)?1:0);
				}
			}			
		}
		else
		{
			if(vif->type == NL80211_IFTYPE_AP)
				wla_cipher_key(wcb, vp->cipher_type, KEY_TYPE_PAIRWISE_KEY,
					0, 0, 0);
		}
	}
}

void wla_lock(void)
{
	MAC_INFO *info = wla_mac_info();
	struct wla_softc *sc = (struct wla_softc *)info->sc;

	if(sc->lock_counter == 0)
		spin_lock_bh(&sc->lock);

	curfunc = __builtin_return_address(0);

	sc->lock_counter++;	
}

void wla_unlock(void)
{
	MAC_INFO *info = wla_mac_info();
	struct wla_softc *sc = (struct wla_softc *)info->sc;

	if(sc->lock_counter == 0)
		panic(" leak lock but unlock\n");
	if(--sc->lock_counter == 0 )
		spin_unlock_bh(&sc->lock);
}

void wla_sleep(unsigned int sleep_tick)
{
	msleep(sleep_tick*10);
}

void wla_beacon_t(struct wla_softc *sc)
{
	MAC_INFO *info = (MAC_INFO *)sc->mac_info;
	unsigned int val;

#if defined(CONFIG_CHEETAH_SMART_CONFIG)
        // disable Beacon TX during smart-config period
        if (__smartcfg_enable == 0)
            wla_software_intr((void *)info->ts_status);
#else
	wla_software_intr((void *)info->ts_status);
#endif

	val = MACREG_READ32(MAC_INT_MASK_REG);
	MACREG_WRITE32(MAC_INT_MASK_REG,val & ~(MAC_INT_PRETBTT));

}

void wla_mgmt_process(struct wbuf *wb, void *wcb)
{
	struct sk_buff *skb;
	struct ieee80211_rx_status rx_status;
	struct ieee80211_tx_info *txi;
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = (struct wla_softc *)info->sc;
	ehdr *ef;
	struct net_device *dev;
	int bss_desc=0;

	skb = (struct sk_buff *)(*(unsigned int *)((char *)wb - SKB_WB_OFFSET));

	//struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;

	if(wb->flags & WBUF_TX_REPORT_STATUS)
	{
		txi = IEEE80211_SKB_CB(skb);
		{
			ieee80211_tx_info_clear_status(txi);
			if ((!(txi->flags & IEEE80211_TX_CTL_NO_ACK)) && (wb->flags & WBUF_TX_SUCCESS))
				txi->flags |= IEEE80211_TX_STAT_ACK;
			#if 0
			if(ieee80211_is_data(hdr->frame_control))
			{		
				printk("!!!!!!!!!!!!!!!!!!!!!!! %s  DATA with wifi header txi flags:%d\n",__func__,txi->flags);
				WLA_DUMP_BUF(skb->data, skb->len);
			}
			#endif
			ieee80211_tx_status_irqsafe(sc->hw, skb);
		}
	}
	else {
		if(wb->secst & WBUF_RX_MIC_ERR)
		{	
			if(wcb)
			{
				bss_desc = ((struct wsta_cb *)wcb)->bss_desc;
			}
			else
			{
				bss_desc = wb->bss_desc;
			}
			ef = (ehdr *)((int)wb + wb->data_off);
			dev = sc->bssdev[bss_desc];

			cfg80211_michael_mic_failure(dev,ef->sa,ef->da[0]&0x01? 
			NL80211_KEYTYPE_GROUP:NL80211_KEYTYPE_PAIRWISE,0,NULL,GFP_ATOMIC);
			
			WBUF_FREE(wb);
			return;
		}
		memset(&rx_status, 0, sizeof(rx_status));
		rx_status.freq = sc->curr_center_freq;
		rx_status.band = sc->curr_band;
		rx_status.rate_idx = 0;
		rx_status.signal = -bb_rssi_decode1(wb->rssi); //dbm
		
		memcpy(IEEE80211_SKB_RXCB(skb), &rx_status, sizeof(rx_status));
		skb->data = (char *)wb + wb->data_off;
		skb->len = wb->pkt_len; 
		
		ieee80211_rx_irqsafe(sc->hw, skb);
	}
}

int disable_mac_interrupts(void)
{
    MACREG_WRITE32(MAC_INT_MASK_REG, MAC_INT_FULL_MASK);

    return 0;
}



unsigned char wla_key_chg(struct ieee80211_vif *vif, struct ieee80211_key_conf *key)
{
	unsigned char key_type;

    switch (vif->type) {
    case NL80211_IFTYPE_STATION:
        if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE ) {
			//idb_print("STA PAIRWISE KEY\n");
            key_type = KEY_TYPE_STA_PAIRWISE_KEY;
		}
        else {
			//idb_print("STA GLOBAL KEY\n");
            key_type = KEY_TYPE_GLOBAL_KEY;
		}
        break;
    case NL80211_IFTYPE_AP:
        if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE ) {
			//idb_print("AP PAIRWISE KEY\n");
            key_type = KEY_TYPE_PAIRWISE_KEY;
		}
        else {
			//idb_print("AP GLOBAL KEY\n");
            key_type = KEY_TYPE_GLOBAL_KEY;
		}
        break;
	default:
        if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE )
            key_type = KEY_TYPE_STA_PAIRWISE_KEY;
        else
            key_type = KEY_TYPE_GLOBAL_KEY;
        break;
		
    }
	return key_type;

}

unsigned char wla_cipher_chg(struct ieee80211_key_conf *key)
{
	unsigned char cipher_type;

	switch (key->cipher) 
	{
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			if(key->keylen == 5)
				cipher_type = AUTH_CIPHER_WEP40;
			else if(key->keylen == 13)
				cipher_type = AUTH_CIPHER_WEP104;
			else
				return -EOPNOTSUPP;
			break;
	
		case WLAN_CIPHER_SUITE_TKIP:
			cipher_type = AUTH_CIPHER_TKIP;
			break;
	
		case WLAN_CIPHER_SUITE_CCMP:
			cipher_type = AUTH_CIPHER_CCMP;
			break;
		#if defined(CONFIG_WLA_WAPI) || defined(CONFIG_WAPI)
		case WLAN_CIPHER_SUITE_SMS4:
			cipher_type = AUTH_CIPHER_SMS4;
			break;
		#endif
		default:
			return -EOPNOTSUPP;
			break;
	}
	return cipher_type;

}

int wla_cfg_station(struct wla_softc *sc, struct ieee80211_sta *sta, struct ieee80211_vif *vif, enum sta_notify_cmd cmd)
{
	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	struct wla_sta_priv *sp = (struct wla_sta_priv *)sta->drv_priv;
	struct wsta_cb *wcb;
	struct sta_info *stainfo;
	int supp_rates;
	struct net_device *wdev;
	struct ieee80211_sub_if_data *sdata = NULL;
	char mode = 0;

	sdata = vif_to_sdata(vif);
	wdev = sdata->dev;
	
	if(cmd == STA_NOTIFY_ADD)
	{
		wcb = wla_get_wcb();
		if(wcb == 0)
		{
			WLA_DBG(WLADEBUG, "Cannot get wcb for station\n");
			return -1;
		}

		sp->wcb = wcb;
		wcb->sta = sta;
	
		WLA_DBG(WLADEBUG, "Add wcb %p for station join\n", wcb);
	}
	else
		wcb = sp->wcb;
	
	if(!wcb)
		return 0;

	stainfo = container_of(sta, struct sta_info, sta);

	supp_rates = sta->supp_rates[sc->curr_band];
	if(sta->ht_cap.ht_supported)
	{
		supp_rates |= ((sta->ht_cap.mcs.rx_mask[0] | ((sta->ht_cap.mcs.rx_mask[4] & 0x1) << 8)) << 12);
	}

	if(stainfo->_flags & BIT(WLAN_STA_WME))
	{
		wcb->qos = 1;
	}
	
#ifdef CONFIG_WLA_UAPSD
	/* Trigger enabled */
	if(stainfo->_flags & BIT(WLAN_STA_AC_BE_TRIGGER))
		wcb->apsd_trigger_deliver |= APSD_AC_BE_TRIGGER;
	
	if(stainfo->_flags & BIT(WLAN_STA_AC_BK_TRIGGER))
		wcb->apsd_trigger_deliver |= APSD_AC_BK_TRIGGER;
	
	if(stainfo->_flags & BIT(WLAN_STA_AC_VI_TRIGGER))
		wcb->apsd_trigger_deliver |= APSD_AC_VI_TRIGGER;

	if(stainfo->_flags & BIT(WLAN_STA_AC_VO_TRIGGER))
		wcb->apsd_trigger_deliver |= APSD_AC_VO_TRIGGER;
	
	/* Deliver enabled */
	if(stainfo->_flags & BIT(WLAN_STA_AC_BE_DELIVERY))
		wcb->apsd_trigger_deliver |= APSD_AC_BE_DELIVERY;

	if(stainfo->_flags & BIT(WLAN_STA_AC_BK_DELIVERY))
		wcb->apsd_trigger_deliver |= APSD_AC_BK_DELIVERY;

	if(stainfo->_flags & BIT(WLAN_STA_AC_VI_DELIVERY))
		wcb->apsd_trigger_deliver |= APSD_AC_VI_DELIVERY;

	if(stainfo->_flags & BIT(WLAN_STA_AC_VO_DELIVERY))
		wcb->apsd_trigger_deliver |= APSD_AC_VO_DELIVERY;
#endif

	//wcb->supported_rates = stainfo->sta.supp_rates[local->oper_channel->band];

	wcb->max_ampdu_factor = stainfo->sta.ht_cap.ampdu_factor;

	if(stainfo->sta.ht_cap.cap & IEEE80211_HT_CAP_GRN_FLD)
		wcb->rate_flags |= RATE_FLAGS_HT_GF;
	
	if(stainfo->sta.ht_cap.cap & IEEE80211_HT_CAP_SUP_WIDTH_20_40)
		wcb->rate_flags |= RATE_FLAGS_HT_40M;
	
	if(stainfo->sta.ht_cap.cap & IEEE80211_HT_CAP_SGI_20)
		wcb->rate_flags |= RATE_FLAGS_HT_SGI20;
	
	if(stainfo->sta.ht_cap.cap & IEEE80211_HT_CAP_SGI_40)
		wcb->rate_flags |= RATE_FLAGS_HT_SGI40;

	if(stainfo->_flags & BIT(WLAN_STA_SHORT_PREAMBLE))
		wcb->rate_flags |= RATE_FLAGS_SHORT_PREAMBLE;
	
	/* NL80211_CMD_SET_STATION parm*/
	wcb->supported_rates = supp_rates;
	wcb->aid = sta->aid;

	if(vif->type == NL80211_IFTYPE_AP)
		mode = 0;
	else if (vif->type == NL80211_IFTYPE_STATION)
	{
		if(__sta_mat == 1)
			mode = MAT_LINKED_AP;
		else
			mode = LINKED_AP;
	}

	if(cmd == STA_NOTIFY_ADD)
	{
		wla_get_sta(sta->addr, vp->bss_desc, wcb, mode);
		sp->assoc = 1;
	}
	else if(cmd == STA_NOTIFY_CHANGE)
		wla_chg_sta(sta->addr, vp->bss_desc, wcb, mode);

	if(stainfo->_flags & BIT(WLAN_STA_AUTHORIZED)) {
		wla_forward(DATA_CAN_FORWARD, wcb);
		
		if (vif->type == NL80211_IFTYPE_STATION)
			info->bcn_mon_timestamp = WLA_CURRENT_TIME + WLA_BCN_MON_TIME;  
	}
	else
		wla_forward(DATA_NO_FORWARD, wcb);

	return 0;
}

void wla_remove_station(struct wla_softc *sc, struct ieee80211_sta *sta, struct ieee80211_vif *vif)
{
	struct wla_sta_priv *sp = (struct wla_sta_priv *)sta->drv_priv;
	struct wsta_cb *wcb = sp->wcb;

	sp->wcb = 0;
	wla_release_sta(wcb);
}

unsigned char wla_transfer_channel_num(unsigned short freq)
{
	int i;
	for(i=0; i< (sizeof(wla_channels_2ghz)/sizeof(wla_channels_2ghz[0])); i++)
	{
		if(wla_channels_2ghz[i].center_freq == freq)
			break;
	}
	return (i+1); 
}

void wla_free_sc(struct wla_softc *sc)
{
    if(sc->dev)
    {
        device_unregister(sc->dev);
        sc->dev = NULL;
    }
}


int wla_beacon_get(struct ieee80211_vif *vif, u8 dtim)
{
	struct ieee80211_sub_if_data *sdata = NULL;
	struct ieee80211_if_ap *ap = NULL;
	struct beacon_data *beacon;
	struct wla_vif_priv *vp = (void *) vif->drv_priv;
	struct wbuf *wb1=NULL, *wb2=NULL, *wb3=NULL;
	MAC_INFO *info = WLA_MAC_INFO;
	struct ieee802_11_elems elems;
	int bw40mhz_intolerant;

	rcu_read_lock();

	sdata = vif_to_sdata(vif);

	if (vif->type == NL80211_IFTYPE_AP) 
	{
		ap = &sdata->u.ap;
		beacon = rcu_dereference(ap->beacon);

		if (ap && beacon) 
		{
			u8 *pos;
			u32 ofs;

			/* Note: check the bandwidth for the 20/40 Mhz coexistence */
			ieee802_11_parse_elems(beacon->head, beacon->head_len, 0, &elems);
			if(elems.ht_operation == NULL)
			{
				ieee802_11_parse_elems(beacon->tail, beacon->tail_len, 0, &elems);
			}

			if(elems.ht_operation)
			{
				if(elems.ht_operation->ht_param & IEEE80211_HT_PARAM_CHA_SEC_OFFSET)
					bw40mhz_intolerant = 0;
				else
					bw40mhz_intolerant = 1;
				
				wla_cfg(SET_40MHZ_INTOLERANT, bw40mhz_intolerant);
				
				WLA_DBG(WLADEBUG, "%s(): find the ht_operation, primary_chan=%d, ht_param=0x%x, bw40mhz_intolerant=%d\n", __FUNCTION__, elems.ht_operation->primary_chan, elems.ht_operation->ht_param, bw40mhz_intolerant);
			}

			ofs = sizeof(struct wbuf);
			wb1 = WBUF_ALLOC(0);
    		if(!wb1)
				goto out; //orig: return 0;
			memset((char *)wb1+ofs, 0, 256);
			memcpy((char *)(wb1)+ofs,beacon->head,beacon->head_len);

			wb1->flags = 0; /* beacon does not need flags */
			wb1->pkt_len = beacon->head_len;
			wb1->data_off = ofs; // not include wbuf
			wb1->bss_desc = vp->bss_desc;

			if((wb2 = WBUF_ALLOC(0)) == 0)
			{
				WBUF_FREE(wb1);
				goto out; //orig: return 0;
			}
			pos = (char *)wb2 + ofs;
			wb2->data_off = ofs;
			wb2->pkt_len = 6;
			*(pos++) = WLAN_EID_TIM;
			*(pos++) = 4;	/* length */
			*(pos++) = 3;
			*(pos++) = dtim;
			*(pos++) = 0;
			*(pos++) = 0;

			if (beacon->tail)
			{
				if((wb3 = WBUF_ALLOC(0)) == 0)
				{
					WBUF_FREE(wb1);
					WBUF_FREE(wb2);
					goto out;
				}
				pos = (char *)wb3+ofs;
				memset(pos, 0, 512);
				memcpy(pos,beacon->tail,beacon->tail_len);
				wb3->data_off = ofs;
				wb3->pkt_len = beacon->tail_len;
			}

			wla_add_beacon(info, wb1, wb2, wb3);

		} 
		else
			goto out;
	}
out:
	rcu_read_unlock();
	return 0;
}

void modify_macaddr(struct ieee80211_vif *vif, char *macaddr)
{
	struct macaddr_pool *mp = &maddr_tables;
	struct wla_vif_priv *vp = (void *) vif->drv_priv;
	
	if(mp->m[(unsigned char)vp->bss_desc].type == vif->type)
	{		
		memcpy(mp->m[(unsigned char)vp->bss_desc].addr, macaddr, 6);
		program_macaddr_table(mp);
	}

	return;
}

int wla_cdb_get_int(unsigned short index, int def_val)
{
    switch(index)
    {
        case $parm_hw_mode:
            return def_val;
        case $parm_wl_power_backoff:
            return def_val;
        case $parm_rfchip:
            return def_val;
        case $parm_ex_ldo:
            return def_val;
        default:
            break;
    }
    return -1;
}

unsigned int wla_current_time(void)
{
	return jiffies;
}

void *mac_malloc(unsigned int size, unsigned char flags)
{
    unsigned char *ptr, *ptr1;
	
//	if(flags & DMA_MALLOC_ALIGN)
	size = ((size + DCACHE_LINE_SIZE - 1) & ~(DCACHE_LINE_SIZE-1)) + (DCACHE_LINE_SIZE << 1);

//    if(flags & DMA_MALLOC_ATOMIC)
        ptr = kmalloc(size, GFP_ATOMIC);
//    else
//        ptr = kmalloc(size, GFP_KERNEL);

	if(((unsigned int)ptr & (DCACHE_LINE_SIZE-1)) != 0)
	{
		WLA_DBG(WLADEBUG,"!!!!! PTR %x\n",(unsigned int)ptr);
		panic("ALLOC NOT ALIGN\n");
	}

    if(ptr)
    {
        if(flags & DMA_MALLOC_BZERO)
            memset(ptr, 0, size);

        if(flags & DMA_MALLOC_UNCACHED)
        {
            DCACHE_FLUSH((unsigned int)ptr, size);
			ptr = (void *)NONCACHED_ADDR(ptr);
        }

//		if(flags & DMA_MALLOC_ALIGN)
		{
			ptr1 = (unsigned char *)(((unsigned int)ptr & ~(DCACHE_LINE_SIZE-1)) + DCACHE_LINE_SIZE);
		
			*((int *)CACHED_ADDR(ptr1) - 1) = (int)ptr;

			return (void *)ptr1;
		}
    }
    else if(flags & DMA_MALLOC_ASSERTION)
    {
        panic("mac_malloc(): allocation failure, size %d\n", size);
    }
	
    return (void *)ptr;
}


void mac_free(void *ptr)
{
#if 1
    if(ptr)
    {
		if(((unsigned int)ptr & (DCACHE_LINE_SIZE-1) ) != 0)
		{
			WLA_DBG(WLADEBUG,"!!!!! PTR %x\n",(unsigned int)ptr);
			panic("FREE NOT ALIGN\n");
		}
        
		if(((unsigned int)ptr & (DCACHE_LINE_SIZE-1)) == 0)
		{
			ptr = (char *)*((int *)CACHED_ADDR(ptr) - 1);
		}

        if((unsigned int)ptr >= UNCACHE_BASE)
		{
            kfree((void *)CACHED_ADDR(ptr));
		}
        else
		{
            kfree(ptr);
		}
     }

#else		
    if(ptr)
    {
        if((unsigned long)ptr >= UNCAC_BASE)
            kfree(CAC_ADDR(ptr));
        else
            kfree(ptr);
    }
#endif	
}

void *wla_zalloc(size_t size)
{
	void *alloc = kmalloc(size,GFP_KERNEL);
	if (alloc)
	{
		memset(alloc, 0, size);
	}
	
	return alloc;
}

void wla_dump_buf_with_offset_32bit(unsigned int *p, u32 s)
{
	int i;
	while ((int)s > 0) {
	    for (i = 0;  i < 4;  i++) {
	        if (i < (int)s/4) {
	            printk("%08X ", p[i] );
	        } else {
	            printk("         ");
	        }
	    }
	    printk("\n");
	    s -= 16;
	    p += 4;
	}
}

void wla_dump_buf_with_offset_16bit(unsigned short *p, u32 s)
{
	int i;
	while ((int)s > 0) {
	    for (i = 0;  i < 8;  i++) {
	        if (i < (int)s/2) {
	      printk("%04X ", p[i] );
	      if (i == 3) printk(" ");
	        } else {
	      printk("     ");
	        }
	    }
	    printk("\n");
	    s -= 16;
	    p += 8;
	}
}

void wla_dump_buf_with_offset_8bit(unsigned char *p, u32 s)
{
	int i, c;
	while ((int)s > 0) {
	    for (i = 0;  i < 16;  i++) {
	        if (i < (int)s) {
	            printk("%02X ", p[i] & 0xFF);
	        } else {
	            printk("   ");
	        }
	    if (i == 7) printk(" ");
	    }
	    printk(" |");
	    for (i = 0;  i < 16;  i++) {
	        if (i < (int)s) {
	            c = p[i] & 0xFF;
	            if ((c < 0x20) || (c >= 0x7F)) c = '.';
	        } else {
	            c = ' ';
	        }
	        printk("%c", c);
	    }
	    printk("|\n");
	    s -= 16;
	    p += 16;
	}
}

void wla_dump_buf(void* buf, u32 len, u8 offset)
{
	if (offset == 32)
		wla_dump_buf_with_offset_32bit((unsigned int *)buf, len);
	else if (offset == 16)
		wla_dump_buf_with_offset_16bit((unsigned short *)buf, len);
	else if (offset == 8)
		wla_dump_buf_with_offset_8bit((unsigned char *)buf, len);

}

struct wbuf *wbuf_alloc(u32 size)
{
	struct sk_buff *skb;
	struct wbuf *wb = NULL;
	int _size;
	if(size == 0)
		_size = 1600;
	
	skb = dev_alloc_skb(_size);

	if(skb == NULL)
		return NULL;
	
	// ?? skb allocate has do this work ??
	DCACHE_INVALIDATE((unsigned int)skb->head,(unsigned int)skb->end-(unsigned int)skb->head);
	*((unsigned int *)skb->head) = (unsigned int)skb;

	wb = (struct wbuf *)(((char *)skb->head)+ SKB_WB_OFFSET);
	wb->flags = WBUF_FIXED_SIZE;
	wb->wb_next = NULL;

	return wb;
}

void wbuf_free(struct wbuf *wb)
{
	struct sk_buff *skb;

	if(wb->wb_next)
	{
		if((wb->flags & (WBUF_CHAIN| WBUF_EXT)) && ((unsigned int)wb->wb_next & 0x80000000))
		{		
			static int count=0;
			count++;
			WLA_DBG(WLADEBUG,"!!!! WB NEXT:%p repeat:%d\n",wb->wb_next,count);
				wbuf_free(wb->wb_next);
		}
	}
	skb = (struct sk_buff *)(*(unsigned int *)((char *)wb - SKB_WB_OFFSET));
	dev_kfree_skb(skb);
}

void wla_wbuf_flush_header(struct wbuf *wb_old, struct wbuf *wb_new, int copy)
{
    char *dst, *src;
    unsigned char off, new_off;
    unsigned int len, new_len, flush_len;

    off = wb_old->data_off;
    len = wb_old->pkt_len + off;
#if defined(WBUF_IP_HEADER_ALIGN)
    /* Maybe wbuf from ether driver data offset is align to 4 bytes in cheetah platform.
       In worst, IP header would not align to 4 bytes and occur exception.
       Shift wbuf data offset to do IP header alignment */
    if(((int)((char *)wb_old + off) & 3) == 0)
    {
        new_off = off + 2;
        new_len = len + 2;
    }
    else
#endif
    {
        new_off = off;
        new_len = len;
    }
    src = (char *)CACHED_ADDR(wb_old);
    dst = (char *)CACHED_ADDR(wb_new);
    if(copy)
    {
        memcpy(dst, src, off);
        memcpy((char *)dst + new_off, (char *)src + off, wb_old->pkt_len);
        flush_len = new_len;
    }
    else
    {
        /* Only copy wbuf length, then use WBUF_EXT and wb_next to indicate original wbuf to send */
        memcpy(dst, src, sizeof(struct wbuf));
        wb_new->flags |= WBUF_EXT;
        wb_new->wb_next = wb_old;
		
		//FIXME:
        //mclrefcnt(wb_old)++;
        flush_len = sizeof(struct wbuf);
    }
    /* flush wbuf header to ensure get right information by non-cached wb_new */
    DCACHE_FLUSH((unsigned int)dst, flush_len);

    wb_new->data_off = new_off;
}

struct wbuf *wbuf_copy(struct wbuf *wb_old, struct wbuf *wb_new)
{
	struct wbuf *wb_new_;

	wb_new_ = wbuf_alloc(0);
	if(!wb_new_)
		return NULL;
	wla_wbuf_flush_header(wb_old, wb_new_, 1);
	return wb_new_;

}

struct wbuf *wla_wbuf_action(struct wbuf *wb_old, struct wbuf *wb_new)
{
//	if(wb_old->flags & WBUF_TX_REQ_STATUS)
//		return wla_wbuf_clone(wb_old);
//	else
		return wbuf_copy(wb_old, wb_new);
}


struct wbuf *wla_wbuf_clone(struct wbuf *wb)
{
	panic("%s Not Implement !!!\n",__func__);
}



struct wbuf *wbuf_txbuf(struct wbuf *wb_old, struct wbuf *wb_new)
{
//	if(wb_new->flags & WBUF_EXT)
//		return wb_old;
//	else
	    return wb_new;

	//panic("%s Not Implement !!!\n",__func__);
}

int wbuf_datap(struct wbuf *wb_old, struct wbuf *wb_new)
{
//	if(wb_new->flags & WBUF_EXT)
//		return (int)wb_old + wb_new->data_off;
//	else
	    return (int)wb_new + wb_new->data_off;

	//panic("%s Not Implement !!!\n",__func__);
}

char *wmaddr_ntoa (char *mac)
{
  static char buf[18];
  sprintf (buf, "%02x:%02x:%02x:%02x:%02x:%02x",
       mac[0] & 0xff, mac[1] & 0xff, mac[2] & 0xff,
       mac[3] & 0xff, mac[4] & 0xff, mac[5] & 0xff);
  return buf;
}

int isspace(int c)
{
    if (c == ' ')
        return 1;
    else
        return 0;
}

char *wmaddr_aton(char *s)
{
    static char buf[6];
    int i;
    long l;
    char *pp;

    while (isspace(*s))
        s++;

    for (i = 0; i < 6; i++) 
    {
        l = simple_strtol(s, &pp, 16);
        if (pp == s || l > 0xFF || l < 0)
            return (NULL);
        if (!(*pp == ':' || (i == 5 && (isspace(*pp) || *pp == '\0'))))
            return (NULL);
        buf[i] = (u_char)l;
        s = pp + 1;
    }

    return (pp ? buf : NULL);
}       


void wm_ba_event(u8 cmd, u8 tid, u32 psta)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = (struct wla_softc *)info->sc;
	struct ieee80211_hw *hw;
	struct ieee80211_sta *sta = (struct ieee80211_sta *)psta ;
#ifdef CONFIG_WMAC_RECOVER
	struct wla_sta_priv *sp;
	u8 bss_desc;
	struct ieee80211_sub_if_data *sdata;
	struct ieee80211_vif *vif;
	struct wsta_cb *wcb;
	int i;
#endif
	
	if(sta == NULL)
		return;

	hw = (struct ieee80211_hw *)sc->hw;

	switch (cmd)
	{
		case BA_SESSION_INITIATION:
			if(!(__ampdu_tx_mask & (1 << tid)))
				ieee80211_start_tx_ba_session(sta, tid, 0);
			break;
		
		case BA_SESSION_TEARDOWN:
			ieee80211_stop_tx_ba_session(sta, tid);
			break;
#ifdef CONFIG_WMAC_RECOVER
		case BA_SESSION_TEARDOWN_ALL:
			sp = (struct wla_sta_priv *)sta->drv_priv;
			wcb = (struct wsta_cb *)sp->wcb;

			if(!wcb)
				return;
			bss_desc = wcb->bss_desc;
			sdata = IEEE80211_DEV_TO_SUB_IF(sc->bssdev[bss_desc]);
			vif = &sdata->vif;
			WLA_DBG(WLADEBUG,"%s: bss_desc(%d) mac(%s)\n", __func__, bss_desc, wmaddr_ntoa((char *)(vif->bss_conf.bssid)) );
			ieee80211_stop_rx_ba_session(vif, 0xffff, sta->addr);
			for(i=0; i<8; i++)
			{
				ieee80211_stop_tx_ba_session(sta, i);
			}
			break;
#endif
		default:
			panic("!!! Not handle this cmd:%d !!!\n",cmd);
	}

	return;
}

void wla_gets(char *buf)
{
	memcpy(buf,"c",1);
}

MAC_INFO *wla_mac_info()
{
	return info;
}

/*!-----------------------------------------------------------------------------
 * function: wla_forward_to_ethernet()
 *
 *      \brief	Forwarding to ethernet function. Input frames from wlan and forward.
 *		\param 
 *      \return	1: SUCCESS
 *				0: FAIL
 +----------------------------------------------------------------------------*/

int wla_forward_to_ethernet(struct wbuf *wb, ehdr *ef)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct sk_buff *skb;
	struct wla_softc *sc = (struct wla_softc *)info->sc;
	struct wsta_cb *wcb = info->wcb_tbls[wb->sa_idx];

	if(!info->mac_is_ready)
	{
		WBUF_FREE(wb);
		return 0;
	}
	sc = (struct wla_softc *)info->sc;
	skb = (struct sk_buff *)(*(unsigned int *)((char *)wb - SKB_WB_OFFSET));

	skb->data = (char *)ef;
	skb->len = (unsigned int)wb->pkt_len;

	skb->tail =(char *)skb->data + skb->len;

	/* Note: the wb->bss_desv is not equal to wcb->bss_desc WHY ??*/
	skb->dev = sc->bssdev[wcb->bss_desc];
	if(!skb->dev)
	{
		WBUF_FREE(wb);
		return 0;
	}
	skb->protocol = eth_type_trans(skb, skb->dev);
	netif_rx(skb);

	return 1;
}

struct wbuf *wla_skb_to_wb(struct sk_buff **orig_skb)
{
	struct wbuf *wb = NULL;
	struct sk_buff *skb, *nskb;

	if (unlikely(skb_headroom(*orig_skb) < HEADROOM_LEAST))
	{
		nskb = skb_realloc_headroom(*orig_skb, HEADROOM_LEAST);
		if(!nskb)
			return wb;
		dev_kfree_skb_any(*orig_skb); // prevent * orig skb been free if no new skb
		*orig_skb = nskb;
	}

	skb = *orig_skb;

	wb = (struct wbuf *)((unsigned int)skb->head + SKB_WB_OFFSET);
	*((unsigned int *)skb->head) = (unsigned int)skb;

	memset(wb,0,sizeof(struct wbuf));
	
	wb->data_off = (unsigned int)skb->data - (unsigned int)wb;
	wb->pkt_len = skb->len;

	return wb;
}
/*!-----------------------------------------------------------------------------
 * function: wla_ndo_start_xmit(skb, dev)
 *
 *      \brief	
 *		\param 
 *      \return	1: SUCCESS
 *				0: FAIL
 +----------------------------------------------------------------------------*/
//send data: wlan dev hook func:ndo_start_xmit
netdev_tx_t wla_data_send(struct sk_buff *skb, struct net_device *dev)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_vif *vif = &sdata->vif;
	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	
	MAC_INFO *info = WLA_MAC_INFO;
	struct wbuf *wb;
	int from_bss; 

#if 0
	from_bss = *(char *)skb->cb;
//new
	if(from_bss & 0x80)
			;
	else
	{
		from_bss = vp->bss_desc;
	}
#endif
	if(!info->mac_is_ready)
	{
		goto fail;
	}

	wb = wla_skb_to_wb(&skb);
	if(wb == NULL)
		goto fail;
	
	wb->tid = skb->priority & IEEE80211_QOS_CTL_TAG1D_MASK;
	wb->wh = 0;

	from_bss = ETH_TO_BSS(vp->unit);
	ethernet_parse_and_forward(info, wb, from_bss);

	return NETDEV_TX_OK;
fail:
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

#ifdef CONFIG_WMAC_RECOVER
void wla_recover(void)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = (struct wla_softc *)info->sc;
	struct ieee80211_hw *hw = (struct ieee80211_hw *)sc->hw;
	int i;

	if(!info->mac_is_ready)
		return;

	wla_lock();
#if 0
	rcu_read_lock();
#endif
	sc->started = 0;

	/* Prohibit ethernet to wifi */
	wla_mac_ethernet_forward_setup(0);
#if (FASTPATH_DESCR_COUNT > 0)
	MACREG_UPDATE32(ERR_EN, 0, FASTPATH_WIFI_TO_WIFI);
#endif
	MACREG_WRITE32(DA_MISS_TOCPU, 1);

	info->mac_is_ready = 0;

	cyg_drv_interrupt_mask(IRQ_WIFI);
	/* clear reorder buffer, psbaq, and ampdu */
	wla_pre_clean(info);

	/* Save LMAC setting */
	for(i=0; i<12; i++)
	{
		info->restore_lmac[i] = MACREG_READ32(LMAC_CNTL+(i<<2));
	}

	wla_mac_stop(info);

	/* Resotre beacon buffer */
	save_beacon_list(info, (struct beacon_desc *)info->beacon_descriptors_list);
	wla_beacon_flush(info);
	wla_mac_reset(info);

	WLA_DBG(WLADEBUG,"%s:%s cnt(%d)\n", wiphy_name(hw->wiphy), __func__, ++info->mac_recover_cnt);

	/* MAC recover */
	wla_mac_recover(info);

	wla_mac_start(sc->mac_info);
	if(info->curr_channel)
	{
		wla_set_channel(info);
		wla_set_bandwidth_type(info);
	}

	/* Without cache sync for address table, target can not receive frames once __no_ampdu is set */
	{
		char addr[6];
		int bcap = 0;

		/* Cache sync for DS address table */
		for(i=0; i < MAX_DS_TABLE_ENTRIES; i++)
			mac_addr_lookup_engine_find(addr, i, &bcap, IN_DS_TBL|BY_ADDR_IDX);
		/* Cache sync for staion address table */
		for(i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
			mac_addr_lookup_engine_find(addr, i, &bcap, BY_ADDR_IDX);
		mac_rx_linkinfo_cache_sync();
	}

	sc->started = 1;
	info->mac_is_ready = 1;

	MACREG_WRITE32(DA_MISS_TOCPU, 0);
#if (FASTPATH_DESCR_COUNT > 0)
	MACREG_UPDATE32(ERR_EN, FASTPATH_WIFI_TO_WIFI, FASTPATH_WIFI_TO_WIFI);
#endif
	/* Permit ethernet to wifi */
	wla_mac_ethernet_forward_setup(1);
	wla_unlock();

	/* In linux, RX delba will block by mac is not ready. */ 
	/* Prohibit AMPDU */
	wla_notify_no_ampdu();
#if 0
	rcu_read_unlock();
#endif

	udelay(1000000);
	cyg_drv_interrupt_unmask(IRQ_WIFI);
	wla_cfg(SET_BEACON_PARAMS, 100, 3);
	/* Restore LMAC setting */
	for(i=0; i<12; i++)
	{
		MACREG_WRITE32(LMAC_CNTL+(i<<2), info->restore_lmac[i]);
	}
	for(i=0; i<info->bss_num; i++)
	{
		struct mbss_t *wif;
	
		wif = &info->mbss[i];
	
		if(wif->role != WIF_AP_ROLE)
			continue;
		/* separate the interrupt & beacon setting */
		wla_set_beacon_interrupt(i, wif->role, 1);
	}
	update_beacon_list(info);
}
#endif

void wla_hw_sync_work(struct work_struct *work)
{
#if defined(CONFIG_CHEETAH_SMART_CONFIG)
	if(__smartcfg_enable)
	{
#if defined(CONFIG_WMAC_RECOVER)
	    if(info->mac_recover)
	    {
                wla_lock();

                bb_register_write(0x12, 0x05);
                bb_register_write(0x13, 0x03);

                MACREG_UPDATE32(RTSCTS_SET, 0, LMAC_FILTER_ALL_PASS);
                MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_SNIFFER_MODE);
                MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);

                wla_recover();

                MACREG_UPDATE32(RTSCTS_SET, LMAC_FILTER_ALL_PASS, LMAC_FILTER_ALL_PASS);
                MACREG_UPDATE32(RX_ACK_POLICY, RX_SNIFFER_MODE, RX_SNIFFER_MODE);
                MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);
            
                bb_register_write(0x12, 0x05);
                bb_register_write(0x13, 0x07);

                info->mac_recover = 0;

                wla_unlock();

                //printk("MAC recovery done\n");
            }
#endif

	    wla_add_timer(100);
	    return;
	}
#endif
	wla_hw_sync();
}

void wla_txfail_timer(unsigned long data)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;

	if((info->wl_txq.pkt_count - info->wl_sw_retq.pkt_count) > info->wl_sw_retq.max) {
		sc->tx_fail_times++;
		info->mac_recover=1;
	}
	else {
		timer_g.expires = jiffies + 1000;
		mod_timer(&timer_g, timer_g.expires);
	}
}

void wla_txfail_chk(void)
{
	timer_g.expires = jiffies + 1000;
	mod_timer(&timer_g, timer_g.expires);
}

void rfc_handler(unsigned int reg, unsigned int val)
{
	wla_rf_reg(reg, val);
	rf_write(reg, val);
}

void txvga_handler(unsigned int ch, unsigned int val)
{
	wla_rf_txvga_reg(ch-1, val);
}

void freq_handler(unsigned int ch, unsigned int val)
{
	wla_rf_freq_ofs_reg(val);
}

static struct hw_setup hw_set[] = {
	HW_MAP("rfc", rfc_handler, "%x", "%x"),
	HW_MAP("txvga", txvga_handler, "%d", "%d"),
	HW_MAP("freq_ofs", freq_handler, NULL, "%x"),
};

void parse_parm(char *p, unsigned int idx)
{
	char *token;
	unsigned int reg, val;
	char buf[512] = {0};
	char *s;
	char *pend=NULL;
	char *vp;

	s = buf;
	p = strchr(p, '=');
	pend = strchr(p, 0xa);
	p += 1;
	memcpy(buf, p, pend-p);
	buf[pend-p+1]='\0';

	if(strlen(buf)==0)
		return;

	if(!strchr(s, ',')) {
		if((vp=strchr(s,'=')) == NULL) {
			sscanf(s, hw_set[idx].val, &val);
			hw_set[idx].handler(0, val);
		}
		else {
			s[vp-s]='\0';
			vp++;
			sscanf(s, hw_set[idx].reg, &reg);
			sscanf(vp, hw_set[idx].val, &val);
			hw_set[idx].handler(reg, val);
		}
	}
	else {
		while ((token = strsep(&s, ",")) != NULL)
		{
			
			if((vp = strchr(token, '=')) != 0)
			{
				vp++;
				sscanf(token, hw_set[idx].reg, &reg);
				sscanf(vp, hw_set[idx].val, &val);
				hw_set[idx].handler(reg, val);
			}
		}
	}
}

void wla_hw_setup(void)
{
	struct file *fp =NULL;
	char *p=NULL;
	unsigned int i;

	char read_buf[2048] = "";

	initkernelenv();
	fp = openfile("/proc/bootvars", O_CREAT | O_RDONLY, 0);
	
	if(fp == NULL)
		goto done;
	
	readfile(fp, read_buf, sizeof(read_buf));

	for(i=0; i < ARRAY_SIZE(hw_set); i++) {
		p =	strstr(read_buf, hw_set[i].name);
		if(p != NULL) {
			parse_parm(p, i);
		}
	}
done:	
	closefile(fp);
	dinitkernelenv();
}

void wla_get_hw_mac(unsigned short idx, unsigned char *mac)
{
	struct file *fp = NULL;
	char *p = NULL, *pend = NULL;

	char read_buf[2048] = "";
	char mac_name[3][8] = {"mac0", "mac1", "mac2"};

	initkernelenv();
	fp = openfile("/proc/bootvars", O_CREAT | O_RDONLY, 0);
	
	if(fp == NULL)
		goto done;
	
	readfile(fp, read_buf, sizeof(read_buf));

	if((p = strstr(read_buf, mac_name[idx])) != NULL)
	{
		p = strchr(p, '=');
		pend = strchr(p, 0xa);
		p += 1;
		memcpy(mac, p, pend-p);
		mac[pend-p+1]='\0';
	}
done:	
	closefile(fp);
	dinitkernelenv();
}

void hw_cdb_get(const char *name, unsigned char *cdb)
{
	struct file *fp = NULL;
	char *p = NULL, *pend = NULL;

	char read_buf[2048] = "";
	//char mac_name[3][8] = {"mac0", "mac1", "mac2"};

	initkernelenv();
	fp = openfile("/proc/bootvars", O_CREAT | O_RDONLY, 0);
	
	if(fp == NULL)
		goto done;
	
	readfile(fp, read_buf, sizeof(read_buf));

	if((p = strstr(read_buf, name)) != NULL)
	{
		p = strchr(p, '=');
		pend = strchr(p, 0xa);
		p += 1;
		memcpy(cdb, p, pend-p);
		cdb[pend-p+1]='\0';
	}
done:	
	closefile(fp);
	dinitkernelenv();
}

void hw_cdb_set(const char *name, const char *cdb)
{
	struct file *fp = NULL;
    char str[300] = {0};

	initkernelenv();
	fp = openfile("/proc/bootvars", O_CREAT | O_RDONLY, 0);
	
	if(fp == NULL)
		goto done;

    sprintf(str, "%s %s\n", name, cdb);
	
	writefile(fp, str, strlen(str));
done:	
	closefile(fp);
	dinitkernelenv();
}
