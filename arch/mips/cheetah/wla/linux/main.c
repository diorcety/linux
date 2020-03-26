/*
 * Copyright (c) 2012   Montage Inc.,  All rights reserved.
 * 
 * WLA driver
 */
#include <linux/autoconf.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>
#include <net/ieee80211_radiotap.h>
#include <linux/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <asm/bug.h>
#include <linux/ethtool.h>

#include <os_compat.h>
#include <wla_debug.h>
#include <rf.h>
#include <wla_bb.h>
#include <linux_wla.h>

#include <linux/wireless.h>
#include <net/iw_handler.h>
#include <asm/mach-cheetah/common.h>
//#include <asm/mach-cheetah/idb.h>

#define SMRT_TIME	90

MAC_INFO *info;

static const struct ethtool_ops wla_ethtool_ops;

struct timer_list timer_g;

//iwpriv handler

int wla_ioctl_reset(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	return 0;
}

int wla_wd_info(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
//	char *ptr = (char *)wrqu->data.pointer;

	return 0;
}

static const struct iw_priv_args wla_privtab[] = {
    { SIOCIWFIRSTPRIV + 0x0, 0, 0, "force_reset" },
    { SIOCIWFIRSTPRIV + 0x1, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, 0, "wd" },
};

static const iw_handler wla_private_handler[] = {
    [0] = (iw_handler) wla_ioctl_reset,
    [1] = (iw_handler) wla_wd_info,
};

const struct iw_handler_def wla_handler_def = {
    .num_private		= ARRAY_SIZE(wla_private_handler),
    .num_private_args	= ARRAY_SIZE(wla_privtab),
    .private			= wla_private_handler,
    .private_args		= wla_privtab,
};

//END

extern struct wlan_rate wla_rates[];

//
void wla_setup_rates(struct wla_softc *sc, enum ieee80211_band band)
{
#if 0
	struct wlan_rate *rate_table = NULL;
    struct ieee80211_supported_band *sband;
    struct ieee80211_rate *rate;
    int i, maxrates;
    
	sband = &sc->bands[band];
    rate = sc->rates[band];

    switch (band)
    {
        case IEEE80211_BAND_2GHZ:
			rate_table =(struct wlan_rate *)&wla_rates[0];
			maxrates = 32;//sizeof(rate_table)/sizeof(struct wlan_rate);
            break;
        case IEEE80211_BAND_5GHZ:
            rate_table = NULL;
            break;
        default:
            break;
    }
    
	if (rate_table == NULL)
		return;

    sband->n_bitrates = 0;

    for (i = 0; i < maxrates; i++)
    {
		if(rate_table[i].flags == HT_RATE)
			break;	
		rate[i].bitrate = rate_table[i].bitrate;
        sband->n_bitrates++;
    }
	
	return;
	
#else
    const struct wla_rate_table *rate_table = NULL;
    struct ieee80211_supported_band *sband;
    struct ieee80211_rate *rate;
    int i, maxrates;

    rate_config ratecfg;

    switch (band)
    {
        case IEEE80211_BAND_2GHZ:
            rate_table = &wla_11g_ratetable; //FIXME
            break;
        case IEEE80211_BAND_5GHZ:
            rate_table = &wla_11a_ratetable; //FIXME
            break;
        default:
            break;
    }

    if (rate_table == NULL)
        return;

    sband = &sc->bands[band];
    rate = sc->rates[band];

    maxrates = rate_table->rate_cnt;

    sband->n_bitrates = 0;

    for (i = 0; i < maxrates; i++)
    {
        rate[i].bitrate = rate_table->info[i].ratekbps / 100;
#if 1
        ratecfg.ch_offset = rate_table->info[i].ch_offset;
        ratecfg.format = rate_table->info[i].format;
        ratecfg.sgi = rate_table->info[i].sgi;
        ratecfg.ht = rate_table->info[i].ht;
        ratecfg.txcnt = 2;
        ratecfg.tx_rate = rate_table->info[i].tx_rate;

        /* hw_value => rate code / full rate setting,  hw_value_short => arthur rate_table index */
        rate[i].hw_value = ratecfg.val;
        rate[i].hw_value_short = i;

        if (rate_table->info[i].short_preamble)
        {
            rate[i].flags = IEEE80211_RATE_SHORT_PREAMBLE;
        }
#endif
        sband->n_bitrates++;

    }
#endif	
}
static int wla_add_interface(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif)
{
	struct wla_softc *sc = hw->priv;
	struct wla_vif_priv *vp = (void *) vif->drv_priv;
	struct ieee80211_sub_if_data *sdata = NULL;
	struct net_device *dev;
#ifdef CONFIG_CHEETAH_SMART_CONFIG
	struct smartcfg *cfg = &sc->smrtcfg;
#endif

	MAC_INFO *info = WLA_MAC_INFO;
	char type=0;
	short bss_desc;
	char val=0;
	int ret = -EIO;
	char vif_id;

	(void)val;
	(void)ret;
	wla_lock();
	
	WLA_DBG(WLADEBUG, "%s:%s (type=%d mac_addr=%pM)\n",
	       wiphy_name(hw->wiphy), __func__, vif->type,
	       vif->addr);

	sdata = vif_to_sdata(vif);
	dev = sdata->dev;

	dev->ethtool_ops = &wla_ethtool_ops;

	/*Change data path*/
	dev->netdev_ops->ndo_start_xmit = wla_data_send;
	dev->priv_flags |= IFF_WLA_DEV;

	if(info->bss_num >= MAC_MAX_BSSIDS)
		goto done;

	memset(vp, 0, sizeof(struct wla_vif_priv));

	if(vif->type == NL80211_IFTYPE_AP)
	{
		type = AP_MACADDR;
		vp->role = WIF_AP_ROLE;
		vif_id = __apid;
	}
	else if(vif->type == NL80211_IFTYPE_STATION)
	{
		type = STA_MACADDR;
		vp->role = WIF_STA_ROLE;
		vif_id = __staid;
	}
	else
		goto done;

	//get bss_desc and set up hw mac bssid
	if((bss_desc = register_macaddr(type, vif->addr)) == RETURN_ERR)
		goto done;

	vp->unit = vif_id;
	vp->name = dev->name;

	vp->bss_desc = bss_desc; // important parm;

	sc->bssdev[bss_desc] = dev;

	dev->priv_flags &= ~IFF_BSS_MASK;
	dev->priv_flags |= (bss_desc << IFF_BSS_SHIFT);
	dev->priv_flags &= ~IFF_BSS_MASK;
	dev->priv_flags |= (bss_desc << IFF_BSS_SHIFT);
	//set up mac_info mbss
	wla_add_wif(bss_desc, (vp->role & 0xff), vif_id);

	/*FIXME: 
		When we add an interface, we first use the lowest bitrae at its basic rate
	*/

	if(__fix_txrate)
	{
		if (__fix_txrate == 21)
			wla_cfg(SET_TX_RATE,FIXED_TX_RATE,bss_desc,__rate_flags,MCS_32);
		else
			wla_cfg(SET_TX_RATE,FIXED_TX_RATE,bss_desc,__rate_flags,__fix_txrate);
	}

	wla_cfg(SET_TX_RATE,BASIC_TX_RATE,bss_desc,RATE_FLAGS_SHORT_PREAMBLE,LOWEST_BITRATE);
	wla_cfg(SET_TX_RATE,MULTICAST_TX_RATE,bss_desc,RATE_FLAGS_SHORT_PREAMBLE,LOWEST_BITRATE);
	
#ifdef CONFIG_CHEETAH_SMART_CONFIG
		cfg->bss_desc = bss_desc;
#endif	
	
	ret = 0;
done:
	wla_unlock();
	return ret;
}

static void wla_remove_interface(
	struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct wla_softc *sc = hw->priv;
	struct wla_vif_priv *vp = (void *) vif->drv_priv;
	char bss_desc = vp->bss_desc;

	struct ieee80211_sub_if_data *sdata = NULL;
	struct net_device *dev;
	MAC_INFO *info = WLA_MAC_INFO;

	char type=0;

	WLA_DBG(WLADEBUG, "%s:%s (type=%d mac_addr=%pM)\n",
	       wiphy_name(hw->wiphy), __func__, vif->type,
	       vif->addr);

	if(!sc->started)
	{
        WLA_DBG(WLADEBUG, "remove interface when sc is not started\n");
		goto done;
	}

	sdata = vif_to_sdata(vif);
	dev = sdata->dev;

	dev->ethtool_ops = NULL;
	
	if(vif->type == NL80211_IFTYPE_AP)
	{
		type = AP_MACADDR;
	}
	else if(vif->type == NL80211_IFTYPE_STATION)
	{
		type = STA_MACADDR;
	}

	clean_macaddr(vif->addr,type);
	
	sc->bssdev[(unsigned char)bss_desc] = NULL;

	info->bss_num--;

	wla_cfg(SET_TX_RATE,BASIC_TX_RATE,bss_desc,0,-1);
	wla_cfg(SET_TX_RATE,MULTICAST_TX_RATE,bss_desc,0,-1);
	
/*
To do
1. clear hw mac table bsase on vp->bss_desc
*/
done:
	return;
}

struct beacon_prepare_args
{
	struct ieee80211_hw *hw;
	struct sk_buff *beacon_skb[MAC_MAX_BSSIDS];
};

//FIXME:Need to modify
#if 1
static int wla_config(struct ieee80211_hw *hw, u32 changed)
{
	struct wla_softc *sc = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;
	int err = 0;
	int chan, mode;

	WLA_DBG(WLADEBUG, "%s:%s\n", wiphy_name(hw->wiphy), __func__);

	if (changed & IEEE80211_CONF_CHANGE_LISTEN_INTERVAL)
	{
		/* TODO */
	}

	if (changed & IEEE80211_CONF_CHANGE_PS) 
	{
		/* TODO */
	}

	if (changed & IEEE80211_CONF_CHANGE_POWER)
	{
		MAC_INFO *info = WLA_MAC_INFO;
		//if(conf->power_level > MAX_TX_POWER)
		//	conf->power_level = MAX_TX_POWER;
		//wla_tx_power(conf->power_level);
		wla_set_channel(info);
	}

	if (changed & IEEE80211_CONF_CHANGE_RETRY_LIMITS) 
	{
		/* TODO */
	}

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) 
	{
		enum nl80211_channel_type type;
		struct ieee80211_channel *curchan = hw->conf.chandef.chan;
		static char acs=0;
		struct ieee80211_local *local;
			
		local = container_of(hw, struct ieee80211_local, hw);
		
		type = cfg80211_get_chandef_type(&conf->chandef);

		WLA_DBG(WLADEBUG, "%s (freq=%d MHz) (channel_type=%d)\n", __func__, curchan->center_freq, type);

		/* update survey stats for the old channel before switching */
		if(acs)
		{
			chan = wla_transfer_channel_num(sc->curr_center_freq);
			
			//spin_lock_irqsave(&sc->lock, flags);
			wla_update_survey_stats(sc,!(hw->conf.flags & IEEE80211_CONF_OFFCHANNEL));
			//spin_unlock_irqrestore(&sc->lock, flags);
		}
		memcpy(&sc->channel, conf->chandef.chan, sizeof(struct ieee80211_channel));
	
		chan = wla_transfer_channel_num(conf->chandef.chan->center_freq);
		
		if(type == NL80211_CHAN_HT40PLUS)
			mode = BW40MHZ_SCA;
		else if(type == NL80211_CHAN_HT40MINUS)
			mode = BW40MHZ_SCB;
		else	
			mode = BW40MHZ_SCN; //FIXME: BW40MHZ_SCN or BW40MHZ_AUTO ??
		

		if(local->scanning)
			acs = 1;
		else
			acs = 0;
		
		wla_cfg(SET_CHANNEL,chan,mode);

		if (acs)
		{
			/* reset bb rx counter */
			bb_rx_counter_ctrl(BB_RX_CNT_RESET);
			/* enable bb rx counter */
			bb_rx_counter_ctrl(BB_RX_CNT_ENABLE);
			/* reset noise floor to 0xff */
			bb_read_noise_floor();
		}
		
		sc->curr_center_freq = curchan->center_freq;
	}

	return err;
}
#endif

static void wla_configure_filter(struct ieee80211_hw *hw,
					    unsigned int changed_flags,
					    unsigned int *total_flags,
					    u64 multicast)
{
	struct wla_softc *sc = hw->priv;
	//u32 set_filter=RX_FILTER_CTRL_FRAME|RX_FILTER_PROBE_REQ|RX_FILTER_BEACON|RX_FILTER_PROBE_RESP;
	u32 set_filter=RX_FILTER_PROBE_REQ|RX_FILTER_BEACON|RX_FILTER_PROBE_RESP;
	u32 del_filter=0;
	
	wla_lock();
	
	WLA_DBG(WLADEBUG, "%s:%s\n", wiphy_name(hw->wiphy), __func__);

	sc->rx_filter = 0;
#if 0
	if (*total_flags & FIF_CONTROL)
	{
		sc->rx_filter |= FIF_CONTROL;
		del_filter |= RX_FILTER_CTRL_FRAME;
	}
#endif
	if (*total_flags & FIF_PROBE_REQ)
	{
		sc->rx_filter |= FIF_PROBE_REQ;
		del_filter |= RX_FILTER_PROBE_REQ;
	}
	if (*total_flags & FIF_BCN_PRBRESP_PROMISC)
	{
		sc->rx_filter |= FIF_BCN_PRBRESP_PROMISC;
		del_filter |= (RX_FILTER_BEACON | RX_FILTER_PROBE_RESP);
	}
#if 0
	u32 err_filter=0;
	u32 err_mask=ERR_EN_FCS_ERROR_TOCPU;
	if (*total_flags & FIF_FCSFAIL)
	{
		sc->rx_filter |= FIF_FCSFAIL;
		err_filter |= ERR_EN_FCS_ERROR_TOCPU;
	}
	MACREG_UPDATE32(ERR_EN, err_filter, err_mask);
#endif
	*total_flags = sc->rx_filter;
	set_filter -= del_filter;

	wla_cfg(SET_RX_FILTER, set_filter);
	wla_cfg(DEL_RX_FILTER, del_filter);

	wla_unlock();
}

static int wla_set_key(struct ieee80211_hw *hw,
			 enum set_key_cmd cmd,
			 struct ieee80211_vif *vif,
			 struct ieee80211_sta *sta,
			 struct ieee80211_key_conf *key)
{
	struct wla_softc *sc = hw->priv;
	int ret=0;

	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	struct wla_sta_priv *sp = (struct wla_sta_priv *)sta->drv_priv;
	struct wsta_cb *wcb=NULL; 
	unsigned char key_type;
	unsigned char is_txkey=0;

//	wla_lock();
	
	WLA_DBG(WLADEBUG, "Set HW Key cmd %d, len %d key:%x\n", cmd, key->keylen,key->key[0]);
//	idb_print("%s HW Key, len %d key:%x\n", cmd==0?"ADD":"Remove", key->keylen,key->key[0]);

	vp->cipher_type = wla_cipher_chg(key);

	key_type = wla_key_chg(vif,key);
	
	if((vp->cipher_type != CIPHER_TYPE_WEP40) && (vp->cipher_type != CIPHER_TYPE_WEP104))
		is_txkey = 1;

	switch (cmd)
	{
		case SET_KEY:
			if(key_type == KEY_TYPE_GLOBAL_KEY)
				wla_cipher_key((struct wsta_cb *)(int)vp->bss_desc, vp->cipher_type, key_type, 
					(char *)&key->key[0], key->keyidx, is_txkey);
			else
			{
				wcb = (struct wsta_cb *)sp->wcb;
				
				if(vp->cipher_type != CIPHER_TYPE_SMS4)
					is_txkey = 0;

				wla_cipher_key(wcb, vp->cipher_type, key_type, 
					(char *)&key->key[0], key->keyidx, is_txkey);
				key->hw_key_idx = wcb->captbl_idx; 
			}
			break;

		case DISABLE_KEY:
			if(key_type == KEY_TYPE_GLOBAL_KEY)
				wla_cipher_key((struct wsta_cb *)(int)vp->bss_desc, AUTH_CIPHER_NONE, key_type, 
					0, 0, 0);
			else
			{		
				mac_rekey_start(key->hw_key_idx,key_type,vp->cipher_type,0);
				memset(&sc->mac_info->private_keys[key->hw_key_idx], 0, sizeof(cipher_key));
				mac_rekey_done();
			}
			break;

		default:
			ret = -EINVAL;
			break;
	}

//	wla_unlock();
	return ret;
}

static void wla_default_unicast_key(struct ieee80211_hw *hw, struct ieee80211_vif *vif, int key_idx)
{
	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	cipher_key *ghwkey;
	int gidx;
	MAC_INFO *info = WLA_MAC_INFO;

	WLA_DBG(WLADEBUG, "cipher type:%d THE default key idx:%d\n",vp->cipher_type,key_idx);
	
	if((vp->cipher_type == AUTH_CIPHER_WEP40) || (vp->cipher_type == AUTH_CIPHER_WEP104))
	{		
		if (key_idx >= 0)
		{		
			gidx = vp->bss_desc;
			ghwkey = &info->group_keys[gidx].group_key;

			if(vif->type == NL80211_IFTYPE_AP)
				ghwkey->wep_key.txkeyid = key_idx;
			
			if(vif->type == NL80211_IFTYPE_STATION)
			{
				struct peer_ap *ap;
				struct wsta_cb *wcb;
				int index;

				ap = info->mbss[(unsigned char)vp->bss_desc].ln_ap;
				index = ap->sta_captbl;
				wcb = info->wcb_tbls[index];
				
				wla_cipher_key(wcb, vp->cipher_type, KEY_TYPE_STA_PAIRWISE_KEY,
						(char *)&(ghwkey->wep_key.key[key_idx]),key_idx,1);
			}
		}
	}
}

static int wla_ampdu_action(struct ieee80211_hw *hw, struct ieee80211_vif *vif, enum ieee80211_ampdu_mlme_action action, struct ieee80211_sta *sta, u16 tid, u16 *ssn, u8 buf_size)
{
    struct wla_sta_priv *sp = (struct wla_sta_priv *)sta->drv_priv;
	struct wsta_cb *wcb = sp->wcb;
	unsigned char n_order;

    int ret = -EOPNOTSUPP;

	if(wcb == 0)
	{
		WLA_DBG(WLADEBUG,"AMPDU: wcb is NULL, should be release !!!\n");
		goto done;
	}

	if(tid>=MAX_QOS_TIDS)
		goto done;

/*FIXME: Need to review code*/
	switch (action)
	{
		case IEEE80211_AMPDU_RX_START:
			if(__no_ampdu)
				break;
/*check status*/
//			printk("%s %d\n",__func__,__LINE__);
			if(wla_ampdu(GET_AMPDU_RX_STATUS, wcb, tid, 0, 0) != 0)
		    {
				WLA_DBG(WLADEBUG,"AMPDU_RX: Start exist, remove first\n");
				wla_ampdu(STOP_AMPDU_RX, wcb, tid, 0, 0);
			}

			WLA_DBG(WLADEBUG,"AMPDU_RX_START the original bufsize:%d ssn:%d\n",buf_size,*ssn);
			for(n_order = 6; n_order > 0; n_order--)
			{
				if((1 << n_order) == buf_size)
					break;
			}
			ret = wla_ampdu(START_AMPDU_RX, wcb, tid, *ssn, n_order);
			break;
		
		case IEEE80211_AMPDU_RX_STOP:
//			printk("%s %d\n",__func__,__LINE__);
			if(wla_ampdu(GET_AMPDU_RX_STATUS, wcb, tid, 0, 0) != 0)
			{		
				WLA_DBG(WLADEBUG,"AMPDU_RX: Stop\n");
				ret = wla_ampdu(STOP_AMPDU_RX, wcb, tid, 0, 0);
			}
			break;

		case IEEE80211_AMPDU_TX_START:
			wcb->ba_tx[tid].state = BA_ESTABLISHING_STATE;
			wcb->ba_tx[tid].req_num = 0; // Not sure !!!
			wcb->ba_tx[tid].start_seq = *ssn = wla_get_ssn(wcb,tid);

			if(wla_ampdu(GET_AMPDU_TX_STATUS, wcb, tid, 0, 0) != 0)
			{		
				WLA_DBG(WLADEBUG,"AMPDU_TX: exist just return\n");
				break;
			}
			WLA_DBG(WLADEBUG,"AMPDU_TX_START ssn:%d\n",*ssn);

			ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
			ret = 0;
			break;

		case IEEE80211_AMPDU_TX_STOP_CONT:
		case IEEE80211_AMPDU_TX_STOP_FLUSH:
		case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
			WLA_DBG(WLADEBUG," AMPDU_TX_STOP\n");
			/*eCos have a transient state change from BA_TEARDOWNING_STATE to */
			wcb->ba_tx[tid].state = BA_TEARDOWNING_STATE;
			/*check status*/
			if(wla_ampdu(GET_AMPDU_TX_STATUS, wcb, tid, 0, 0) != 0)		
				wla_ampdu(STOP_AMPDU_TX, wcb, tid, 0, 0);
			if(action == IEEE80211_AMPDU_TX_STOP_CONT)
				ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
			wcb->ba_tx[tid].state = BA_IDLE_STATE;
			ret = 0;
			break;

		case IEEE80211_AMPDU_TX_OPERATIONAL:
			/*eCos has do this check*/
			WLA_DBG(WLADEBUG,"A-MPDU TX operational, TID %d, SSN %d, BUFSIZE %d\n", tid, wcb->ba_tx[tid].start_seq, buf_size);
			/* Do we need some protect value of ssn or bufsize*/
			ret = wla_ampdu(START_AMPDU_TX, wcb, tid, wcb->ba_tx[tid].start_seq, buf_size);
			wcb->ba_tx[tid].state = BA_ESTABLISHED_STATE;
			break;
	}

done:
	return ret;
}

static int wla_get_survey(struct ieee80211_hw *hw, int idx, struct survey_info *survey)
{
	struct wla_softc *sc = hw->priv;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *chan;

	sband = hw->wiphy->bands[IEEE80211_BAND_2GHZ];

	if (!sband || idx >= sband->n_channels) {
		return -ENOENT;
	}
	
	chan = &sband->channels[idx];

	memcpy(survey, &sc->survey[idx], sizeof(*survey));
	survey->channel = chan;

	return 0;
}

static void wla_bss_info_changed(struct ieee80211_hw *hw,
					    struct ieee80211_vif *vif,
					    struct ieee80211_bss_conf *binfo,
					    u32 changed)
{
	struct wla_vif_priv *vp = (void *)vif->drv_priv;
	struct wla_softc *sc = hw->priv;
	struct ieee80211_sub_if_data *sdata = NULL;
	struct ieee80211_if_ap *ap = NULL;
	MAC_INFO *info = WLA_MAC_INFO;
	int i;

	(void)sdata;
	(void)ap;
	wla_lock();
	
	WLA_DBG(WLADEBUG," %s:%s vif type:%d (changed=0x%x)\n",
	       wiphy_name(hw->wiphy), __func__, vif->type, changed);

	/* FIXME: Ignore command while MAC is recovering */
	if(info->mac_recover)
		goto out;

	if (changed & BSS_CHANGED_BSSID) 
	{
		WLA_DBG(WLADEBUG,"%s:%s: BSSID changed: %pM\n",
			   wiphy_name(hw->wiphy), __func__,
			   binfo->bssid);
#if 0		
		// if vif support only one type. bssid in bssinfo might be only one. 
		if(vif->type == NL80211_IFTYPE_STATION)
		{
			memcpy(vp->bssinfo->associated_bssid, (u8 *) info->bssid, ETH_ALEN);
		}
		else
		{
			memcpy(vp->bssinfo->bssid, (u8 *) info->bssid, ETH_ALEN);
		}
		modify_macaddr(vp->bss_desc, info->bssid);
#endif		
	}
	
	if(changed & BSS_CHANGED_BEACON)
	{
		WLA_DBG(WLADEBUG,"VIF type:%d: Change Beacon Data\n",vif->type);
		if(vif->type == NL80211_IFTYPE_AP)
			wla_beacon_get(vif,binfo->dtim_period);
	}

	if(changed & BSS_CHANGED_BEACON_INT)
	{
		WLA_DBG(WLADEBUG,"VIF type:%d : BCNINT: %d DTIM:%d\n",
			   vif->type, binfo->beacon_int,binfo->dtim_period);
		
		if(vif->type == NL80211_IFTYPE_AP)
		{
			info->beacon_interval = binfo->beacon_int;
			wla_cfg(SET_BEACON_PARAMS,binfo->beacon_int,binfo->dtim_period);
		}
		/* Put here due to only call once */
		if(vif->type == NL80211_IFTYPE_STATION)
		{
			wla_start_tsync(vp->bss_desc,binfo->beacon_int,binfo->dtim_period,0);
			wla_set_beacon_interrupt(vp->bss_desc, vp->role, 1);
			vp->bcn_threshold = 100*8/binfo->beacon_int; /*if lost 80 percent beacon we did not reset beacon monitor timer*/
		}
	}

	if(changed & BSS_CHANGED_BEACON_ENABLED)
	{
		WLA_DBG(WLADEBUG,"[%s] Beacon\n",binfo->enable_beacon?"Enable":"Disable");
		if(vif->type == NL80211_IFTYPE_AP)
		{
			if(binfo->enable_beacon)
			{
				if(!sc->ap_exist)
				{		
					wla_start_tsync(vp->bss_desc,vif->bss_conf.beacon_int,vif->bss_conf.dtim_period,0);
					wla_set_beacon_interrupt(vp->bss_desc,vp->role,1);
				}
				info->int_mask &= ~MAC_INT_PRETBTT;
				sc->ap_exist = 1;
			}	
			else
			{		
				info->int_mask |= MAC_INT_PRETBTT;
			}
		}
	}
	
	if (changed & BSS_CHANGED_ASSOC) {
		WLA_DBG(WLADEBUG,"  %s: ASSOC: assoc=%d aid=%d\n",
		       wiphy_name(hw->wiphy), binfo->assoc, binfo->aid);
		vp->aid = binfo->aid;
	}

//done
	if (changed & BSS_CHANGED_ERP_CTS_PROT) {
		WLA_DBG(WLADEBUG,"  %s: ERP_CTS_PROT: %d\n",
		       wiphy_name(hw->wiphy), binfo->use_cts_prot);
		if(binfo->use_cts_prot >= 0)
		{
			u32 protection_word = 0;
			if (binfo->use_cts_prot)
				protection_word = (PROTECT_MODE_HT_NON_MEMBER<<PROTECT_SHIFT)|BG_PROTECT|ERP_PROTECT;
			MACREG_UPDATE32(RTSCTS_SET, protection_word, PROTECT_MODE|BG_PROTECT|ERP_PROTECT);
		}
	}

	if (changed & BSS_CHANGED_ERP_PREAMBLE) {
		WLA_DBG(WLADEBUG,"  %s: ERP_PREAMBLE: %d\n",
		       wiphy_name(hw->wiphy), binfo->use_short_preamble);
	}

//done
	if (changed & BSS_CHANGED_ERP_SLOT) {
		static u32 slot_time;
		WLA_DBG(WLADEBUG,"  %s: ERP_SLOT: %d\n",
		       wiphy_name(hw->wiphy), binfo->use_short_slot);

		if(vif->type == NL80211_IFTYPE_AP)
		{		
			/* set short slot time or long slot time */
			if(binfo->use_short_slot)
			{		
				wla_cfg(SET_SLOTTIME,SLOTTIME_9US);
				slot_time = SLOTTIME_9US;
			}
			else
			{		
				wla_cfg(SET_SLOTTIME,SLOTTIME_20US);
				slot_time = SLOTTIME_20US;
			}
		}
		else if(vif->type == NL80211_IFTYPE_STATION)
		{
			if(binfo->assoc)
			{
				if(binfo->use_short_slot)
					wla_cfg(SET_SLOTTIME,SLOTTIME_9US);
				else
					wla_cfg(SET_SLOTTIME,SLOTTIME_20US);
			}
			else
					wla_cfg(SET_SLOTTIME,slot_time);
		}
	}

	if (changed & BSS_CHANGED_HT) {
		WLA_DBG(WLADEBUG,"  %s: HT: op_mode=0x%x\n",
		       wiphy_name(hw->wiphy),
		       binfo->ht_operation_mode);
	}

//FIXME: Note change rate should not clear the cipher setting 
	if (changed & BSS_CHANGED_BASIC_RATES) {
		WLA_DBG(WLADEBUG,"  %s: BASIC_RATES: 0x%llx\n",
		       wiphy_name(hw->wiphy),
		       (unsigned long long) binfo->basic_rates);
		/*Set the original at the lowest basic rate*/
		for(i=LOWEST_BITRATE-1 ; i<=HIGHEST_BITRATE-1 ; i++)
		{		
			if(binfo->basic_rates & BIT(i))
			{	
				wla_chg_tx_rate(vp->bss_desc,i+1,BASIC_TX_RATE);
				break;
			}
		}

		/*Set the multicast at the highest basic rate but here use the lowest rate*/
		for(i=HIGHEST_BITRATE-1 ; i>=LOWEST_BITRATE-1 ; i--)
		{		
			if(binfo->basic_rates & BIT(i))
			{		
				wla_chg_tx_rate(vp->bss_desc,i+1,MULTICAST_TX_RATE);
				break;
			}
		}	
	}
out:
	wla_unlock();
}


static int wla_sta_add(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif,
			 struct ieee80211_sta *sta)
{
	struct wla_softc *sc = hw->priv;
	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	int ret;

	ret = wla_cfg_station(sc, sta, vif, STA_NOTIFY_ADD);
	
	if(vp->cipher_type != AUTH_CIPHER_NONE)
		wla_hal_cipher_key(sta, vif,1);

	return ret;
}

static int wla_sta_remove(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta)
{
	struct wla_softc *sc = hw->priv;
	struct wla_vif_priv *vp = (struct wla_vif_priv *)vif->drv_priv;
	
//	struct wla_sta_priv *sp = (struct wla_sta_priv *)sta->drv_priv;
//	struct wsta_cb *wcb = sp->wcb;

	if(vp->cipher_type != AUTH_CIPHER_NONE)
		wla_hal_cipher_key(sta, vif, 0);

//	printk("Del wcb:%p for station leave\n",wcb);
	wla_remove_station(sc, sta, vif);

	return 0;
}

static void wla_sta_notify(struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif,
				      enum sta_notify_cmd cmd,
				      struct ieee80211_sta *sta)
{
	struct wla_softc *sc = hw->priv;

	switch (cmd) 
	{
		case STA_NOTIFY_CHANGE:
			wla_cfg_station(sc, sta, vif, cmd);
			break;
		default:
			break;

	}
}

int mac_cw_value_to_hw_encoding(int cw_val)
{
	int encoding;

	if(cw_val>=1023)
		encoding = 10;
	else if(cw_val>=511)
		encoding = 9;
	else if(cw_val>=255)
		encoding = 8;
	else if(cw_val>=127)
		encoding = 7;
	else if(cw_val>=63)
		encoding = 6;
	else if(cw_val>=31)
		encoding = 5;
	else if(cw_val>=15)
		encoding = 4;
	else if(cw_val>=7)
		encoding = 3;
	else if(cw_val>=3)
		encoding = 2;
	else if(cw_val>=1)
		encoding = 1;
	else
		encoding = 0;

	return encoding;
}

#define MAC_AC_BK	0
#define MAC_AC_BE	1
#define MAC_AC_VI   2
#define MAC_AC_VO   3

void mac_apply_ac_parameters(int ac, int cw_min, int cw_max, int aifs, int txop)
{
	if(ac == MAC_AC_BK)
	{
		if((cw_min >= 0) && (cw_max >= 0))
		{
			MACREG_UPDATE32(CW_SET, (mac_cw_value_to_hw_encoding(cw_max) << 4) | (mac_cw_value_to_hw_encoding(cw_min)),
						CW_SET_BK_CWMIN | CW_SET_BK_CWMAX);
		}

		if(aifs >= 0)
		{
			MACREG_UPDATE32(AIFS_SET, (aifs << 16), AIFS_SET_AIFSN_BK);
		}

		if(txop >= 0)
		{
			MACREG_UPDATE32(TXOP_LIMIT, (txop), TXOP_LIMIT_BK);
		}
	}
	else if(ac == MAC_AC_BE)
	{
		if((cw_min >= 0) && (cw_max >= 0))
		{
			MACREG_UPDATE32(CW_SET, (mac_cw_value_to_hw_encoding(cw_max) << 12) | (mac_cw_value_to_hw_encoding(cw_min) << 8),
						CW_SET_BE_CWMIN | CW_SET_BE_CWMAX);
		}

		if(aifs >= 0)
		{
			MACREG_UPDATE32(AIFS_SET, (aifs << 20), AIFS_SET_AIFSN_BE);
		}

		if(txop >= 0)
		{
			MACREG_UPDATE32(TXOP_LIMIT, (txop << 8), TXOP_LIMIT_BE);
		}
	}
	else if(ac == MAC_AC_VI)
	{
		if((cw_min >= 0) && (cw_max >= 0))
		{
			MACREG_UPDATE32(CW_SET, (mac_cw_value_to_hw_encoding(cw_max) << 20) | (mac_cw_value_to_hw_encoding(cw_min) << 16),
						CW_SET_VI_CWMIN | CW_SET_VI_CWMAX);
		}

		if(aifs >= 0)
		{
			MACREG_UPDATE32(AIFS_SET, (aifs << 24), AIFS_SET_AIFSN_VI);
		}

		if(txop >= 0)
		{
			MACREG_UPDATE32(TXOP_LIMIT, (txop << 16), TXOP_LIMIT_VI);
		}
	}
	else if(ac == MAC_AC_VO)
	{
		if((cw_min >= 0) && (cw_max >= 0))
		{
			MACREG_UPDATE32(CW_SET, (mac_cw_value_to_hw_encoding(cw_max) << 28) | (mac_cw_value_to_hw_encoding(cw_min) << 24),
							CW_SET_VO_CWMIN | CW_SET_VO_CWMAX);
		}

		if(aifs >= 0)
		{
			MACREG_UPDATE32(AIFS_SET, (aifs << 28), AIFS_SET_AIFSN_VO);
		}

		if(txop >= 0)
		{
			MACREG_UPDATE32(TXOP_LIMIT, (txop << 24), TXOP_LIMIT_VO);
		}
	}
}

static int wla_conf_tx(
	struct ieee80211_hw *hw, struct ieee80211_vif *vif, u16 queue,
	const struct ieee80211_tx_queue_params *params)
{
	int ac;

	struct ieee80211_bss_conf *binfo = &vif->bss_conf;
	MAC_INFO *info = WLA_MAC_INFO;

	wla_lock();
	
	WLA_DBG(WLADEBUG,"%s:%s (queue=%d txop=%d cw_min=%d cw_max=%d "
	       "aifs=%d)\n",
	       wiphy_name(hw->wiphy), __func__, queue,
	       params->txop, params->cw_min, params->cw_max, params->aifs);

	if(queue == NL80211_TXQ_Q_VO)
		ac = MAC_AC_VO;
	else if(queue == NL80211_TXQ_Q_VI)
		ac = MAC_AC_VI;
	else if(queue == NL80211_TXQ_Q_BE)
		ac = MAC_AC_BE;
	else if(queue == NL80211_TXQ_Q_BK)
		ac = MAC_AC_BK;
	else
		goto done;

	if(vif->type == NL80211_IFTYPE_AP)
		wla_ac_paramters(ac, params->cw_max, params->cw_min, params->aifs, params->txop);
	else if((vif->type == NL80211_IFTYPE_STATION) && !info->ap_exist)
	{
		if(binfo->assoc)
			wla_ac_paramters(ac, params->cw_max, params->cw_min, params->aifs, params->txop);
	}

done:
	wla_unlock();
	return 0;
}

static int wla_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	int ret = 0;

	wla_lock();
	
	if(((int )value) >= 0)
	{
		WLA_DBG(WLADEBUG, "set rts threshold %d\n", (int) value);
		wla_cfg(SET_RTS_THRESHOLD,value);
	}
	
	wla_unlock();

	return ret;
}

int wla_set_frag_threshold(struct ieee80211_hw *hw, u32 value)
{
	int ret = 0;
	
	/*If frag threshold >=2346 the value will be -1. by hostapd*/
	if(((int)value) >= 0)
	{
		WLA_DBG(WLADEBUG, "set frag threshold %d\n", (int) value);
		
		wla_cfg(SET_FRAG_THRESHOLD,value);
	}

	return ret;
}

//mac80211 hook tx handler
static void wla_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control, struct sk_buff *skb)
{
	struct wla_softc *sc = hw->priv;
	struct wbuf *wb;
	struct ieee80211_sta *sta;
	struct wla_sta_priv *sp;
	struct wla_vif_priv *vp;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
	__le16 fc;

	(void)sc;
	wla_lock();

//debug
#if 0
	struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
	if(ieee80211_is_data(hdr->frame_control))
	{
		printk("!!!!!!!!!!!!!!!!!!!!!!! %s  DATA with wifi header\n",__func__);
		WLA_DUMP_BUF(skb->data, skb->len);
	}
#endif
	if(txi->control.vif)
	{
			;
	}
	else 
		goto fail;

	wb = wla_skb_to_wb(&skb);
	
	if(wb == NULL)
		goto fail;
	
	fc = hdr->frame_control;

	if(txi->flags & IEEE80211_TX_CTL_NO_ACK)
		wb->flags |= WBUF_TX_NO_ACK;

	if(txi->flags & IEEE80211_TX_CTL_REQ_TX_STATUS)
		wb->flags |= WBUF_TX_REQ_STATUS;
	
	if(ieee80211_is_probe_resp(fc))
		wb->flags |= WBUF_TX_INSERT_TS;

	if(ieee80211_is_action(fc) || ieee80211_is_disassoc(fc) || ieee80211_is_deauth(fc))
		wb->flags |= WBUF_TX_CHECK_PS;
	
	if(is_broadcast_ether_addr(hdr->addr1))
		wb->bcmc=1;
	
	vp = (struct wla_vif_priv *)(&txi->control.vif->drv_priv);
	wb->bss_desc = vp->bss_desc & 0x7;

	sta =(struct ieee80211_sta *)(control->sta);
	/*FIXME: maybe we can use sta_info->flags to check WLAN_STA_AUTHORIZED and remove sp->assoc*/
	if(sta)
	{	
		sp = (struct wla_sta_priv *)(&sta->drv_priv[0]);
		if(sp->assoc)
		{	
			wb->sta_idx = sp->wcb->captbl_idx;
		}
		else
			wb->sta_idx = 0;
	}
	else
		wb->sta_idx = 0;
	
	//WLA_DUMP_BUF(skb->data,skb->len);
	wb->tid = 7; 

	/*FIXME: need to add condition to determine*/
	wb->ap = 1;
	wb->wh = 1;

	if(ieee80211_is_mgmt(fc))
		wb->flags |= (WBUF_TX_BASIC_RATE|WBUF_MGMT_FRAME);
	else if(ieee80211_is_data_qos(fc))
		wb->qos = 1;

	/* FIXME: EAPOL and WAI of AP */
	if(ieee80211_is_data_present(fc))
	{
		/* WAPI doesn't need to use pairwise key to protect group key negotiation but CCMP,
		   protection is decided in hostapd or supplicant */
		wb->flags |= WBUF_TX_SPEC_RATE;
		if(!ieee80211_has_protected(fc))
			wb->flags |= WBUF_TX_NO_SECU;
	}

	wla_mgt_send(wb);
	goto done;

fail:
	dev_kfree_skb(skb);

done:
	wla_unlock();
}

#if defined(ARTHUR_DEBUG)
static int arthur_irq_counter = 0;
#define ARTHUR_SPURIOUS_IRQ_THRESHOLD	3000
u32 last_jiffies;
u32 intr_status;
#endif

static irqreturn_t wla_isr(int irq, void *data)
{
	struct wla_softc *sc = data;
	unsigned int status, mask;
	MAC_INFO *info = WLA_MAC_INFO;
	
	mask = MACREG_READ32(MAC_INT_MASK_REG);
	disable_mac_interrupts();

	status = MACREG_READ32(MAC_INT_STATUS) & ~(mask & MAC_INT_TXRX_MASK);

	if((status & MAC_INT_PRETBTT) && ((info->int_mask & MAC_INT_PRETBTT) != MAC_INT_PRETBTT))
	{
		info->pre_tbtt_int++;

		info->ts_status = MACREG_READ32(TS_INT_STATUS);
		MACREG_WRITE32(TS_INT_STATUS, info->ts_status);
		info->ts_ap_status = MACREG_READ32(TS_AP_INT_STATUS);
		MACREG_WRITE32(TS_AP_INT_STATUS, info->ts_ap_status);
		
		/* TS0 of PRE_TBTT_SW */
		if(info->ts_status)
		{
			tasklet_hi_schedule(&sc->beacon_tasklet);
		}

		#ifdef TBTT_EXCEPTION_HANDLE
        /* TBTT ERROR EXCEPTION */
        if(info->ts_ap_status & (TBTT_ER_STATUS_0 | TBTT_ER_STATUS_1))
        {
            wla_update_next_tbtt(info, info->ts_ap_status);
        }
		#endif // TBTT_EXCEPTION_HANDLE
	}
	
	if(status & MAC_INT_HW_BUF_FULL)
	{
		info->hw_buf_full++;
	}

	if(status & MAC_INT_RX_DESCR_FULL)
	{
		info->rx_desc_full++;
	}

	if(status & MAC_INT_RX_FIFO_FULL)
	{
		info->rx_fifo_full++;
	}
	
	//check if already pending, skip schedule
	if(status & MAC_INT_TXRX_MASK)
	{
#if defined	USE_TASKLET
		tasklet_schedule(&sc->irq_tasklet);
#elif defined USE_WORKQUEUE
		queue_work(sc->irq_wq, &sc->irq_work);
#elif defined USE_NAPI
	struct napi_struct *napi = &sc->napi;
	if (likely(napi_schedule_prep(napi)))
		__napi_schedule(napi);
#endif
	}

	MACREG_WRITE32(MAC_INT_CLR, status & ~MAC_INT_TXRX_MASK);

	if(info->mac_is_ready)
	{
		/* enable non tx/rx interrupts */
		if(MAC_INT_TXRX_MASK & status)
			MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask | MAC_INT_TXRX_MASK);
		else
			MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask);
	}

	return IRQ_HANDLED;
}

#if defined(USE_NAPI)
static int wla_napi_poll(struct napi_struct *napi, int budget)
{
	unsigned int status, work_done=0, work_to_do=budget;
	MAC_INFO *info = WLA_MAC_INFO;
	
	status = MACREG_READ32(MAC_INT_STATUS);
	MACREG_WRITE32(MAC_INT_CLR, status & MAC_INT_TXRX_MASK);

	if(status & (MAC_INT_SW_RTURNQ | MAC_INT_SW_RTURNQ_FULL))
	{
		wla_tx_done(info);
	}

	if(status & (MAC_INT_SW_RX | MAC_INT_RX_DESCR_FULL))
	{
		work_done = wla_rxq(info, &info->wl_rxq);
	}
	
	if(info->mac_is_ready && (work_done < work_to_do))
	{	
		napi_complete(napi);
		MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask);
	}
	
	return work_done;
}

#else
static void wla_irq_dsr(struct wla_softc *sc)
{
	unsigned int status;
	MAC_INFO *info = WLA_MAC_INFO;
#if defined USE_WORKQUEUE
	struct wla_softc *psc;
#endif
	status = MACREG_READ32(MAC_INT_STATUS);
	MACREG_WRITE32(MAC_INT_CLR, status & MAC_INT_TXRX_MASK);

	if(status & (MAC_INT_SW_RTURNQ | MAC_INT_SW_RTURNQ_FULL))
	{
		wla_tx_done(info);
	}

	if(status & (MAC_INT_SW_RX | MAC_INT_RX_DESCR_FULL))
	{
		wla_rxq(info, &info->wl_rxq);
	}
	
	status = MACREG_READ32(MAC_INT_STATUS);
	if(status & MAC_INT_TXRX_MASK)
	{
#if defined	USE_TASKLET
		tasklet_schedule(&sc->irq_tasklet);
#elif defined USE_WORKQUEUE
		psc = (struct wla_softc *)info->sc;
		queue_work(psc->irq_wq, &psc->irq_work);
#endif		
	}
	else
	if(info->mac_is_ready)
	{		
		MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask);
	}
}
#endif

static struct device_driver wla_driver = {
	.name = "wla"
};

#if !defined(ARTHUR_RANDOM_WIFI_MAC_ADDR)
const char wifi_default_mac_addr[ETH_ALEN] =  { 0x00, 0x12, 0x34, 0x56, 0x78, 0x88 };
#endif

static int wla_start(struct ieee80211_hw *hw)
{
	struct wla_softc *sc = hw->priv;
	MAC_INFO *info = WLA_MAC_INFO;

	WLA_DBG(WLADEBUG,"%s:%s\n", wiphy_name(hw->wiphy), __func__);
	
	if(__uapsd_enable)
		hw->wiphy->flags |= WIPHY_FLAG_AP_UAPSD;
	else
		hw->wiphy->flags &= ~WIPHY_FLAG_AP_UAPSD;

	if(!sc->started)
	{

//Do we need this flag to remember which index is ap when we are sta
//		sc->sta_mode_ap_index = -1;

		sc->mac_info = info;
		info->sc =(char *)sc;

		/*FIXME: Need to see the role ???*/
		info->wla_mgt_handler = &wla_mgmt_process;
		info->wla_eapol_handler = (void *)&wla_forward_to_ethernet;


		/* Setup WiFi feature*/
		{
			/* Recover detection */
			wla_set_recovery(__recovery);

			/* HW/SW PATH */
			wla_cfg(SET_FORWARD_MODE,__sw_forward);

			/* AP isolated */
			wla_set_ap_isolated(__ap_isolated);

			/* BSS isolated */
			wla_set_bss_isolated(__bss_isolated);

			/* Power saveing mode*/
			wla_cfg(SET_POWER_SAVING,__pw_save);

			/* Tx Power level*/
			info->txpowlevel = __tx_power;
		}

		if(0 != request_irq(IRQ_WIFI, wla_isr, 0, "WIFI", hw->priv))
		   panic("wla_start(): request irq failed.\n");

		wla_mac_start(sc->mac_info);

		sc->started = 1;
		info->mac_is_ready = true;

		/*WDS SETUP*/
		wla_wds_setup(sc);

		#if defined(USE_NAPI)
		napi_enable(&sc->napi);
		#endif
		wla_hw_setup();
	}

	return 0;
}

/*
 Note: When mac80211 call this func the local->open_count == 0
*/
static void wla_stop(struct ieee80211_hw *hw)
{
	struct wla_softc *sc = hw->priv;
	MAC_INFO *info = WLA_MAC_INFO;
	struct wds_peer *wds, *prev_wds;

//	wla_lock();
	/* 2016/06/01, Roger asks adding recovery mechanism in wla_stop to test TX stuck */
	wla_recover();

	if(sc->started)
	{
		sc->started = 0;
		info->mac_is_ready = 0;
		info->ap_exist = 0;
		info->ibss_exist = 0;
		
		free_irq(IRQ_WIFI, hw->priv);

		wla_mac_stop(info);

/*mac80211 should flush workqueue if we use WQ_RC ??*/
		wla_del_timer();
		/*Because cause ether hang follow eCos do not reset*/
		//wla_mac_reset(sc->mac_info);
		
		/*Follow eCos do not release*/
		//wla_mac_release(sc->mac_info);
		
		sc->mac_info = NULL;
		sc->ap_exist = 0;
		
		if(sc->wds_mode)
		{
			wds = sc->all_wds_peer;
			while(wds)
			{		
				prev_wds = wds;
				wds = wds->next;
				wla_release_sta(prev_wds->wcb);
				kfree(prev_wds);
			}
			sc->all_wds_peer = 0;
		}
		wla_mac_clean(info);

		sc->cur_basic_rates = 0;
		sc->curr_center_freq = 0;
		
		wla_acs_reset(hw);

#if defined USE_NAPI
		napi_disable(&sc->napi);
#endif
	}

//	wla_unlock();
}

static struct ieee80211_hw *wla_hw;
static struct class *wla_class;

static const struct ieee80211_ops wla_ops =
{
	.tx					= wla_tx,
	.start				= wla_start,
	.stop				= wla_stop,
	.add_interface		= wla_add_interface,
	.remove_interface	= wla_remove_interface,

//FIXME:Need to refine
	.config				= wla_config,				//rate control
	
	.configure_filter	= wla_configure_filter,

	.bss_info_changed	= wla_bss_info_changed,

//FIXME:Need to refine
	.sta_add         	= wla_sta_add,
	.sta_remove	    	= wla_sta_remove,
	.sta_notify			= wla_sta_notify,			//Remove sta; Change sta

	.conf_tx			= wla_conf_tx,				//ok
	.set_key			= wla_set_key,				//Remove key
	.set_default_unicast_key = wla_default_unicast_key,

	.ampdu_action		= wla_ampdu_action,

	.get_survey			= wla_get_survey,

	.set_rts_threshold	= wla_set_rts_threshold,	//ok
	.set_frag_threshold = wla_set_frag_threshold,	//ok
};

static const struct ieee80211_iface_limit wla_if_limits[] = {
    {.max = 2, .types = BIT(NL80211_IFTYPE_AP) 
#ifdef CONFIG_WLA_P2P
					  | BIT(NL80211_IFTYPE_P2P_GO)
#endif
					  },
    {.max = 2, .types = BIT(NL80211_IFTYPE_STATION)
#ifdef CONFIG_WLA_P2P
					  | BIT(NL80211_IFTYPE_P2P_CLIENT)
#endif
					  },

};

static const struct ieee80211_iface_combination wla_if_comb = {
    .limits = wla_if_limits,
    .n_limits = ARRAY_SIZE(wla_if_limits),
    .max_interfaces = 2, 
    .num_different_channels = 1,
    .beacon_int_infra_match = true,
};

static u32 wla_get_link(struct net_device *dev)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	int bss_desc;
	int ret = 0;
	
	for (bss_desc=0; bss_desc <= 3; bss_desc++)
	{
		if( sc->bssdev[bss_desc] == dev)
			break;
	}
	
	if(bss_desc == 4)
	{		
		ret = 0;
		goto done;
	}

	if(info->mbss[bss_desc].role == WIF_AP_ROLE)
		ret = 1;
	else if(info->mbss[bss_desc].role == WIF_STA_ROLE)
	{
		struct peer_ap *ln_ap = (struct peer_ap *)info->mbss[bss_desc].ln_ap;

		if(ln_ap)
		{
			struct wsta_cb *cb = info->wcb_tbls[ln_ap->sta_captbl];

			if(cb && cb->controlled_port)
			{
				ret = 1;
			}
		}
	}
		
done:
	return ret;
}

static const struct ethtool_ops wla_ethtool_ops = {
	.get_link			= wla_get_link,
};

static int __init wla_init(void)
{
	int err = 0;
	u8 addr[ETH_ALEN];
	struct wla_softc *sc = NULL;

    enum ieee80211_band band;
	struct page *page;
	MAC_INFO *macinfo;

	info = (MAC_INFO *)get_zeroed_page(GFP_KERNEL);
	page = virt_to_page(info);

	SetPageReserved(page);
	macinfo = info;

	if(wla_hw)
	{
		WLA_DBG(WLADEBUG,"cheetah: one instance already initialled\n");
		return -EIO;
	}

	wla_class = class_create(THIS_MODULE, "cheetah_wla");
	if(IS_ERR(wla_class))
	{
		err = PTR_ERR(wla_class);
		wla_class = NULL;
        goto failed;
	}
	
	//Need to refine struct wla_softc
    wla_hw = ieee80211_alloc_hw(sizeof(struct wla_softc), &wla_ops);
    if(!wla_hw)
	{
        WLA_DBG(WLADEBUG,"WLA: ieee80211_alloc_hw failed\n");
        err = -ENOMEM;
        goto failed;
    }

#if defined(ARTHUR_RANDOM_WIFI_MAC_ADDR)
    random_ether_addr(addr);
#else
	memcpy(addr, (u8 *) wifi_default_mac_addr, ETH_ALEN);
#endif

    sc = wla_hw->priv;
    sc->hw = wla_hw;
	sc->curr_band = IEEE80211_NUM_BANDS;	/* init. to invalid band (neither 2G nor 5G) */
	mutex_init(&sc->mutex);
	spin_lock_init(&sc->lock);

	wla_hw->extra_tx_headroom = 32;		/* for skb->data to be properly aligned */

	wla_debug_init(sc);

	wla_procfs_init(sc);
	
	wla_test_init();

    sc->dev = device_create(wla_class, NULL, 0, wla_hw, "wla");
    if(IS_ERR(sc->dev))
	{
        WLA_DBG(WLADEBUG,
               "wla: device_create "
               "failed (%ld)\n", PTR_ERR(sc->dev));
        err = -ENOMEM;
		sc->dev = NULL;
        goto failed;
    }
    sc->dev->driver = &wla_driver;
	
	macinfo->sc =(char *)sc;

    SET_IEEE80211_DEV(wla_hw, sc->dev);
    SET_IEEE80211_PERM_ADDR(wla_hw, addr);

    wla_hw->channel_change_time = 1;
    wla_hw->queues = 4;
	wla_hw->max_rates = 4;
	wla_hw->max_rate_tries = 4;
	wla_hw->max_signal = 127;

	wla_hw->max_rx_aggregation_subframes = 64;

	wla_hw->flags = IEEE80211_HW_AP_LINK_PS |
                IEEE80211_HW_HAS_RATE_CONTROL |
		IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING |
		IEEE80211_HW_AMPDU_AGGREGATION |
#ifdef CONFIG_MONTAGE_HW_REPORT_TX_ACK
		IEEE80211_HW_REPORTS_TX_ACK_STATUS |	/* disable to use probe req to keep alive */
#endif
		IEEE80211_HW_SIGNAL_DBM;

#if 0
		IEEE80211_HW_SUPPORTS_PS |
		IEEE80211_HW_PS_NULLFUNC_STACK |
#endif

    wla_hw->wiphy->interface_modes =
        BIT(NL80211_IFTYPE_STATION) |
        BIT(NL80211_IFTYPE_AP)
#ifdef CONFIG_WLA_P2P
		| BIT(NL80211_IFTYPE_P2P_GO)
		| BIT(NL80211_IFTYPE_P2P_CLIENT)
#endif
		;


	wla_hw->wiphy->iface_combinations = &wla_if_comb;
	wla_hw->wiphy->n_iface_combinations = 1;

	wla_hw->wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL;

#if 0
		BIT(NL80211_IFTYPE_P2P_CLIENT) |
		BIT(NL80211_IFTYPE_P2P_GO) |
		BIT(NL80211_IFTYPE_AP_VLAN) |
		BIT(NL80211_IFTYPE_WDS) |
		BIT(NL80211_IFTYPE_MONITOR);
#endif

	wla_hw->rate_control_algorithm = NULL;

#if defined(ENABLE_BEACON_QUEUE)
	/* we do not use mac80211 layer PS buffering functions */
#else
	wla_hw->flags |= IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING;
#endif

    /* ask mac80211 to reserve space for magic */
    wla_hw->vif_data_size = sizeof(struct wla_vif_priv);
    wla_hw->sta_data_size = sizeof(struct wla_sta_priv);


//FIXME: Initiallize rate
#if 1
    memcpy(sc->channels_2ghz, wla_channels_2ghz, sizeof(wla_channels_2ghz));
    memcpy(sc->channels_5ghz, wla_channels_5ghz, sizeof(wla_channels_5ghz));

#if 0
//set up sc legacy_table ???
#if defined(DEFAULT_USE_5G_CHANNEL)
	err = arthur_init_rate_table(sc, IEEE80211_BAND_5GHZ);
#else
	err = arthur_init_rate_table(sc, IEEE80211_BAND_2GHZ);
#endif
	if(err)
	{
        printk(KERN_DEBUG "arthur: malloc rate table failed\n");
        goto failed;
	}
#endif
	wla_setup_rates(sc, IEEE80211_BAND_2GHZ);
	wla_setup_rates(sc, IEEE80211_BAND_5GHZ);

    for (band = IEEE80211_BAND_2GHZ; band < IEEE80211_NUM_BANDS; band++) 
	{
        struct ieee80211_supported_band *sband = &sc->bands[band];
        switch (band) 
		{
			case IEEE80211_BAND_2GHZ:
				sband->channels = sc->channels_2ghz;
				sband->n_channels = ARRAY_SIZE(wla_channels_2ghz);
				break;
			case IEEE80211_BAND_5GHZ:
				sband->channels = sc->channels_5ghz;
				sband->n_channels = ARRAY_SIZE(wla_channels_5ghz);
				break;
			default:
				break;
        }

		sband->bitrates = sc->rates[band];
        
		sband->ht_cap.ht_supported = true;
        sband->ht_cap.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
                            IEEE80211_HT_CAP_GRN_FLD |
							IEEE80211_HT_CAP_SGI_20 |
                            IEEE80211_HT_CAP_SGI_40 ;

        sband->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
        sband->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;

        memset(&sband->ht_cap.mcs, 0, sizeof(sband->ht_cap.mcs));
        sband->ht_cap.mcs.rx_mask[0] = 0xff;

#if defined(ENABLE_HT_TWO_STREAMS)
        // two spatial streams
		sband->ht_cap.mcs.rx_mask[1] = 0xff;
#endif
        sband->ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
#endif
		// announce MCS-32
		sband->ht_cap.mcs.rx_mask[4] = 0x01;

#if defined(SUPPORT_5G_CHANNEL)
		wla_hw->wiphy->bands[band] = sband;
#else
		if(band == IEEE80211_BAND_2GHZ)
			wla_hw->wiphy->bands[band] = sband;
		else 
			wla_hw->wiphy->bands[band] = NULL;
#endif
    }

    err = ieee80211_register_hw(wla_hw);
    if(err < 0)
	{
        WLA_DBG(WLADEBUG,"WLA: "
               "ieee80211_register_hw failed (%d)\n", err);
        goto failed;
    }

	/* Instead of cfg80211_ops */
	wla_cfg80211_ops(wla_hw);

		//sc->irq_wq_rc = create_rt_workqueue("cta_wla_rc");
		INIT_DELAYED_WORK(&sc->hw_sync_work, wla_hw_sync_work);
		/*For tx fail check*/
		init_timer(&timer_g);
		timer_g.data = 0;
		timer_g.function = (void *)wla_txfail_timer;

#ifdef CONFIG_CHEETAH_SMART_CONFIG
        INIT_DELAYED_WORK(&sc->smartcfg_work, wla_chgchan_inform);
        INIT_DELAYED_WORK(&sc->smartcfg_usrwork, smart_inform_user);
		memset(&sc->smrtcfg, 0, sizeof(struct smartcfg));
#endif

	wla_mac_reset(macinfo);

	/* Move rf_init and bb_init from wla_start to wla_init*/
	#if defined(DEFAULT_USE_5G_CHANNEL) && defined(SUPPORT_5G_CHANNEL)
		if(sc->curr_band != IEEE80211_BAND_5GHZ)
		{
			if(rf_init_11a());	/* XXX: to fix */
				goto failed;
			sc->curr_band = IEEE80211_BAND_5GHZ;
		}
	#else
		if(sc->curr_band != IEEE80211_BAND_2GHZ)
		{
			if(rf_init())
				goto failed;
			sc->curr_band = IEEE80211_BAND_2GHZ;
		}
	#endif

		if(wla_mac_init(macinfo) < 0)
			return -EIO;

		bb_init();

#if defined (USE_TASKLET)
		tasklet_init(&sc->irq_tasklet, (void (*)(unsigned long))
				wla_irq_dsr, (unsigned long)wla_hw->priv);
#elif defined USE_WORKQUEUE
		sc->irq_wq = create_rt_workqueue("cta_wla");
		INIT_WORK(&sc->irq_work, (work_func_t)wla_irq_dsr);
#elif defined USE_NAPI
		init_dummy_netdev(&sc->napi_dev);
		netif_napi_add(&sc->napi_dev, &sc->napi, wla_napi_poll,
			   RX_DESCRIPTOR_COUNT);
#endif
		/*
			Beacon:Use high tasklet
		*/
		tasklet_init(&sc->beacon_tasklet, (void (*)(unsigned long))
				wla_beacon_t, (unsigned long)wla_hw->priv);

		WLA_DBG(WLADEBUG, "%s: hwaddr %pM registered\n",
			wiphy_name(wla_hw->wiphy),
			wla_hw->wiphy->perm_addr);

	br_forward_lookup_hook = wla_sta_forward_path;
	monitor_wifi_status_hook = wla_mac_status;
	set_wifi_recover_hook = wla_set_recover;
	return 0;

failed:
	if(wla_hw)
	{
		wla_procfs_exit();
		wla_debug_exit();
		wla_test_exit();
		wla_free_sc(sc);

		ieee80211_free_hw(wla_hw);
		wla_hw = NULL;
	}

	if(wla_class)
	{
		class_destroy(wla_class);
		wla_class = NULL;
	}

//FIXME: Need to Modify
//	arthur_rate_control_unregister();
	return err;
}


static void __exit wla_exit(void)
{
	struct wla_softc *sc;
	struct page *page;
	MAC_INFO *info;

	if(NULL==wla_hw)
	{   
		WLA_DBG(WLADEBUG, "WLA: not initialled\n");
		return;
	}

	if(wla_hw)
	{
		sc = wla_hw->priv;

		info = sc->mac_info;

#if defined USE_TASKLET
		tasklet_kill(&sc->irq_tasklet);
#elif defined USE_WORKQUEUE
		flush_workqueue(sc->irq_wq);
		destroy_workqueue(sc->irq_wq);
#endif
		tasklet_kill(&sc->beacon_tasklet);

//FIXME: Need to add proc.c and debugfs.c
		wla_procfs_exit();
		wla_debug_exit();
		wla_test_exit();

		wla_free_sc(sc);

		ieee80211_unregister_hw(wla_hw);

		ieee80211_free_hw(wla_hw);
		wla_hw = NULL;
	}

	if(wla_class)
	{
		class_destroy(wla_class);
		wla_class = NULL;
	}

	if(info)
	{
		page = virt_to_page(info);
		ClearPageReserved(page);
		free_page((unsigned long)info);
	}

//FIXME: Need to modify rate
//	arthur_rate_control_unregister();
}


late_initcall(wla_init);
module_exit(wla_exit);
MODULE_LICENSE("GPL");

