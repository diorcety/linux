#ifdef CONFIG_CHEETAH_SMART_CONFIG
/* define this for Cheetah A0/A1/A2 */
//#define CHEETAH_A2
#if !defined(CHEETAH_A2)
/* use new code for Cheetah A3 and above */
#include "smartcfg2.c"
#else
/*=============================================================================+
|                                                                              |
| Copyright 2015                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
#include <os_compat.h>
#include <linux_wla.h>
#include <mac_common.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <asm/mach-cheetah/common.h>
#include <wla_debug.h>
#include <rf.h>
#include <wla_util.h>
#include <net/regulatory.h>
#include <net/cfg80211.h>

#define SC_DBG(...) printk(KERN_NOTICE __VA_ARGS__)
//#define SC_DBG(...) printk(KERN_EMERG __VA_ARGS__)
//#define SC_DBG(...)

#define KEEP_ADDR_LOOKUP_DATA

#define NEW_FRAME_DISPATCHER

#define CHANNEL_HOPPING

#define CHANNEL_BLACKLIST

#define ENABLE_40MHZ_CHANNEL

extern MAC_INFO *info;
u32 __smartcfg_enable = 0;
u32 __smartcfg_dbgmask = 0;
u32 __smartcfg_filter_mode = 0;
u8 *cfgdata = NULL;
u8 ampdu_sniffer_captbl_idx;

static unsigned short curr_center_freq;
static unsigned char curr_channel;
static char bandwidth_type;

#define MAX_CFGDATA_SIZE 256
unsigned long cfgdata_valid[MAX_CFGDATA_SIZE / sizeof(unsigned long)];

#define SMRTCFG			"/lib/wdk/omnicfg"
#define SMRT_PATH		"PATH=/usr/sbin:/usr/bin:/sbin:/bin:/lib/wdk"
#define SMRTDONE		"done"
#define SMRUSRSTOP		"state"

#define CHGCHANDELAY                            (HZ / 2)
#define SMART_DECODE_STATUS_CHECK_INTERVAL      (CHGCHANDELAY * 5)

static void smrt_data_decode(unsigned char *mac, struct smartcfg *cfg, unsigned int len);

#define SIGNATURE_MAC_COUNT 2

u8 smrt_chan_multicast_addr_list[SIGNATURE_MAC_COUNT][6] = {
    {0x01, 0x00, 0x5E, 0x63, 0x68, 0x6E},
    {0x01, 0x00, 0x5E, 0x6E, 0x65, 0x6C},
};

u8 smrt_cfgdata_addr[6] = {0x01, 0x00, 0x5E, 0x73, 0x69, 0x00};

#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
//#define MULTICAST_PATTERN
#ifdef MULTICAST_PATTERN
u8 pattern_addr_list[3][6] = {
	{0x01, 0x00, 0x5E, 0x00, 0x00, 0xFE},
	{0x01, 0x00, 0x5E, 0x01, 0x00, 0xFE},
	{0x01, 0x00, 0x5E, 0x02, 0x00, 0xFE},
};
#endif
#endif

static inline int is_cfgdata_addr(unsigned char *mac)
{
    return ((smrt_cfgdata_addr[0]==mac[0])
            &&(smrt_cfgdata_addr[1]==mac[1])
            &&(smrt_cfgdata_addr[2]==mac[2])
            &&(smrt_cfgdata_addr[3]==mac[3])
            &&(smrt_cfgdata_addr[4]==mac[4]));
}

static inline unsigned char get_cfgdata_idx(unsigned char *mac)
{
    return mac[5];
}

#if defined(CHEETAH_A2)
static void smart_get_sta(char *sta_addr, char bss_desc, struct wsta_cb *wcb, char mode)
{
    sta_cap_tbl* captbl;
	sta_cap_tbl* rate_captbl;
    int index, i;
    int addr_index;
    sta_basic_info bcap;
    sta_tx_status_tbl *tx_tbls;
    sta_rx_status_tbl *rx_tbls;
    ucast_qinfo *qinfos;
    struct peer_ap *ap;
	int flag;

    /* always allocate QOS table size, even for non-QOS STA */
    int tx_tbl_num = QOS_TX_STATUS_TBLS;
    int rx_tbl_num = QOS_RX_STATUS_TBLS;
    int mpdu_win_num = DEFAULT_LEGACY_RX_MPDU_WINDOW;

    if (!wcb)
    {
        panic("forget to assign wcb!!\n");
    }

	flag = ds_table(mode);
    bcap.val = 0;
    /* find duplicated address and release it */
    index = mac_addr_lookup_engine_find(sta_addr, (int)&addr_index, 0, flag);
    if (index >= 0)
    {
        WLA_DBG(WLADEBUG, "duplicated addr and link to captbl[%d], release it!\n", index);
        mac_addr_lookup_engine_update(info, addr_index, 0, BY_ADDR_IDX);
    }

    if (0 > (index = sta_cap_get_free(info)))
        return;

    ampdu_sniffer_captbl_idx = index;
    SC_DBG("Use captbl %d for sniffer mode:%d\n", ampdu_sniffer_captbl_idx,mode);

    captbl = &info->sta_cap_tbls[index];

    captbl->BG = 1;
    captbl->gsn = 1;
    captbl->QOS = 0;
    captbl->nsd = 1;    // not sounding

    if (wcb->qos)
    {
        captbl->NE = 1;
        captbl->gsn = 0;
        captbl->QOS = 1;

        tx_tbl_num = QOS_TX_STATUS_TBLS;
        rx_tbl_num = QOS_RX_STATUS_TBLS;
        mpdu_win_num = DEFAULT_QOS_RX_MPDU_WINDOW;
    }

    /*	
        | tx_tbl |
        | rx_tbl |
        | tx_baps_qinfos |
    */
#ifdef CONFIG_SRAM_WLA
    tx_tbls = &all_tx_tbls[index*QOS_TX_STATUS_TBLS];
    rx_tbls = &all_rx_tbls[index*QOS_RX_STATUS_TBLS];
    qinfos =  &all_qinfos[index*QOS_TX_STATUS_TBLS];

    memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS);
    memset(rx_tbls, 0, sizeof(sta_rx_status_tbl)*QOS_RX_STATUS_TBLS);
#else
    tx_tbls = (sta_tx_status_tbl *)WLA_MALLOC(sizeof(sta_tx_status_tbl) * tx_tbl_num +
                                              sizeof(sta_rx_status_tbl) * rx_tbl_num +
                                              sizeof(ucast_qinfo) * tx_tbl_num,
                                              DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN);
    rx_tbls = (sta_rx_status_tbl*)(&tx_tbls[tx_tbl_num]);
#endif	// CONFIG_SRAM_WLA
    for (i=0; i<tx_tbl_num; i++)
    {
        tx_tbls[i].long_retry = HT_LONG_RETRY_DEFAULT;
        tx_tbls[i].short_retry = HT_SHORT_RETRY_DEFAULT;
        tx_tbls[i].ack_policy = MAC_ACK_POLICY_NORMAL_ACK;
    }

    captbl->TX_tbl_addr = (u32) PHYSICAL_ADDR(tx_tbls);
    captbl->RX_tbl_addr = (u32) PHYSICAL_ADDR(rx_tbls);

    /* these settings will be used when toggling gsn in TX descr */
    captbl->gack_policy = MAC_ACK_POLICY_NORMAL_ACK;
    captbl->glong_retry = LEGACY_LONG_RETRY_DEFAULT;
    captbl->gshort_retry = LEGACY_SHORT_RETRY_DEFAULT;

    captbl->bssid = bss_desc; /* why set it in cap-table ? */

    captbl->MPDU_spacing = wcb->min_mpdu_start_spacing;
    /* AP manager assign wcb */
    wcb->captbl_idx = index;
    info->wcb_tbls[index] = wcb;
    wla_rc_init(info, wcb); 

    bcap.bit.bssid = bss_desc;
    bcap.bit.vld = 1;

    bcap.bit.tosw = 1;
    bcap.bit.rx_ampdu = 255;
    bcap.bit.eth_header = 0;
    /* The cached cap-table should write back before inserting addr-table */
//	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));

    /* write back TX and RX part of stacap */
//	mac_stacap_cache_sync(index, true);
//	mac_rx_linkinfo_cache_sync();

    /* [31:8] is basic capability, [7:0] is lookup index */
    addr_index = mac_addr_lookup_engine_update(info, (int)sta_addr, (bcap.val | index), flag);

    wcb->addr_idx = addr_index;
    wcb->hw_cap_ready = 1;

    ap = 0;
	switch(mode)
    {
		case LINKED_AP:
			captbl->ess_idx = addr_index; 
			captbl->parent_ap = 1;
			captbl->AP = 1;

			ap = new_peer_ap(info, MAT_LINKED_AP);
			ap->sta_captbl = index;
			ap->bss_desc = bss_desc;
			info->mbss[(u32)bss_desc].ln_ap = ap;
			/* FIXED ME: update BASIC_RX_RATE only */
			i = info->mbss[(u32)bss_desc].tx_rate[BASIC_TX_RATE].rate_captbl;
			rate_captbl = &info->rate_tbls[i];
			rate_captbl->ess_idx = addr_index; 
			rate_captbl->parent_ap = 1;
			rate_captbl->AP = 1;
			break;

#if 0
        /* STA MODE */
        captbl->parent_ap = 0;
        captbl->AP = 0;
        info->mbss[(u32)bss_desc].sta_count++;
#endif
		default:
			/* STA MODE */
			captbl->parent_ap = 0;
			captbl->AP = 0;
			info->mbss[(u32)bss_desc].sta_count++;
			break; 
    }
    wcb->linked_ap = ap;
    wcb->mode = mode;
    wcb->bss_desc = bss_desc;

//	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));
//	mac_stacap_cache_sync(index, true);

#if defined(WLA_PSBA_QUEUE)
    /* initialize psbaq for BA-noagg/PS/AMPDU */
#ifndef CONFIG_SRAM_WLA
    qinfos = (ucast_qinfo *)&rx_tbls[rx_tbl_num];
#endif

    for (i=0; i<tx_tbl_num; i++)
    {
        memset(&qinfos[i], 0, sizeof(ucast_qinfo));
        qinfos[i].tx_head_ptr = 0xffff;
        qinfos[i].tx_tail_ptr = 0xffff;
        qinfos[i].tx_queue_len = MAX_PS_QUEUE_LENGTH;
    }

    info->sta_qinfo_tbls[index].qinfo_addr = PHYSICAL_ADDR((char *)qinfos);
    info->sta_qinfo_tbls[index].sta_idx = addr_index;
    info->sta_qinfo_tbls[index].qos = wcb->qos; 
    info->sta_qinfo_tbls[index].vld = 1;

    DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));

    /* write back TX and RX part of stacap */
    mac_stacap_cache_sync(index, true);
    mac_rx_linkinfo_cache_sync();
#endif

    return; 
}

/* Non reentrant function */
int smrt_release_sta(struct wsta_cb *wcb)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	int captbl_idx;

	if(wcb == 0)
		return 0;

	if(wcb->hw_cap_ready == 0)
		return 0;
	
	captbl_idx = wcb->captbl_idx;
	captbl = &info->sta_cap_tbls[captbl_idx];
	if(!captbl->valid)
	{
		WLA_DBG(WLADEBUG, "???%s(%d), captbl->valid=%d\n", __func__, __LINE__, captbl->valid);
		return 0;
	}

	//DCACHE_FLUSH((unsigned int)captbl, sizeof(sta_cap_tbl));  /* captbl should be located non-cached memory */
	captbl = (sta_cap_tbl *)NONCACHED_ADDR(captbl);

	captbl->valid = 0;

	/* write back TX and RX part of stacap */
	mac_stacap_cache_sync(captbl_idx, true);
	mac_rx_linkinfo_cache_sync(); 

	wla_lock();
	wcb->hw_cap_ready = 0;
	wcb->sta = 0;
	wla_unlock();

	return 0;
}

static void smart_ampdu(unsigned char *addr)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    struct wsta_cb *wcb=NULL;
    sta_basic_info bcap;

    wla_lock();
    if (!cfg->wcb)
    {
        if ((!(is_zero_ether_addr(addr)))&&(0==(addr[0] & 0x01)))
        {
            wcb = wla_get_wcb();
            if(wcb)
            {
                smart_get_sta(addr, cfg->bss_desc, wcb, 0);
                cfg->wcb = wcb;
            }
            else
                goto done;
        }
        else
            goto done;
    }

    if (mac_addr_lookup_engine_find(addr, 0, 0, 0) < 0)
    {
        SC_DBG("BA-session setup %pM chan:%d\n", addr, cfg->cur_chan);
        mac_addr_lookup_engine_find(0, cfg->wcb->addr_idx, (int *)&bcap, BY_ADDR_IDX); 
        mac_addr_lookup_engine_update(info, (int)addr, bcap.val, 0);
    }

done:   
    wla_unlock();
}
#endif

void smart_inform_user(struct work_struct *work)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;

    char *argv[5] = {NULL, NULL, NULL, NULL, NULL};
    char *envp[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
    unsigned int i;
#if defined(KEEP_ADDR_LOOKUP_DATA)
    sta_basic_info bcap;
    unsigned char addr[6];
#endif     

    int data_len, mode, ssid_len, pass_len;
    int err = 0;

    wla_lock();

#if defined(KEEP_ADDR_LOOKUP_DATA)
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
    {
        mac_addr_lookup_engine_find(addr, i, &bcap.val, BY_ADDR_IDX);
        if(bcap.bit.vld)
        {
            if((0==(addr[0] & 0x01)) && (bcap.bit.captbl_idx==ampdu_sniffer_captbl_idx))
            {
                SC_DBG("Remove addr table %d %pM %08x\n", i, addr, bcap.val);
                mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX);
            }
            else
            {
                bcap.bit.tosw = 0;
                mac_addr_lookup_engine_update(info, i, bcap.val, BY_ADDR_IDX);
            }
        }
    }
#else
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
        mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX);
#endif

#if 1
    if(cfg->wcb)
    {    
        smrt_release_sta(cfg->wcb);
        wla_post_release_sta(cfg->wcb);
        cfg->wcb = NULL;
    }
#endif

#if defined(CHEETAH_A2)
    MACREG_UPDATE32(RTSCTS_SET, 0, LMAC_FILTER_ALL_PASS);
    MACREG_UPDATE32(ERR_EN, 0, ERR_EN_DATA_TA_MISS_TOCPU);
    MACREG_UPDATE32(RX_ACK_POLICY, RX_AMPDU_REORDER_ENABLE, RX_AMPDU_REORDER_ENABLE);
    rf_write(0,0x3E954);
    rf_write(0x15,0x3000);
    rf_write(0x19,0x3555);
#else
    MACREG_UPDATE32(RTSCTS_SET, 0, LMAC_FILTER_ALL_PASS);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_SNIFFER_MODE);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);
#endif

    info->curr_center_freq = curr_center_freq;
    info->curr_channel = curr_channel;
    info->bandwidth_type = bandwidth_type;

    SC_DBG("Change back to channel %d %d\n", info->curr_channel, info->bandwidth_type);
    wla_set_channel(info);
    wla_set_bandwidth_type(info);

    /* inform to user-space */
    argv[0] = SMRTCFG; 
    envp[0] = SMRT_PATH;

    if (cfg->state == SMART_DONE)
    {
        data_len = cfgdata[0] + 1;
        mode = cfgdata[1];
        ssid_len = cfgdata[2];
        if(data_len > (ssid_len + 3))
            pass_len = cfgdata[3 + ssid_len];
        else
            pass_len = 0;

        cfgdata[3 + ssid_len] = 0;
        cfgdata[4 + ssid_len + pass_len] = 0;

        argv[1] = SMRTDONE; 

        envp[1] = kmalloc((sizeof("MODE=") + 2), GFP_ATOMIC);
        if(envp[1]==NULL)
            err = 1;
        else
            //sprintf(envp[1], "MODE=%d", mode);
            snprintf(envp[1],(sizeof("MODE=") + 2), "MODE=%d", mode);

        SC_DBG("envp[1] %s\n", envp[1]);

        envp[2] = kmalloc((sizeof("SSID=")+ssid_len+1), GFP_ATOMIC);
        if(envp[2]==NULL)
            err = 1;
        else
            //sprintf(envp[2], "SSID=%s", &cfgdata[3]);
            snprintf(envp[2],(sizeof("SSID=")+ssid_len+1), "SSID=%s", &cfgdata[3]);

        SC_DBG("envp[2] %s\n", envp[2]);

        if(pass_len)
        {
            envp[3] = kmalloc((sizeof("PASS=")+pass_len+1), GFP_ATOMIC);
            if(envp[3]==NULL)
                err = 1;
            else
                //sprintf(envp[3], "PASS=%s", &cfgdata[4+ssid_len]);
                snprintf(envp[3],(sizeof("PASS=")+pass_len+1), "PASS=%s", &cfgdata[4+ssid_len]);

            SC_DBG("envp[3] %s\n", envp[3]);
        }
    }
    else if (cfg->state == SMART_USRSTOP)
    {
        argv[1] = SMRUSRSTOP;
        envp[1] = NULL;
    }

    if(cfgdata)
    {
        kfree(cfgdata);
        cfgdata = NULL;
    }

    __smartcfg_enable = 0;

    wla_unlock();

    if(!err)
        call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);

    if (envp[1])
        kfree(envp[1]);
    if (envp[2])
        kfree(envp[2]);
    if (envp[3])
        kfree(envp[3]);

    return;
}

void smart_start(struct smartcfg *cfg, int chansec)
{
    struct wla_softc *sc = info->sc;
    unsigned char channel, bss_desc;
    unsigned int i;
#if defined(KEEP_ADDR_LOOKUP_DATA)
    sta_basic_info bcap;
#endif

    wla_lock();

    if(__smartcfg_enable)
    {
        SC_DBG("smart_start(): already started, ignored\n");
        wla_unlock();
        return;
    }

    __smartcfg_filter_mode = 0;

    cfgdata = kzalloc(sizeof(u8) * (MAX_CFGDATA_SIZE + 1), GFP_ATOMIC);
    if(cfgdata==NULL)
    {
        SC_DBG("smart_start(): cfgdata malloc failure\n");
        wla_unlock();
        return;
    }

    SC_DBG("Smart-config start\n");

    curr_center_freq = info->curr_center_freq;
    curr_channel = info->curr_channel;
    bandwidth_type = info->bandwidth_type;
    ampdu_sniffer_captbl_idx = 0xff;
    memset(cfgdata_valid, 0, sizeof(cfgdata_valid));

    __smartcfg_enable = 1;

#if defined(KEEP_ADDR_LOOKUP_DATA)
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
    {
        mac_addr_lookup_engine_find(NULL, i, &bcap.val, BY_ADDR_IDX);
        if(bcap.bit.vld)
        {
            bcap.bit.tosw = 1;
            mac_addr_lookup_engine_update(info, i, bcap.val, BY_ADDR_IDX);
        }
    }
#else
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
        mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX);
#endif

//	cancel_delayed_work(&sc->smartcfg_work);
    channel = cfg->cur_chan;
    bss_desc = cfg->bss_desc;
    memset(cfg, 0, sizeof(struct smartcfg));

    cfg->num_of_channel = 11;
    if(cfg80211_regdomain)
    {
        SC_DBG("Regulatory domain %c%c\n", cfg80211_regdomain->alpha2[0], cfg80211_regdomain->alpha2[1]);
        if((cfg80211_regdomain->alpha2[0]=='C')&&(cfg80211_regdomain->alpha2[1]=='N'))
        {
            SC_DBG("13 channels\n");
            cfg->num_of_channel = 13;
        }
    }

    cfg->fixchan = chansec;
    if (chansec == 2)
        cfg->cur_chan = 4;
    else if (chansec == 3)
        cfg->cur_chan = 8;
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
	else if (chansec == 4) {
		cfg->cur_chan = 0;
	}
#endif
    else
        cfg->cur_chan = channel;
    cfg->bss_desc=bss_desc;

#if defined(CHEETAH_A2)
    /*stop tx & rx still can normally receive frame*/
    rf_write(0,0x3E9DF);
    rf_write(0x15,0x1000);
    rf_write(0x19,0x27555);

    MACREG_UPDATE32(RTSCTS_SET, LMAC_FILTER_ALL_PASS, LMAC_FILTER_ALL_PASS);
    MACREG_UPDATE32(ERR_EN, ERR_EN_DATA_TA_MISS_TOCPU, ERR_EN_DATA_TA_MISS_TOCPU);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);
#else
    MACREG_UPDATE32(RTSCTS_SET, LMAC_FILTER_ALL_PASS, LMAC_FILTER_ALL_PASS);
    MACREG_UPDATE32(RX_ACK_POLICY, RX_SNIFFER_MODE, RX_SNIFFER_MODE);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);
#endif

    __smartcfg_filter_mode = 1;

    cfg->total = jiffies;

#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
	if(chansec != 4)
#endif
		ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_work, 0);
    wla_unlock();
}

static void smart_decode(char *data, struct smartcfg *cfg,  unsigned int len)
{
    struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)data;
    __le16 fc;

    fc = hdr->frame_control;
    if (ieee80211_is_data(fc))
    {
        if (is_multicast_ether_addr(hdr->addr3) && (!memcmp(hdr->addr2, cfg->ta, 6)))
        {
            if(ieee80211_is_data_qos(fc))
                smrt_data_decode(hdr->addr3, cfg, len);
            else
                smrt_data_decode(hdr->addr3, cfg, len+2);
        }
    }

    return;
}

static void smart_signature_check(char *mac, struct smartcfg *cfg)
{
    int i;

    for (i=0; i<SIGNATURE_MAC_COUNT; i++)
    {
        if (!memcmp(smrt_chan_multicast_addr_list[i], mac, 6))
            cfg->signature_count++;
    }
}

static int smrt_chan_chkaddr(char *mac, struct smartcfg *cfg, unsigned int len)
{
    int i;
    int ret = 0;

    for (i=0; i<SIGNATURE_MAC_COUNT; i++)
    {
        if (cfg->macmatch & (1<<i))
            continue;
        if (!memcmp(smrt_chan_multicast_addr_list[i], mac, 6)) // make sure channel.
        {
            SC_DBG("Signature %pM matched\n", mac);

            cfg->macmatch |= (1 << i);
            if (cfg->macmatch == (1<<SIGNATURE_MAC_COUNT)-1)   // We found channel special multicast address
            {
                cfg->next_decode_status_check_time = jiffies + SMART_DECODE_STATUS_CHECK_INTERVAL;
                cfg->state = SMART_DECODE;
                ret = 1;
                goto done;
            }
        }
    }

    if (cfg->macmatch)
        cfg->state = SMART_REMAINCHAN;
    else
        cfg->state = SMART_INIT;

done:   
    return ret;
}

static void smrt_data_decode(unsigned char *mac, struct smartcfg *cfg, unsigned int len)
{
    unsigned char idx;
    u8 byte;

    smart_signature_check(mac, cfg);

    if (is_cfgdata_addr(mac))
    {
        if(len >= cfg->offset)
        {
            idx = get_cfgdata_idx(mac);
            byte = (u8) (len - cfg->offset);

            //SC_DBG("Idx %d Len %d Byte %02x\n", idx, len, byte);
            cfg->valid_pkt_count++;

            if(test_bit(idx, cfgdata_valid))
            {
                if(cfgdata[idx]!=byte)
                {
                    SC_DBG("Inconsistent Data[%d]=%02x %02x, reset decoded data\n", idx, (u8) cfgdata[idx], (u8) byte);

                    memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
                    cfg->datacnt = 0;
                    cfg->datalen = 0;
                }
            }
            else
            {
                set_bit(idx, cfgdata_valid);

                cfgdata[idx] = byte;
                cfg->datacnt++;
    
                SC_DBG("Data[%d]=%02x\n", idx, (u8) cfgdata[idx]);
    
                if(idx==0)
                {
                    cfg->datalen = byte + 1;                
                    SC_DBG("Config data length = %d\n", cfg->datalen);
                }
            }
        }
        else
        {
            goto Exit;
        }
    }

    if (cfg->datalen && (cfg->datacnt == cfg->datalen))
    {
        struct wla_softc *sc = info->sc;
        int i;

        cfg->total = (long)jiffies - (long)cfg->total;
        for(i=0;i<cfg->datalen;i++)
        {
            if(isprint(cfgdata[i]))
               SC_DBG("CFGDATA[%d] 0x%02x %c\n", i, cfgdata[i], cfgdata[i]);
            else
               SC_DBG("CFGDATA[%d] 0x%02x\n", i, cfgdata[i]);
        }

        __smartcfg_filter_mode = 0;

        cfg->state=SMART_DONE;
        ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_usrwork, 0);
    }

Exit:
    return;
}

static void smart_estimate(char *data, struct smartcfg *cfg,  unsigned int len)
{
    struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)data;
    __le16 fc;

    fc = hdr->frame_control;

#if defined(CHEETAH_A2)
    /*FIXME:Note we might move these check and ampdu for every pkt. to avoid other state did not do this work*/
    /*For AMPDU*/
    if (ieee80211_is_nullfunc(fc) || ieee80211_is_back(fc))
    {
        if (ieee80211_is_nullfunc(fc)) {
            if(!is_valid_ether_addr(hdr->addr2)) {
				SC_DBG("NULL FUNC add invalid %pM DROP\n", hdr->addr2);
				WLA_DUMP_BUF(hdr, len);
				return;
			}
            smart_ampdu(hdr->addr2);
		}
        else {
            if(!is_valid_ether_addr(hdr->addr1)) {
				SC_DBG("BA add invalid %pM DROP\n", hdr->addr1);
				WLA_DUMP_BUF(hdr, len);
				return;
			}
			smart_ampdu(hdr->addr1);
		}
        return;
    }
#endif

    /*We can check pkt len size*/

    if (!ieee80211_is_data(fc))
        return;

    /*should only handle multicast addr*/
    if (is_multicast_ether_addr(hdr->addr3))
    {
        if (smrt_chan_chkaddr(hdr->addr3, cfg, len))
            goto decode;
    }

    return;

decode:
    cfg->chantime = (long)jiffies - (long)cfg->total;
    memcpy(cfg->ta, hdr->addr2, ETH_ALEN);

    if(ieee80211_is_data_qos(fc))
        cfg->offset = len - SMART_BASELEN;
    else
        cfg->offset = len - SMART_BASELEN + 2;

    SC_DBG("Locked STA %pM, channel:%d base_len:%d offset:%d\n",cfg->ta, cfg->cur_chan, len, cfg->offset);
}

void smart_stop(struct smartcfg *cfg)
{
    struct wla_softc *sc = info->sc;

    wla_lock();

    if(0==__smartcfg_enable)
    {
        SC_DBG("smart_stop(): already stopped, ignored\n");
        wla_unlock();
        return;
    }

    __smartcfg_filter_mode = 0;

    cfg->state = SMART_USRSTOP;
    //cancel_delayed_work(&sc->smartcfg_work);
    cancel_delayed_work_sync(&sc->smartcfg_work);
    
    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_usrwork, 0);

    wla_unlock();
    
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
	smrtcfg_clear_aplist(0);
#endif

    SC_DBG("Smart-config stopped\n");
}

#if defined(CHANNEL_HOPPING)
const u8 scan_channel_list_b[] = { 1, 6, 11, 2, 5, 8, 3, 9, 4, 7, 10 };
const u8 scan_channel_list_d[] = { 1, 6, 11, 2, 13, 5, 8, 3, 9, 4, 12, 7, 10 };
const u8 *scan_channel_list;
u8 num_of_channel;
#endif

int has_two_secondary_channels(int channel)
{
    if(num_of_channel==13)
        return ((channel >= 5) && (channel <= 9));
    else
        return ((channel >= 5) && (channel <= 7));
}

void wla_chgchan_inform(struct work_struct *work)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    static u8 remainchan_state_count = 0;
    static u8 channel_below = 0;

    // Timeout concern
        
    wla_lock();

    if(cfg->num_of_channel==13)
    {
        num_of_channel = 13;
        scan_channel_list = scan_channel_list_d;
    }
    else
    {
        num_of_channel = 11;
        scan_channel_list = scan_channel_list_b;
    }

#if defined(CHANNEL_BLACKLIST)
Restart:
#endif

    if (cfg->state == SMART_INIT) //In this state we should change channel step by step
    {
        remainchan_state_count = 0;
        if (!cfg->swupdwn)
        {
            if (cfg->fixchan == 1) // chan 1-4
                cfg->cur_chan=cfg->cur_chan>=4?1:++cfg->cur_chan;
            else if (cfg->fixchan == 2) // chan 5-8
                cfg->cur_chan=cfg->cur_chan>=8?5:++cfg->cur_chan;
            else if (cfg->fixchan == 3) // chan 9-11
                cfg->cur_chan=cfg->cur_chan>=num_of_channel?9:++cfg->cur_chan;
            else // all chan
            {
#if defined(CHANNEL_HOPPING)
                if(cfg->channel_index >= num_of_channel)
                    cfg->channel_index = 0;

                cfg->cur_chan = scan_channel_list[cfg->channel_index];

                cfg->channel_index++;
                if(cfg->channel_index >= num_of_channel)
                    cfg->channel_index = 0;

#else
                cfg->cur_chan=cfg->cur_chan>=num_of_channel?1:++cfg->cur_chan;
#endif

#if defined(CHANNEL_BLACKLIST)
                if(cfg->black_list_count && (cfg->black_list_channel_num==cfg->cur_chan))
                {
                    cfg->black_list_count--;
                    SC_DBG("Skip channel %d\n", cfg->cur_chan);
                    goto Restart;
                }
#endif
            }

#if defined(ENABLE_40MHZ_CHANNEL)
            if (has_two_secondary_channels(cfg->cur_chan))
                cfg->swupdwn = 2;
            else
                cfg->swupdwn = 0;
#else
            cfg->swupdwn = 0;
#endif
        }

        if ((cfg->swupdwn) && has_two_secondary_channels(cfg->cur_chan))
        {
            if(((cfg->swupdwn==2) && (cfg->channel_below_first))
                ||((cfg->swupdwn==1) && (!cfg->channel_below_first)))
            {
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCB);
                SC_DBG("Chan:%d-\n",cfg->cur_chan);
                channel_below = 1;
            }
            else
            {
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCA);
                SC_DBG("Chan:%d+\n",cfg->cur_chan);
                channel_below = 0;
            }
            cfg->swupdwn--;
        }
        else
        {
#if defined(ENABLE_40MHZ_CHANNEL)
            if (cfg->cur_chan < 5)
            {    
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCA);
                SC_DBG("Chan:%d+\n",cfg->cur_chan);
            }
            else
            {    
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCB);
                SC_DBG("Chan:%d-\n",cfg->cur_chan);
            }
#else
            wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCN);
            SC_DBG("Chan:%d\n",cfg->cur_chan);
#endif
        }

        goto timer;
    }
    /*FIXME: Add error handle if we cannot enter DECODE STATE when we are in channel 5 6 7 (8 9) */
    else if (cfg->state == SMART_REMAINCHAN) //In this state we just remain on chan 
    {
        cfg->macmatch = 0;
        remainchan_state_count++;
        if(remainchan_state_count>4)
        {
            SC_DBG("Lock attempt failed\n");
            cfg->swupdwn = 0;
            cfg->state = SMART_INIT;

#if defined(CHANNEL_BLACKLIST)
            cfg->black_list_count = 1;
            cfg->black_list_channel_num = cfg->cur_chan;
#endif
        }        
        goto timer;
    }
    else if (cfg->state == SMART_DECODE)
    {
        if(time_after(jiffies, cfg->next_decode_status_check_time))
        {
            if(cfg->signature_count==cfg->signature_count_prev)
            {
                SC_DBG("Unlock the STA and try again\n");

                cfg->swupdwn = 0;
                cfg->macmatch = 0;

                /* swap the above/below change order */
                cfg->channel_below_first = (cfg->channel_below_first) ? 0 : 1;

                cfg->state = SMART_INIT;

#if defined(CHANNEL_BLACKLIST)
                cfg->black_list_count = 1;
                cfg->black_list_channel_num = cfg->cur_chan;
#endif
            }
#if defined(ENABLE_40MHZ_CHANNEL)
            else if (has_two_secondary_channels(cfg->cur_chan))
            {
                if(cfg->valid_pkt_count==cfg->valid_pkt_count_prev)
                {
                    if(channel_below)
                    {
                        wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCA);
                        SC_DBG("Chan:%d+\n",cfg->cur_chan);
                        channel_below = 0;
                    }
                    else
                    {
                        wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCB);
                        SC_DBG("Chan:%d-\n",cfg->cur_chan);
                        channel_below = 1;
                    }
                }
            }
#endif

            cfg->signature_count_prev = cfg->signature_count;
            cfg->valid_pkt_count_prev = cfg->valid_pkt_count;
            cfg->next_decode_status_check_time = jiffies + SMART_DECODE_STATUS_CHECK_INTERVAL;
        }
        goto timer;
    }
    wla_unlock();
    return;

timer:
    //cancel_delayed_work(&sc->smartcfg_work); /* XXX */
    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_work, CHGCHANDELAY);
    wla_unlock();
}

#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
void smrtcfg_clear_aplist(char flag)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    int i;
	
	if(flag)
		__smartcfg_enable = 0;

#if 0
	/*Please determine use it or not*/
	if(cfg->aplist_stop)
	{		
   		WLA_DBG(WLADEBUG, "NO CLEAR AP tables\n");
		goto done;
	}
#endif

	for(i = 0; i<MAX_PEER_AP; i++)
		info->linked_ap[i].type = 0;
	info->linked_ap_count=0;
    
	if(cfg->wcb_ap)
    {
		struct wsta_cb *wcb = cfg->wcb_ap;
        cfg->wcb_ap = NULL;
        smrt_release_sta(wcb);
        wla_post_release_sta(wcb);
    }
	
	for (i=0; i < MAX_DS_TABLE_ENTRIES; i++)
        mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX|IN_DS_TBL);

	for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
        mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX);

	if(flag)
		__smartcfg_enable = 1;
}

void wla_chgchan_fixed(struct smartcfg *cfg, int chan)
{
    wla_lock();

	if(chan != cfg->cur_chan) {
		if (has_two_secondary_channels(cfg->cur_chan))
			cfg->swupdwn = 2;
		else
			cfg->swupdwn = 0;
	}

	if (has_two_secondary_channels(cfg->cur_chan)) {
		if(cfg->swupdwn == 2)
			wla_cfg(SET_CHANNEL, chan, BW40MHZ_SCA);
		else if(cfg->swupdwn == 1)
			wla_cfg(SET_CHANNEL, chan, BW40MHZ_SCB);
		
		if(cfg->swupdwn > 0)
			cfg->swupdwn--;
	}
	else
		if(chan < 5)
			wla_cfg(SET_CHANNEL, chan, BW40MHZ_SCA);
		else
			wla_cfg(SET_CHANNEL, chan, BW40MHZ_SCB);

    cfg->cur_chan = chan;

	/*Resset ap ds table*/
	smrtcfg_clear_aplist(1);

    wla_unlock();
}

#ifdef MULTICAST_PATTERN
int is_pattern_ether_addr(const u8 *addr)
{
	int i;

	for(i=0; i<sizeof(pattern_addr_list)/6; i++)
	{
		char *pattern = (char *)&pattern_addr_list[i];
		if((addr[3]==(0xff&pattern[3])) && (addr[4]==(0xff&pattern[4])) && (addr[5]==(0xff&pattern[5])))
			return 1;
	}
	return 0;
}
#endif

static void smart_addap_tods(unsigned char *addr)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    struct wsta_cb *wcb=NULL;
    sta_basic_info bcap;
	int addr_index;
	struct wsta_cb *wcb_ap = cfg->wcb_ap;
	
	bcap.val = 0;

    wla_lock();
    if (!wcb_ap)
    {
        if ((!(is_zero_ether_addr(addr)))&&(0==(addr[0] & 0x01)))
        {
            wcb = wla_get_wcb();
            if(wcb)
            {
                smart_get_sta(addr, cfg->bss_desc, wcb, LINKED_AP);
                cfg->wcb_ap = wcb;
            }
            else
                goto done;
        }
        else
            goto done;
    }
	else {
		if ((addr_index=mac_addr_lookup_engine_find(addr, 0, 0, IN_DS_TBL)) < 0)
		{
			mac_addr_lookup_engine_find(0, wcb_ap->addr_idx, (int *)&bcap, IN_DS_TBL|BY_ADDR_IDX); 
			addr_index=mac_addr_lookup_engine_update(info, (int)addr, bcap.val, IN_DS_TBL);
		}
	}

done:   
    wla_unlock();
}

int smart_usr_inform(struct wbuf *wb, char *ptr)
{
    struct sk_buff *skb, *skb2;
    struct ieee80211_rx_status rx_status;
    struct wla_softc *sc = (struct wla_softc *)info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;

    struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)ptr;
    __le16 fc;

    fc = hdr->frame_control;

    if(ieee80211_is_beacon(fc)) {
		smart_addap_tods(hdr->addr2);
		goto done;
	}

	if(ieee80211_is_nullfunc(fc) || ieee80211_is_back(fc)) {
        if(ieee80211_is_nullfunc(fc))
            smart_ampdu(hdr->addr2);
        else        
            smart_ampdu(hdr->addr1);
		goto done;
    }

    if(!ieee80211_is_data_present(fc))
		goto done;

    skb = (struct sk_buff *)(*(unsigned int *)((char *)wb - SKB_WB_OFFSET));
    skb->data = ptr;
    skb->len = wb->pkt_len;

    if(info->w_rx_dump == 3) {
        if(is_multicast_ether_addr(hdr->addr3))
            WLA_DUMP_BUF(ptr, wb->pkt_len);
        if(is_multicast_ether_addr(hdr->addr1))
            WLA_DUMP_BUF(ptr, wb->pkt_len);
	}
#ifdef MULTICAST_PATTERN
	/*We got addr3 is multicast and match the pattern. lucky*/
    if(is_multicast_ether_addr(hdr->addr3))
		if(is_pattern_ether_addr(hdr->addr3)) {
			cfg->aplist_stop = 1;
			smart_addap_tods(hdr->addr1);
		}

	/*Find the pattern*/
	if(is_multicast_ether_addr(hdr->addr1))
		if(is_pattern_ether_addr(hdr->addr1)) {
			cfg->aplist_stop = 1;
		}
#endif
	skb2 = skb_clone(skb, GFP_ATOMIC);

    memset(&rx_status, 0, sizeof(rx_status));

    /* troy:fix channel initial error */
    if (0 == cfg->cur_chan)
        cfg->cur_chan = 1;

    rx_status.freq = wla_channels_2ghz[cfg->cur_chan-1].center_freq;
    rx_status.band = sc->curr_band;
    rx_status.signal = -bb_rssi_decode1(wb->rssi);
    rx_status.antenna = 1;
    memcpy(IEEE80211_SKB_RXCB(skb2), &rx_status, sizeof(rx_status));

    ieee80211_rx_irqsafe(sc->hw, skb2);

done:
   return 0;
}
#endif

int smart_state(char *ptr, unsigned int len)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;

    switch (cfg->state)
    {
        case SMART_INIT:
        case SMART_REMAINCHAN:
            smart_estimate(ptr, cfg, len);
            break;
        case SMART_DECODE:
            smart_decode(ptr, cfg, len);
            break;
        default:
            break;
    }

    return 0;
}

#if !defined(NEW_FRAME_DISPATCHER)
void smartcfg_frame_dispatcher(struct wbuf *wb)
{
	struct wlan_qoshdr *fm;

	if(!wb->wh)
	{
        goto Exit;
	}

	fm = (struct wlan_qoshdr *)((int)wb + wb->data_off);

	switch(fm->type)
	{
		case WLAN_FC_TYPE_MGT:
		{
			//if(info->filter & RX_FILTER_MGMT_FRAME)
			//	break;

            if(fm->subtype == WLAN_FC_SUBTYPE_PROBE_REQ)
            {    
    			wb->flags |= WBUF_MGMT_FRAME;
    
    			//WLA_DBG(WLADEBUG, "RX: MGT frame\n");
    			//WLA_DUMP_BUF(fm, 128);
    			if(info->wla_mgt_handler)
    			{
    				info->wla_mgt_handler(wb, NULL);
    				return;
    			}
            }

			break;
		}
		case WLAN_FC_TYPE_CTRL:
		{
			//if(info->filter & RX_FILTER_CTRL_FRAME)
			//		break;

			if(fm->subtype == WLAN_FC_SUBTYPE_BA)
            {
				smart_state((char *)fm, wb->pkt_len);
				//WLA_DBG(WLADEBUG, "RX: ctrl frame, length=%d\n", wb->pkt_len);
				//WLA_DUMP_BUF((char *)fm,  wb->pkt_len);
			}
			break;
		}
		//case WLAN_FC_TYPE_DATA:
		default:
			break; 
	}
Exit:
	WBUF_FREE(wb);
}
#endif

int smartcfg_process(struct rx_q *rxq)
{
    volatile rx_descriptor *rx_desc;
	volatile buf_header *bhdr=NULL;
	struct wbuf *wb, *wb1;
	u16 bhdr_head_index, bhdr_tail_index;
	u8 hw_buf;
	u16 flags;
	int count = 0;
	struct wbuf *wb_ptr;
    u8 *data;

	wla_lock();
	/* handle HW RX Queue */
	for(rx_desc = &rxq->desc_hdr[rxq->index]; (rx_desc->own == 0) && (++count <= rxq->max); 
			rx_desc = &rxq->desc_hdr[rxq->index])
	{
		/* initialize temp variables per desacriptor */
		wb = wb1 = 0;
		flags = WBUF_RX_FRAME|WBUF_FIXED_SIZE;

		bhdr_head_index = rx_desc->bhdr_head_index;
		bhdr_tail_index = rx_desc->bhdr_tail_index;

		if(!rx_desc->swb)
		{
			/* check legal range */
			if((bhdr_head_index >= info->hw_buf_headers_count)||
				(bhdr_tail_index >= info->hw_buf_headers_count))
			{
				WLA_DBG(WLAWARN, "%s(%d) Wrong head idx=%x, tail idx=%x, limit=%x\n", __func__,
					__LINE__,  bhdr_head_index, bhdr_tail_index, info->hw_buf_headers_count);
#if defined(CONFIG_WMAC_RECOVER)
				wla_rx_monitor(&info->mon_wronghead_cnt, "Wrong head");
#else
				panic("Wrong head !!!");
#endif
			}

			hw_buf = 1;
			WLA_DBG(WLAWARN, "??????HW frame's Drop cause: %x, rx_desc[%d]->pkt_length=%d\n", ((u32 *)rx_desc)[3], rxq->index, rx_desc->pkt_length);
			WLA_DUMP_BUF_16BIT((char *)rxq->desc_hdr, info->wl_rxq.max*16);
		}
		else
		{
			/* check legal range */
			if((bhdr_head_index >= info->sw_tx_bhdr_start)||
				(bhdr_head_index < info->hw_buf_headers_count)||
				(bhdr_tail_index >= info->sw_tx_bhdr_start)||
				(bhdr_tail_index < info->hw_buf_headers_count))
			{
				WLA_DBG(WLAWARN, "??????%s(%d) Wrong head idx=%x, tail idx=%x, limit=%x\n", 
					__func__, __LINE__, bhdr_head_index, bhdr_tail_index,
					info->sw_allocated_buf_headers_count);
				WLA_DBG(WLADEBUG, "rx_desc->pkt_length=%d\n", rx_desc->pkt_length);
				goto fail2;
			}

			if(rx_desc->pkt_length == 0)
			{
				/* UMAC may drop BAR with zero length to CPU */
				WLA_DBG(WLAWARN, "WARNING: zero rx_desc->pkt_length, %08x-%08x-%08x-%08x, rxq->index=%d\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3], rxq->index);

				goto fail2;
			}

            hw_buf = 0;

		}

		/* Give up all tasks once MAC shutdown */
		if(!info->mac_is_ready)
			goto fail;

        if(0==__smartcfg_filter_mode)
            goto fail;

		bhdr = idx_to_bhdr(info, bhdr_head_index);

        wb_ptr = dptr_to_wb((char *)(u32)bhdr->dptr);
        data = (u8 *)((int)wb_ptr + (int)get_bhdr_offset((buf_header *)bhdr));

#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE
		smart_usr_inform(wb_ptr, data);
		goto fail;
#endif
		if(rx_desc->drop)
		{
            smart_state(data, (0x1fff & rx_desc->pkt_length));
            goto fail;
		}

		if(((bhdr->offset_h << 6) | bhdr->offset ) == 0)
		{
			WLA_DBG(WLAWARN, "Zero data offset: %08x, %08x, %08x, %08x, bhdr=%p(%08x)\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3], bhdr, *(u32 *)bhdr);

#if defined(CONFIG_WMAC_RECOVER)
			wla_rx_monitor(&info->mon_zerodataofs_cnt, "Zero data offset");
#else
			panic("wla_rx_wlq():Zero data offset\n");
#endif
		}

#ifdef CONFIG_SRAM_WLA
#error not supported 
#endif
		if(wb)
		{
		}
		/* multiple buffers or Hardware buffers need to copy to one buffer */
		else if((bhdr_head_index != bhdr_tail_index) || (!rx_desc->swb))
		{
			goto fail;
		}
		/* signle buffer, just exchange buffer's address to gain performance */
		else
		{
#if defined(NEW_FRAME_DISPATCHER)
            struct wlan_qoshdr *fm;

            wb = dptr_to_wb((char *)(unsigned int)bhdr->dptr);
            wb->wb_next=0;
            wb->rb_next=0;

            if (wb->flags != WBUF_FIXED_SIZE)
            {
                WLA_DBG(WLAERROR, "wla_rx_wlq(): wrong wb->flags:%x, bhr_head=%d, index=%d\n", wb->flags, bhdr_head_index, rxq->index);
                WLA_MSG("~~~~~~~~wb=%p\n", wb);
#if defined(CONFIG_WMAC_RECOVER)
                wla_rx_monitor(&info->mon_wrongflag_cnt, "wrong wb->flags");
#else
                panic("wla_rx_wlq(): wrong wb->flags !!!");
#endif
            }

            if (!wb->wh)
                goto fail;

            wb->data_off = wb->data_off + WLAN_TX_MIN_RESERVED_LEADIND_SPACE;

            fm = (struct wlan_qoshdr *)((int)wb + wb->data_off);
            switch (fm->type)
            {
                case WLAN_FC_TYPE_MGT:
                    {
                        if (fm->subtype == WLAN_FC_SUBTYPE_PROBE_REQ)
                        {
                            unsigned int *bhdr_ptr;
    
                            if ((wb1 = WBUF_ALLOC(0)) == 0)
                            {
                                WLA_DBG(WLAERROR, "wla_rx_wlq(): wbuf_alloc failed\n");
                                goto fail;
                            }
    
                            wb->flags |= WBUF_MGMT_FRAME;
                            if (info->wla_mgt_handler)
                            {
                                //SC_DBG("Probe request\n");
                                info->wla_mgt_handler(wb, NULL);
                            }
    
                            bhdr_ptr = (unsigned int *)bhdr;
                            bhdr_ptr[0] = (1 << 15);
                            bhdr_ptr[1] = (int)wb_to_dptr(wb1);
    #if defined(WBUF_IS_CACHED)
                            DCACHE_INVALIDATE((unsigned int)wb1, 1600);
    #endif
                        }
                        break;
                    }
                case WLAN_FC_TYPE_CTRL:
                    {
                        if (fm->subtype == WLAN_FC_SUBTYPE_BA)
                        {
                            smart_state((char *)fm, wb->pkt_len);
                        }
                        break;
                    }
                    //case WLAN_FC_TYPE_DATA:
                default:
                    break; 
            }
#else
			unsigned int *bhdr_ptr;

			if((wb1 = WBUF_ALLOC(0)) == 0)
			{
				WLA_DBG(WLAERROR, "wla_rx_wlq(): wbuf_alloc failed\n");
				goto fail;
			}

			wb = dptr_to_wb((char *)(unsigned int)bhdr->dptr);
			wb->wb_next=0;
			wb->rb_next=0;
///
			if(wb->flags != WBUF_FIXED_SIZE)
			{
				WLA_DBG(WLAERROR, "wla_rx_wlq(): wrong wb->flags:%x, bhr_head=%d, index=%d\n", wb->flags, bhdr_head_index, rxq->index);
				WLA_DUMP_BUF_16BIT((char *)rxq->desc_hdr, info->wl_rxq.max*16);
				WLA_MSG("~~~~~~~~wb=%p\n", wb);
				WLA_DUMP_BUF_16BIT(((char *)wb)-16, 128);
#if defined(CONFIG_WMAC_RECOVER)
				wla_rx_monitor(&info->mon_wrongflag_cnt, "wrong wb->flags");
#else
				panic("wla_rx_wlq(): wrong wb->flags !!!");
#endif
			}
			wb->data_off = wb->data_off + WLAN_TX_MIN_RESERVED_LEADIND_SPACE;

			/* caution: !!! the operation should be atomic */
			bhdr_ptr = (unsigned int *)bhdr;
			bhdr_ptr[0] = (1 << 15);
			bhdr_ptr[1] = (int)wb_to_dptr(wb1);
#if defined(WBUF_IS_CACHED)
			DCACHE_INVALIDATE((unsigned int)wb1, 1600);
#endif
#endif
		}
#if !defined(NEW_FRAME_DISPATCHER)
		//wb->bss_desc = rx_desc->bssid;  /* FIXME: enable for client mode power saving test */

		wb->flags |= flags;
		if(rx_desc->hit & DA_HIT_ADDR)
			wb->sta_idx = rx_desc->addr_index;
#endif
		rxq->pkt_count++;
fail:	/* There are valid buffer headers and release to hardware. */
		mac_free_buffer_hdr(bhdr_head_index, bhdr_tail_index, hw_buf);
/* The descriptor has invalid buffer header, just return descriptor */
fail2:
		//rx_desc->bhdr_head_index = rx_desc->bhdr_tail_index = 0;
		rx_desc->own = 1;
		rxq->index = (rxq->index + 1) % rxq->max;

		MACREG_WRITE32(CPU_RD_SWDES_IND, 1);     /* kick RX queue logic as it might be pending on get free RX descriptor to write */

#if !defined(NEW_FRAME_DISPATCHER)
		/* sent the frame/packet to the upper layer */
		if(wb)
		{
			smartcfg_frame_dispatcher(wb);
		}
#endif
	}

	wla_unlock();

    return count;
}
#endif // CHEETAH_A2
#endif /* CONFIG_CHEETAH_SMART_CONFIG */
